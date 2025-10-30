#include <fstream>
#include <memory>
#include <regex>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/examples/multibody/deformable/parameters/elastoplastic_sim_params.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_point_cloud_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_bool(write_files, true, "Enable dumping MPM data to files.");
DEFINE_double(simulation_time, 10.0, "Desired duration of the simulation [s].");
DEFINE_int32(testcase, 0, "Test Case.");

using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::math::RotationMatrix;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::Body;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::PackageMap;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::gmpm::MpmConfigParams;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace {

static constexpr const char* kHandModel =
    "package://drake_models/allegro_hand_description/urdf/"
    "allegro_hand_description_right.urdf";
static const double kDoughRadius = 0.03;

int DoMain() {
  // int DoMain(int argc, char* argv[]) {
  // Load parameters.
  ElastoPlasticSimParams sim_params =
      drake::yaml::LoadYamlFile<ElastoPlasticSimParams>(
          drake::FindResource("drake/examples/multibody/deformable/parameters/"
                              "elastoplastic_sim_params.yaml")
              .get_absolute_path_or_throw());

  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = sim_params.dt;
  plant_config.discrete_contact_approximation = "lagged";

  // Build the simulation plant.
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  // Set some contact properties.
  ProximityProperties compliant_hydro_props;
  const CoulombFriction<double> surface_friction(1.0, 1.0);
  AddContactMaterial(sim_params.damping, {}, surface_friction,
                     &compliant_hydro_props);
  AddCompliantHydroelasticProperties(0.01, 1e6, &compliant_hydro_props);

  // Add the hand.
  Parser parser(&plant, &scene_graph);
  //   ModelInstanceIndex hand_index =
  parser.AddModelsFromUrl(kHandModel)[0];
  Quaterniond hand_quat = {
      sim_params.fixed_base_robot[0], sim_params.fixed_base_robot[1],
      sim_params.fixed_base_robot[2], sim_params.fixed_base_robot[3]};
  Vector3d hand_pos = {sim_params.fixed_base_robot[4],
                       sim_params.fixed_base_robot[5],
                       sim_params.fixed_base_robot[6]};
  RigidTransformd X_WH = RigidTransformd(hand_quat, hand_pos);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName(sim_params.fixed_base_robot_frame),
                   X_WH);

  // mpm stuff
  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();
  deformable_model.RegisterMpmParticle(
      {sim_params.q_init_object[4] - kDoughRadius,
       sim_params.q_init_object[5] - kDoughRadius,
       sim_params.q_init_object[6] - kDoughRadius},
      {sim_params.q_init_object[4] + kDoughRadius,
       sim_params.q_init_object[5] + kDoughRadius,
       sim_params.q_init_object[6] + kDoughRadius},
      sim_params.ppc, 1.0 / 64.0);

  MpmConfigParams mpm_config;
  mpm_config.domain_bits = 6;
  mpm_config.grid_block_spacing = 1.152;
  mpm_config.youngs_modules = 2e4;
  mpm_config.poisson_ratio = 0.4;
  mpm_config.particle_yield_stress = 1e3;
  mpm_config.particle_plasticity = true;
  mpm_config.particle_linear_corotated = false;
  mpm_config.density = 1000.0;
  mpm_config.rpic_damping = 0.2;

  mpm_config.substep_dt = sim_params.substep;
  mpm_config.write_files = FLAGS_write_files;
  mpm_config.contact_stiffness = sim_params.stiffness;
  mpm_config.contact_damping = sim_params.damping;
  mpm_config.contact_friction_mu = sim_params.friction;
  // Seems to be a boundary condition.  111 fixes the bottom z height.
  // mpm_config.mpm_bc = 111;
  deformable_model.SetMpmConfig(std::move(mpm_config));

  plant.Finalize();

  /* Add a visualizer that emits LCM messages for visualization. */
  if (sim_params.visualize_drake_sim) {
    geometry::DrakeVisualizerParams visualize_params;
    visualize_params.show_mpm =
        geometry::DrakeVisualizerParams::ShowMpmOpt::kParticleMpm;
    auto& visualizer = geometry::DrakeVisualizerd::AddToBuilder(
        &builder, scene_graph, nullptr, visualize_params);

    // NOTE (changyu): MPM shortcut port shuould be explicit connected for
    // visualization.
    builder.Connect(
        plant.get_output_port(plant.deformable_model().mpm_output_port_index()),
        visualizer.mpm_input_port());
  }

  // meshcat viz
  auto meshcat = std::make_shared<geometry::Meshcat>();
  if (FLAGS_write_files) {
    auto meshcat_params = drake::geometry::MeshcatVisualizerParams();
    meshcat_params.show_mpm =
        drake::geometry::MeshcatVisualizerParams::ShowMpmOpt::kParticleMpm;
    auto& meshcat_visualizer =
        drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
            &builder, scene_graph, meshcat, meshcat_params);
    visualization::ApplyVisualizationConfig(
        visualization::VisualizationConfig{
            .default_proximity_color = geometry::Rgba{1, 0, 0, 0.25},
            .enable_alpha_sliders = true,
        },
        &builder, nullptr, nullptr, nullptr, meshcat);

    builder.Connect(
        plant.get_output_port(plant.deformable_model().mpm_output_port_index()),
        meshcat_visualizer.mpm_input_port());
  }

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  // Draw the diagram.
  std::ignore = std::system("mkdir -p /mnt/data0/bibit/diagrams/mpm_drake/");
  std::string path = "/mnt/data0/bibit/diagrams/mpm_drake/elastoplastic";
  std::ofstream out(path);
  out << diagram->GetGraphvizString();
  out.close();
  std::regex r(" ");
  path = std::regex_replace(path, r, "\\ ");
  std::string cmd = "dot -Tsvg " + path + " -o " + path + ".svg";
  std::ignore = std::system(cmd.c_str());
  cmd = "rm " + path;
  std::ignore = std::system(cmd.c_str());

  /* Build the simulator and run! */
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  //   auto& mutable_context = simulator.get_mutable_context();
  //   auto& plant_context =
  //   plant.GetMyMutableContextFromRoot(&mutable_context);

  // plant.SetPositions(&plant_context, left_iiwa,
  // left_iiwa_initial_joint_values); plant.SetPositions(&plant_context,
  // right_iiwa,
  //                    right_iiwa_initial_joint_values);
  // plant.SetPositions(&plant_context, left_wsg, Eigen::Vector2d(-0.03, 0.03));
  // plant.SetPositions(&plant_context, right_wsg, Eigen::Vector2d(-0.03,
  // 0.03));

  simulator.Initialize();
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  if (FLAGS_write_files) {
    meshcat->StartRecording();
    simulator.AdvanceTo(FLAGS_simulation_time);
    meshcat->StopRecording();
    meshcat->PublishRecording();
    std::ofstream htmlFile("/home/bibit/drake/elastoplastic.html");
    htmlFile << meshcat->StaticHtml();
    htmlFile.close();
  } else {
    simulator.AdvanceTo(FLAGS_simulation_time);
  }

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::DoMain();
}
// int main(int argc, char* argv[]) {
//   drake::examples::DoMain(argc, argv);
// }
