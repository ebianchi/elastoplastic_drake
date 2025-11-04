#include <fstream>
#include <memory>
#include <regex>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/examples/multibody/deformable/mpm_points_sender.h"
#include "drake/examples/multibody/deformable/parameters/elastoplastic_lcm_params.h"
#include "drake/examples/multibody/deformable/parameters/elastoplastic_sim_params.h"
#include "drake/examples/multibody/deformable/parameters/mpm_params.h"
#include "drake/examples/multibody/deformable/robot_lcm_systems.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_point_cloud_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/subvector_pass_through.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_bool(write_files, true, "Enable dumping MPM data to files.");
DEFINE_double(simulation_time, 10.0, "Desired duration of the simulation [s].");
DEFINE_int32(testcase, 0, "Test Case.");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

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
using drake::systems::AddActuationRecieverAndStateSenderLcm;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
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

static const std::string kHandModel =
    "package://drake_models/allegro_hand_description/urdf/"
    "allegro_hand_description_right.urdf";
static const double kDoughRadius = 0.03;
static const std::string kDiagramFolder =
    "/mnt/data0/bibit/diagrams/mpm_drake/";
static const std::string kRelativeParamFolder =
    "drake/examples/multibody/deformable/parameters/";

int DoMain() {
  // int DoMain(int argc, char* argv[]) {
  // Load parameters.
  ElastoPlasticSimParams sim_params =
      drake::yaml::LoadYamlFile<ElastoPlasticSimParams>(
          drake::FindResource(kRelativeParamFolder +
                              "elastoplastic_sim_params.yaml")
              .get_absolute_path_or_throw());
  ElastoPlasticLCMChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<ElastoPlasticLCMChannels>(
          drake::FindResource(kRelativeParamFolder +
                              "elastoplastic_lcm_params.yaml")
              .get_absolute_path_or_throw());
  MPMParams mpm_params = drake::yaml::LoadYamlFile<MPMParams>(
      drake::FindResource(kRelativeParamFolder + "mpm_params.yaml")
          .get_absolute_path_or_throw());

  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = sim_params.dt;
  plant_config.discrete_contact_approximation = "lagged";

  // Build the simulation plant.
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  // Set some contact properties.
  ProximityProperties compliant_hydro_props;
  const CoulombFriction<double> surface_friction(
      mpm_params.contact_friction_mu, mpm_params.contact_friction_mu);
  AddContactMaterial(mpm_params.contact_damping, {}, surface_friction,
                     &compliant_hydro_props);
  AddCompliantHydroelasticProperties(0.01, 1e6, &compliant_hydro_props);

  // Add the hand.
  Parser parser(&plant, &scene_graph);
  ModelInstanceIndex hand_index = parser.AddModelsFromUrl(kHandModel)[0];
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
      mpm_params.points_per_cell, mpm_params.cell_side_length);

  MpmConfigParams mpm_config;
  mpm_config.domain_bits = mpm_params.domain_bits;
  mpm_config.grid_block_spacing = mpm_params.grid_block_spacing;
  mpm_config.youngs_modules = mpm_params.youngs_modulus;
  mpm_config.poisson_ratio = mpm_params.poisson_ratio;
  mpm_config.particle_yield_stress = mpm_params.particle_yield_stress;
  mpm_config.particle_plasticity = mpm_params.particle_plasticity;
  mpm_config.particle_linear_corotated = mpm_params.particle_linear_corotated;
  mpm_config.density = mpm_params.density;
  mpm_config.rpic_damping = mpm_params.rpic_damping;

  mpm_config.substep_dt = sim_params.mpm_substep;
  mpm_config.write_files = FLAGS_write_files;
  mpm_config.contact_stiffness = mpm_params.contact_stiffness;
  mpm_config.contact_damping = mpm_params.contact_damping;
  mpm_config.contact_friction_mu = mpm_params.contact_friction_mu;
  // Seems to be a boundary condition.  111 fixes the bottom z height.
  // mpm_config.mpm_bc = 111;
  deformable_model.SetMpmConfig(std::move(mpm_config));

  plant.Finalize();

  // Publishing
  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.robot_input_channel,
      lcm_channel_params.robot_state_channel, sim_params.robot_publish_rate,
      hand_index, sim_params.publish_efforts, sim_params.actuator_delay);
  auto particle_positions_sender =
      builder.AddSystem<drake::systems::MPMPointsSender>(
          "particle_positions_sender");
  auto particle_positions_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<drake::lcmt_material_points>(
          lcm_channel_params.mpm_channel, lcm,
          1.0 / sim_params.object_publish_rate));
  builder.Connect(
      plant.get_output_port(plant.deformable_model().mpm_output_port_index()),
      particle_positions_sender->get_input_port_mpm());
  builder.Connect(particle_positions_sender->get_output_port_particles(),
                  particle_positions_publisher->get_input_port());

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
  std::ignore = std::system(("mkdir -p " + kDiagramFolder).c_str());
  std::string path = kDiagramFolder + "/elastoplastic";
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
