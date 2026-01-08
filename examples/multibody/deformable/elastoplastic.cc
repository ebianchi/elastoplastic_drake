#include <fstream>
#include <memory>
#include <regex>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/examples/multibody/deformable/helpers/deform_sim_utils.h"
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
DEFINE_double(simulation_time, 1.5, "Desired duration of the simulation [s].");
DEFINE_int32(testcase, 0, "Test Case.");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

using drake::geometry::AddContactMaterial;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::systems::AddActuationRecieverAndStateSenderLcm;
using drake::systems::Context;
using drake::systems::lcm::LcmPublisherSystem;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace deformable {

int DoMain() {
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
  MpmParams mpm_params = drake::yaml::LoadYamlFile<MpmParams>(
      drake::FindResource(kRelativeParamFolder + "mpm_params.yaml")
          .get_absolute_path_or_throw());

  // Put together the diagram.
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = sim_params.dt;
  plant_config.discrete_contact_approximation = "lagged";

  // Build the simulation plant.
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);
  Parser parser(&plant, &scene_graph);

  // Add the robot and environment.
  ModelInstanceIndex robot_index;
  if (sim_params.use_franka) {
    robot_index = AddFrankaToPlant(&plant, &scene_graph, true, true, true);
  } else {
    robot_index = AddGenericRobotToPlant(&plant, &scene_graph, sim_params);
  }

  // Add the object (dough) as an MPM entity.
  AddMpmBlockToPlant(&plant, mpm_params, sim_params, FLAGS_write_files);

  plant.Finalize();

  // Set up LCM communication systems.
  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.robot_input_channel,
      lcm_channel_params.robot_state_channel, sim_params.robot_publish_rate,
      robot_index, sim_params.publish_efforts, sim_params.actuator_delay);
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

  // Visualize with meshcat.
  auto meshcat = std::make_shared<geometry::Meshcat>();
  meshcat->SetCameraPose(sim_params.camera_pose, sim_params.camera_target);
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

  // Set up the simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  // Start with the robot at the initial position.
  auto& mutable_context = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&mutable_context);
  Eigen::VectorXd q_init_robot =
      sim_params.use_franka ? kQInitFranka : sim_params.q_init_robot;
  plant.SetPositions(&plant_context, robot_index, q_init_robot);

  simulator.Initialize();
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  //   if (FLAGS_write_files) {
  //     meshcat->StartRecording();
  //     simulator.AdvanceTo(FLAGS_simulation_time);
  //     meshcat->StopRecording();
  //     meshcat->PublishRecording();
  //     std::ofstream htmlFile(kHtmlFolder + "elastoplastic.html");
  //     htmlFile << meshcat->StaticHtml();
  //     htmlFile.close();
  //   } else {
  //     simulator.AdvanceTo(FLAGS_simulation_time);
  //   }
  // Run indefinitely for simulation purposes.  Can use the shorter cutoff time
  // above for testing.
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace deformable
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::deformable::DoMain();
}
