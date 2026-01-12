#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/examples/multibody/deformable/parameters/elastoplastic_sim_params.h"
#include "drake/examples/multibody/deformable/parameters/mpm_params.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace deformable {

/// Path constants.
static const std::string kDiagramFolder =
    "/mnt/data0/bibit/diagrams/mpm_drake/";
static const std::string kHtmlFolder = "/mnt/data0/bibit/mpm_drake/recordings/";
static const std::string kRelativeParamFolder =
    "drake/examples/multibody/deformable/parameters/";

/// Constants for the Franka, end effector, and environment.
static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/panda_arm.urdf";
static constexpr const char* kEndEffectorModel =
    "drake/examples/multibody/deformable/models/end_effector_full.urdf";
static constexpr const char* kEndEffectorName = "end_effector_tip";
static constexpr const char* kGroundFrankaModel =
    "drake/examples/multibody/deformable/models/ground_franka.urdf";
static constexpr const char* kPlatformModel =
    "drake/examples/multibody/deformable/models/platform.urdf";
static constexpr const char* kBoxModel =
    "drake/examples/multibody/deformable/models/box.urdf";
inline const Eigen::VectorXd kQInitFranka =
    (Eigen::VectorXd(7) << 2.2, 0.8, -1.7, -2.4, 0.93, 2.04, -0.09).finished();

/// Tool attachment frame is the offset from the Panda's link7 frame to its
/// flange where an end effector can be attached.
static const Eigen::Vector3d kToolAttachmentFrame = {0, 0, 0.107};
static const Eigen::Vector3d kFrankaToGroundOffset = {0, 0, -0.029};
static const Eigen::Vector3d kFrankaToPlatformOffset = {0, 0, -0.0145};
static const Eigen::Vector3d kWorldToFrankaOffset = {0, 0, 0};
static const Eigen::Vector3d kWorldToGroundOffset =
    kWorldToFrankaOffset + kFrankaToGroundOffset;
static const Eigen::Vector3d kWorldToBoxOffset = {0.4, 0.25, 0};

/// Add a generic robot and ground model to a given multibody plant and scene
/// graph, based on the simulation parameters.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param sim_params
/// @return the ModelInstanceIndex of the robot in the plant
drake::multibody::ModelInstanceIndex AddGenericRobotToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const ElastoPlasticSimParams& sim_params);

/// Add the Franka to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param include_ee whether to include the EE
/// @param include_ground_and_platform whether to include the ground and
/// platform.
/// @param include_box whether to add a box upon which dough can fall into the
/// workspace.  Will be fixed to environment.
/// @return the ModelInstanceIndex of the Franka in the plant
drake::multibody::ModelInstanceIndex AddFrankaToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ee = true,
    const bool& include_ground_and_platform = true,
    const bool& include_box = false);

/// Add an MPM block to a given multibody plant.
/// @param plant a pointer to the MultibodyPlant
/// @param mpm_params the MPM parameters for the dough
/// @param sim_params the simulation parameters
/// @param write_files whether to write the MPM data files to disk
void AddMpmBlockToPlant(drake::multibody::MultibodyPlant<double>* plant,
                        const MpmParams& mpm_params,
                        const ElastoPlasticSimParams& sim_params,
                        const bool& write_files);

}  // namespace deformable
}  // namespace examples
}  // namespace drake
