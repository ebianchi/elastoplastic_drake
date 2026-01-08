#include "deform_sim_utils.h"

#include <iostream>

#include "common/find_resource.h"

#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace deformable {

using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::gmpm::MpmConfigParams;
using Eigen::Quaterniond;
using Eigen::Vector3d;

ModelInstanceIndex AddGenericRobotToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const ElastoPlasticSimParams& sim_params) {
  Parser parser(plant, scene_graph);

  // Add the ground.
  parser.AddModels(sim_params.ground_model);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"));

  // Add the robot.
  ModelInstanceIndex robot_index = parser.AddModels(sim_params.robot_model)[0];
  Quaterniond robot_quat = {
      sim_params.fixed_base_robot[0], sim_params.fixed_base_robot[1],
      sim_params.fixed_base_robot[2], sim_params.fixed_base_robot[3]};
  Vector3d robot_pos = {sim_params.fixed_base_robot[4],
                        sim_params.fixed_base_robot[5],
                        sim_params.fixed_base_robot[6]};
  RigidTransformd X_WR = RigidTransformd(robot_quat, robot_pos);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName(sim_params.fixed_base_robot_frame),
                    X_WR);

  return robot_index;
}

ModelInstanceIndex AddFrankaToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    const bool& include_ee,
                                    const bool& include_ground_and_platform,
                                    const bool& include_box) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  RigidTransformd X_WI = RigidTransformd::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"),
                    X_WI);

  if (include_ee) {
    parser.AddModels(FindResourceOrThrow(kEndEffectorModel));
    RigidTransformd T_EE_W =
        RigidTransformd(drake::math::RotationMatrix<double>(
                            drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
                        kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"), T_EE_W);
  }

  if (include_ground_and_platform) {
    parser.AddModels(FindResourceOrThrow(kGroundFrankaModel));
    parser.AddModels(FindResourceOrThrow(kPlatformModel));

    RigidTransformd X_F_P = RigidTransformd(kFrankaToPlatformOffset);
    RigidTransformd X_F_G_franka = RigidTransformd(kFrankaToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("ground"), X_F_G_franka);
    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("platform"), X_F_P);
  }

  if (include_box) {
    parser.AddModels(FindResourceOrThrow(kBoxModel));
    RigidTransformd X_WB = RigidTransformd(kWorldToBoxOffset);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("box"), X_WB);
  }

  return franka_index;
}

void AddMpmBlockToPlant(drake::multibody::MultibodyPlant<double>* plant,
                        const MpmParams& mpm_params,
                        const ElastoPlasticSimParams& sim_params,
                        const bool& write_files) {
  DeformableModel<double>& deformable_model = plant->mutable_deformable_model();
  deformable_model.RegisterMpmParticle(
      {sim_params.q_init_object[4] - sim_params.object_half_widths[0],
       sim_params.q_init_object[5] - sim_params.object_half_widths[1],
       sim_params.q_init_object[6] - sim_params.object_half_widths[2]},
      {sim_params.q_init_object[4] + sim_params.object_half_widths[0],
       sim_params.q_init_object[5] + sim_params.object_half_widths[1],
       sim_params.q_init_object[6] + sim_params.object_half_widths[2]},
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
  mpm_config.write_files = write_files;
  mpm_config.contact_stiffness = mpm_params.contact_stiffness;
  mpm_config.contact_damping = mpm_params.contact_damping;
  mpm_config.contact_friction_mu = mpm_params.contact_friction_mu;
  // Seems to be a boundary condition.  111 fixes the bottom z height.
  // mpm_config.mpm_bc = 111;
  deformable_model.SetMpmConfig(std::move(mpm_config));
}

}  // namespace deformable
}  // namespace examples
}  // namespace drake
