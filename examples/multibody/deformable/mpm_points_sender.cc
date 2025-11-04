#include <algorithm>
#include <iostream>

#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/examples/multibody/deformable/mpm_points_sender.h"
#include "drake/geometry/kinematics_vector.h"

namespace drake {
namespace systems {

using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MPMPointsSender::MPMPointsSender(std::string name) {
  this->set_name(name);

  input_port_mpm_ = this->DeclareAbstractInputPort(
                            "mpm", Value<multibody::gmpm::MpmPortData<
                                       multibody::gmpm::config::GpuT>>())
                        .get_index();

  output_port_ = this->DeclareAbstractOutputPort(
                         "lcmt_material_points", drake::lcmt_material_points(),
                         &MPMPointsSender::OutputParticlePositionsLcm)
                     .get_index();
}

void MPMPointsSender::OutputParticlePositionsLcm(
    const drake::systems::Context<double>& context,
    drake::lcmt_material_points* output) const {
  const auto& mpm_outputs = this->EvalInputValue<
      multibody::gmpm::MpmPortData<multibody::gmpm::config::GpuT>>(
      context, input_port_mpm_);

  std::vector<Eigen::Vector3d> point_positions = mpm_outputs->pos;
  int n_points = static_cast<int>(point_positions.size());
  std::cout << "MPMPointsSender: number of MPM points = " << n_points
            << std::endl;

  // Convert to std::vectors.
  std::vector<std::vector<float>> points(n_points, std::vector<float>(3, 0));
  for (int i = 0; i < n_points; i++) {
    for (int j = 0; j < 3; j++) {
      points[i][j] = point_positions[i](j);
    }
  }

  // Set the fields of the LCM message.
  output->utime = context.get_time() * 1e6;
  output->num_points = n_points;
  output->points = points;
}

}  // namespace systems
}  // namespace drake
