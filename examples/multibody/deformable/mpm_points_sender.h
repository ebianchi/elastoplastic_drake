#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_material_points.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace drake {
namespace systems {

/// Converts particle positions to LCM type lcmt_material_points.
class MPMPointsSender : public systems::LeafSystem<double> {
 public:
  MPMPointsSender(std::string name);

  const systems::InputPort<double>& get_input_port_mpm() const {
    return this->get_input_port(input_port_mpm_);
  }

  const systems::OutputPort<double>& get_output_port_particles() const {
    return this->get_output_port(output_port_);
  }

 private:
  void OutputParticlePositionsLcm(
      const drake::systems::Context<double>& context,
      drake::lcmt_material_points* output) const;

  systems::InputPortIndex input_port_mpm_;
  systems::OutputPortIndex output_port_;
};

}  // namespace systems
}  // namespace drake
