#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct MpmParams {
  int domain_bits;
  double grid_block_spacing;
  double youngs_modulus;
  double poisson_ratio;
  double particle_yield_stress;
  bool particle_plasticity;
  bool particle_linear_corotated;
  double density;
  double rpic_damping;
  double contact_stiffness;
  double contact_damping;
  double contact_friction_mu;
  int points_per_cell;
  double cell_side_length;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(domain_bits));
    a->Visit(DRAKE_NVP(grid_block_spacing));
    a->Visit(DRAKE_NVP(youngs_modulus));
    a->Visit(DRAKE_NVP(poisson_ratio));
    a->Visit(DRAKE_NVP(particle_yield_stress));
    a->Visit(DRAKE_NVP(particle_plasticity));
    a->Visit(DRAKE_NVP(particle_linear_corotated));
    a->Visit(DRAKE_NVP(density));
    a->Visit(DRAKE_NVP(rpic_damping));
    a->Visit(DRAKE_NVP(contact_stiffness));
    a->Visit(DRAKE_NVP(contact_damping));
    a->Visit(DRAKE_NVP(contact_friction_mu));
    a->Visit(DRAKE_NVP(points_per_cell));
    a->Visit(DRAKE_NVP(cell_side_length));
  }
};
