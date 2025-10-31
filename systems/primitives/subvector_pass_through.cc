/* Directly ported over from dairlib at:
https://github.com/DAIRLab/dairlib/blob/bibit/deform/systems/primitives/subvector_pass_through.cc
*/

// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/systems/primitives/subvector_pass_through-inl.h"

#include "drake/common/default_scalars.h"

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SubvectorPassThrough);
