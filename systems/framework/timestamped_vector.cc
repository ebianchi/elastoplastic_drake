/* Directly ported over with minimal edits from dairlib at:
https://github.com/DAIRLab/dairlib/blob/bibit/deform/systems/framework/timestamped_vector.cc
*/

#include "drake/systems/framework/timestamped_vector.h"

#include "drake/common/default_scalars.h"

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TimestampedVector);
