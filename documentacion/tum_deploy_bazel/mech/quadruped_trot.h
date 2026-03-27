

#pragma once

#include "mech/quadruped_command.h"
#include "mech/quadruped_context.h"

namespace mjmech {
namespace mech {

struct TrotResult {
  std::vector<QuadrupedCommand::Leg> legs_R;
  base::KinematicRelation desired_RB;
};

/// Execute the trot gait.
TrotResult QuadrupedTrot(
    QuadrupedContext* context,
    const std::vector<QuadrupedCommand::Leg>& old_legs_R);

}
}
