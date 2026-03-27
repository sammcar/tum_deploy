
#pragma once

#include "base/point3d.h"

namespace mjmech {
namespace mech {

struct TrajectoryState {
  base::Point3D pose_l;
  base::Point3D velocity_l_s;
  base::Point3D acceleration_l_s2;
};

TrajectoryState CalculateAccelerationLimitedTrajectory(
    const TrajectoryState& start,
    const base::Point3D& target_l,
    double target_velocity_l_s,
    double max_acceleration_l_s2,
    double delta_s);

}
}
