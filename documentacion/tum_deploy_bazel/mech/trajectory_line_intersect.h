

#pragma once

#include <Eigen/Core>

namespace mjmech {
namespace mech {

/// Given a trajectory and a line segment, return the time required for
/// the trajectory to intersect the line.  negative may be returned if
/// the trajectory would have intersected the line in the past, and
/// infinity will be returned if the trajectory will never intersect
/// the line.
///
/// The trajectory is specified as a path starting from (0, 0) facing
/// positive X.
double TrajectoryLineIntersectTime(
    const Eigen::Vector2d& velocity,
    double omega,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2);

}
}
