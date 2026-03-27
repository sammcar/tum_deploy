

#pragma once

#include <Eigen/Core>

#include "base/bezier.h"

namespace mjmech {
namespace mech {

/// A unit-less trajectory generator for leg swings.  It uses 2nd
/// derivative smooth curves to get the leg off the ground and out of
/// its current velocity, to a target point, and then back down to
/// moving at the given velocity.
///
/// start.z() must equal end.z() for now.
///
/// @param world_blend fraction of phase to spend ramping in and out
/// of the world velocity
class SwingTrajectory {
 public:
  SwingTrajectory()
      : SwingTrajectory({}, {}, {}, 1.0, 0.1, 1.0) {}

  SwingTrajectory(const Eigen::Vector3d& start,
                  const Eigen::Vector3d& start_velocity,
                  const Eigen::Vector3d& end,
                  double height,
                  double world_blend,
                  double swing_time_s);

  struct Result {
    double phase = 0.0;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity_s;
    Eigen::Vector3d acceleration_s2;
  };

  Result Advance(double delta_s, const Eigen::Vector3d& world_velocity_s);

 private:
  Eigen::Vector3d start_;
  Eigen::Vector3d start_velocity_;
  Eigen::Vector3d end_;
  double world_blend_;
  double swing_time_s_;

  double phase_ = 0.0;
  Eigen::Vector3d current_;

  base::Bezier<double> zinterp_;
  base::Bezier<Eigen::Vector2d> xymove_;
};

}
}
