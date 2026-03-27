
#pragma once

namespace mjmech {
namespace mech {

/// Propagate a leg given a linear and angular velocity of the CoM.
class PropagateLeg {
 public:
  PropagateLeg(const Eigen::Vector3d& v_R,
               const Eigen::Vector3d& w_R,
               double period_s)
      : v_R_(v_R),
        w_R_(w_R),
        pose_T2_T1_(
          Sophus::SO3d(
              Eigen::AngleAxisd(-period_s * w_R.z(), Eigen::Vector3d::UnitZ())
              .toRotationMatrix()),
          -v_R_ * period_s) {}

  struct Result {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
  };

  Result operator()(const Eigen::Vector3d& position_R) const {
    Result result;
    result.position = pose_T2_T1_ * position_R;
    result.velocity = -v_R_ - w_R_.cross(position_R);
    return result;
  }

 private:
  Eigen::Vector3d v_R_;
  Eigen::Vector3d w_R_;
  Sophus::SE3d pose_T2_T1_;
};

}
}
