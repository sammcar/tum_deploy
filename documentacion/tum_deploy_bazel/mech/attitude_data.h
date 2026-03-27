
#pragma once

#include "mjlib/base/visitor.h"

#include "base/euler.h"
#include "base/point3d.h"
#include "base/quaternion.h"

namespace mjmech {
namespace mech {

struct AttitudeData {
  boost::posix_time::ptime timestamp;

  base::Quaternion attitude;
  base::Point3D rate_dps;
  base::Euler euler_deg;
  base::Point3D accel_mps2;

  base::Point3D bias_dps;
  base::Quaternion attitude_uncertainty;
  base::Point3D bias_uncertainty_dps;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(attitude));
    a->Visit(MJ_NVP(rate_dps));
    a->Visit(MJ_NVP(euler_deg));
    a->Visit(MJ_NVP(accel_mps2));
    a->Visit(MJ_NVP(bias_dps));
    a->Visit(MJ_NVP(attitude_uncertainty));
    a->Visit(MJ_NVP(bias_uncertainty_dps));
  }
};

}
}
