

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "mjlib/base/eigen.h"

#include "base/common.h"

namespace mjmech {
namespace base {

using Point3D = Eigen::Vector3d;

inline double Point3DHeadingDeg(const Point3D& p) {
  return Degrees(WrapNegPiToPi(0.5 * M_PI - std::atan2(p.y(), p.x())));
}

}
}
