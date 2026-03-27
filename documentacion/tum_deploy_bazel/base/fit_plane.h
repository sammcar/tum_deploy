

#pragma once

#include <vector>

#include <Eigen/Core>

namespace mjmech {
namespace base {

/// For a plane defined as a*x + b*y + c = z
///
/// Yes, this only works for planes that are mostly level.
struct Plane {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
};

Plane FitPlane(const std::vector<Eigen::Vector3d>& points);

}
}
