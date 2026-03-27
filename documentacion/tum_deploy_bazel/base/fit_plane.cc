

#include "base/fit_plane.h"

#include <Eigen/Dense>

namespace mjmech {
namespace base {

Plane FitPlane(const std::vector<Eigen::Vector3d>& points) {
  Eigen::MatrixXd A(points.size(), 3);
  Eigen::MatrixXd B(points.size(), 1);

  for (size_t i = 0; i < points.size(); i++) {
    A(i, 0) = points[i].x();
    A(i, 1) = points[i].y();
    A(i, 2) = 1.0;
    B(i) = points[i].z();
  }

  Eigen::MatrixXd result = A.bdcSvd(
      Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
  return Plane{result(0), result(1), result(2)};
}

}
}
