

#pragma once

#include <vector>

#include <Eigen/Core>

namespace mjmech {
namespace base {

/// Given the leg X/Y positions in the M frame, return a ratio of
/// force to apply which minimizes the amount of angular acceleration
/// incurred.
std::vector<double> OptimizeLegForce(const std::vector<Eigen::Vector2d>&);

}
}
