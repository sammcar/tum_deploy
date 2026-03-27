

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mjmech {
namespace base {

/// Given a source image and a destination region, return a draw
/// location that maximizes the usable size while maintaining the
/// aspect ratio of the source.
Eigen::AlignedBox2i MaintainAspectRatio(Eigen::Vector2i source,
                                        Eigen::Vector2i dest);

}
}
