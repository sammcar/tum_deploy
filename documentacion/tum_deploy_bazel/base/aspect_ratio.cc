

#include "base/aspect_ratio.h"

namespace mjmech {
namespace base {

Eigen::AlignedBox2i MaintainAspectRatio(Eigen::Vector2i source,
                                        Eigen::Vector2i dest) {
  int x = 0;
  int y = 0;
  int display_w = dest.x();
  int display_h = dest.y();

  // Enforce an aspect ratio.
  const double desired_aspect_ratio =
      static_cast<double>(std::abs(source.x())) /
      static_cast<double>(std::abs(source.y()));
  const double actual_ratio =
      static_cast<double>(display_w) /
      static_cast<double>(display_h);
  if (actual_ratio > desired_aspect_ratio) {
    const int w = display_h * desired_aspect_ratio;
    const int remaining = display_w - w;
    x = remaining / 2;
    display_w = w;
  } else if (actual_ratio < desired_aspect_ratio) {
    const int h = display_w / desired_aspect_ratio;
    const int remaining = display_h - h;
    y = remaining / 2;
    display_h = h;
  }

  return {
    Eigen::Vector2i(x, y),
        Eigen::Vector2i(x + display_w, y + display_h)
        };
}

}
}
