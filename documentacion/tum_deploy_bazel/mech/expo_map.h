
#pragma once

#include <cmath>

namespace mjmech {
namespace mech {

/// A two rate piecewise linear mapping with a central deadband.
///
/// For the positive side, the input-output mapping:
///
///  0     - 0.0
///  0.05  - 0.0
///  0.3   - 0.1
///  1.0   - 1.0
class ExpoMap {
 public:
  struct Options {
    double deadband = 0.05;
    double slow_range = 0.30;
    double slow_value = 0.10;

    Options() {}
  };

  ExpoMap(const Options& options = Options()) : options_(options) {}

  double operator()(double value) const {
    const auto& o = options_;

    if (std::abs(value) < o.deadband) { return 0.0; }
    const double sign = std::copysign(1.0, value);
    if (std::abs(value) < o.slow_range) {
      return o.slow_value * (value - o.deadband * sign) /
          (o.slow_range - o.deadband);
    }
    return (value - (sign * o.slow_range)) /
        (1.0 - o.slow_range) *
        (1.0 - o.slow_value) + sign * o.slow_value;
  }

 private:
  Options options_;
};

}
}
