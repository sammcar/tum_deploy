

#pragma once

namespace mjmech {
namespace base {

/// Interpolate over a cubic bezier function with fixed control points
/// such that velocity is 0 at start and end and the acceleration is
/// continuous within that range.
template <typename T>
class Bezier {
 public:
  Bezier(T start, T end)
      : start_(start),
        end_(end) {}

  T position(double phase) const {
    const double bezier =
        phase * phase * phase + 3.0 * (phase * phase * (1.0 - phase));
    return start_ + bezier * delta_;
  }

  T velocity(double phase) const {
    const double bezier = 6 * phase * (1.0 - phase);
    return bezier * delta_;
  }

  T acceleration(double phase) const {
    const double bezier = 6 - 12 * phase;
    return bezier * delta_;
  }

 private:
  T start_;
  T end_;
  T delta_{end_ - start_};
};

}
}
