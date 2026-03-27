

#pragma once

namespace mjmech {
namespace base {

template <typename T>
T Interpolate(T start, T end, double scale) {
  return (end - start) * scale + start;
}

}
}
