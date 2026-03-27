

#pragma once

namespace mjmech {
namespace base {

template <typename T, typename InType>
T Saturate(InType value) {
  if (value >= static_cast<InType>(std::numeric_limits<T>::max())) {
    return std::numeric_limits<T>::max();
  }
  if (value <= static_cast<InType>(std::numeric_limits<T>::min())) {
    return std::numeric_limits<T>::min();
  }
  return static_cast<T>(value);
}

}
}
