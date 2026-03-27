

#include "quaternion.h"

#include <fmt/format.h>

namespace mjmech {
namespace base {

std::string Quaternion::str() const {
  return fmt::format("%f %f %f %f", w_, x_, y_, z_);
}

}
}
