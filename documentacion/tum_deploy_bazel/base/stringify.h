

#pragma once

#include <sstream>
#include <string>

namespace mjmech {
namespace base {

template <typename T>
std::string Stringify(const T& rhs) {
  std::ostringstream ostr;
  ostr << rhs;
  return ostr.str();
}

}
}
