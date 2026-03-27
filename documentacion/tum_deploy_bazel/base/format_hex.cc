

#include "base/format_hex.h"

#include <sstream>

#include <fmt/format.h>

namespace mjmech {
namespace base {

std::string FormatHex(std::string_view data) {
  std::ostringstream ostr;
  for (char c : data) {
    ostr << fmt::format("{:02x}", static_cast<int>(static_cast<uint8_t>(c)));
  }
  return ostr.str();
}

}
}
