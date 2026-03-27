

#include "base/git_info.h"

#include <cstring>

namespace mjmech {
namespace base {

namespace {

uint8_t ParseHexNibble(uint8_t c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  return 0;
}

uint8_t ParseHexByte(const char* data) {
  return (ParseHexNibble(data[0]) << 4) | ParseHexNibble(data[1]);
}

}

GitInfo::GitInfo() {
  if (std::strlen(kGitHash) != 40) {
    dirty = true;
  } else {
    for (size_t i = 0; i <= 20; i++) {
      hash[i] = ParseHexByte(&kGitHash[i * 2]);
    }

    dirty = kGitDirty[0] != '0';
  }
}

char kGitHash[41] __attribute__((weak)) = {};
char kGitDirty[10] __attribute__((weak)) = {};
}
}
