

#pragma once

#include <array>
#include <cstdint>

#include "mjlib/base/visitor.h"

namespace mjmech {
namespace base {

struct GitInfo {
  GitInfo();

  std::array<uint8_t, 20> hash = {{}};
  bool dirty = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(hash));
    a->Visit(MJ_NVP(dirty));
  }
};

extern char kGitHash[41];
extern char kGitDirty[10];

}
}
