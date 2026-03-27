

#pragma once

#include "mjlib/base/visitor.h"

namespace mjmech {
namespace base {

/// Euler angles are in roll, pitch, then yaw.
///  +roll -> right side down
///  +pitch -> forward edge up
///  +yaw -> clockwise looking down
struct Euler {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(roll));
    a->Visit(MJ_NVP(pitch));
    a->Visit(MJ_NVP(yaw));
  }
};

inline Euler operator*(const Euler& lhs, double s) {
  return Euler{lhs.roll * s, lhs.pitch * s, lhs.yaw * s};
}

inline Euler operator*(double s, const Euler& lhs) {
  return Euler{lhs.roll * s, lhs.pitch * s, lhs.yaw * s};
}

}
}
