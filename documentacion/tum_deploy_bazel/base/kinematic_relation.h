

#pragma once

namespace mjmech {
namespace base {

/// Describes the kinematic relation between two rigid body frames.
struct KinematicRelation {
  Sophus::SE3d pose;  // Absolute position and orientation
  base::Point3D v;  // velocity
  base::Point3D w;  // angular rate

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(pose));
    a->Visit(MJ_NVP(v));
    a->Visit(MJ_NVP(w));
  }
};

}
}
