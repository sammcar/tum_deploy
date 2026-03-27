

#include "mjlib/base/visitor.h"

#include "base/point3d.h"

namespace mjmech {
namespace mech {

struct ImuData {
  boost::posix_time::ptime timestamp;

  base::Point3D rate_dps;
  base::Point3D accel_mps2;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(rate_dps));
    a->Visit(MJ_NVP(accel_mps2));
  }
};

}
}
