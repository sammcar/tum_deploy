
#pragma once

#include "mjlib/io/async_types.h"

#include "mech/attitude_data.h"

namespace mjmech {
namespace mech {

class ImuClient {
 public:
  virtual ~ImuClient() {}

  virtual void ReadImu(AttitudeData*, mjlib::io::ErrorCallback) = 0;
};

}
}
