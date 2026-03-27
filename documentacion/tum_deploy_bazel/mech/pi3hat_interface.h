
#pragma once

#include "mjlib/multiplex/asio_client.h"

#include "mech/imu_client.h"
#include "mech/rf_client.h"

namespace mjmech {
namespace mech {

/// The pi3hat has multiple functions.  This is just an interface that
/// combines all of them for use in an mjlib::io::Selector.
class Pi3hatInterface : public ImuClient,
                        public RfClient,
                        public mjlib::multiplex::AsioClient {
 public:
  ~Pi3hatInterface() override {}

  virtual void Cycle(
      AttitudeData*,
      const Request*, Reply*,
      mjlib::io::ErrorCallback callback) = 0;
};

}
}
