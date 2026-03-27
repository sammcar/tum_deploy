

#pragma once

#include "mjlib/io/async_stream.h"

#include "mech/rf_client.h"

namespace mjmech {
namespace mech {

/// An implementation of RfClient that uses the nrfusb.
class NrfusbClient : public RfClient {
 public:
  struct Options {
    uint32_t id0 = 5678;
    uint32_t id1 = 88754;

    Options() {}
  };
  NrfusbClient(mjlib::io::AsyncStream*, const Options& = Options());
  ~NrfusbClient() override;

  void AsyncWaitForSlot(
      int* remote, uint16_t* bitfield, mjlib::io::ErrorCallback) override;

  Slot rx_slot(int remote, int slot_idx) override;
  void tx_slot(int remote, int slot_idx, const Slot&) override;
  Slot tx_slot(int remote, int slot_idx) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
