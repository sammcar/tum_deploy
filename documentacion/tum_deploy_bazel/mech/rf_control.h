
#pragma once

#include <functional>
#include <memory>

#include <clipp/clipp.h>

#include "base/context.h"
#include "mech/rf_client.h"
#include "mech/quadruped_control.h"

namespace mjmech {
namespace mech {

/// Listens to RF commands, using those to command QuadrupedControl.
/// Also, exposes telemetry back out to the RF interface.
class RfControl {
 public:
  // To be called at AsyncStart time.
  using RfGetter = std::function<RfClient*()>;

  RfControl(const base::Context&, QuadrupedControl* control, RfGetter);
  ~RfControl();

  void AsyncStart(mjlib::io::ErrorCallback);

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
