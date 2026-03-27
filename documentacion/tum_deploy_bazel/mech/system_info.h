

#pragma once

#include <memory>

#include <clipp/clipp.h>

#include "base/context.h"

namespace mjmech {
namespace mech {

/// Record information about the system on a periodic basis.
class SystemInfo : boost::noncopyable {
 public:
  SystemInfo(base::Context&);
  ~SystemInfo();

  void AsyncStart(mjlib::io::ErrorCallback);

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
