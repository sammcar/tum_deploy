

#pragma once

#include "mjlib/base/error_code.h"
#include "mjlib/base/fail.h"

namespace mjmech {
namespace base {

class FailFunctor {
 public:
  template <typename... Args>
  void operator()(const mjlib::base::error_code& ec, Args...) {
    mjlib::base::FailIf(ec);
  }
};

}
}
