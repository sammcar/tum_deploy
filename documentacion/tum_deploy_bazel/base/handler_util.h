

#pragma once

#include <list>
#include <memory>

#include "mjlib/base/error_code.h"
#include "mjlib/io/async_types.h"

#include "logging.h"

namespace mjmech {
namespace base {
/// Join one or more ErrorHandler callbacks into a single callback.
/// The first failure will cause the target to be emitted, and it will
/// be called with success after all registered handlers are called.
class ErrorHandlerJoiner
    : public std::enable_shared_from_this<ErrorHandlerJoiner> {
 public:
  ErrorHandlerJoiner(mjlib::io::ErrorCallback handler)
      : handler_(std::move(handler)),
        log_(GetUniqueLogInstance("joiner")) {
  }

  mjlib::io::ErrorCallback Wrap(const std::string& message) {
    BOOST_ASSERT(!done_);
    outstanding_.push_front(true);
    log_.debugStream()
        << "Scheduling [" << message  << "], "
        << outstanding_.size() << " pending";
    auto it = outstanding_.begin();
    auto me = shared_from_this();
    return [me, it, message](mjlib::base::error_code ec) {
      if (me->done_) {
        me->log_.warnStream()
            << "Complete [" << message << "]: result " << (!!ec)
            << ", but we are already done";
        // Another callback already finished early, eat this handler
        // and return.
        return;
      }

      me->outstanding_.erase(it);
      me->log_.debugStream()
          << "Complete [" << message << "]: result " << (!!ec)
          <<  ", " << me->outstanding_.size() << " pending";
      if (ec || me->outstanding_.empty()) {
        ec.Append(message);
        me->log_.debug("All complete");
        me->handler_(ec);
        me->done_ = true;
      }
    };
  }

  bool done_ = false;
  mjlib::io::ErrorCallback handler_;
  std::list<bool> outstanding_;
  LogRef log_;
};
}
}
