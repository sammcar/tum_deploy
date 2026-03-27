

#pragma once

#include <sstream>

#include <boost/asio/spawn.hpp>
#include <boost/system/system_error.hpp>

#include <fmt/format.h>

#include "stringify.h"

namespace mjmech {
namespace base {

/// The following routine can be used to wrap coroutines such that
/// boost::system_error information is captured.  The default
/// boost::exception_ptr ignores this exception, making it challenging
/// to even report what happened.
template <typename Coroutine>
auto ErrorWrap(Coroutine coro) {
  return [=](boost::asio::yield_context yield) {
    try {
      return coro(yield);
    } catch (boost::system::system_error& e) {
      std::throw_with_nested(
          std::runtime_error(
              fmt::format("system_error: {}: {}",
                          e.what(), Stringify(e.code()))));
    } catch (std::runtime_error& e) {
      std::throw_with_nested(std::runtime_error(e.what()));
    }
  };
}

}
}
