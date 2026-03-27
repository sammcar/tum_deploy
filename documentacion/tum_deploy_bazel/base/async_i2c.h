

#pragma once

#include <boost/asio/any_io_executor.hpp>

#include "mjlib/io/async_types.h"

namespace mjmech {
namespace base {

class AsyncI2C : boost::noncopyable {
 public:
  virtual ~AsyncI2C() {}
  virtual boost::asio::any_io_executor get_executor() = 0;

  virtual void AsyncRead(
      uint8_t device, uint8_t address,
      mjlib::io::MutableBufferSequence buffers, mjlib::io::ReadHandler) = 0;

  virtual void AsyncWrite(
      uint8_t device, uint8_t address,
      mjlib::io::ConstBufferSequence buffers, mjlib::io::WriteHandler) = 0;
};

typedef std::shared_ptr<AsyncI2C> SharedI2C;
typedef std::function<void (mjlib::base::error_code, SharedI2C)> SharedI2CHandler;

}
}
