

#pragma once

#include <memory>

#include <boost/noncopyable.hpp>
#include <boost/asio/any_io_executor.hpp>
#include <boost/asio/io_context.hpp>

#include "mjlib/io/realtime_executor.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/telemetry/file_writer.h"

namespace mjmech {
namespace base {

class TelemetryRemoteDebugServer;
class TelemetryRegistry;

struct Context : boost::noncopyable {
  Context();
  ~Context();

  boost::asio::io_context context;
  mjlib::io::RealtimeExecutor rt_executor{context.get_executor()};
  boost::asio::any_io_executor executor{rt_executor};
  std::unique_ptr<mjlib::telemetry::FileWriter> telemetry_log;
  std::unique_ptr<TelemetryRemoteDebugServer> remote_debug;
  std::unique_ptr<TelemetryRegistry> telemetry_registry;
  std::unique_ptr<mjlib::io::StreamFactory> factory;
};

}
}
