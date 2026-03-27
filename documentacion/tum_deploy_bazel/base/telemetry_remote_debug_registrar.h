

#pragma once

#include <boost/signals2/signal.hpp>

#include "telemetry_remote_debug_server.h"

namespace mjmech {
namespace base {
class TelemetryRemoteDebugServer;

/// A registrar which supports emitting data over the network in a
/// human readable format for online diagnostics and debugging.
///
/// The implementation of this class is largely a pass-through, since
/// tuple's can't hold noncopyable things, we delegate to a shared_ptr
/// to something that is non-copyable.
class TelemetryRemoteDebugRegistrar {
 public:
  TelemetryRemoteDebugRegistrar(TelemetryRemoteDebugServer* server)
      : server_(server) {}

  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    server_->Register(name, signal);
  }

  TelemetryRemoteDebugServer* const server_;
};
}
}
