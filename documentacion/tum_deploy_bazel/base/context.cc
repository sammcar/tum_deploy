

#include "context_full.h"

namespace mjmech {
namespace base {

Context::Context()
    : telemetry_log(std::make_unique<mjlib::telemetry::FileWriter>([]() {
          mjlib::telemetry::FileWriter::Options options;
          options.blocking = false;
          return options;
        }())),
      remote_debug(std::make_unique<TelemetryRemoteDebugServer>(executor)),
      telemetry_registry(std::make_unique<TelemetryRegistry>(
                             context, telemetry_log.get(), remote_debug.get())),
      factory(std::make_unique<mjlib::io::StreamFactory>(executor))
{
}

Context::~Context() {}

}
}
