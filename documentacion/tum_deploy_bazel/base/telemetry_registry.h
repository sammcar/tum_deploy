

#pragma once

#include <map>

#include <boost/asio/io_context.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/telemetry/file_writer.h"

#include "base/telemetry_log_registrar.h"
#include "base/telemetry_remote_debug_registrar.h"

namespace mjmech {
namespace base {
/// Maintain a local publish-subscribe model used for data objects.
class TelemetryRegistry : boost::noncopyable {
 public:
  TelemetryRegistry(boost::asio::io_context& context,
                    mjlib::telemetry::FileWriter* log,
                    TelemetryRemoteDebugServer* debug)
      : log_(context, log), debug_(debug) {}

  /// Register a serializable object, and return a function object
  /// which when called will disseminate the
  /// object to any observers.
  template <typename DataObject>
  std::function<void (const DataObject*)>
  Register(const std::string& record_name) {
    // NOTE jpieper: Ideally this would return auto, and just let the
    // compiler sort out what type the lambda is.  But since C++11
    // doesn't support that yet, we return the type erasing
    // std::function.

    auto* ptr = new Concrete<DataObject>();
    auto result = [ptr](const DataObject* object) { ptr->signal(object); };

    log_.Register(record_name, &ptr->signal);
    debug_.Register(record_name, &ptr->signal);

    records_.insert(
        std::make_pair(
            record_name, std::unique_ptr<Base>(ptr)));

    return result;
  };

  template <typename DataObject>
  void Register(const std::string& record_name,
                boost::signals2::signal<void (const DataObject*)>* signal) {
    signal->connect(Register<DataObject>(record_name));
  }

 private:
  struct Base {
    virtual ~Base() {}
  };

  template <typename Serializable>
  struct Concrete : public Base {
    virtual ~Concrete() {}

    boost::signals2::signal<void (const Serializable*)> signal;
  };

  std::map<std::string, std::unique_ptr<Base> > records_;

  TelemetryLogRegistrar log_;
  TelemetryRemoteDebugRegistrar debug_;
};

}
}
