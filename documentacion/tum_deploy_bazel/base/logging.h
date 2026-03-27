

#pragma once

#include <log4cpp/Category.hh>

#include <clipp/clipp.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/visitor.h"

namespace mjmech {
namespace base {

clipp::group MakeLoggingOptions();

// Call after program_options has been notified, or if you do not support
// program options.
void InitLogging();

typedef log4cpp::Category& LogRef;
// Get a LogRef with a given name.
//  - there is a .-separated hierachy -- '-t cd' will enable 'cd.stats' logger
//  - set the first element to C++ file name (lowercase, _-separated)
LogRef GetLogInstance(const std::string& name);

// Same as GetLogInstance, but appends a unique suffix to a name.
LogRef GetUniqueLogInstance(const std::string& name);

LogRef GetSubLogger(LogRef parent, const std::string& name);


//class TelemetryRegistry;
//void WriteLogToTelemetryLog(TelemetryRegistry*);


// TODO theamk: ideally, all of the stuff below should be hidden, and a single
// function (with signature above) should be exposed. However,
// TelemetryRegsitry is way too templated for this to work, so we have to
// expose much more than we wanted to.
struct TextLogMessage {
  boost::posix_time::ptime timestamp;
  std::string thread;
  std::string priority;
  int priority_int;
  std::string ndc;
  std::string category;
  std::string message;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(thread));
    a->Visit(MJ_NVP(priority));
    a->Visit(MJ_NVP(priority_int));
    a->Visit(MJ_NVP(ndc));
    a->Visit(MJ_NVP(category));
    a->Visit(MJ_NVP(message));
  }
};
typedef boost::signals2::signal<void (const TextLogMessage*)
                                > TextLogMessageSignal;

TextLogMessageSignal* GetLogMessageSignal();

}
}
