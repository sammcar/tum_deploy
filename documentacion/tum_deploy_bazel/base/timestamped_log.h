

#pragma once

#include <string>

#include "mjlib/telemetry/file_writer.h"

namespace mjmech {
namespace base {

enum TimestampMode {
  kTimestamped,
  kShort,
};

void OpenMaybeTimestampedLog(mjlib::telemetry::FileWriter* writer,
                             std::string_view filename,
                             TimestampMode);

}
}
