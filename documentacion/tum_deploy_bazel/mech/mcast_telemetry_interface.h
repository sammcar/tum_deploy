

#pragma once

#include <string>

#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace mjmech {
namespace mech {

/// The multi-cast system can associate additional telemetry with
/// video frames as they are sent.  This interface provides a mechanism
/// to provide that additional telemetry. */
class McastTelemetryInterface {
 public:
  virtual ~McastTelemetryInterface() {}

  /// Include @p data with @p name in packets that are sent until @p
  /// expiration has passed, after which no data with this name will
  /// be included.  Any previous information associated with @p name
  /// is overwritten.
  virtual void SetTelemetry(const std::string& name,
                            const std::string& data,
                            boost::posix_time::ptime expiration) = 0;
};
}
}
