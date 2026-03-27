

#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/io/now.h"
#include "mjlib/telemetry/binary_write_archive.h"
#include "mjlib/telemetry/file_writer.h"


namespace mjmech {
namespace base {
/// A registrar which emits every instance of every record to a
/// TelemetryLog instance using the TelemetryArchive for
/// serialization.
///
/// NOTE: Ideally this would be noncopyable, but std::tuple doesn't
/// currently allow construction of noncopyable members.
///
/// NOTE: In the future, this could have policies around which records
/// are written to the log and at what rate.
class TelemetryLogRegistrar {
 public:
  TelemetryLogRegistrar(boost::asio::io_context& context,
                        mjlib::telemetry::FileWriter* telemetry_log)
      : context_(context),
        telemetry_log_(telemetry_log) {}

  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    const auto identifier = telemetry_log_->AllocateIdentifier(name);
    telemetry_log_->WriteSchema(
        identifier,
        mjlib::telemetry::BinarySchemaArchive::template schema<T>());
    signal->connect(std::bind(&TelemetryLogRegistrar::HandleData<T>,
                              this, identifier,
                              std::placeholders::_1));
  }

  template <typename T>
  void HandleData(mjlib::telemetry::FileWriter::Identifier identifier,
                  const T* data) {
    // If the log isn't open, don't even bother serializing things.
    if (!telemetry_log_->IsOpen()) { return; }

    auto buffer = telemetry_log_->GetBuffer();
    mjlib::telemetry::BinaryWriteArchive(*buffer).Accept(data);
    telemetry_log_->WriteData(
        mjlib::io::Now(context_), identifier, std::move(buffer));
  }

  boost::asio::io_context& context_;
  mjlib::telemetry::FileWriter* const telemetry_log_;
};
}
}
