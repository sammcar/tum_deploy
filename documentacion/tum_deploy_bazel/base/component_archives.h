

#pragma once

#include <map>

#include "mjlib/base/clipp_archive.h"

#include "base/handler_util.h"


namespace mjmech {
namespace base {
struct EnableArchive {
  EnableArchive(std::map<std::string, bool>& enabled): enabled(enabled) {}

  template <typename T>
  EnableArchive& Accept(T* value) {
    value->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    Helper(pair.name(), pair.value(), 0);
  }

  template <typename T>
  auto Helper(const char* name, T* value, int)
      -> decltype((*value)->AsyncStart(mjlib::io::ErrorCallback())) {
    enabled[name] = true;
  }

  template <typename T>
  void Helper(const char*, T*, long) {}

  std::map<std::string, bool>& enabled;
};

struct StartArchive {
  StartArchive(mjlib::io::ErrorCallback handler)
      : joiner(std::make_shared<ErrorHandlerJoiner>(std::move(handler))) {}

  template <typename Serializable>
  static void Start(Serializable* serializable,
                    mjlib::io::ErrorCallback callback) {
    StartArchive archive(std::move(callback));
    archive.Accept(serializable);
  }

  template <typename T>
  StartArchive& Accept(T* value) {
    value->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    Helper(pair.name(), pair.value(), 0);
  }

  template <typename T>
  auto Helper(const char* name, T* value, int)
      -> decltype((*value)->AsyncStart(mjlib::io::ErrorCallback())) {
    (*value)->AsyncStart(
        joiner->Wrap(std::string("starting: '") + name + "'"));
  }

  template <typename T>
  void Helper(const char*, T*, long) {}

  std::shared_ptr<ErrorHandlerJoiner> joiner;
};

class ClippComponentArchive {
 public:
  template <typename T>
  ClippComponentArchive& Accept(T* value) {
    value->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    VisitHelper(pair, pair.value(), static_cast<int32_t>(0));
  }

  template <typename NameValuePair, typename Serializable>
  auto VisitHelper(const NameValuePair& pair,
                   Serializable* serializable,
                   int32_t) -> decltype((*serializable)->program_options()) {
    group_.merge(
        clipp::with_prefix(std::string(pair.name()) + ".",
                           (*pair.value())->program_options()));
    return {};
  }

  template <typename NameValuePair, typename Serializable>
  auto VisitHelper(const NameValuePair& pair,
                   Serializable*,
                   int64_t) {
    group_.merge(
        mjlib::base::ClippArchive(std::string(pair.name()) + ".")
        .Accept((*pair.value())->parameters()).release());
  }

  clipp::group release() {
    return std::move(group_);
  }

  clipp::group group() {
    return group_;
  }

 private:
  clipp::group group_;
};

}
}
