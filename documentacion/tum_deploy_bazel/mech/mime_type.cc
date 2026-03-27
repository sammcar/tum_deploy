

#include "mech/mime_type.h"

namespace mjmech {
namespace mech {

std::string_view GetMimeType(std::string_view path) {
  const auto ext = [&]() {
    const auto pos = path.rfind(".");
    if (pos == std::string_view::npos) { return std::string_view(); }
    return path.substr(pos);
  }();

  struct Mapping {
    std::string_view extension;
    std::string_view mime_type;
  };
  constexpr Mapping mappings[] = {
    { ".htm", "text/html" },
    { ".html", "text/html" },
    { ".css", "text/css" },
    { ".txt", "text/plain" },
    { ".js", "application/javascript" },
    { ".json", "application/json" },
    { ".xml", "application/xml" },
    { ".png", "image/png" },
    { ".jpeg", "image/jpeg" },
    { ".jpg", "image/jpg" },
    { ".gif", "image/gif" },
    { ".ico", "image/vnd.microsoft.icon" },
    { ".svg", "image/svg+xml" },
  };
  for (const auto& mapping : mappings) {
    if (ext == mapping.extension) {
      return mapping.mime_type;
    }
  }
  return "application/text";
}

}
}
