
#pragma once

#include <string>

/// A simple tokenizer.  It can split on multiple delimiters, and
/// reports multiple consecutive delimiters as empty tokens.
class Tokenizer {
 public:
  Tokenizer(const std::string& source, const char* delimiters)
      : source_(source),
        delimiters_(delimiters),
        position_(source_.cbegin()) {}

  std::string next() {
    if (position_ == source_.end()) { return std::string(); }
    const auto start = position_;
    auto next = position_;
    bool found = false;
    for (; next != source_.end(); ++next) {
      if (std::strchr(delimiters_, *next) != nullptr) {
        position_ = next;
        ++position_;
        found = true;
        break;
      }
    }
    if (!found) { position_ = next; }
    return std::string(start, next);
  }

  std::string remaining() const {
    return std::string(position_, source_.end());
  }

 private:
  const std::string source_;
  const char* const delimiters_;
  std::string::const_iterator position_;
};
