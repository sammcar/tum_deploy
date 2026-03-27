

#pragma once

namespace mjmech {
namespace base {

/// Manages ownership of a system file descriptor.
class SystemFd {
 public:
  SystemFd() : fd_(-1) {}
  SystemFd(int fd) : fd_(fd) {}

  SystemFd(SystemFd&& rhs) {
    fd_ = rhs.fd_;
    rhs.fd_ = -1;
  }

  SystemFd& operator=(SystemFd&& rhs) {
    fd_ = rhs.fd_;
    rhs.fd_ = -1;
    return *this;
  }

  ~SystemFd();

  SystemFd(const SystemFd&) = delete;
  SystemFd& operator=(const SystemFd&) = delete;

  operator int() { return fd_; }

 private:
  int fd_ = -1;
};

}
}
