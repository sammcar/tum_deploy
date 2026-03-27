

#include "base/system_fd.h"

#include <unistd.h>

namespace mjmech {
namespace base {

SystemFd::~SystemFd() {
  if (fd_ >= 0) ::close(fd_);
}

}
}
