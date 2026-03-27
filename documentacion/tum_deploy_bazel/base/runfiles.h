

#pragma once

#include <memory>
#include <string>

#include <boost/filesystem.hpp>

#include "tools/cpp/runfiles/runfiles.h"

#include "mjlib/base/system_error.h"

namespace mjmech {
namespace base {
class Runfiles {
 public:
  Runfiles() {
    std::string error;
    runfiles_.reset(
        bazel::tools::cpp::runfiles::Runfiles::Create(
            boost::filesystem::canonical("/proc/self/exe").native(), &error));
    if (!runfiles_) {
      throw mjlib::base::system_error::einval("Error creating runfiles: " + error);
    }
  }

  std::string Rlocation(std::string_view path) const {
    return runfiles_->Rlocation("com_github_mjbots_mech/" + std::string(path));
  }

 private:
  std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles_;
};

class TestRunfiles {
 public:
  TestRunfiles() {
    std::string error;
    runfiles_.reset(bazel::tools::cpp::runfiles::Runfiles::CreateForTest(&error));
    if (!runfiles_) {
      throw mjlib::base::system_error::einval("Error creating runfiles: " + error);
    }
  }

  std::string Rlocation(std::string_view path) const {
    return runfiles_->Rlocation("com_github_mjbots_mech/" + std::string(path));
  }

 private:
  std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles_;
};
}
}
