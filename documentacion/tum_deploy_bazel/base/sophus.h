

#pragma once

#include <sophus/se3.hpp>

#include "mjlib/base/eigen.h"
#include "mjlib/base/visitor.h"

namespace mjlib {
namespace base {

template <typename Scalar>
struct ExternalSerializer<Sophus::SO3<Scalar>> {
  struct Wrapper {
    Wrapper(Sophus::SO3<Scalar>* wrapped = nullptr)
        : wrapped_(wrapped ? wrapped : &g_static_wrapped_) {}

    template <typename Archive>
    void Serialize(Archive* a) {
      auto* data = wrapped_->data();
      a->Visit(mjlib::base::MakeNameValuePair(&data[0], "x"));
      a->Visit(mjlib::base::MakeNameValuePair(&data[1], "y"));
      a->Visit(mjlib::base::MakeNameValuePair(&data[2], "z"));
      a->Visit(mjlib::base::MakeNameValuePair(&data[3], "w"));
    }

    Sophus::SO3<Scalar>* wrapped_;
    static inline Sophus::SO3<Scalar> g_static_wrapped_;
  };

  template <typename PairReceiver>
  void Serialize(Sophus::SO3<Scalar>* value, PairReceiver receiver) {
    Wrapper wrapper(value);
    receiver(mjlib::base::MakeNameValuePair(&wrapper, ""));
  }
};

template <typename Scalar>
struct ExternalSerializer<Sophus::SE3<Scalar>> {
  struct Wrapper {
    Wrapper(Sophus::SE3<Scalar>* wrapped = nullptr)
        : wrapped_(wrapped ? wrapped : &g_static_wrapped_) {}

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(mjlib::base::MakeNameValuePair(
                   &wrapped_->so3(), "so3"));
      a->Visit(mjlib::base::MakeNameValuePair(
                   &wrapped_->translation(), "translation"));
    }

    Sophus::SE3<Scalar>* wrapped_;
    static inline Sophus::SE3<Scalar> g_static_wrapped_;
  };

  template <typename PairReceiver>
  void Serialize(Sophus::SE3<Scalar>* value, PairReceiver receiver) {
    Wrapper wrapper(value);
    receiver(mjlib::base::MakeNameValuePair(&wrapper, ""));
  }
};

}
}
