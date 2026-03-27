

#pragma once

namespace mjmech {
namespace base {
template <typename T>
class circular_buffer {
 public:
  circular_buffer() { data_.resize(2); }

  void push_back(T&& value) {
    if (full()) { resize(data_.size() * 2); }
    data_[insert_] = std::move(value);
    insert_ = (insert_ + 1) % data_.size();
  }

  void pop_front() {
    remove_ = (remove_ + 1) % data_.size();
  }

  T& front() { return data_[remove_]; }
  const T& front() const { return data_[remove_]; }

  T& back() { return data_[insert_]; }
  const T& back() const { return data_[insert_]; }

  bool empty() const { return insert_ == remove_; }
  bool full() const {
    return ((insert_ + 1) % data_.size()) == remove_;
  }

  size_t capacity() const { return data_.size() - 1; }

 private:
  void resize(size_t size) {
    std::vector<T> new_data(size);
    size_t new_offset = 0;
    while (!empty()) {
      new_data[new_offset] = std::move(front());
      pop_front();
      new_offset++;
    }
    data_.swap(new_data);
    remove_ = 0;
    insert_ = new_offset;
  }

  std::vector<T> data_;
  size_t insert_ = 0;
  size_t remove_ = 0;
  size_t size_ = 0;
};
}
}
