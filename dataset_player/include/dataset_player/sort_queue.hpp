// Copyright <2021> [Copyright rossihwang@gmail.com]

#pragma once

#include <cstdio>
#include <queue>

namespace slamtk {

template <typename T>
struct StampedData {
  double stamp;
  T data;
  StampedData(double stamp, T data)
    : stamp(stamp),
      data(data) {}
  StampedData()
    : stamp(0.0) {}
};

template <typename T>
bool operator<(const StampedData<T>& a, const StampedData<T>& b) {
  return b.stamp < a.stamp;
}

template <typename T>
class SortQueue {
 protected:
  size_t limit_size_;
  std::priority_queue<StampedData<T>> q_;

 public:
  SortQueue(size_t size)
    : limit_size_(size) {

  }
  ~SortQueue() = default;
  void push(T data, double stamp) {
    q_.push(StampedData<T>(stamp, data));
  }
  void pop() {
    q_.pop();
  }
  StampedData<T> top() {
    return q_.top();
  }
  bool full() {
    return limit_size_ <= q_.size();
  }
  size_t size() {
    return q_.size();
  }
}; 

}  // namespace slamtk