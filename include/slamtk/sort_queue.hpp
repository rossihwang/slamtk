// Copyright <2021> [Copyright rossihwang@gmail.com]

#pragma once

#include <cstdio>
#include <list>

namespace toolkits {

template <typename T>
class SortQueue {
 protected:
  size_t limit_size_;
  std::list<std::pair<double, T>> list_;

 public:
  SortQueue(size_t size)
    : limit_size_(size) {

  }
  ~SortQueue() = default;
  void push(T data, double stamp) {
    list_.push_back(std::make_pair(stamp, data));
    list_.sort([](const std::pair<double, T>& a, const std::pair<double, T>& b) {
                return a.first < b.first;
              });
    // for (auto l: list_) {
    //   printf("%.6f, ", l.first);
    // }
    // std::cout << std::endl;
  }
  void pop() {
    list_.pop_front();
  }
  std::pair<double,  T> front() {
    return list_.front();
  }
  bool full() {
    return limit_size_ <= list_.size();
  }
  size_t size() {
    return list_.size();
  }
}; 

}  // namespace toolkits