#pragma once
#include <iostream>
#include <iterator>
#include <vector>

namespace ext {
  /**
   * @brief queue
   *
   */
  template <typename T>
  class queue {
    size_t size_;
    std::vector<T> data_;

  public:
    queue() = default;
    queue(const size_t& size) : size_(size) { data_.reserve(size); }
    queue(const queue& vq) {
      size_ = vq.queue_size();
      data_ = vq.data();
    }
    // queue(const queue &&vq)
    // {
    //     // size_ = std::move();
    //     // data_ = std::move();
    //     return *this;
    // }
    void set_queue_size(size_t size) {
      size_ = size;
      data_.reserve(size);
    }
    size_t size() { return data_.size(); }
    bool empty() { return data_.empty(); }
    T* data() { return data_.data(); }
    const T* data() const { return data_.data(); }
    typename std::vector<T>::iterator begin() { return data_.begin(); }
    typename std::vector<T>::iterator end() { return data_.end(); }
    T front() { return data_.front(); }
    T back() { return data_.back(); }
    T at(size_t n) { return data_.at(n); }
    void push_back(const T& x) { data_.push_back(x); }
    void push_back(T&& x) { data_.push_back(x); }
    void pop_back() { data_.pop_back(); }
    // ext
    void pop() { data_.erase(data_.begin()); }
    void push(T&& x) { data_.push_back(x); }
    void push(const T& x) { data_.push_back(x); }
    bool push_limit(T&& x) {
      data_.push_back(x);
      if (data_.size() > size_) {
        pop();
        return true;
      }
      return false;
    }
    bool push_limit(const T& x) {
      data_.push_back(x);
      if (data_.size() > size_) {
        pop();
        return true;
      }
      return false;
    }
    size_t queue_size() { return data_.size(); }

    T& operator[](const size_t n) { return data_.at(n); }
    constexpr T operator[](const size_t n) const { return data_.at(n); }
    constexpr bool operator==(const queue& v) const { return (size() == v.size()); }
    constexpr bool operator!=(const queue& v) const { return !(size() == v.size()); }
  };
} // namespace ext