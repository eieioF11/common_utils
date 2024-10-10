#pragma once
// std
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
// Eigen
#include <Eigen/Dense>

namespace common_utils {
  template <class T>
  constexpr inline Eigen::VectorX<T> set_vectorx(std::vector<T>& vec) {
    Eigen::Map<Eigen::VectorX<T>> p(vec.data(), vec.size());
    return p;
  }
  template <class T>
  constexpr inline Eigen::VectorX<T> set_vectorx(std::vector<T> vec) {
    Eigen::Map<Eigen::VectorX<T>> p(vec.data(), vec.size());
    return p;
  }
} // namespace common_utils