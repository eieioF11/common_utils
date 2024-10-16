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
  /**
   * @brief vector -> Eigen::VectorX
   *
   * @tparam T
   * @param vec std::vector<T>
   * @return Eigen::VectorX<T>
   */
  template <class T>
  constexpr inline Eigen::VectorX<T> set_vectorx(std::vector<T>& vec) {
    Eigen::Map<Eigen::VectorX<T>> p(vec.data(), vec.size());
    return p;
  }

  /**
   * @brief vector -> Eigen::VectorX
   *
   * @tparam T
   * @param vec std::vector<T>
   * @return Eigen::VectorX<T>
   */
  template <class T>
  constexpr inline Eigen::VectorX<T> set_vectorx(std::vector<T> vec) {
    Eigen::Map<Eigen::VectorX<T>> p(vec.data(), vec.size());
    return p;
  }

  /**
   * @brief approx_eq Eigen::VectorX version
   *
   * @tparam T
   * @param  a Eigen::VectorX<T>
   * @param  b Eigen::VectorX<T>
   * @param  range T default:1e-12
   * @return bool
   */
  template <class T>
  constexpr inline bool approx_eq(const Eigen::VectorX<T>& a,const Eigen::VectorX<T>& b, T range = 1e-12) {
    return (bool)((a - b).norm() < range);
  }
} // namespace common_utils