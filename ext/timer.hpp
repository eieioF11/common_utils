#pragma once
#include "../utility/unit_utils.hpp"
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace ext {
  /**
   * @brief タイマーライブラリ
   *
   */
  template <typename FLOATING_TYPE>
  class Timer;
  using Timerf = Timer<float>;
  using Timerd = Timer<double>;

  template <typename FLOATING_TYPE>
  class Timer {
  public:
    Timer() { reset(); }

    /**
     * @brief Reset
     *
     */
    void reset() { start_ = std::chrono::high_resolution_clock::now(); }

    /**
     * @brief 計測開始
     *
     */
    void start() { reset(); }

    /**
     * @brief startもしくはresetからの経過時間取得
     *
     * @tparam UNIT
     */
    template <typename UNIT = common_utils::unit::time::s>
    FLOATING_TYPE elapsed() {
      using namespace common_utils;
      std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
      return unit_cast<unit::time::s, UNIT, FLOATING_TYPE>(std::chrono::duration_cast<std::chrono::duration<FLOATING_TYPE>>(end - start_).count());
    }

    /**
     * @brief 実行周期の取得
     *
     * @tparam UNIT
     */
    template <typename UNIT = common_utils::unit::time::s>
    FLOATING_TYPE period() {
      using namespace common_utils;
      std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
      FLOATING_TYPE period_time = std::chrono::duration_cast<std::chrono::duration<FLOATING_TYPE>>(end - start_).count();
      start_                    = std::chrono::high_resolution_clock::now();
      return period_time;
    }

    /**
     * @brief 実行周期判定
     *
     * @tparam UNIT
     * @param t : 実行周期
     */
    template <typename UNIT = common_utils::unit::time::s>
    bool cyclic(FLOATING_TYPE t) {
      using namespace common_utils;
      FLOATING_TYPE cyclic_time                                       = unit_cast<unit::time::s, UNIT, FLOATING_TYPE>(t);
      std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
      FLOATING_TYPE now_time = std::chrono::duration_cast<std::chrono::duration<FLOATING_TYPE>>(end - start_).count();
      if (now_time > cyclic_time) {
        reset();
        return true;
      }
      return false;
    }

  private:
    std::chrono::high_resolution_clock::time_point start_;
  };
} // namespace ext
