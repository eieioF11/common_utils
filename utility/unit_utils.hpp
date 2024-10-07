#pragma once
#include "math_utils.hpp"

namespace common_utils {

  // 単位変換
  /*
  Ex:
  //SI単位変換 milli->基準単位
  double v = unit_cast<unit::si::m, unit::si::base>(90.0);
  double v = unit_cast<unit::si::m>(90.0);
  //時間の変換 分->秒
  double v = unit_cast<unit::time::m, unit::time::s>(1.0);
  double v = unit_cast<unit::time::m>(1.0);
  //角度単位変換
  double rad = unit_cast<unit::angle::deg,unit::angle::rad>(90.0);
  double deg = unit_cast<unit::angle::rad,unit::angle::deg>(constants::PI/2);
  double rad = unit_cast<unit::angle::rad>(90.0);
  double deg = unit_cast<unit::angle::deg>(constants::PI / 2);
  */

  enum class angle_unit_type { RADIAN, DEGREE };

  inline namespace unit {
    inline namespace si {
      using femto = std::femto;
      using pico  = std::pico;
      using nano  = std::nano;
      using micro = std::micro;
      using milli = std::milli;
      using centi = std::centi;
      using deci  = std::deci;
      using base  = std::ratio<1>; // 基準
      using deca  = std::deca;
      using hecto = std::hecto;
      using kilo  = std::kilo;
      using mega  = std::mega;
      using giga  = std::giga;
      using tera  = std::tera;
      using peta  = std::peta;
      using exa   = std::exa;

      using f  = femto;
      using p  = pico;
      using n  = nano;
      using u  = micro;
      using m  = milli;
      using c  = centi;
      using d  = deci;
      using da = deca;
      using h  = hecto;
      using k  = kilo;
      using M  = mega;
      using G  = giga;
      using T  = tera;
      using P  = peta;
      using E  = exa;
    } // namespace si
    inline namespace time {
      using nanoseconds  = si::nano;
      using microseconds = si::micro;
      using milliseconds = si::milli;
      using seconds      = std::ratio<1>;
      using minutes      = std::ratio<60>;
      using hours        = std::ratio<3600>;

      using ns = nanoseconds;
      using us = microseconds;
      using ms = milliseconds;
      using s  = seconds;
      using m  = minutes;
      using h  = hours;
    } // namespace time
    inline namespace angle {
      inline constexpr angle_unit_type degree = angle_unit_type::DEGREE;
      inline constexpr angle_unit_type radian = angle_unit_type::RADIAN;
      inline constexpr angle_unit_type deg    = degree;
      inline constexpr angle_unit_type rad    = radian;
    } // namespace angle
  } // namespace unit

  /**
   * @brief SI単位と時間の変換
   *
   * @tparam IN_UNIT
   * @tparam OUT_UNIT
   * @tparam T
   * @param value
   * @return constexpr T
   */
  template <typename IN_UNIT, typename OUT_UNIT = std::ratio<1>, typename T = double>
  static constexpr T unit_cast(T value) {
    std::chrono::duration<T, IN_UNIT> in(value);
    std::chrono::duration<T, OUT_UNIT> out = std::chrono::duration_cast<std::chrono::duration<T, OUT_UNIT>>(in);
    return out.count();
  }

  /**
   * @brief 角度単位の変換
   *
   * @tparam IN_UNIT
   * @tparam OUT_UNIT
   * @tparam T
   * @param value
   * @return constexpr T
   */
  template <angle_unit_type IN_UNIT, angle_unit_type OUT_UNIT, typename T = double>
  static constexpr T unit_cast(T value) {
    if (IN_UNIT == angle_unit_type::DEGREE) {
      if (OUT_UNIT == angle_unit_type::RADIAN) return value * constants::DEG_TO_RAD;
    } else {
      if (OUT_UNIT == angle_unit_type::DEGREE) return value * constants::RAD_TO_DEG;
    }
    return value;
  }

  /**
   * @brief 角度単位の変換
   *
   * @tparam OUT_UNIT
   * @tparam T
   * @param value
   * @return constexpr T
   */
  template <angle_unit_type OUT_UNIT, typename T = double>
  static constexpr T unit_cast(T value) {
    if (OUT_UNIT == angle_unit_type::RADIAN)
      return value * constants::DEG_TO_RAD;
    else
      return value * constants::RAD_TO_DEG;
  }

} // namespace common_utils