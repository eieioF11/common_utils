#pragma once
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace common_utils {

  /**
   * @brief format
   *
   * @param fmt
   * @param args
   * @return std::string
   */
  template <class... Args>
  inline std::string format(const char* fmt, Args... args) {
    size_t len = std::snprintf(nullptr, 0, fmt, args...);
    std::string buf;
    buf.resize(len);
    std::snprintf(buf.data(), len + 1, fmt, args...);
    return buf;
  }

  /**
   * @brief 文字列分割
   *
   * @param str 文字列
   * @param delim 区切り文字
   * @return std::vector<std::string>
   */
  inline std::vector<std::string> split(const std::string str, const std::string delim = "/") {
    std::vector<std::string> out;
    size_t start;
    size_t end = 0;
    while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
      end = str.find(delim, start);
      out.push_back(str.substr(start, end - start));
    }
    return out;
  }

  /**
   * @brief vectorを文字列に変換
   *
   * @param std::vector<T> vector
   * @return std::string
   */
  template <typename T>
  std::string to_string(const std::vector<T>& data) {
    std::stringstream ss;
    std::ostream_iterator<T> out_it(ss, ", ");
    ss << "[";
    std::copy(data.begin(), data.end() - 1, out_it);
    ss << data.back() << "]";
    return ss.str();
  }

} // namespace common_utils
