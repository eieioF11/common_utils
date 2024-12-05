#pragma once
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "string_utils.hpp"

// color code
#define LOG_COLOR_RESET "\033[m"
#define LOG_BLACK "\e[30m"
#define LOG_RED "\e[31m"
#define LOG_GREEN "\e[32m"
#define LOG_YELLOW "\e[33m"
#define LOG_BLUE "\e[34m"
#define LOG_MAGENTA "\e[35m"
#define LOG_CYAN "\e[36m"
#define LOG_WHITE "\e[37m"
// log
#define LOG_DEBUG std::cout << LOG_GREEN << "[DEBUG]"
#define LOG_INFO std::cout << "[INFO]"
#define LOG_WARN std::cout << LOG_YELLOW << "[WARN]"
#define LOG_ERROR std::cout << LOG_RED << "[ERROR]"
#define LOG_ENDL LOG_COLOR_RESET << std::endl

namespace common_utils {

  /**
   * @brief LogLevel
   *
   */
  enum class LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, NONE = 4 };

  /**
   * @brief Get the log level object
   *
   * @param std::string log_level
   * @return LogLevel
   */
  LogLevel get_log_level(std::string log_level) {
    std::unordered_map<std::string, LogLevel> log_level_map = {
        {"DEBUG", LogLevel::DEBUG}, {"INFO", LogLevel::INFO}, {"WARN", LogLevel::WARN}, {"ERROR", LogLevel::ERROR}};
    if (log_level_map.find(log_level) != log_level_map.end()) return log_level_map.at(log_level);
    return LogLevel::NONE;
  }

  /**
   * @brief Get the log level str object
   *
   * @param LogLevel log_level
   * @return std::string
   */
  std::string get_log_level_str(LogLevel log_level) {
    std::unordered_map<LogLevel, std::string> log_level_map = {
        {LogLevel::DEBUG, "DEBUG"}, {LogLevel::INFO, "INFO"}, {LogLevel::WARN, "WARN"}, {LogLevel::ERROR, "ERROR"}};
    if (log_level_map.find(log_level) != log_level_map.end()) return log_level_map.at(log_level);
    return "NONE";
  }

  std::ofstream dm_cout("/dev/null", std::ios::out | std::ios::app);

  /**
   * @brief Logger
   *
   */
  class LoggerBase {
  protected:
    LogLevel log_level_;

  private:
    LogLevel out_level_ = LogLevel::DEBUG;
    std::string make_log_level(LogLevel log_level) {
      std::unordered_map<LogLevel, std::string> log_color_map = {
          {LogLevel::DEBUG, LOG_GREEN}, {LogLevel::INFO, LOG_COLOR_RESET}, {LogLevel::WARN, LOG_YELLOW}, {LogLevel::ERROR, LOG_RED}};
      std::string log = "[" + get_log_level_str(log_level) + "]";
      if (log_color_map.find(log_level) != log_color_map.end()) return log_color_map.at(log_level) + log;
      return log;
    }

    std::ostream& log_out(LogLevel log_level) {
      int level = static_cast<int>(log_level);
      if (level >= static_cast<int>(log_level_)) return std::cout;
      return dm_cout;
    }

  public:
    LoggerBase() : log_level_(LogLevel::NONE) {}
    LoggerBase(LogLevel log_level) : log_level_(log_level) {}
    void set_out_level(LogLevel out_level) { out_level_ = out_level; }
    virtual void set_log_level(LogLevel log_level) { log_level_ = log_level; }
    /**
     * @brief print
     *
     * @param LogLevel log_level
     * @param const char* fmt
     * @param Args... args
     */
    template <class... Args>
    void print(LogLevel log_level, const char* fmt, Args... args) {
      log_out(log_level) << make_log_level(log_level);
      log_out(log_level) << common_utils::format(fmt, args...);
      log_out(log_level) << LOG_COLOR_RESET;
    }
    /**
     * @brief printf
     *
     * @param LogLevel log_level
     * @param const char* fmt
     * @param Args... args
     */
    template <class... Args>
    void printf(LogLevel log_level, const char* fmt, Args... args) {
      print(log_level, fmt, args...);
    }

    /**
     * @brief println
     *
     * @param LogLevel log_level
     * @param const char* fmt
     * @param Args... args
     */
    template <class... Args>
    void println(LogLevel log_level, const char* fmt, Args... args) {
      log_out(log_level) << make_log_level(log_level);
      log_out(log_level) << common_utils::format(fmt, args...);
      log_out(log_level) << LOG_COLOR_RESET << std::endl;
    }

    /**
     * @brief out
     *
     * @param LogLevel log_level
     * @return std::ostream&
     */
    std::ostream& out(LogLevel log_level) {
      log_out(log_level) << make_log_level(log_level);
      return log_out(log_level);
    }

    /**
     * @brief out
     *
     * @return std::ostream&
     */
    std::ostream& out() {
      log_out(out_level_) << make_log_level(out_level_);
      return log_out(out_level_);
    }
  };

  class Logger : public LoggerBase {
  private:
    LoggerBase ERROR_;
    LoggerBase WARN_;
    LoggerBase INFO_;
    LoggerBase DEBUG_;
    LoggerBase NONE_;
		void init_logger() {
			ERROR_.set_out_level(LogLevel::ERROR);
			WARN_.set_out_level(LogLevel::WARN);
			INFO_.set_out_level(LogLevel::INFO);
			DEBUG_.set_out_level(LogLevel::DEBUG);
			NONE_.set_out_level(LogLevel::NONE);
		}
  public:
    Logger() : LoggerBase() {
			init_logger();
    }
    void set_log_level(LogLevel log_level) {
      log_level_ = log_level;
      ERROR_.set_log_level(log_level);
      WARN_.set_log_level(log_level);
      INFO_.set_log_level(log_level);
      DEBUG_.set_log_level(log_level);
      NONE_.set_log_level(log_level);
    }
    LoggerBase& ERROR() { return ERROR_; }
    LoggerBase& WARN() { return WARN_; }
    LoggerBase& INFO() { return INFO_; }
    LoggerBase& DEBUG() { return DEBUG_; }
    LoggerBase& NONE() { return NONE_; }
    /**
     * @brief endl
     *
     * @return "\033[m\r\n"
     */
    static constexpr std::string_view endl{"\033[m\r\n"};
  };

  // Logger logger;
  // std::ostream& operator<<(std::ostream& os, const LogLevel& log_level) { return logger.out(log_level); }
  std::ostream& operator<<(std::ostream& os, LoggerBase& logger) { return logger.out(); }

} // namespace common_utils