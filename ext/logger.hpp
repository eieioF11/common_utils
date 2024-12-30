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

#include "../utility/string_utils.hpp"

namespace ext {

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
   * @brief LoggerBase
   *
   */
  class LoggerBase {
  protected:
    static constexpr const char* COLOR_RESET{"\033[m"};
    static constexpr const char* BLACK{"\e[30m"};
    static constexpr const char* RED{"\e[31m"};
    static constexpr const char* GREEN{"\e[32m"};
    static constexpr const char* YELLOW{"\e[33m"};
    static constexpr const char* BLUE{"\e[34m"};
    static constexpr const char* MAGENTA{"\e[35m"};
    static constexpr const char* CYAN{"\e[36m"};
    static constexpr const char* WHITE{"\e[37m"};

    LogLevel log_level_;
    std::string name_;

    std::string make_name_str(const std::string& name) { return "[" + name + "]"; }

    std::string make_log_level(LogLevel log_level) {
      std::unordered_map<LogLevel, std::string> log_color_map = {{LogLevel::DEBUG, LoggerBase::GREEN},
                                                                 {LogLevel::INFO, LoggerBase::COLOR_RESET},
                                                                 {LogLevel::WARN, LoggerBase::YELLOW},
                                                                 {LogLevel::ERROR, LoggerBase::RED}};
      std::string log                                         = "[" + get_log_level_str(log_level) + "]" + name_ + ": ";
      if (log_color_map.find(log_level) != log_color_map.end()) return log_color_map.at(log_level) + log;
      return log;
    }

    std::ostream& log_out(LogLevel log_level) {
      int level = static_cast<int>(log_level);
      if (level >= static_cast<int>(log_level_)) {
        if (log_level == LogLevel::ERROR || log_level == LogLevel::WARN) return std::cerr;
        return std::cout;
      }
      return dm_cout;
    }

    template <class... Args>
    void print(LogLevel log_level, bool endl, const char* fmt, Args... args) {
      log_out(log_level) << make_log_level(log_level);
      log_out(log_level) << common_utils::format(fmt, args...);
      if (endl)
        log_out(log_level) << LoggerBase::COLOR_RESET << std::endl;
      else
        log_out(log_level) << LoggerBase::COLOR_RESET;
    }

  public:
    LoggerBase() : log_level_(LogLevel::NONE) {}
    LoggerBase(LogLevel log_level) : log_level_(log_level) {}
    LoggerBase(LoggerBase& logger) : log_level_(logger.log_level_), name_(logger.name_) {}
    LoggerBase(LoggerBase&& logger) : log_level_(std::move(logger.log_level_)), name_(std::move(logger.name_)) {}
    virtual void set_name(std::string name) { name_ = make_name_str(name); }
    virtual void set_log_level(LogLevel log_level) { log_level_ = log_level; }
    /**
     * @brief Get name
     *
     * @return std::string
     */
    std::string get_name() { return name_; }
    /**
     * @brief Get log level
     *
     * @return LogLevel
     */
    LogLevel get_log_level() { return log_level_; }
  };

  /**
   * @brief LoggerType
   *
   */
  template <LogLevel out_level_>
  class LoggerType : public LoggerBase {
  public:
    LoggerType() : LoggerBase() {}
    LoggerType(LoggerType& logger) : LoggerBase(logger.log_level_) {}
    LoggerType(LoggerType&& logger) : LoggerBase(std::move(logger.log_level_)) {}

    /**
     * @brief print
     *
     * @param const char* fmt
     * @param Args... args
     */
    template <class... Args>
    void print(const char* fmt, Args... args) {
      LoggerBase::print(out_level_, false, fmt, args...);
    }

    /**
     * @brief printf
     *
     * @param const char* fmt
     * @param Args... args
     */
    template <class... Args>
    void printf(const char* fmt, Args... args) {
      LoggerBase::print(out_level_, false, fmt, args...);
    }

    /**
     * @brief println
     *
     * @param const char* fmt
     * @param Args... args
     */
    template <class... Args>
    void println(const char* fmt, Args... args) {
      LoggerBase::print(out_level_, true, fmt, args...);
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

    template <class T>
    std::ostream& operator<<(T v) {
      std::ostream& os = out();
      os << v;
      return os;
    }
  };

  /**
   * @brief Logger
   *
   */
  class Logger : public LoggerBase {
  private:
    LoggerType<LogLevel::ERROR> ERROR_;
    LoggerType<LogLevel::WARN> WARN_;
    LoggerType<LogLevel::INFO> INFO_;
    LoggerType<LogLevel::DEBUG> DEBUG_;
    LoggerType<LogLevel::NONE> NONE_;

  public:
    Logger() : LoggerBase() {}
    Logger(Logger& logger) : LoggerBase(logger.log_level_) { name_ = logger.name_; }
    Logger(Logger&& logger) : LoggerBase(std::move(logger.log_level_)) { name_ = std::move(logger.name_); }

    Logger& operator=(const Logger& logger) {
      log_level_ = logger.log_level_;
      name_      = logger.name_;
    }
    Logger& operator=(Logger&& logger) {
      log_level_ = std::move(logger.log_level_);
      name_      = std::move(logger.name_);
    }

    /**
     * @brief set_name
     *
     * @param std::string name
     */
    void set_name(std::string name) {
      name_ = make_name_str(name);
      ERROR_.set_name(name);
      WARN_.set_name(name);
      INFO_.set_name(name);
      DEBUG_.set_name(name);
      NONE_.set_name(name);
    }

    /**
     * @brief set_log_level
     *
     * @param LogLevel log_level
     */
    void set_log_level(LogLevel log_level) {
      log_level_ = log_level;
      ERROR_.set_log_level(log_level);
      WARN_.set_log_level(log_level);
      INFO_.set_log_level(log_level);
      DEBUG_.set_log_level(log_level);
      NONE_.set_log_level(log_level);
    }
    /**
     * @brief error
     *
     * @return LoggerType<LogLevel::ERROR>&
     */
    LoggerType<LogLevel::ERROR>& ERROR() { return ERROR_; }
    /**
     * @brief warn
     *
     * @return LoggerType<LogLevel::WARN>&
     */
    LoggerType<LogLevel::WARN>& WARN() { return WARN_; }
    /**
     * @brief info
     *
     * @return LoggerType<LogLevel::INFO>&
     */
    LoggerType<LogLevel::INFO>& INFO() { return INFO_; }
    /**
     * @brief debug
     *
     * @return LoggerType<LogLevel::DEBUG>&
     */
    LoggerType<LogLevel::DEBUG>& DEBUG() { return DEBUG_; }
    /**
     * @brief none
     *
     * @return LoggerType<LogLevel::NONE>&
     */
    LoggerType<LogLevel::NONE>& NONE() { return NONE_; }

    /**
     * @brief print
     *
     * @param LogLevel log_level
     * @param const char* fmt
     * @param Args... args
     */
    template <class... Args>
    void print(LogLevel log_level, const char* fmt, Args... args) {
      LoggerBase::print(log_level, false, fmt, args...);
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
      LoggerBase::print(log_level, false, fmt, args...);
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
      LoggerBase::print(log_level, true, fmt, args...);
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
     * @brief endl
     *
     * @return "\033[m\r\n"
     */
    static constexpr std::string_view endl{"\033[m\r\n"};
  };

  std::ostream& operator<<(std::ostream& os, LogLevel& log_level) {
    os << get_log_level_str(log_level);
    return os;
  }

  template <LogLevel out_level>
  std::ostream& operator<<(std::ostream& os, LoggerType<out_level>& logger) {
    LogLevel level     = logger.get_log_level();
    LogLevel out_log_level = out_level;
    os << "Log Level:" << level << "," << "Out Log Level:" << out_log_level;
    return os;
  }

  std::ostream& operator<<(std::ostream& os, Logger& logger) {
    LogLevel level = logger.get_log_level();
    os << "Log Level:" << level;
    return os;
  }

} // namespace ext
