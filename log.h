#ifndef LOG_H
#define LOG_H

#include <vector>
#include <map>
#include <string>
#include <spdlog/spdlog.h>
#undef ERROR

/** @brief Logging elements
 *
 * @note Since spdlog is header-only, each executable or library will
 * instantiate and use its own "global" registry.
 * Therefore, one must not use spdlog registry directly to configure or use
 * project's logging.
 */
namespace Log {

/// Return a registered logger, initialize one based on default if needed
std::shared_ptr<spdlog::logger> getLogger(std::string const& name);
/// Return default logger
std::shared_ptr<spdlog::logger> getDefaultLogger();


/** @brief Helper to register static loggers, defined at module level
 *
 * To use a static logger:
 *  - define a StaticLogger variable
 *  - create and configure the loggers (typically, from main())
 *  - call StaticLogger::finalize() to set the StaticLogger pointers
 */
class StaticLogger
{
 public:
  StaticLogger(const char* name);

  spdlog::logger* operator->() { return logger_.get(); }

  /** @brief Finalize static loggers
   *
   * This method must be called after loggers have been setup and
   * before they are used.
   *
   * Static loggers alreadey initialized are finalized. Static loggers
   * initialized afterwards will be finalized immediately on creation.
   */
  static void finalize();

 protected:
  /// Setup a single static logger
  void setup();

 private:
  static inline bool finalized_ = false;
  std::shared_ptr<spdlog::logger> logger_;
  const char* name_;
};

/// Setup loggers
class Setup
{
 public:
  void setDefaultLevel(spdlog::level::level_enum level) { default_level_ = level; }
  void addDefaultSink(spdlog::sink_ptr sink) { default_sinks_.push_back(sink); }
  void setLevel(std::string name, spdlog::level::level_enum level) { levels_[name] = level; }
  /// Configure logger using an option string (`[logger=]level`)
  void configure(std::string s);
  /// Finalize configuration, create the loggers and set levels
  void finalize();

  static spdlog::level::level_enum stringToLevel(std::string s);

  /// Create and return a default sink
  static spdlog::sink_ptr createDefaultSink();

 private:
  std::vector<spdlog::sink_ptr> default_sinks_;
  spdlog::level::level_enum default_level_ = spdlog::level::info;
  std::map<std::string, spdlog::level::level_enum> levels_;
};

}


#endif
