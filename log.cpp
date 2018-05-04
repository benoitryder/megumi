#include <vector>
#include <memory>
#include "log.h"
// include sinks AFTER spdlog.h (needd on win32)
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/stdout_color_sinks.h>


namespace Log {

std::shared_ptr<spdlog::logger> getLogger(std::string const& name)
{
  auto logger = spdlog::get(name);
  if(!logger) {
    auto default_logger = spdlog::get("");
    if(!default_logger) {
      throw std::logic_error("no default logger");
    }
    auto const& sinks = default_logger->sinks();
    logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    spdlog::register_logger(logger);
    logger->set_level(default_logger->level());
  }
  return logger;
}

std::shared_ptr<spdlog::logger> getDefaultLogger()
{
  return spdlog::get("");
}


std::vector<StaticLogger*>& getStaticLoggers()
{
  static std::vector<StaticLogger*> loggers;
  return loggers;
}


StaticLogger::StaticLogger(const char* name): name_(name)
{
  getStaticLoggers().push_back(this);
  if(finalized_) {
    setup();
  }
}

void StaticLogger::finalize()
{
  assert(!finalized_);
  finalized_ = true;

  for(auto& instance : getStaticLoggers()) {
    instance->setup();
  }
}

void StaticLogger::setup()
{
  auto logger = getLogger(name_);
  if(!logger) {
    logger = getDefaultLogger();
  }
  if(!logger) {
    throw std::runtime_error(fmt::format("logger {} not configured", name_));
  }
  logger_ = logger;
}

spdlog::level::level_enum Setup::stringToLevel(std::string s)
{
  if(s == "trace" || s == "T") {
    return spdlog::level::trace;
  } else if(s == "debug" || s == "D") {
    return spdlog::level::debug;
  } else if(s == "info" || s == "I") {
    return spdlog::level::info;
  } else if(s == "warn" || s == "warning" || s == "W") {
    return spdlog::level::warn;
  } else if(s == "err" || s == "error" || s == "E") {
    return spdlog::level::err;
  } else if(s == "critical" || s == "C") {
    return spdlog::level::critical;
  } else if(s == "off" || s == "O") {
    return spdlog::level::off;
  } else {
    throw std::domain_error("invalid log level: " + s);
  }
}

void Setup::configure(std::string s)
{
  auto pos = s.find('=');
  if(pos == std::string::npos) {
    setDefaultLevel(stringToLevel(s));
  } else {
    setLevel(s.substr(0, pos), stringToLevel(s.substr(pos+1)));
  }
}

void Setup::finalize()
{
  auto default_logger = std::make_shared<spdlog::logger>("", default_sinks_.begin(), default_sinks_.end());
  default_logger->set_level(default_level_);
  spdlog::register_logger(default_logger);
  for(auto pair : levels_) {
    auto logger = std::make_shared<spdlog::logger>(pair.first, default_sinks_.begin(), default_sinks_.end());
    spdlog::register_logger(logger);
    logger->set_level(pair.second);
  }
}


spdlog::sink_ptr Setup::createDefaultSink()
{
  spdlog::sink_ptr sink;
  if(!isatty(_fileno(stderr))) {
    // not a terminal: no color
    sink = std::make_shared<spdlog::sinks::stderr_sink_mt>();
  } else {
    sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  }
  sink->set_pattern("%T  %L %n  %v");
  return sink;
}


}

