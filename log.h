#ifndef LOG_H
#define LOG_H

#include <sstream>
#include <boost/format.hpp>

namespace Log {

/// Log severities
enum Severity {
  INFO = 0,
  NOTICE,
  WARNING,
  ERROR,
};


/// Set minimum report level (not thread-safe)
void setMinimumSeverity(Severity severity);


/// Message logger
class Message
{
 public:
  Message(Severity severity);
  ~Message();

  std::ostringstream& stream() { return stream_; }

  /// Write message to destination
  void flush();

 private:
  Severity severity_;
  std::ostringstream stream_;
};


// For DLOG implementation in NDEBUG mode
struct VoidMessage
{
  VoidMessage() {}
  template <typename T> void operator&(const T&) const {}
};

}


// Main log macro
#define LOG(severity) \
    ::Log::Message(::Log::Severity::severity).stream()
// Use boost::format formatting
#define LOGF(severity, f)  LOG(severity) << boost::format(f)

// Log message if condition is not fullfilled
#define CHECK(severity, cond)  (cond) ? (void)0 : ::Log::VoidMessage() & LOG(severity)
#define CHECKF(severity, cond, f)  (cond) ? (void)0 : ::Log::VoidMessage() & LOGF(severity, f)


// Debug message
#ifndef NDEBUG
#define DLOG(severity)  LOG(severity)
#define DLOGF(severity, f)  LOGF(severity, f)
#define DCHECK(severity, cond)  CHECK(severity, cond)
#define DCHECKF(severity, cond, f)  CHECKF(severity, conf, f)
#else
#define DLOG(severity)  true ? (void)0 : ::Log::VoidMessage() & LOG(severity)
#define DLOGF(severity, f)  true ? (void)0 : ::Log::VoidMessage() & LOGF(severity, f)
#define DCHECK(severity, cond)  DLOG(severity)
#define DCHECKF(severity, cond, f)  DLOGF(severity, f)
#endif

#endif
