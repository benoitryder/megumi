#include <iostream>
#include <iomanip>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include "log.h"


namespace Log {

static const char* severity_names[] = {
  "INFO",
  "NOTICE",
  "WARNING",
  "ERROR",
};


static Severity minimum_severity = Severity::INFO;

void setMinimumSeverity(Severity severity)
{
  minimum_severity = severity;
}


Message::Message(Severity severity):
    severity_(severity)
{
  struct ::timeval tnow;
  ::gettimeofday(&tnow, NULL);
  struct ::tm tloc = {};
  time_t timep = tnow.tv_sec;
  localtime_r(&timep, &tloc);

  stream_
      << std::setfill('0')
      << std::setw(2) << tloc.tm_hour << ':'
      << std::setw(2) << tloc.tm_min  << ':'
      << std::setw(2) << tloc.tm_sec  << '.'
      << std::setw(6) << tnow.tv_usec
      << ' ' << severity_names[severity]
      << " - "
      ;
}

Message::~Message()
{
  flush();
}

void Message::flush()
{
  if(severity_ >= minimum_severity) {
    std::cout << stream_.str() << std::endl << std::flush;
  }
}


}

