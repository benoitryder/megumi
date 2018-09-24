#ifndef WIN32_H
#define WIN32_H

#include <windows.h>
#undef ERROR
#include <exception>
#include <string>


/** @brief Exception class for Win32 errors
 *
 * @warning On mingw, GetLastError() must be called before the \e throw
 * statement since it resets the error.
 * That is also why there is no constructor with default error code.
 */
class win32_error: public std::exception
{
 public:
  win32_error(DWORD code) noexcept:
      code_(code), msg_(error_code_to_string(code)) {}
  win32_error(DWORD code, const std::string& msg) noexcept:
      code_(code), msg_(msg+": "+error_code_to_string(code)) {}

  virtual ~win32_error() {}
  DWORD code() const noexcept { return code_; }
  const char* what() const noexcept override { return msg_.c_str(); }

 private:
  static std::string error_code_to_string(DWORD code)
  {
    LPTSTR err = NULL;
    if(!FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER|FORMAT_MESSAGE_FROM_SYSTEM, NULL, code, 0, (LPTSTR)&err, 0, NULL)) {
      return "unknown";
    } else {
      std::string ret(err);
      LocalFree(err);
      return ret;
    }
  }

  DWORD code_;
  std::string msg_;
};


#endif
