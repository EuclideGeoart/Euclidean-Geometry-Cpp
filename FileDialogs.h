#pragma once
#include <string>

class FileDialogs {
 public:
  // Returns empty string if cancelled
  static std::string SaveFile(const char *filter, const char *defaultExt);
  static std::string OpenFile(const char *filter);
};
