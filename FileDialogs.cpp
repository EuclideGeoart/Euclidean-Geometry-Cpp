#include "FileDialogs.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#endif

// Linker Check: Ensure -lcomdlg32 is added to the build command (handled in CMakeLists.txt)!

#ifndef _WIN32
namespace {

std::string trimWhitespace(std::string value) {
  auto notSpace = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
  value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
  return value;
}

std::string shellEscapeSingleQuotes(const std::string& input) {
  std::string escaped;
  escaped.reserve(input.size() + 8);
  for (char ch : input) {
    if (ch == '\'') {
      escaped += "'\\''";
    } else {
      escaped += ch;
    }
  }
  return escaped;
}

std::string extractPatternFromFilter(const char* filter, const char* fallbackPattern) {
  if (!filter || !*filter) return fallbackPattern ? std::string(fallbackPattern) : std::string("*.*");

  std::string firstPart(filter);
  auto openParen = firstPart.find('(');
  auto closeParen = firstPart.find(')', openParen == std::string::npos ? 0 : openParen + 1);
  if (openParen != std::string::npos && closeParen != std::string::npos && closeParen > openParen + 1) {
    std::string inside = firstPart.substr(openParen + 1, closeParen - openParen - 1);
    inside = trimWhitespace(inside);
    if (!inside.empty()) return inside;
  }

  std::size_t wildcardPos = firstPart.find("*.");
  if (wildcardPos != std::string::npos) {
    std::size_t end = wildcardPos + 2;
    while (end < firstPart.size() && (std::isalnum(static_cast<unsigned char>(firstPart[end])) || firstPart[end] == '*')) {
      ++end;
    }
    return firstPart.substr(wildcardPos, end - wildcardPos);
  }

  return fallbackPattern ? std::string(fallbackPattern) : std::string("*.*");
}

std::string runZenityCommand(const std::string& command) {
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    std::cerr << "[Linux] Failed to launch zenity command." << std::endl;
    return std::string();
  }

  std::string output;
  char buffer[512];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  int rc = pclose(pipe);
  if (rc != 0) {
    return std::string();
  }

  output = trimWhitespace(output);
  return output;
}

std::string ensureDefaultExtension(const std::string& path, const char* defaultExt) {
  if (!defaultExt || !*defaultExt || path.empty()) return path;

  std::string ext(defaultExt);
  if (!ext.empty() && ext[0] == '.') {
    ext.erase(ext.begin());
  }
  if (ext.empty()) return path;

  std::size_t slashPos = path.find_last_of("/");
  std::size_t dotPos = path.find_last_of('.');
  bool hasExt = (dotPos != std::string::npos) && (slashPos == std::string::npos || dotPos > slashPos);
  if (hasExt) return path;

  return path + "." + ext;
}

}  // namespace
#endif

std::string FileDialogs::SaveFile(const char *filter, const char *defaultExt) {
#ifdef _WIN32
  // --- WINDOWS IMPLEMENTATION ---
  OPENFILENAMEA ofn;
  char szFile[260] = {0};

  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = NULL;
  ofn.lpstrFile = szFile;
  ofn.nMaxFile = sizeof(szFile);
  ofn.lpstrFilter = filter;
  ofn.nFilterIndex = 1;
  ofn.lpstrFileTitle = NULL;
  ofn.nMaxFileTitle = 0;
  ofn.lpstrInitialDir = NULL;
  ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR | OFN_OVERWRITEPROMPT;
  ofn.lpstrDefExt = defaultExt;

  if (GetSaveFileNameA(&ofn) == TRUE) {
    return std::string(ofn.lpstrFile);
  }
  return std::string();

#else
  std::string pattern = extractPatternFromFilter(filter, "*.fluxgeo");
  std::string command = "zenity --file-selection --save --confirm-overwrite --title='Save As' --file-filter='" +
                        shellEscapeSingleQuotes(pattern) + "' 2>/dev/null";

  std::string path = runZenityCommand(command);
  if (path.empty()) return std::string();

  return ensureDefaultExtension(path, defaultExt);
#endif
}

std::string FileDialogs::OpenFile(const char *filter) {
#ifdef _WIN32
  // --- WINDOWS IMPLEMENTATION ---
  OPENFILENAMEA ofn;
  char szFile[260] = {0};

  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = NULL;
  ofn.lpstrFile = szFile;
  ofn.nMaxFile = sizeof(szFile);
  ofn.lpstrFilter = filter;
  ofn.nFilterIndex = 1;
  ofn.lpstrFileTitle = NULL;
  ofn.nMaxFileTitle = 0;
  ofn.lpstrInitialDir = NULL;
  ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

  if (GetOpenFileNameA(&ofn) == TRUE) {
    return std::string(ofn.lpstrFile);
  }
  return std::string();

#else
  std::string pattern = extractPatternFromFilter(filter, "*.fluxgeo");
  std::string command = "zenity --file-selection --title='Open Project' --file-filter='" +
                        shellEscapeSingleQuotes(pattern) + "' 2>/dev/null";

  return runZenityCommand(command);
#endif
}