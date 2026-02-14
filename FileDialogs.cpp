#include "FileDialogs.h"
#include <iostream>

#ifdef _WIN32
    #include <windows.h>
    #include <commdlg.h>
#endif

// Linker Check: Ensure -lcomdlg32 is added to the build command (handled in CMakeLists.txt)!

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
  // --- LINUX STUB (Placeholder) ---
  // Since we haven't integrated Zenity/GTK yet, we just log an error and return empty.
  // This prevents build errors.
  std::cerr << "[Linux] Save File Dialog not yet implemented." << std::endl;
  return ""; 
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
  // --- LINUX STUB (Placeholder) ---
  std::cerr << "[Linux] Open File Dialog not yet implemented." << std::endl;
  return "";
#endif
}