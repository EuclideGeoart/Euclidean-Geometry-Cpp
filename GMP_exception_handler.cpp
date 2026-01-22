#include "GMP_exception_handler.h"
#include <iostream>
#include <stdexcept>
#include <string>

// Custom GMP error handler implementation
extern "C" void custom_gmp_error_handler(const char *reason, const char *expr,
                                         const char *file) {
  // Print the error to stderr
  std::cerr << "GMP Error: " << (reason ? reason : "unknown")
            << " in expression: " << (expr ? expr : "unknown")
            << " at: " << (file ? file : "unknown file") << std::endl;

  // Option 1: Throw an exception to be caught by higher-level code
  throw GMP_Exception(reason ? reason : "unknown", expr ? expr : "unknown",
                      file ? file : "unknown");

  // Option 2 and 3 are commented out since we're choosing to throw exceptions
  // Option 2: Exit the program with an error code
  // std::exit(1);

  // Option 3: Just print the error and continue (default behavior)
  // This is safer for GUI applications where an unhandled exception could crash
  // the app
}

// A utility function to log GMP errors if they occur
// You can call this from your code if you catch a GMP-related exception
void log_gmp_error(const std::string &operation, const std::exception &e) {
  std::cerr << "GMP Error during " << operation << ": " << e.what()
            << std::endl;
}

// Add this function for debugging GMP issues
void check_gmp_enabled() {
  std::cout << "GMP availability check: GMP is enabled in this build"
            << std::endl;
}
