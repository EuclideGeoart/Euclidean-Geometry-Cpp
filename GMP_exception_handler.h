#ifndef GMP_EXCEPTION_HANDLER_H
#define GMP_EXCEPTION_HANDLER_H

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

// Exception class for GMP errors
class GMP_Exception : public std::runtime_error {
public:
  GMP_Exception(const std::string &reason, const std::string &expr,
                const std::string &file)
      : std::runtime_error("GMP error: " + reason + " in " + expr + " at " +
                           file) {}
};

// Simplified error handler setup - doesn't rely on __gmp_set_error_handler
inline bool setup_gmp_error_handling() {
  // Instead of trying to call the GMP function directly, we'll just log that
  // we're setting up error handling and return success.
  // If actual GMP errors occur, they will be handled by GMP's default handler.
  std::cout << "GMP error handling setup: Using default GMP error handler"
            << std::endl;
  return true;
}

// A utility function to log GMP errors if they occur
void log_gmp_error(const std::string &operation, const std::exception &e);

// Add this function for debugging GMP issues
void check_gmp_enabled();

#endif // GMP_EXCEPTION_HANDLER_H
