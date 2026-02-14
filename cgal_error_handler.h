#ifndef CGAL_ERROR_HANDLER_H
#define CGAL_ERROR_HANDLER_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

// Remove problematic include
// #include "CGALExceptions.h"

#include <CGAL/assertions.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/exceptions.h>
#include <iostream>
#include <stdexcept>
#include <string>

// Custom CGAL error handlers to prevent exit() calls
inline void cgal_custom_assertion_fail_handler(const char *condition,
                                               const char *expr,
                                               const char *file, int line,
                                               const char *msg) {
  std::cerr << "CGAL assertion failure: " << expr << std::endl
            << "  Condition: " << (condition ? condition : "unknown")
            << std::endl
            << "  File: " << file << " Line: " << line << std::endl;
  if (msg)
    std::cerr << "  Message: " << msg << std::endl;

  // Just use standard exception directly
  throw std::runtime_error(std::string("CGAL assertion failure: ") +
                           (msg ? msg : "unknown"));
}

// Updated with correct signature (5 parameters instead of 4)
inline void cgal_custom_warning_handler(const char *condition, const char *expr,
                                        const char *file, int line,
                                        const char *msg) {
  std::cerr << "CGAL warning: " << expr << std::endl
            << "  Condition: " << (condition ? condition : "unknown")
            << std::endl
            << "  File: " << file << " Line: " << line << std::endl;
  if (msg)
    std::cerr << "  Message: " << msg << std::endl;
  // Just log warnings, don't throw or terminate
}

// Function to install our custom error handlers
inline void install_custom_cgal_error_handlers() {
  CGAL::set_error_handler(cgal_custom_assertion_fail_handler);
  CGAL::set_warning_handler(cgal_custom_warning_handler);
  // Don't set the error behavior to CGAL::CONTINUE as that may lead to
  // undefined behavior; instead let exceptions propagate
}

#endif // CGAL_ERROR_HANDLER_H
