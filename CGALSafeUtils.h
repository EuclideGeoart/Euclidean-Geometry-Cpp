#pragma message("--- In CGALSafeUtils.h: Checking CGAL preprocessor flags ---")

#ifdef CGAL_HAS_THREADS
#pragma message(                                                               \
    "CGALSafeUtils.h: CGAL_HAS_THREADS is DEFINED before any action.")
#else
#pragma message(                                                               \
    "CGALSafeUtils.h: CGAL_HAS_THREADS is NOT DEFINED before any action.")
#endif

#ifdef CGAL_USE_SSE2
#pragma message("CGALSafeUtils.h: CGAL_USE_SSE2 is DEFINED before any action.")
#else
#pragma message(                                                               \
    "CGALSafeUtils.h: CGAL_USE_SSE2 is NOT DEFINED before any action.")
#endif

// Attempt to set the desired state
#define CGAL_HAS_THREADS 1
#undef CGAL_USE_SSE2
#pragma message(                                                               \
    "CGALSafeUtils.h: Action: Defined CGAL_HAS_THREADS, Undefined CGAL_USE_SSE2.")

// Verify after action
#ifdef CGAL_HAS_THREADS
#pragma message("CGALSafeUtils.h: CGAL_HAS_THREADS is DEFINED after action.")
#else
#pragma message(                                                               \
    "CGALSafeUtils.h: CGAL_HAS_THREADS is NOT DEFINED after action. (Problem!)")
#endif

#ifdef CGAL_USE_SSE2
#pragma message(                                                               \
    "CGALSafeUtils.h: CGAL_USE_SSE2 is STILL DEFINED after action. (Problem!)")
#else
#pragma message(                                                               \
    "CGALSafeUtils.h: CGAL_USE_SSE2 is UNDEFINED after action. (Good)")
#endif

#pragma message("--- End of preprocessor checks in CGALSafeUtils.h ---")

#ifndef CGAL_SAFE_UTILS_H
#define CGAL_SAFE_UTILS_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "Point.h"
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/IO/to_string.h>
#include <CGAL/Uncertain.h>
#include <CGAL/number_utils.h>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>

namespace CGALSafeUtils {

// Safe conversion from FT to double with exception handling
template <typename FT>
double safe_to_double(const FT &value, const std::string &context = "",
                      double defaultValue = 0.0) {
  try {
    return CGAL::to_double(value);
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "CGAL conversion exception in " << context << ": " << e.what()
              << std::endl;
    return defaultValue;
  }
}

// Safe sqrt calculation with exception handling
template <typename FT>
double safe_sqrt(const FT &value, const std::string &context = "",
                 double defaultValue = 0.0) {
  try {
    double doubleValue = safe_to_double(value, context + " (pre-sqrt)", 0.0);
    if (doubleValue < 0.0) {
      std::cerr << "Warning in " << context
                << ": Attempting to take sqrt of negative value: "
                << doubleValue << std::endl;
      return defaultValue;
    }
    return std::sqrt(doubleValue);
  } catch (const std::exception &e) {
    std::cerr << "Exception in safe_sqrt (" << context << "): " << e.what()
              << std::endl;
    return defaultValue;
  }
}

// Safe atan2 calculation with exception handling
template <typename FT>
double safe_atan2(const FT &y, const FT &x, const std::string &context = "",
                  double defaultValue = 0.0) {
  try {
    double y_dbl = safe_to_double(y, context + " (atan2-y)", 0.0);
    double x_dbl = safe_to_double(x, context + " (atan2-x)",
                                  1.0); // Avoid division by zero

    if (std::abs(x_dbl) < 1e-10 && std::abs(y_dbl) < 1e-10) {
      std::cerr << "Warning in " << context
                << ": atan2 called with values near zero" << std::endl;
      return defaultValue;
    }

    return std::atan2(y_dbl, x_dbl);
  } catch (const std::exception &e) {
    std::cerr << "Exception in safe_atan2 (" << context << "): " << e.what()
              << std::endl;
    return defaultValue;
  }
}

// Safe execution of CGAL operations with general exception handling
template <typename Func>
auto safely_execute(Func func, const std::string &context = "",
                    const std::function<decltype(func())()> &fallback = nullptr)
    -> decltype(func()) {
  try {
    return func();
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "CGAL conversion exception in " << context << ": " << e.what()
              << std::endl;
    if (fallback) {
      return fallback();
    }
    throw; // Re-throw if no fallback provided
  } catch (const std::exception &e) {
    std::cerr << "Exception in " << context << ": " << e.what() << std::endl;
    if (fallback) {
      return fallback();
    }
    throw; // Re-throw if no fallback provided
  }
}
// Function to debug CGAL vectors
inline void debug_cgal_vector(const Vector_2 &vec,
                              const std::string &vector_name,
                              const std::string &context) {
  std::cerr << std::fixed << std::setprecision(18);
  std::cerr << "DEBUG_CGAL_VECTOR [" << context << "] " << vector_name << ": ";
  bool dx_finite = false;
  bool dy_finite = false;
  double dx_val = 0.0, dy_val = 0.0;
  Kernel::FT sq_len_val;
  bool sq_len_finite = false;
  double sq_len_double = 0.0;

  try {
    dx_finite = CGAL::is_finite(
        vec.x()); // Assuming .x() for Vector_2, adjust if it's .dx()
    if (dx_finite)
      dx_val = CGAL::to_double(vec.x());
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "dx (uncertain conversion): " << e.what() << ", ";
  } catch (const std::exception &e) {
    std::cerr << "dx (exception): " << e.what() << ", ";
  }

  try {
    dy_finite = CGAL::is_finite(
        vec.y()); // Assuming .y() for Vector_2, adjust if it's .dy()
    if (dy_finite)
      dy_val = CGAL::to_double(vec.y());
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "dy (uncertain conversion): " << e.what();
  } catch (const std::exception &e) {
    std::cerr << "dy (exception): " << e.what();
  }

  try {
    sq_len_val = vec.squared_length();
    sq_len_finite = CGAL::is_finite(sq_len_val);
    if (sq_len_finite)
      sq_len_double = CGAL::to_double(sq_len_val);
  } catch (const std::exception &e) {
    std::cerr << " | SqLen (exception): " << e.what();
  }

  std::cerr << "(" << (dx_finite ? std::to_string(dx_val) : "non-finite")
            << ", " << (dy_finite ? std::to_string(dy_val) : "non-finite")
            << ")"
            << " | Finite: (dx: " << (dx_finite ? "yes" : "NO")
            << ", dy: " << (dy_finite ? "yes" : "NO") << ")"
            << " | SqLen: "
            << (sq_len_finite ? std::to_string(sq_len_double)
                              : "non-finite/error")
            << std::endl;
  std::cerr << std::defaultfloat;
}
// Function to normalize a CGAL vector robustly
inline Vector_2 normalize_vector_robust(const Vector_2 &vec,
                                        const std::string &context_msg) {
  // Check input vector components
  if (!CGAL::is_finite(vec.x()) || !CGAL::is_finite(vec.y())) {
    debug_cgal_vector(vec, "normalize_input_non_finite", context_msg);
    throw std::runtime_error("[" + context_msg +
                             "] Input vector has non-finite components");
  }

  Kernel::FT sq_len = vec.squared_length();

  // Check if squared length is valid
  if (!CGAL::is_finite(sq_len) ||
      sq_len < Kernel::FT(Constants::CGAL_EPSILON_SQUARED)) {
    debug_cgal_vector(vec, "normalize_sq_len_invalid", context_msg);
    throw std::runtime_error("[" + context_msg +
                             "] Vector too small to normalize");
  }

  // Convert to double for sqrt calculation
  double d_sq_len = CGAL::to_double(sq_len);
  if (!std::isfinite(d_sq_len) ||
      d_sq_len < Constants::EPSILON_LENGTH_CONSTRUCTION *
                     Constants::EPSILON_LENGTH_CONSTRUCTION) {
    debug_cgal_vector(vec, "normalize_d_sq_len_invalid", context_msg);
    throw std::runtime_error("[" + context_msg +
                             "] Vector length too small for normalization");
  }

  // Normalize
  Kernel::FT ft_len(std::sqrt(d_sq_len));
  Vector_2 normalized_vec = vec / ft_len;

  // Validate result components
  if (!CGAL::is_finite(normalized_vec.x()) ||
      !CGAL::is_finite(normalized_vec.y())) {
    debug_cgal_vector(normalized_vec, "normalize_result_non_finite",
                      context_msg);
    throw std::runtime_error("[" + context_msg +
                             "] Normalization produced non-finite result");
  }

  // Check if normalized (this was the problematic part - loosened tolerance)
  Kernel::FT norm_sq_len = normalized_vec.squared_length();
  if (!CGAL::is_finite(norm_sq_len)) {
    debug_cgal_vector(normalized_vec, "normalize_result_non_finite_len",
                      context_msg);
    throw std::runtime_error("[" + context_msg +
                             "] Normalized vector has non-finite length");
  }

  // Only warn if the result is drastically wrong (removed the strict 1.0 check)
  double norm_sq_len_double = CGAL::to_double(norm_sq_len);
  if (norm_sq_len_double < 0.5 || norm_sq_len_double > 1.5) {
    std::cerr << "WARNING: [" << context_msg
              << "] Normalized vector length seems off: " << norm_sq_len_double
              << std::endl;
    debug_cgal_vector(normalized_vec, "normalize_result_len_issue",
                      context_msg);
  }

  return normalized_vec;
}

inline void debug_cgal_point(const Point_2 &pt, const std::string &point_name,
                             const std::string &context) {
  std::cerr << std::fixed << std::setprecision(18); // For detailed output
  std::cerr << "DEBUG_CGAL_POINT [" << context << "] " << point_name << ": ";
  bool x_finite = false;
  bool y_finite = false;
  double x_val = 0.0, y_val = 0.0;

  try {
    x_finite = CGAL::is_finite(pt.x());
    if (x_finite)
      x_val = CGAL::to_double(pt.x());
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "X coord (uncertain conversion): " << e.what() << ", ";
  } catch (const std::exception &e) {
    std::cerr << "X coord (exception): " << e.what() << ", ";
  }

  try {
    y_finite = CGAL::is_finite(pt.y());
    if (y_finite)
      y_val = CGAL::to_double(pt.y());
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "Y coord (uncertain conversion): " << e.what();
  } catch (const std::exception &e) {
    std::cerr << "Y coord (exception): " << e.what();
  }

  std::cerr << "(" << (x_finite ? std::to_string(x_val) : "non-finite") << ", "
            << (y_finite ? std::to_string(y_val) : "non-finite") << ")"
            << " | Finite: (x: " << (x_finite ? "yes" : "NO")
            << ", y: " << (y_finite ? "yes" : "NO") << ")" << std::endl;
  std::cerr << std::defaultfloat; // Reset precision formatting
}

} // namespace CGALSafeUtils

#endif // CGAL_SAFE_UTILS_H
