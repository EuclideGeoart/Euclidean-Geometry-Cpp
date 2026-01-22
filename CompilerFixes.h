#pragma once

// Define CGAL_HEADER_ONLY first to prevent compiler_config.h issue
#define CGAL_HEADER_ONLY 1
// Add this to avoid the compiler_config.h error
#define CGAL_NO_COMPILER_CONFIG 1

// Prevent template parsing issues in Boost and CGAL
#ifdef __clang__
#define BOOST_DISABLE_ASSERTS
#define BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE
#define BOOST_NO_CXX98_FUNCTION_BASE

// CGAL specific fixes
#define CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES
#define CGAL_HAS_NO_THREADS
#define CGAL_DISABLE_ROUNDING_MATH_CHECK

// Tell Boost we're using clang-cl
#define BOOST_COMPILER_CONFIG "boost/config/compiler/clang.hpp"
#endif

// Prevent SFML Vector2 conflicts with CGAL
namespace sf {
    // Forward declare Vector2 types to ensure they're in sf:: namespace
    template<typename T> class Vector2;
    typedef Vector2<float> Vector2f;
    typedef Vector2<int> Vector2i;
}
