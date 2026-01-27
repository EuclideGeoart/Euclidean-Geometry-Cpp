#ifndef CHAR_TRAITS_FIX_H
#define CHAR_TRAITS_FIX_H

#include <cstring>
#include <string> // For std::char_traits

// Forward declaration for sf::Uint32 if not already visible
// Or include <SFML/Config.hpp> if it's light enough and provides Uint32
namespace sf {
typedef unsigned int Uint32;
}

// Provide a minimal specialization for std::char_traits<unsigned int>
// This is a workaround for SFML with libc++ and SFML_USE_CHAR32_STRING issues.
// Specialization removed as it conflicts with standard library implementation
// on this platform (GCC/MSYS2).
/*
namespace std {
template <> struct char_traits<unsigned int> {
  // ... removed ...
};
}
*/

#endif // CHAR_TRAITS_FIX_H