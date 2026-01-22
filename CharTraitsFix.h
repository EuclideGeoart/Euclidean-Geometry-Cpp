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
namespace std {
template <> struct char_traits<unsigned int> {
  using char_type = unsigned int;
  using int_type = unsigned long; // Or another suitable int type for EOF
  using off_type = streamoff;
  using pos_type = streampos;
  using state_type = mbstate_t;

  static constexpr void assign(char_type &r, const char_type &a) noexcept {
    r = a;
  }
  static constexpr bool eq(char_type a, char_type b) noexcept { return a == b; }
  static constexpr bool lt(char_type a, char_type b) noexcept { return a < b; }

  static constexpr int compare(const char_type *s1, const char_type *s2,
                               size_t n) {
    for (size_t i = 0; i < n; ++i) {
      if (lt(s1[i], s2[i]))
        return -1;
      if (lt(s2[i], s1[i]))
        return 1;
    }
    return 0;
  }

  static constexpr size_t length(const char_type *s) {
    size_t i = 0;
    while (!eq(s[i], char_type(0)))
      ++i;
    return i;
  }

  static constexpr const char_type *find(const char_type *s, size_t n,
                                         const char_type &a) {
    for (size_t i = 0; i < n; ++i) {
      if (eq(s[i], a))
        return s + i;
    }
    return nullptr;
  }

  static char_type *move(char_type *s1, const char_type *s2, size_t n) {
    if (n == 0)
      return s1;
    // memmove is suitable here
    return static_cast<char_type *>(memmove(s1, s2, n * sizeof(char_type)));
  }

  static char_type *copy(char_type *s1, const char_type *s2, size_t n) {
    if (n == 0)
      return s1;
    // memcpy is suitable here
    return static_cast<char_type *>(memcpy(s1, s2, n * sizeof(char_type)));
  }

  static char_type *assign(char_type *s, size_t n, char_type a) {
    for (size_t i = 0; i < n; ++i)
      assign(s[i], a);
    return s;
  }

  static constexpr int_type not_eof(int_type c) noexcept {
    return eq_int_type(c, eof()) ? ~eof() : c;
  }

  static constexpr char_type to_char_type(int_type c) noexcept {
    return static_cast<char_type>(c);
  }

  static constexpr int_type to_int_type(char_type c) noexcept {
    return static_cast<int_type>(c);
  }

  static constexpr bool eq_int_type(int_type c1, int_type c2) noexcept {
    return c1 == c2;
  }

  static constexpr int_type eof() noexcept {
    return static_cast<int_type>(-1); // Or ULONG_MAX or similar
  }
};
} // namespace std

#endif // CHAR_TRAITS_FIX_H