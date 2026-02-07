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
    using char_type = unsigned int;
    using int_type = unsigned int;
    using off_type = std::streamoff;
    using pos_type = std::streampos;
    using state_type = std::mbstate_t;

    static void assign(char_type& c1, const char_type& c2) noexcept { c1 = c2; }
    static bool eq(const char_type& c1, const char_type& c2) noexcept { return c1 == c2; }
    static bool lt(const char_type& c1, const char_type& c2) noexcept { return c1 < c2; }

    static int compare(const char_type* s1, const char_type* s2, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            if (lt(s1[i], s2[i])) return -1;
            if (lt(s2[i], s1[i])) return 1;
        }
        return 0;
    }

    static size_t length(const char_type* s) {
        size_t len = 0;
        while (!eq(s[len], char_type(0))) ++len;
        return len;
    }

    static const char_type* find(const char_type* s, size_t n, const char_type& a) {
        for (size_t i = 0; i < n; ++i) {
            if (eq(s[i], a)) return s + i;
        }
        return nullptr;
    }

    static char_type* move(char_type* s1, const char_type* s2, size_t n) {
        return static_cast<char_type*>(std::memmove(s1, s2, n * sizeof(char_type)));
    }

    static char_type* copy(char_type* s1, const char_type* s2, size_t n) {
        return static_cast<char_type*>(std::memcpy(s1, s2, n * sizeof(char_type)));
    }

    static char_type* assign(char_type* s, size_t n, char_type a) {
        for (size_t i = 0; i < n; ++i) s[i] = a;
        return s;
    }

    static constexpr int_type not_eof(int_type c) noexcept { return eq_int_type(c, eof()) ? 0 : c; }
    static constexpr char_type to_char_type(int_type c) noexcept { return char_type(c); }
    static constexpr int_type to_int_type(char_type c) noexcept { return int_type(c); }
    static constexpr bool eq_int_type(int_type c1, int_type c2) noexcept { return c1 == c2; }
    static constexpr int_type eof() noexcept { return static_cast<int_type>(-1); }
};
}
*/

#endif // CHAR_TRAITS_FIX_H