# Building FluxGeo

FluxGeo requires a C++20 compliant compiler.

## Dependencies

* **CMake** (3.15+)
* **SFML** (2.5+)
* **CGAL** (The core geometry kernel)
* **GMP & MPFR** (Required by CGAL for exact arithmetic)
* **ImGui** (Included in source)

---

## 🪟 Windows Build (MSYS2)

We recommend using **MSYS2 (UCRT64 or Clang64)** to handle dependencies easily.

### 1. Install Dependencies
Open your MSYS2 terminal and run:
```bash
pacman -S mingw-w64-ucrt-x86_64-toolchain \
          mingw-w64-ucrt-x86_64-cmake \
          mingw-w64-ucrt-x86_64-sfml \
          mingw-w64-ucrt-x86_64-cgal \
          mingw-w64-ucrt-x86_64-gmp \
          mingw-w64-ucrt-x86_64-mpfr
