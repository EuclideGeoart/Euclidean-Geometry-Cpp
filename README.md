# FluxGeo

**A Modern, C++ Euclidean Geometry Engine with Native LaTeX Support.**

FluxGeo is a dynamic geometry software (similar to GeoGebra) built for performance and precision. It allows users to construct complex geometric figures, perform exact calculations, and render publication-quality mathematical text using a native LaTeX engine.

![FluxGeo Demo](docs/images/image_e3c4df.png)
*Real-time LaTeX rendering with dynamic preview*

## Key Features

* **Dynamic Geometry Engine**: Construct Points, Lines, Circles, Polygons, and Intersections that update in real-time.
* **Native LaTeX Rendering**: Powered by MicroTeX, FluxGeo renders complex math formulas (`\int`, `\sum`, `\sqrt{}`) directly on the canvas without external webview dependencies.
* **Vector Export**: Export your constructions to **SVG** with raw LaTeX strings, ready for editing in Inkscape (TexText) or publication.
* **Smart Snapping**: Intelligent snapping system for precise construction.
* **Save/Load System**: Robust JSON-based serialization that preserves geometry dependencies and text objects.

## 📸 Screenshots

| Construction & Math | Interactive Text Editor |
|:-------------------:|:-----------------------:|
| ![Geometry](docs/images/image_d7773c.jpg) | ![Editor](docs/images/image_9cd77d.png) |
| *Construct complex relationships* | *Type math with live preview* |

## Quick Start

### Windows (Pre-built)
1.  Download the latest release from the [Releases Page](#).
2.  Unzip and run `FluxGeo.exe`.
3.  *Note: Ensure the `res/` folder and font files are in the same directory as the executable.*

### Building from Source
FluxGeo is built with **C++20** and uses **CMake**.
See [BUILD.md](BUILD.md) for detailed compilation instructions for Windows (MSYS2) and Linux.

## 🛠️ Tech Stack
* **Core**: C++20 Standard
* **Rendering**: SFML 2.6
* **Math Kernel**: CGAL (Computational Geometry Algorithms Library)
* **UI**: ImGui + ImGui-SFML
* **LaTeX Engine**: MicroTeX (Custom Integration)

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
