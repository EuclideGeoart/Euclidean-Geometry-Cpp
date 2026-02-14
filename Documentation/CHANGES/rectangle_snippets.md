# Rectangle.cpp — Snippets added/adjusted by assistant

This file contains the code snippets that were added or adjusted in `Rectangle.cpp` so you can paste or review them if "Apply" did not show them.

## 1) Insertion at start of `updateSFMLShape()`
(keeps non-explicit rectangles synchronized before rendering)

```cpp
// --- Added: ensure non-explicit rectangles sync corners before drawing ---
if (!m_useExplicitVertices) {
  if (m_isRotatable) {
    syncRotatableFromAnchors();
  } else {
    updateDimensionsFromCorners();
  }
  syncDependentCorners();
}
```

Placed immediately at the top of `Rectangle::updateSFMLShape()` before the SFML shape setup.

---

If you want, I can also (a) create a small patch file with `git apply`-style diff, (b) open the file at the exact lines for you, or (c) add any other snippets you recall were missing. Which would you prefer next?