# Point Snapping & Line/Segment Creation

This document describes how the editor resolves clicks into points when creating lines and segments, and why existing points are reused instead of creating duplicates.

## Core Flow
Line and segment creation uses `handleLineCreation()` in [HandleMousePress.cpp](HandleMousePress.cpp). The click resolution is delegated to `createSmartPointFromClick()`.

### 1) Object Hit Test (Top Priority)
`createSmartPointFromClick()` uses `findClosestObject()` with the current snap tolerance to find the nearest selectable object. The decision order is:

- **Existing point types (reused)**
  - `Point`, `ObjectPoint`, `IntersectionPoint`  
  - Result: the existing point is returned and reused as the line endpoint.

- **Line / Segment**
  - Result: an `ObjectPoint` is created on the line using the projected relative position.

- **Circle**
  - Result: an `ObjectPoint` is created on the circle using the angular position around the center.

- **Shapes (Rectangle / Triangle / Polygon / RegularPolygon)**
  - Result: an `ObjectPoint` is created on the nearest edge at the projected relative position.

If any of the above returns a point, no further creation occurs.

### 2) Fallback: Smart Point Creation
If no object is hit, the click falls back to `PointUtils::createSmartPoint()` in [PointUtils.cpp](PointUtils.cpp):

- Handles intersections (existing or newly created)
- Optionally creates object-projected points depending on the tool and modifier keys
- Finally creates a free point if nothing else matches

## Why This Prevents Duplicates
Because `createSmartPointFromClick()` returns existing point instances *before* any new point creation, clicks on existing points (including line endpoints and intersection points) reuse the original point rather than create a duplicate.

## Related Functions
- `handleLineCreation()` in [HandleMousePress.cpp](HandleMousePress.cpp)
- `createSmartPointFromClick()` in [HandleMousePress.cpp](HandleMousePress.cpp)
- `findClosestObject()` in [HandleMousePress.cpp](HandleMousePress.cpp)
- `PointUtils::createSmartPoint()` in [PointUtils.cpp](PointUtils.cpp)
