# Geometry Tool: Snapping & ObjectPoint Fixes

## 1. Strict Dynamic Snapping Tolerance (Ghost Snap Fix)

**Summary:**
- Enforces visual-logic consistency for snapping.
- Uses `getDynamicSnapTolerance(editor)` for zoom-aware tolerance.
- Snap targets are cleared if the mouse distance exceeds tolerance.
- Created objects are only dependent if the preview visually snaps to a point.

**Key Implementation:**
- Mouse move handler and Rectangle tool now double-check snap distance before accepting a snap.
- Rectangle preview and creation logic use validated snap position.
- Defensive programming: multiple validation layers prevent stale or invalid snaps.

**Code Locations:**
- `HandleEvents.cpp`: Snap detection logic, universal snapping system, rectangle preview.
- `HandleMousePress.cpp`: Rectangle tool creation logic.

---

## 2. ObjectPoint & Virtual Vertex Connectivity Fixes

**Summary:**
- Rectangle and Rotatable Rectangle tools now accept ObjectPoints (including perpendicular/parallel origins) as valid corners.
- Virtual vertex snapping: clicking near a shape vertex (RegularPolygon, Polygon, Rectangle) creates a real Point for snapping.
- ObjectPoint tool can now create points on shape edges (rectangle, polygon, regular polygon, triangle).

**Key Implementation:**
- Rectangle tools skip constrained-point rejection for ObjectPoints.
- `createSmartPointFromClick` includes a pass to create points on shape vertices.
- ObjectPoint tool extended to support shape edges.

**Code Locations:**
- `HandleMousePress.cpp`: Rectangle/RotatableRectangle tool logic, smart point creation, ObjectPoint tool.

---

## Testing & Validation
- Confirmed fixes by running the build and testing snapping and ObjectPoint creation across all relevant tools.
- If Circle3P snapping fails, add a dedicated pass for its vertices.

---

## Authors
- For further improvements or bug reports, contact the project maintainer.
