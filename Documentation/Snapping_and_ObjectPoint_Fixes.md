# Geometry Tool: Bug Fixes & Improvements

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

## 3. Label Generation Overflow Fix (LaTeX Subscripts)

**Summary:**
- Fixed label corruption after 26 points (A-Z).
- Now generates LaTeX-style subscript labels: A, B, ..., Z, A_{1}, B_{1}, ..., Z_{1}, A_{2}, ...
- Labels render correctly in the LaTeX rendering system.

**Key Implementation:**
- Changed from Unicode subscripts to LaTeX format: `_{1}`, `_{2}`, etc.
- Modified `toLatexSubscript()` helper function in `PointUtils.cpp`.

**Code Locations:**
- `PointUtils.cpp`: `toLatexSubscript()` function, `getNextLabels()` method.
- `PointUtils.h`: Documentation updated to reflect LaTeX format.

---

## 4. ObjectPoint Hit-Testing Fix (Ghost Selection)

**Summary:**
- Fixed rectangular selection incorrectly selecting ObjectPoints via their parent line/circle geometry.
- ObjectPoints now only select when the point itself is inside the selection box.

**Key Implementation:**
- Removed duplicate ObjectPoint selection pass that used global bounds.
- ObjectPoints now use position-based selection (checks if point position is inside box).

**Code Locations:**
- `HandleEvents.cpp`: Selection box logic for ObjectPoints.

---

## 5. Resurrection Visibility Bug Fix

**Summary:**
- Fixed dependent objects (Lines, Circles, Perpendiculars, Parallels) not reappearing when their parent points become valid again.
- Objects were getting stuck in hidden state even after parents were restored.

**Root Cause:**
- Main update loop in `GeometryEditor::updateAllGeometry()` was checking `isValid()` before calling `update()`.
- This prevented hidden objects from running their update logic to check if they should resurrect.

**Key Implementation:**
1. **GeometryEditor.cpp**: Removed `isValid()` checks from all update loops:
   - Points, Lines, Circles, ObjectPoints now always call `update()`
   - Allows objects to run resurrection logic internally

2. **Line.cpp**: Already had correct resurrection logic (lines 1237-1257):
   - Checks parent validity
   - Sets `visible = true` if parents are valid (RESURRECTION)
   - Sets `visible = false` and returns if parents are invalid
   - Continues to geometry calculation after resurrection

3. **Circle.cpp**: Added resurrection logic similar to Line:
   - Checks center point validity for standard circles
   - Checks diameter points validity for semicircles
   - Sets visibility appropriately before geometry calculation
   - Always recalculates geometry after resurrecting

**Code Locations:**
- `GeometryEditor.cpp`: `updateAllGeometry()` method (lines 2194-2250).
- `Line.cpp`: `update()` method (lines 1229-1330).
- `Circle.cpp`: `update()` method (lines 318-390).

---

## Testing & Validation
- Confirmed fixes by running the build and testing snapping and ObjectPoint creation across all relevant tools.
- Verified label generation beyond Z works correctly with LaTeX subscripts.
- Tested rectangular selection to ensure ObjectPoints are only selected by their actual position.
- Verified dependent objects (perpendiculars, parallels, transformed circles) now resurrect when parent points are restored.

---

## Authors
- Fixes implemented by GitHub Copilot (Claude Sonnet 4.5)
- For further improvements or bug reports, contact the project maintainer.
