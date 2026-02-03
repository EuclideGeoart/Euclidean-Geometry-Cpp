# Selection Priority & Chained Transformation Logic (V2.1 Fix)

## Overview
This document details the critical logic changes introduced to resolve "Fighting" between overlapping objects and to enable robust Chained Transformations (e.g., Rotating a Reflected shape).

## 1. Selection Priority (The "Source First" Rule)
**Problem:** When a Dependent Shape (Result) overlaps its Source Shape (Original), clicking to drag the Source would often select the Result instead. Since Results are often locked/dependent, the user could not edit the shape.

**The Fix:**
We implemented a **Two-Pass Priority Search** in `HandleMousePress::findClosestObject`:

1.  **Pass 1 (High Priority):** Search *only* for **Independent/Source** points within the tolerance range.
    * Condition: `!object->isDependent()`
    * If found, return immediately.
2.  **Pass 2 (Low Priority):** If no independent point is found, search for **Dependent** points.

**Result:** Users can always click "through" a reflection to grab the original vertex underneath.

## 2. Chained Transformations (Recursive Updates)
**Problem:** Transforming a shape that was already a result of another transformation (e.g., A -> B -> C) caused C to detach or fail to update when A moved.

**The Fix:**
We switched to a **Recursive Observer Pattern**:
1.  **Registration:** When `C` is created from `B`, `C` registers itself as an observer of `B`.
2.  **Update Loop (`GeometricObject::update`):**
    * **Self-Update:** Object checks if it has a `m_parentSource`. If yes, it calls `Transforms::recalculateGeometry(this)` to snap to the correct math coordinates.
    * **Visual Update:** Syncs SFML shape.
    * **Notify Children:** Iterates through `m_observers` and calls `child->update()`.

**Graph Flow:**
`User Moves A` -> `A.update()` -> `Notify B` -> `B.recalculate()` -> `B.update()` -> `Notify C` -> `C.recalculate()` -> `C.update()`

## 3. "Translate by Vector" Hardening
To prevent "Shadowing" (where the shape being moved blocks the vector selection):
* **Step 1 (Source Selection):** Locked to only accept objects (no point creation).
* **Step 2 (Vector Selection):** Explicitly **IGNORES** the Source Object ID during the search.
* **Vector Creation:** If points P1 and P2 are clicked manually, a visual `Vector` line is permanently created to represent the transformation.