# Loader Fixes – February 2026

## Summary
This update fixes project loading for polygons, triangles, circles, and tangent lines. All shapes are now restored with correct IDs, outlines, and dependencies from JSON files.

## Key Fixes
- **Polygons, Triangles, Regular Polygons:**
  - If a shape is loaded with `id: 0`, a new unique ID is assigned and `maxId` is updated.
  - Ensures all shapes are registered and drawn correctly.

- **Circles:**
  - Circles with `id: 0` are assigned a new unique ID and added to `editor.circles`.
  - Ensures thickness and outline are restored.

- **Tangent Lines:**
  - Tangent lines reference valid circles with proper IDs.
  - If a tangent's circle has `id: 0`, it is assigned a new ID and added to `editor.circles`.

## Code Snippets

### Circle Loader Fix
```cpp
// In getOrCreate (ProjectSerializer.cpp):
unsigned int circleId = jObj.value("id", 0u);
if (circleId == 0u) {
    circleId = ++maxId;
} else {
    bumpMaxId(circleId);
}
// ...
obj->setID(circleId);
```

### Ensure Circles in Editor
```cpp
if (auto ci = std::dynamic_pointer_cast<Circle>(obj)) {
    ci->setThickness(jObj.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
    if (std::find(editor.circles.begin(), editor.circles.end(), ci) == editor.circles.end()) {
        editor.circles.push_back(ci);
    }
}
```

### Tangent Line Loader Fix
```cpp
if (cir && cir->getID() == 0u) {
    cir->setID(++maxId);
    bumpMaxId(cir->getID());
    if (std::find(editor.circles.begin(), editor.circles.end(), cir) == editor.circles.end()) {
        editor.circles.push_back(cir);
    }
}
```

### Polygon/Triangle ID Assignment
```cpp
unsigned int id = jShape.value("id", 0u);
if (id == 0) {
    id = ++maxId;
} else {
    bumpMaxId(id);
}
```

## Impact
- All shapes now load and render correctly from JSON, including outlines and dependencies.
- Fixes issues with missing edges/outlines for polygons and circles, and missing tangents.

---
*Committed: February 2026*
