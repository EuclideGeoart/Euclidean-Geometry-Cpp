# Deserializer Refactoring: Strict 2-Pass ID-to-Pointer Mapping

## Architecture Overview

### Current Problems
1. **Point creation scattered across shape loaders** - Rectangles, Polygons, etc. call `getOrCreatePointFromCoords()`
2. **No enforcement of "points-first" policy** - Shapes can create points on-demand
3. **Inconsistent mapping** - Some code uses IDs, some uses coordinates

### New Architecture: 3-Pass System

```
┌─────────────────────────────────────────────────────────────┐
│ PASS 0.5: PRE-SCAN & CREATE ALL VERTEX POINTS             │
│ - Scan all shape JSON for vertices without IDs             │
│ - Create Point objects from coordinates                     │
│ - Register in masterMap                                     │
│ - GOAL: Ensure 100% of points exist before shapes load     │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ PASS 1: IDENTITY PASS - Load Points & ObjectPoints        │
│ - Load points from "points" array                          │
│ - Load intersection points                                  │
│ - Create ObjectPoint STRUCTURES (host links deferred)      │
│ - ALL objects get registered in masterMap[id] = ptr        │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ PASS 2: STRUCTURAL LINK PASS - Build Shapes               │
│ - Load Lines, Circles, Rectangles, Polygons, Triangles     │  
│ - STRICT RULE: Only use masterMap.at(id) lookups           │
│ - NO point creation allowed - fail if point missing        │
│ - Link ObjectPoints to their hosts via setHost()           │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ PASS 3: DEPENDENCY GRAPH PASS - Restore Transformations   │
│ - Find parent objects via parentSourceID lookup            │
│ - Restore transformation links (Reflect, Rotate, etc.)     │
│ - Re-establish observer pattern (parent->child updates)    │
│ - Restore IntersectionPoint dependencies                    │
└─────────────────────────────────────────────────────────────┘
```

## Implementation Changes

### 1. Add Pass 0.5 (Line ~930)

Insert BEFORE "PASS 1: LOAD ALL POINTS":

```cpp
// ========================================================================
// PASS 0.5: PRE-SCAN SHAPES TO CREATE MISSING VERTEX POINTS
// ========================================================================
std::cout << "[Deserializer] Pass 0.5: Pre-scanning vertices..." << std::endl;

// Lambda to pre-create points from shape coordinates
auto preCreateVertexPoints = [&](const json& shapeArray) {
  if (!shapeArray.is_array()) return;
  for (const auto& shape : shapeArray) {
    // Skip if all vertex IDs exist
    bool hasAllIds = false;
    if (shape.contains("vertexIds") && shape["vertexIds"].is_array()) {
      hasAllIds = true;
      for (const auto& idJson : shape["vertexIds"]) {
        if (idJson.get<int>() <= 0 || !getPointById(idJson.get<int>())) {
          hasAllIds = false;
          break;
        }
      }
    }
    if (hasAllIds) continue;
    
    // Create from coordinates
    if (shape.contains("vertices")) {
      for (const auto& vJson : shape["vertices"]) {
        double x = 0.0, y = 0.0;
        if (readVertex(vJson, x, y) && !findPointAt(x, y)) {
          int id = allocateId();
          auto pt = std::make_shared<Point>(
              Point_2(x, y), Constants::CURRENT_ZOOM,
              Constants::POINT_DEFAULT_COLOR, id);
          editor.points.push_back(pt);
          registerPoint(id, pt);
        }
      }
    }
  }
};

// Pre-create all shape vertices
if (data.contains("rectangles")) preCreateVertexPoints(data["rectangles"]);
if (data.contains("polygons")) preCreateVertexPoints(data["polygons"]);
if (data.contains("triangles")) preCreateVertexPoints(data["triangles"]);
if (data.contains("regularPolygons")) preCreateVertexPoints(data["regularPolygons"]);
```

### 2. Remove Point Creation from Shape Loaders

#### Rectangle Loader (Line ~1175):
**REMOVE:**
```cpp
auto corner1 = getOrCreatePointFromCoords(x1, y1, Constants::POINT_DEFAULT_COLOR);
auto corner2 = getOrCreatePointFromCoords(x2, y2, Constants::POINT_DEFAULT_COLOR);
```

**REPLACE WITH:**
```cpp
auto corner1 = findPointAt(x1, y1);
auto corner2 = findPointAt(x2, y2);
if (!corner1 || !corner2) {
  std::cerr << "[Rectangle] FATAL: Missing vertex points! " << std::endl;
  continue; // Skip this rectangle
}
```

#### Polygon Loader (Line ~1240):
**REMOVE:**
```cpp
vertexPoints.push_back(
    getOrCreatePointFromCoords(x, y, Constants::POINT_DEFAULT_COLOR));
```

**REPLACE WITH:**
```cpp
auto pt = findPointAt(x, y);
if (!pt) {
  std::cerr << "[Polygon] Missing vertex at (" << x << "," << y << ")" << std::endl;
  deferPoly = true;
  break;
}
vertexPoints.push_back(pt);
```

#### Line Loader (Line ~1048):
**REMOVE:**
```cpp
startPt = getOrCreatePointFromCoords(...);
endPt = getOrCreatePointFromCoords(...);
```

**REPLACE WITH:**
```cpp
startPt = findPointAt(jLine["startX"].get<double>(), jLine["startY"].get<double>());
endPt = findPointAt(jLine["endX"].get<double>(), jLine["endY"].get<double>());
if (!startPt || !endPt) {
  std::cerr << "[Line] Missing endpoint!" << std::endl;
  pendingLines.push_back(jLine);
  continue;
}
```

### 3. ObjectPoint Host Linking (PASS 2)

In the ObjectPoint loader (Line ~1870), after creating ObjectPoint:

```cpp
// CRITICAL: Set host link immediately after shape is loaded
auto hostObj = getObjectById(hostId);
if (hostObj) {
  if (auto hostLine = std::dynamic_pointer_cast<Line>(hostObj)) {
    newObjPoint->setHost(hostLine);
  } else if (auto hostCircle = std::dynamic_pointer_cast<Circle>(hostObj)) {
    newObjPoint->setHost(hostCircle);
  }
}
```

### 4. Mark User Override Flags on Load

In `applyCommonPointFields()` lambda (Line ~849):

```cpp
if (jPoint.contains("showLabel")) {
  pt->setShowLabel(jPoint.value("showLabel", true));
}
if (jPoint.contains("hasUserOverride")) {
  pt->setHasUserOverride(jPoint.value("hasUserOverride", false));
}
```

In `saveProject()` (Line ~660):

```cpp
ptJson["showLabel"] = pt->getShowLabel();
ptJson["hasUserOverride"] = pt->hasUserOverride(); // NEW
```

## Benefits

✅ **Zero Point Creation in Shape Loaders** - Enforced architecturally  
✅ **Single Source of Truth** - All points come from masterMap  
✅ **Deterministic Loading** - Points always exist before references  
✅ **Better Error Handling** - Missing points logged, not silently created  
✅ **Cleaner Code** - Separation of concerns (identity vs structure vs dependencies)

## Testing Checklist

- [ ] Save/Load projects with rectangles (ID-based and coordinate-based)
- [ ] Save/Load polygons with mixed vertex data
- [ ] Save/Load transformation objects (reflections, rotations)
- [ ] Save/Load ObjectPoints on lines and circles
- [ ] Save/Load IntersectionPoints with line dependencies
- [ ] Verify user override flags persist across save/load
- [ ] Test legacy JSON files (old format compatibility)
