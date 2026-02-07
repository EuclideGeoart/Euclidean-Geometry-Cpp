# FluxGeo Refactoring Plan: HandleMousePress.cpp Modularization

## Document Version
- **Created**: 2026-02-05
- **Status**: Planning Phase
- **Priority**: Medium (Post Bug-Fix)

---

## Executive Summary

HandleMousePress.cpp has grown to **5,245 lines** with **61+ functions**, creating maintenance and debugging challenges. This document outlines a phased approach to modularize the codebase into logical, maintainable modules while preserving all existing functionality.

---

## Current State Analysis

### File Statistics
- **Total Lines**: 5,245
- **Total Functions**: 61+
- **Main Categories**:
  - Primitive Creation Tools: ~18 functions
  - Construction Tools: ~12 functions
  - Transformation Tools: ~8 functions
  - Helper Functions: ~23 functions

### Identified Issues
1. **Cognitive Overload**: Single file handles too many responsibilities
2. **Risk of Merge Conflicts**: Multiple developers editing same large file
3. **Difficult Testing**: Hard to unit test individual tool behaviors
4. **Code Navigation**: Finding specific tool logic requires extensive scrolling

---

## Proposed Module Structure

### Module 1: `PrimitiveCreation.h/cpp`
**Purpose**: Handle all "click-to-draw" basic geometry creation

**Functions to Move**:
- `handleRectangleCreation()` - Axis-aligned rectangles
- `handleRotatableRectangleCreation()` - Free-rotation rectangles
- `handlePolygonCreation()` - N-sided polygons
- `handleRegularPolygonCreation()` - Regular N-gons
- `handleTriangleCreation()` - 3-point triangles
- `handleCircleCreation()` - Center-radius circles
- `handleSemicircleCreation()` - 2-point diameter semicircles
- `handleCircle3pCreation()` - 3-point circumcircle
- `handleLineCreation()` - Finite line segments
- `handleRayCreation()` - Infinite rays
- `handleSegmentCreation()` - Finite segments with endpoints
- `handlePolyLineCreation()` - Connected line segments
- `handleVectorCreation()` - Directed vectors
- `handleEllipseCreation()` - Ellipse drawing (if exists)

**Estimated Lines**: ~1,200

**Dependencies**:
- `GeometryEditor.h`
- `ConstraintUtils.h` (for smart point filtering)
- `PointUtils.h` (for snapping)
- Geometry classes: `Rectangle`, `Polygon`, `Circle`, `Line`, etc.

---

### Module 2: `ConstructionTools.h/cpp`
**Purpose**: Handle reference-based geometric constructions

**Functions to Move**:
- `handleParallelLineCreation()` - Lines parallel to reference
- `handlePerpendicularLineCreation()` - Lines perpendicular to reference
- `handleMidpointCreation()` - Midpoint of segment
- `handleAngleBisectorCreation()` - Angle bisector construction
- `handlePerpendicularBisectorCreation()` - Perpendicular bisector
- `handleTangentCreation()` - Tangent lines to circles
- `handleCompassCreation()` - Compass circle transfer
- `handleCompassToolClick()` - Compass interaction logic
- `handleObjectPointCreation()` - Points on objects
- `handleIntersectionPointCreation()` - Explicit intersection points

**Estimated Lines**: ~900

**Dependencies**:
- `GeometryEditor.h`
- `Line.h`, `Circle.h`
- `PointUtils.h`, `ProjectionUtils.h`
- `ConstructionObjects.h`

---

### Module 3: `TransformationTools.h/cpp`
**Purpose**: Handle geometric transformations (translation, rotation, reflection, dilation)

**Functions to Move**:
- `handleTranslationTool()` - Vector-based translation
- `handleReflectionLineTool()` - Reflection across line
- `handleReflectionPointTool()` - Point reflection (inversion)
- `handleReflectionCircleTool()` - Inversion through circle
- `handleRotationTool()` - Rotation around point
- `handleDilationTool()` - Scaling from center
- `createTransformedPoint()` - Core transformation logic
- `attachTransformMetadata()` - Dependency tracking
- `registerTransformPoint()` - Point registration
- `registerHiddenTransformObjectPoint()` - Hidden point tracking

**Estimated Lines**: ~800

**Dependencies**:
- `GeometryEditor.h`
- `Transforms.cpp` (existing transformation utilities)
- `PointUtils.h`

---

### Module 4: `InteractionHandlers.h/cpp`
**Purpose**: Centralize mouse event handling logic

**Functions to Move** (from HandleEvents.cpp):
- `handleMouseMove()` - Mouse movement, hover, drag, preview updates
- `handleMouseRelease()` - Click release, object finalization
- `handleDragLogic()` - Object dragging (extract from handleMouseMove)
- `handleHoverLogic()` - Hover highlighting (extract from handleMouseMove)
- `handlePreviewUpdates()` - Preview shape updates (extract from handleMouseMove)

**Estimated Lines**: ~600

**Key Additions**:
- **The Guard Clause**: Prevent dragging during creation
  ```cpp
  bool isCreating = editor.isCreatingRectangle || 
                    editor.isCreatingPolygon || 
                    /* ... all creation flags ... */;
  if (!isCreating && editor.dragMode == DragMode::MoveObject) {
      // Only allow dragging if NOT creating
      /* Original Drag Logic */
  }
  ```

**Dependencies**:
- `GeometryEditor.h`
- `PointUtils.h`
- `ConstraintUtils.h`

---

### Module 5: `ToolHelpers.h/cpp`
**Purpose**: Shared utility functions used across multiple tools

**Functions to Move**:
- `createSmartPointFromClick()` - Smart point creation with snapping
- `findClosestObject()` - Object hit detection
- `getDynamicSnapTolerance()` - Zoom-adjusted snap tolerance
- `getDynamicSelectionTolerance()` - Zoom-adjusted selection tolerance
- `screenPixelsToWorldUnits()` - Screen-to-world conversion
- `getPrimeLabel()` - Label generation (A, B, C → A', B', C')
- `applyRectangleVertexLabels()` - Auto-labeling for rectangles
- `assignUnifiedLabels()` - Unified labeling system
- `getPointForShapeVertex()` - Extract vertex points from shapes
- `getOrCreateHelperLineForEdge()` - Edge-to-line conversion
- `findTopNonDependentAt()` - Find selectable objects
- `clearTransformSelection()` - Selection cleanup
- `clearTemporarySelection()` - Temp selection cleanup
- `clearTempSelectedObjects()` - Global selection cleanup
- `ensureHoverVector()` - Vector hover logic

**Estimated Lines**: ~700

**Dependencies**:
- `GeometryEditor.h`
- `PointUtils.h`
- `ConstraintUtils.h`

---

## Refactoring Phases

### Phase 1: Preparation (1 day)
**Goal**: Set up infrastructure without breaking existing code

**Tasks**:
1. ✅ Create `ConstraintUtils.h/cpp` (COMPLETED - bug fix)
2. Create empty module files with headers
3. Add files to `CMakeLists.txt`
4. Verify build succeeds with empty modules

**Deliverables**:
- All 5 new module files created
- Build passes with no errors
- All existing tests pass

---

### Phase 2: Extract Transformation Tools (2-3 days)
**Goal**: Move transformation logic to dedicated module

**Why Start Here?**:
- Smallest module (~800 lines)
- Self-contained logic with minimal dependencies
- Good test case for refactoring process

**Tasks**:
1. Copy transformation functions to `TransformationTools.cpp`
2. Update includes and namespace references
3. Wrap original code in `#if 0 // MOVED TO TransformationTools.cpp ... #endif`
4. Update `handleMousePress()` dispatcher to call new module
5. Compile and test all transformation tools

**Safety**:
- Original code remains in file (commented out) for easy rollback
- Test each transformation type (translate, rotate, reflect, dilate)

**Deliverables**:
- `TransformationTools.cpp` functional
- All transformation tools working identically
- Commit with message: "Refactor: Extract transformation tools to dedicated module"

---

### Phase 3: Extract Construction Tools (3-4 days)
**Goal**: Move construction tool logic to dedicated module

**Tasks**:
1. Copy construction functions to `ConstructionTools.cpp`
2. Handle dependencies (ProjectionUtils, ConstructionObjects)
3. Wrap original code in `#if 0 // MOVED TO ConstructionTools.cpp ... #endif`
4. Update dispatcher
5. Comprehensive testing of parallel, perpendicular, bisector, tangent, compass

**Special Attention**:
- Parallel/Perpendicular line state management
- Compass multi-step interaction flow
- Projection utilities integration

**Deliverables**:
- `ConstructionTools.cpp` functional
- All construction tools tested
- Commit: "Refactor: Extract construction tools to dedicated module"

---

### Phase 4: Extract Primitive Creation Tools (4-5 days)
**Goal**: Move primitive creation to dedicated module

**Tasks**:
1. Copy primitive creation functions to `PrimitiveCreation.cpp`
2. **Inject Bug Fixes**:
   - Add `deselectAllAndClearInteractionState(editor)` at start of rectangle creation
   - Add `ConstraintUtils::isPointConstrainedByLine()` checks for all shape corners
3. Wrap original code in `#if 0 // MOVED TO PrimitiveCreation.cpp ... #endif`
4. Update dispatcher
5. Test all primitive tools (rectangles, polygons, circles, triangles, lines)

**Critical**:
- Rectangle constraint checking (parallel/perpendicular line protection)
- Rotatable rectangle 3-step flow
- Polygon/Regular polygon multi-click flows
- Circle variants (center-radius, 3-point, semicircle)

**Deliverables**:
- `PrimitiveCreation.cpp` functional
- Rectangle bug permanently fixed
- All primitive tools tested
- Commit: "Refactor: Extract primitive creation tools with bug fixes"

---

### Phase 5: Extract Tool Helpers (2 days)
**Goal**: Centralize shared utilities

**Tasks**:
1. Copy helper functions to `ToolHelpers.cpp`
2. Make helpers accessible from other modules
3. Wrap original code
4. Update all module includes to use `ToolHelpers.h`
5. Verify no circular dependencies

**Deliverables**:
- `ToolHelpers.cpp` functional
- All modules using centralized helpers
- Commit: "Refactor: Extract tool helper utilities"

---

### Phase 6: Extract Interaction Handlers (3-4 days)
**Goal**: Modularize event handling from HandleEvents.cpp

**Tasks**:
1. Extract `handleMouseMove()` → `InteractionHandlers::handleMouseMove()`
2. Extract `handleMouseRelease()` → `InteractionHandlers::handleMouseRelease()`
3. **Inject The Guard Clause**:
   ```cpp
   // In handleMouseMove, before drag logic:
   if (editor.isCreatingRectangle || editor.isCreatingPolygon || 
       editor.isCreatingTriangle || /* ... all creation flags ... */) {
       editor.dragMode = DragMode::None;  // Cancel any lingering drag
       return;  // Do not drag objects while creating
   }
   ```
4. Wrap original code in HandleEvents.cpp
5. Extensive testing of mouse interactions

**Critical**:
- Guard clause prevents "zombie dragging" bug
- Preview updates still work correctly
- Hover highlighting intact
- Selection mechanisms unchanged

**Deliverables**:
- `InteractionHandlers.cpp` functional
- Zombie dragging bug permanently fixed
- All mouse interactions tested
- Commit: "Refactor: Extract interaction handlers with drag guard"

---

### Phase 7: Cleanup (1 day)
**Goal**: Remove old commented code, finalize documentation

**Tasks**:
1. Remove all `#if 0 ... #endif` blocks from HandleMousePress.cpp and HandleEvents.cpp
2. Update all documentation and code comments
3. Run full regression test suite
4. Create final commit

**Deliverables**:
- HandleMousePress.cpp reduced from 5,245 to ~500 lines (dispatcher only)
- HandleEvents.cpp streamlined
- Final commit: "Refactor: Remove legacy code after successful modularization"

---

## Dispatcher Pattern (Final HandleMousePress.cpp)

After refactoring, HandleMousePress.cpp will become a clean dispatcher:

```cpp
#include "PrimitiveCreation.h"
#include "ConstructionTools.h"
#include "TransformationTools.h"
#include "ToolHelpers.h"

void handleMousePress(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  
  // Route to appropriate module based on current tool
  switch (editor.m_currentToolType) {
    // Primitives
    case ObjectType::Rectangle:
      PrimitiveCreation::handleRectangle(editor, mouseEvent);
      break;
    case ObjectType::RotatableRectangle:
      PrimitiveCreation::handleRotatableRectangle(editor, mouseEvent);
      break;
    case ObjectType::Polygon:
      PrimitiveCreation::handlePolygon(editor, mouseEvent);
      break;
    // ... (all primitive tools)
      
    // Construction
    case ObjectType::ParallelLine:
      ConstructionTools::handleParallelLine(editor, mouseEvent);
      break;
    case ObjectType::PerpendicularLine:
      ConstructionTools::handlePerpendicularLine(editor, mouseEvent);
      break;
    // ... (all construction tools)
      
    // Transformations
    case ObjectType::Translation:
      TransformationTools::handleTranslation(editor, mouseEvent);
      break;
    case ObjectType::Rotation:
      TransformationTools::handleRotation(editor, mouseEvent);
      break;
    // ... (all transformation tools)
      
    default:
      break;
  }
}
```

**Expected Final Line Count**: ~500 lines (down from 5,245)

---

## Bug Fixes Integrated

### 1. Rectangle Constraint Protection
**Location**: `PrimitiveCreation.cpp` - `handleRectangle()`, `handleRotatableRectangle()`

**Implementation**:
```cpp
// Phase 1: First Corner
if (!editor.isCreatingRectangle) {
  deselectAllAndClearInteractionState(editor);  // ← The Magic Shield
  
  editor.rectangleCorner1 = cgalWorldPos;
  
  // ← Constraint Check
  if (smartPoint && ConstraintUtils::isPointConstrainedByLine(editor, smartPoint)) {
    editor.rectangleCorner1Point = nullptr;  // Force new point creation
  } else {
    editor.rectangleCorner1Point = smartPoint;  // Safe to reuse
  }
  // ...
}

// Phase 2: Second Corner (same logic)
```

### 2. The Guard Clause (Zombie Drag Prevention)
**Location**: `InteractionHandlers.cpp` - `handleMouseMove()`

**Implementation**:
```cpp
void handleMouseMove(GeometryEditor& editor, const sf::Event& event) {
  // ... existing code ...
  
  if (editor.isDragging) {
    // ← THE GUARD CLAUSE
    bool isCreating = editor.isCreatingRectangle || 
                      editor.isCreatingRotatableRectangle ||
                      editor.isCreatingPolygon || 
                      editor.isCreatingTriangle ||
                      editor.m_isPlacingParallel ||
                      editor.m_isPlacingPerpendicular ||
                      /* ... all creation flags ... */;
    
    if (isCreating) {
      editor.dragMode = DragMode::None;  // Kill any lingering drag state
      editor.isDragging = false;
      return;  // STOP! Do not drag objects while creating shapes
    }
    
    // ← Original drag logic (only executes if NOT creating)
    if (editor.dragMode == DragMode::MoveFreePoint) {
      // ... existing drag logic ...
    }
  }
}
```

---

## Testing Strategy

### Unit Tests (New)
For each module, create unit tests:

**Example**: `test_PrimitiveCreation.cpp`
```cpp
TEST(PrimitiveCreation, RectangleDoesNotStealConstrainedPoints) {
  GeometryEditor editor;
  
  // Setup: Create parallel line with endpoint at (5, 5)
  auto parallelPoint = std::make_shared<Point>(Point_2(5, 5), 1.0f);
  auto parallelLine = std::make_shared<Line>(parallelPoint, /* ... */);
  parallelLine->setParallelLine(true);
  editor.lines.push_back(parallelLine);
  
  // Action: Create rectangle with corner near (5, 5)
  sf::Event::MouseButtonEvent click1 = createMouseEvent(5, 5);
  PrimitiveCreation::handleRectangle(editor, click1);
  
  sf::Event::MouseButtonEvent click2 = createMouseEvent(10, 10);
  PrimitiveCreation::handleRectangle(editor, click2);
  
  // Assert: Rectangle has its own points, not the parallel line's point
  ASSERT_NE(editor.rectangles[0]->getCorner1Point(), parallelPoint);
}
```

### Regression Tests (Existing)
- Run all existing test suites after each phase
- Manual testing of all tools in GUI
- Performance benchmarks (ensure no slowdown)

### Integration Tests
- Test cross-module interactions (e.g., transform a constructed object)
- Test complex workflows (create parallel line → rotate it → dilate result)

---

## Risk Mitigation

### High-Risk Areas
1. **State Management**: Ensure creation flags are properly handled across modules
2. **Pointer Lifetimes**: Verify shared_ptr references remain valid
3. **Undo/Redo**: Ensure CommandSystem integration works post-refactor

### Rollback Plan
- Each phase wrapped in `#if 0` blocks for easy revert
- Git commits after each successful phase
- Keep build green at all times

### Performance Monitoring
- Profile before refactoring (baseline)
- Profile after each phase
- Accept max +5% overhead (modular function calls)

---

## Success Criteria

### Code Quality
- ✅ Each module < 1,500 lines
- ✅ Clear separation of concerns
- ✅ No circular dependencies
- ✅ Consistent naming conventions

### Functionality
- ✅ All existing tools work identically
- ✅ No regression in behavior
- ✅ Bugs fixed (rectangle constraints, zombie dragging)

### Maintainability
- ✅ New developers can find tool logic quickly
- ✅ Adding new tools requires editing only 1 module
- ✅ Unit tests cover all tool behaviors

---

## Timeline Summary

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| Phase 1: Preparation | 1 day | Empty modules, build passes |
| Phase 2: Transformations | 2-3 days | TransformationTools.cpp complete |
| Phase 3: Construction | 3-4 days | ConstructionTools.cpp complete |
| Phase 4: Primitives | 4-5 days | PrimitiveCreation.cpp complete + bug fixes |
| Phase 5: Helpers | 2 days | ToolHelpers.cpp complete |
| Phase 6: Interactions | 3-4 days | InteractionHandlers.cpp complete + guard |
| Phase 7: Cleanup | 1 day | Legacy code removed, docs updated |
| **Total** | **16-21 days** | Fully modularized codebase |

---

## Post-Refactoring Benefits

### Developer Experience
- **Faster Navigation**: Find tool logic in ~300-line modules vs 5,000-line file
- **Parallel Development**: Multiple developers can work on different modules
- **Easier Testing**: Unit test individual tools in isolation

### Code Health
- **Reduced Coupling**: Clear module boundaries
- **Better Encapsulation**: Internal helpers hidden within modules
- **Improved Readability**: Logical grouping of related functionality

### Bug Prevention
- **Constraint Guards**: Built into primitive creation
- **State Isolation**: Creation flags managed within modules
- **Drag Protection**: Guard clause prevents cross-tool interference

---

## Next Steps

1. ✅ **Complete Bug Fix** (DONE - ConstraintUtils created and integrated)
2. **Review This Plan**: Get team approval for phased approach
3. **Schedule Phase 1**: Set aside 1 day for module infrastructure setup
4. **Begin Phase 2**: Start with Transformation Tools extraction

---

## Notes

- All module files will use namespace prefix to avoid conflicts (e.g., `PrimitiveCreation::`, `ConstructionTools::`)
- Original HandleMousePress.cpp and HandleEvents.cpp will be preserved in git history
- Documentation will be updated in parallel with code changes
- This is a **living document** - update as refactoring progresses

---

**Document End**
