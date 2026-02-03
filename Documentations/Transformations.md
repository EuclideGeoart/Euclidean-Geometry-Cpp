# Transformation Mechanisms and Chained Transformations

This document summarizes the mechanisms for all supported geometric transformations in the project, including translation, rotation, and scaling, as well as how chained transformations are handled. Code snippets from the codebase are provided for each transformation type to illustrate their implementation and chaining logic.

## Table of Contents
- [Translation by Vector](#translation-by-vector)
- [Rotation](#rotation)
- [Scaling](#scaling)
- [Chained Transformations](#chained-transformations)

---

## Translation by Vector

**Mechanism:**
- Translates a shape by a vector defined by two points.
- Supports chaining: a translated result can be translated again.

**Relevant code (HandleMousePress.cpp):**
```cpp
auto applyVectorTranslation = [&](const std::shared_ptr<Point>& v1, const std::shared_ptr<Point>& v2) -> bool {
  if (sourceShared->getType() == ObjectType::Rectangle ||
      sourceShared->getType() == ObjectType::RectangleRotatable) {
    // ...
    auto V0t = createTransformedPoint(editor, V0, tool, nullptr, nullptr, nullptr, v1, v2);
    // ...
    auto newRect = std::make_shared<Rectangle>(V0t, V1t, h, editor.getCurrentColor(), editor.objectIdCounter++);
    newRect->setDependent(true);
    attachTransformMetadata(sourceShared, newRect, tool, vectorAux, v1, v2);
    // ...
  }
  // ...
}
```
See [Translate-By-Vector.md](Translate-By-Vector.md) for more details.

---

## Rotation

**Mechanism:**
- Rotates a shape around a pivot point by a given angle.
- Supports chaining: a rotated result can be rotated again.

**Relevant code (HandleMousePress.cpp):**
```cpp
if (tool == ObjectType::RotateAroundPoint) {
  // ...
  auto newRect = std::make_shared<Rectangle>(rotatedV0, rotatedV1, h, editor.getCurrentColor(), editor.objectIdCounter++);
  newRect->setDependent(true);
  attachTransformMetadata(sourceShared, newRect, tool, pivot, nullptr, nullptr);
  editor.rectangles.push_back(newRect);
  // ...
}
```

---

## Scaling

**Mechanism:**
- Scales a shape relative to a center point by a scale factor.
- Supports chaining: a scaled result can be scaled again.

**Relevant code (HandleMousePress.cpp):**
```cpp
if (tool == ObjectType::DilateFromPoint) {
  // ...
  auto newRect = std::make_shared<Rectangle>(scaledV0, scaledV1, h, editor.getCurrentColor(), editor.objectIdCounter++);
  newRect->setDependent(true);
  attachTransformMetadata(sourceShared, newRect, tool, center, nullptr, nullptr);
  editor.rectangles.push_back(newRect);
  // ...
}
```

---

## Chained Transformations

**General Mechanism:**
- Each transformation creates a new dependent object, preserving the transformation metadata.
- The result object can be used as the source for further transformations (chaining).
- The code pattern for all transformations is similar:

```cpp
// Example for any transformation
auto newObj = std::make_shared<ShapeType>(... /* transformed vertices */ ...);
newObj->setDependent(true);
attachTransformMetadata(sourceShared, newObj, tool, aux, ...);
editor.<shapeCollection>.push_back(newObj);
```

**attachTransformMetadata** ensures:
- The transformation type and parameters are stored in the new object.
- The dependency chain is maintained for chaining.

**Code for attachTransformMetadata:**
```cpp
auto attachTransformMetadata = [&](const std::shared_ptr<GeometricObject>& source,
                                   const std::shared_ptr<GeometricObject>& created,
                                   ObjectType activeTool,
                                   const std::shared_ptr<GeometricObject>& aux,
                                   const std::shared_ptr<Point>& vStart,
                                   const std::shared_ptr<Point>& vEnd) {
  // ...
  created->setTransformType(t);
  created->restoreTransformation(source, aux, t);
  source->addDependent(created);
  if (t == TransformationType::Translate && vStart && vEnd) {
    created->setTranslationVector(vEnd->getCGALPosition() - vStart->getCGALPosition());
  } else if (t == TransformationType::Rotate) {
    created->setTransformValue(g_transformRotationDegrees);
  } else if (t == TransformationType::Dilate) {
    created->setTransformValue(g_transformDilationFactor);
  }
};
```

**Key Points:**
- The use of `setDependent(true)` and `attachTransformMetadata` ensures that the transformation chain is tracked.
- Chained transformations work by always using the latest result as the new source.

---

## Comparison Table
| Transformation | Chaining Supported | Key Function | Metadata Tracking |
|----------------|-------------------|--------------|------------------|
| Translation    | Yes               | applyVectorTranslation | attachTransformMetadata |
| Rotation       | Yes               | applyRotation         | attachTransformMetadata |
| Scaling        | Yes               | applyScaling          | attachTransformMetadata |

---

For more details and specific code for each transformation, see the relevant sections in `HandleMousePress.cpp` and the dedicated documentation files.
