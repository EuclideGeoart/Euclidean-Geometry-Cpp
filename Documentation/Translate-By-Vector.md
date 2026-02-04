# Translate by Vector: Behavior Notes

This document summarizes the current Translate-by-Vector behavior and the two recent improvements.

## Step 2: Point-Based Vector With Hover-Only Display
When the user defines a translation vector by clicking two points (instead of selecting an existing vector):

- A **vector line is created between the two points**.
- The line is **transparent by default** and only becomes visible on hover/selection.
- The line is stored as a `Vector` type so it can be reused by later translations.

Location: `handleTransformationCreation()` in [HandleMousePress.cpp](HandleMousePress.cpp)

## Step 3: Chained Translate for Rectangles

Translated rectangles (both axis-aligned and rotatable) can now be translated again by vector:

- The translation logic handles `ObjectType::Rectangle` and `ObjectType::RectangleRotatable` together, so you can chain translations on the result rectangle.
- The relevant code is in the `applyVectorTranslation()` lambda in [HandleMousePress.cpp](../HandleMousePress.cpp):

```cpp
if (sourceShared->getType() == ObjectType::Rectangle ||
		sourceShared->getType() == ObjectType::RectangleRotatable) {
	auto rect = std::dynamic_pointer_cast<Rectangle>(sourceShared);
	auto V0 = getPointForShapeVertex(editor, rect, 0);
	auto V1 = getPointForShapeVertex(editor, rect, 1);
	auto V3 = getPointForShapeVertex(editor, rect, 3);
	auto V0t = createTransformedPoint(editor, V0, tool, nullptr, nullptr, nullptr, v1, v2);
	auto V1t = createTransformedPoint(editor, V1, tool, nullptr, nullptr, nullptr, v1, v2);
	auto V3t = createTransformedPoint(editor, V3, tool, nullptr, nullptr, nullptr, v1, v2);
	if (V0t && V1t && V3t) {
		registerPoint(V0t); registerPoint(V1t); registerPoint(V3t);
		double h = std::sqrt(CGAL::to_double(CGAL::squared_distance(V0t->getCGALPosition(), V3t->getCGALPosition())));
		auto newRect = std::make_shared<Rectangle>(V0t, V1t, h, editor.getCurrentColor(), editor.objectIdCounter++);
		newRect->setThickness(rect->getThickness());
		newRect->setDependent(true);
		attachTransformMetadata(sourceShared, newRect, tool, vectorAux, v1, v2);
		applyRectangleVertexLabels(editor, newRect);
		editor.rectangles.push_back(newRect);
		editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newRect)));
		editor.setGUIMessage("Translated rectangle created.");
		clearSelection();
		return true;
	}
}
```

**Key points:**
- The function creates new points for the rectangle's vertices by translating them with the selected vector.
- It then constructs a new rectangle from these points, preserving thickness and dependency.
- The new rectangle is added to the editor and can itself be translated again, enabling chaining.

**Helper function for rectangle edge points:**
```cpp
if (shape->getType() == ObjectType::Rectangle || shape->getType() == ObjectType::RectangleRotatable) {
	// ...
	auto p1 = std::static_pointer_cast<Point>(op1);
	auto p2 = std::static_pointer_cast<Point>(op2);
	// ...
}
```
This ensures correct vertex/edge handling for both rectangle types.
