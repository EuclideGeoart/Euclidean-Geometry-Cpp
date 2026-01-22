#pragma once

#include "CharTraitsFix.h" // Ensure this is very early
#include <string>          // Ensure standard string is included very early

#include "ForwardDeclarations.h"
#include "Types.h"

// Forward declaration for GeometryEditor which is used in function declarations
class GeometryEditor;
class Line;

// Enum for tracking the state of parallel/perpendicular line creation
enum class LineToolMode {
  SelectReference, // First step: selecting reference line/object
  PlaceNewLine // Second step: selecting point to create the new line through
};

// External declarations for global variables used by both HandleEvents.cpp and
// GUI.cpp
extern LineToolMode parallelLineMode;
extern LineToolMode perpLineMode;
extern Line *selectedRefLine;
extern Vector_2 savedDirection;
extern bool isReferenceHorizontalAxis;
extern bool isReferenceVerticalAxis;

// Function declarations
void resetParallelLineState();
void resetPerpLineState();
void handleEscapeKey(GeometryEditor &editor);
