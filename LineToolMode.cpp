#include "LineToolMode.h"
#include "Constants.h" // May be needed by reset functions or handleEscapeKey
#include "GeometryEditor.h" // Include the full definition of GeometryEditor
#include "Line.h"           // Required for selectedRefLine
#include <iostream>


// Global variable definitions
LineToolMode parallelLineMode = LineToolMode::SelectReference;
LineToolMode perpLineMode = LineToolMode::SelectReference;
Line *selectedRefLine = nullptr;
Vector_2 savedDirection(0, 0); // Initialize to a zero vector
bool isReferenceHorizontalAxis = false;
bool isReferenceVerticalAxis = false;

void resetParallelLineState() {
  parallelLineMode = LineToolMode::SelectReference;
  if (selectedRefLine) {
    // If you were highlighting the selectedRefLine, unhighlight it here
    // For example: selectedRefLine->setSelected(false);
  }
  selectedRefLine = nullptr;
  savedDirection = Vector_2(0, 0);
  isReferenceHorizontalAxis = false;
  isReferenceVerticalAxis = false;
  // Potentially reset preview elements if any
}

void resetPerpLineState() {
  perpLineMode = LineToolMode::SelectReference;
  if (selectedRefLine) {
    // If you were highlighting the selectedRefLine, unhighlight it here
    // For example: selectedRefLine->setSelected(false);
  }
  selectedRefLine = nullptr;
  savedDirection = Vector_2(0, 0);
  isReferenceHorizontalAxis = false;
  isReferenceVerticalAxis = false;
  // Potentially reset preview elements if any
}

// Handle Escape key to cancel parallel/perpendicular line operations
void handleEscapeKey(GeometryEditor &editor) {
  bool stateChanged = false;
  if (parallelLineMode == LineToolMode::PlaceNewLine) {
    resetParallelLineState();
    std::cout << "Parallel line creation cancelled by Escape." << std::endl;
    stateChanged = true;
  }
  if (perpLineMode == LineToolMode::PlaceNewLine) {
    resetPerpLineState();
    std::cout << "Perpendicular line creation cancelled by Escape."
              << std::endl;
    stateChanged = true;
  }

  if (stateChanged) {
    // If a tool was cancelled, revert to the default tool (e.g., Move/None)
    // and update the GUI accordingly.
    editor.setCurrentTool(ObjectType::None);
  }
}
