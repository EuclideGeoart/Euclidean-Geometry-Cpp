#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include "GeometryEditor.h"
#include "Types.h"
#include <CGAL/number_utils.h>
#include <iostream>
#include <sstream>
#include <string>


namespace DebugUtils {

// Helper function to print CGAL point information
inline void logPoint(const Point_2 &point, const std::string &label) {
  try {
    double x = CGAL::to_double(point.x());
    double y = CGAL::to_double(point.y());
    std::cout << label << ": (" << x << ", " << y << ")" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error logging " << label << ": " << e.what() << std::endl;
  }
}

// Helper function to print SFML vector information
inline void logVector(const sf::Vector2f &vec, const std::string &label) {
  std::cout << label << ": (" << vec.x << ", " << vec.y << ")" << std::endl;
}

// Helper function for logging object information
template <typename T>
inline void logObject(const T *obj, const std::string &label) {
  if (!obj) {
    std::cout << label << " is null" << std::endl;
    return;
  }

  std::cout << label << " [Type: " << static_cast<int>(obj->getType())
            << ", Valid: " << (obj->isValid() ? "Yes" : "No")
            << ", Selected: " << (obj->isSelected() ? "Yes" : "No")
            << ", Hovered: " << (obj->isHovered() ? "Yes" : "No") << "]"
            << std::endl;
}

// Log editor state
inline void logEditorState(const GeometryEditor &editor) {
  std::cout << "=== Editor State ===\n"
            << "Current Tool: " << static_cast<int>(editor.m_currentToolType)
            << "\n"
            << "Selected Object: " << (editor.selectedObject ? "Yes" : "None")
            << "\n"
            << "Drag Mode: " << static_cast<int>(editor.dragMode) << "\n"
            << "Is Dragging: " << (editor.isDragging ? "Yes" : "No") << "\n"
            << "Is Drawing Selection Box: "
            << (editor.isDrawingSelectionBox ? "Yes" : "No") << "\n"
            << "Is Creating Circle: "
            << (editor.isCreatingCircle ? "Yes" : "No") << "\n"
            << "Is Panning: " << (editor.isPanning ? "Yes" : "No") << "\n"
            << "====================" << std::endl;
}

} // namespace DebugUtils

#endif // DEBUG_UTILS_H
