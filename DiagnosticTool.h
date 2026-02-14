#ifndef DIAGNOSTIC_TOOL_H
#define DIAGNOSTIC_TOOL_H

#include "GeometryEditor.h"
#include <iostream>
#include <string>

namespace DiagnosticTool {
inline void logPointState(const Point &point) {
  std::cout << "POINT INFO: " << &point << " at ("
            << CGAL::to_double(point.getCGALPosition().x()) << ", "
            << CGAL::to_double(point.getCGALPosition().y()) << ")"
            << " isSelected=" << (point.isSelected() ? "true" : "false")
            << " isValid=" << (point.isValid() ? "true" : "false")
            << " isHovered=" << (point.isHovered() ? "true" : "false")
            << std::endl;
}

inline void logLineState(const Line &line) {
  std::cout << "LINE INFO: " << &line << " start=("
            << CGAL::to_double(line.getStartPoint().x()) << ", "
            << CGAL::to_double(line.getStartPoint().y()) << ")"
            << " end=(" << CGAL::to_double(line.getEndPoint().x()) << ", "
            << CGAL::to_double(line.getEndPoint().y()) << ")"
            << " isSegment=" << (line.isSegment() ? "true" : "false")
            << " isSelected=" << (line.isSelected() ? "true" : "false")
            << " isValid=" << (line.isValid() ? "true" : "false") << std::endl;
}

inline void dumpEditorState(const GeometryEditor &editor) {
  std::cout << "\n=== EDITOR STATE DUMP ===" << std::endl;
  std::cout << "Current Tool: " << static_cast<int>(editor.getCurrentTool())
            << std::endl;
  std::cout << "Drag Mode: " << static_cast<int>(editor.dragMode) << std::endl;
  std::cout << "Is Dragging: " << (editor.isDragging ? "true" : "false")
            << std::endl;
  std::cout << "Is Panning: " << (editor.isPanning ? "true" : "false")
            << std::endl;
  std::cout << "Points: " << editor.points.size() << std::endl;
  std::cout << "Object Points: " << editor.ObjectPoints.size() << std::endl;
  std::cout << "Lines: " << editor.lines.size() << std::endl;
  std::cout << "Circles: " << editor.circles.size() << std::endl;
  std::cout << "=== END STATE DUMP ===\n" << std::endl;
}
} // namespace DiagnosticTool

#endif // DIAGNOSTIC_TOOL_H
