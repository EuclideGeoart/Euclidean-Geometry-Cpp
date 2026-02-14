#include "Commands.h"

#include <unordered_map>
#include <utility>

#include "GeometryEditor.h"

MoveCommand::MoveCommand(GeometryEditor& editor, std::map<unsigned int, Point_2> oldPositions, std::map<unsigned int, Point_2> newPositions)
    : m_editor(editor), m_oldPositions(std::move(oldPositions)), m_newPositions(std::move(newPositions)) {}

void MoveCommand::execute() { applyPositions(m_newPositions); }

void MoveCommand::undo() { applyPositions(m_oldPositions); }

void MoveCommand::applyPositions(const std::map<unsigned int, Point_2>& positions) {
  if (positions.empty()) {
    return;
  }

  std::unordered_map<unsigned int, std::shared_ptr<Point>> pointIndex;
  auto allPoints = m_editor.getAllPoints();
  pointIndex.reserve(allPoints.size());
  for (const auto& point : allPoints) {
    if (point) {
      pointIndex[point->getID()] = point;
    }
  }

  for (const auto& entry : positions) {
    auto it = pointIndex.find(entry.first);
    if (it == pointIndex.end() || !it->second) {
      continue;
    }
    it->second->setCGALPosition(entry.second);
    it->second->update();
  }

  m_editor.updateAllGeometry();
}
