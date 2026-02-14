#pragma once

#include <map>
#include <string>

#include "CommandSystem.h"
#include "Types.h"

class GeometryEditor;

class MoveCommand : public Command {
 public:
  MoveCommand(GeometryEditor& editor, std::map<unsigned int, Point_2> oldPositions, std::map<unsigned int, Point_2> newPositions);

  void execute() override;
  void undo() override;
  std::string getName() const override { return "Move"; }

 private:
  void applyPositions(const std::map<unsigned int, Point_2>& positions);

  GeometryEditor& m_editor;
  std::map<unsigned int, Point_2> m_oldPositions;
  std::map<unsigned int, Point_2> m_newPositions;
};
