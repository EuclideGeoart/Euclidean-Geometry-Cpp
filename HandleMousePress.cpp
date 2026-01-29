#include <memory>
#include <vector>

#include <SFML/Graphics.hpp>

#include "CommandSystem.h"
#include "ConstructionObjects.h"
#include "GeometricObject.h"
#include "GeometryEditor.h"
#include "Line.h"
#include "Point.h"

bool handleCompassCreation(GeometryEditor& editor,
                           std::vector<GeometricObject*>& tempSelectedObjects,
                           const sf::Vector2f& worldPos_sfml,
                           float tolerance) {
  GeometricObject* hitPoint = editor.lookForObjectAt(
      worldPos_sfml, tolerance, {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});

  std::shared_ptr<Point> centerPt;
  if (hitPoint) {
    centerPt = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hitPoint));
  }

  bool createdCenter = false;
  if (!centerPt) {
    centerPt = editor.createPoint(editor.toCGALPoint(worldPos_sfml));
    createdCenter = (centerPt != nullptr);
    if (createdCenter) {
      editor.commandManager.pushHistoryOnly(
          std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(centerPt)));
    }
  }

  if (!centerPt) {
    editor.setGUIMessage("Error: Could not create center point.");
    return false;
  }

  std::shared_ptr<CompassCircle> newCircle;

  if (tempSelectedObjects.size() == 1) {
    auto line = std::dynamic_pointer_cast<Line>(editor.findSharedPtr(tempSelectedObjects[0]));
    if (line) {
      newCircle = std::make_shared<CompassCircle>(centerPt, line, 0, editor.getCurrentColor());
    }
  } else if (tempSelectedObjects.size() == 2) {
    auto p1 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[0]));
    auto p2 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[1]));
    if (p1 && p2) {
      newCircle = std::make_shared<CompassCircle>(centerPt, p1, p2, 0, editor.getCurrentColor());
    }
  }

  if (newCircle) {
    editor.circles.push_back(newCircle);
    newCircle->setSelected(true);
    editor.commandManager.pushHistoryOnly(
        std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newCircle)));
    editor.setGUIMessage("Compass Circle constructed.");
  } else {
    editor.setGUIMessage("Error: Invalid compass configuration.");
  }

  for (auto* obj : tempSelectedObjects) {
    if (obj) obj->setSelected(false);
  }
  tempSelectedObjects.clear();

  return newCircle != nullptr;
}
