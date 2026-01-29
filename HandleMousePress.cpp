#include <memory>
#include <vector>

#include <SFML/Graphics.hpp>

#include "CommandSystem.h"
#include "ConstructionObjects.h"
#include "GeometricObject.h"
#include "GeometryEditor.h"
#include "Line.h"
#include "Point.h"
#include "PointUtils.h"
#include "TransformationObjects.h"
#include "Circle.h"
#include "Polygon.h"

extern float g_transformRotationDegrees;
extern float g_transformDilationFactor;

static std::string getPrimeLabel(const std::string& oldLabel) {
  return oldLabel.empty() ? std::string() : (oldLabel + "'");
}

static std::shared_ptr<Point> createTransformedPoint(
    GeometryEditor& editor,
    const std::shared_ptr<Point>& source,
    ObjectType tool,
    const std::shared_ptr<Point>& kernelPoint,
    const std::shared_ptr<Line>& kernelLine,
    const std::shared_ptr<Circle>& kernelCircle,
    const std::shared_ptr<Point>& vecStart,
    const std::shared_ptr<Point>& vecEnd) {
  if (!source) return nullptr;

  std::shared_ptr<Point> newPoint;
  switch (tool) {
    case ObjectType::ReflectAboutLine:
      if (kernelLine) newPoint = std::make_shared<ReflectLine>(source, kernelLine, editor.getCurrentColor());
      break;
    case ObjectType::ReflectAboutPoint:
      if (kernelPoint) newPoint = std::make_shared<ReflectPoint>(source, kernelPoint, editor.getCurrentColor());
      break;
    case ObjectType::ReflectAboutCircle:
      if (kernelCircle) newPoint = std::make_shared<ReflectCircle>(source, kernelCircle, editor.getCurrentColor());
      break;
    case ObjectType::RotateAroundPoint:
      if (kernelPoint) newPoint = std::make_shared<RotatePoint>(source, kernelPoint, g_transformRotationDegrees, editor.getCurrentColor());
      break;
    case ObjectType::TranslateByVector:
      if (vecStart && vecEnd) newPoint = std::make_shared<TranslateVector>(source, vecStart, vecEnd, editor.getCurrentColor());
      break;
    case ObjectType::DilateFromPoint:
      if (kernelPoint) newPoint = std::make_shared<DilatePoint>(source, kernelPoint, g_transformDilationFactor, editor.getCurrentColor());
      break;
    default:
      break;
  }

  if (!newPoint) return nullptr;

  std::string baseLabel = source->getLabel();
  std::string primeLabel = getPrimeLabel(baseLabel);
  if (!primeLabel.empty()) {
    newPoint->setLabel(primeLabel);
    newPoint->setShowLabel(true);
  } else {
    std::string autoLabel = LabelManager::getNextLabel(editor.points);
    newPoint->setLabel(autoLabel);
    newPoint->setShowLabel(true);
  }
  return newPoint;
}

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

bool handleTransformationCreation(GeometryEditor& editor,
                                  std::vector<GeometricObject*>& tempSelectedObjects,
                                  const sf::Vector2f& worldPos_sfml,
                                  float tolerance) {
  auto clearSelection = [&]() {
    for (auto* obj : tempSelectedObjects) {
      if (obj) obj->setSelected(false);
    }
    tempSelectedObjects.clear();
  };

  auto registerPoint = [&](const std::shared_ptr<Point>& newPoint) {
    if (!newPoint) return;
    editor.points.push_back(newPoint);
    editor.commandManager.pushHistoryOnly(
        std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoint)));
    newPoint->setSelected(true);
  };

  ObjectType tool = editor.m_currentToolType;

  // Step 1: Source object
  if (tempSelectedObjects.empty()) {
    GeometricObject* hitObj = editor.lookForObjectAt(
        worldPos_sfml, tolerance,
        {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint, ObjectType::Line,
         ObjectType::LineSegment, ObjectType::Circle, ObjectType::Polygon});
    if (!hitObj) return false;

    tempSelectedObjects.push_back(hitObj);
    hitObj->setSelected(true);

    if (tool == ObjectType::ReflectAboutLine) {
      editor.setGUIMessage("Reflect: Select line to reflect about.");
    } else if (tool == ObjectType::ReflectAboutPoint) {
      editor.setGUIMessage("Reflect: Select center point.");
    } else if (tool == ObjectType::ReflectAboutCircle) {
      editor.setGUIMessage("Invert: Select circle for inversion.");
    } else if (tool == ObjectType::RotateAroundPoint) {
      editor.setGUIMessage("Rotate: Select pivot point.");
    } else if (tool == ObjectType::TranslateByVector) {
      editor.setGUIMessage("Translate: Select vector start point.");
    } else if (tool == ObjectType::DilateFromPoint) {
      editor.setGUIMessage("Dilate: Select center point.");
    }
    return true;
  }

  auto sourceShared = editor.findSharedPtr(tempSelectedObjects[0]);
  if (!sourceShared) {
    clearSelection();
    editor.setGUIMessage("Error: Invalid source object.");
    return false;
  }

  // Translate requires 2-step vector selection
  if (tool == ObjectType::TranslateByVector) {
    if (tempSelectedObjects.size() == 1) {
      GeometricObject* vecStartRaw = editor.lookForObjectAt(
          worldPos_sfml, tolerance, {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});
      if (!vecStartRaw) return false;
      tempSelectedObjects.push_back(vecStartRaw);
      vecStartRaw->setSelected(true);
      editor.setGUIMessage("Translate: Select vector end point.");
      return true;
    }

    GeometricObject* vecEndRaw = editor.lookForObjectAt(
        worldPos_sfml, tolerance, {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});
    if (!vecEndRaw) return false;
    tempSelectedObjects.push_back(vecEndRaw);

    auto v1 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[1]));
    auto v2 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[2]));
    if (!v1 || !v2) {
      clearSelection();
      editor.setGUIMessage("Error: Invalid translation vector.");
      return false;
    }

    auto sourcePoint = std::dynamic_pointer_cast<Point>(sourceShared);
    if (sourcePoint) {
      auto newPoint = createTransformedPoint(editor, sourcePoint, tool, nullptr, nullptr, nullptr, v1, v2);
      if (!newPoint) {
        editor.setGUIMessage("Error: Invalid translation.");
        clearSelection();
        return false;
      }
      registerPoint(newPoint);
      editor.setGUIMessage("Translated point created.");
      clearSelection();
      return true;
    }

    // Translate non-point sources by decomposing to points
    if (sourceShared->getType() == ObjectType::Line || sourceShared->getType() == ObjectType::LineSegment) {
      auto line = std::dynamic_pointer_cast<Line>(sourceShared);
      if (!line) {
        editor.setGUIMessage("Error: Invalid line.");
        clearSelection();
        return false;
      }

      auto p1 = line->getStartPointObjectShared();
      auto p2 = line->getEndPointObjectShared();
      auto p1t = createTransformedPoint(editor, p1, tool, nullptr, nullptr, nullptr, v1, v2);
      auto p2t = createTransformedPoint(editor, p2, tool, nullptr, nullptr, nullptr, v1, v2);
      if (!p1t || !p2t) {
        editor.setGUIMessage("Error: Invalid translation.");
        clearSelection();
        return false;
      }

      registerPoint(p1t);
      registerPoint(p2t);

      auto newLine = std::make_shared<Line>(p1t, p2t, line->isSegment(), line->getColor());
      editor.lines.push_back(newLine);
      editor.commandManager.pushHistoryOnly(
          std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));
      editor.setGUIMessage("Translated line created.");
      clearSelection();
      return true;
    }

    if (sourceShared->getType() == ObjectType::Circle) {
      auto circle = std::dynamic_pointer_cast<Circle>(sourceShared);
      if (!circle) {
        editor.setGUIMessage("Error: Invalid circle.");
        clearSelection();
        return false;
      }
      auto centerRaw = circle->getCenterPointObject();
      auto centerShared = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(centerRaw));
      auto newCenter = createTransformedPoint(editor, centerShared, tool, nullptr, nullptr, nullptr, v1, v2);
      if (!newCenter) {
        editor.setGUIMessage("Error: Invalid translation.");
        clearSelection();
        return false;
      }
      registerPoint(newCenter);
      auto newCircle = std::make_shared<Circle>(newCenter.get(), nullptr, circle->getRadius(), circle->getColor());
      editor.circles.push_back(newCircle);
      editor.commandManager.pushHistoryOnly(
          std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newCircle)));
      editor.setGUIMessage("Translated circle created.");
      clearSelection();
      return true;
    }

    if (sourceShared->getType() == ObjectType::Polygon) {
      auto poly = std::dynamic_pointer_cast<Polygon>(sourceShared);
      if (!poly) {
        editor.setGUIMessage("Error: Invalid polygon.");
        clearSelection();
        return false;
      }
      std::vector<std::shared_ptr<Point>> newVerts;
      newVerts.reserve(poly->getVertexCount());
      for (size_t i = 0; i < poly->getVertexCount(); ++i) {
        auto v = poly->getVertexPoint(i);
        auto vt = createTransformedPoint(editor, v, tool, nullptr, nullptr, nullptr, v1, v2);
        if (!vt) {
          editor.setGUIMessage("Error: Invalid translation.");
          clearSelection();
          return false;
        }
        registerPoint(vt);
        newVerts.push_back(vt);
      }
      auto newPoly = std::make_shared<Polygon>(newVerts, poly->getColor());
      editor.polygons.push_back(newPoly);
      editor.commandManager.pushHistoryOnly(
          std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoly)));
      editor.setGUIMessage("Translated polygon created.");
      clearSelection();
      return true;
    }
  }

  // Step 2: Operator object
  std::shared_ptr<Point> pivotPoint;
  std::shared_ptr<Line> lineObj;
  std::shared_ptr<Circle> circleObj;

  if (tool == ObjectType::ReflectAboutLine) {
    GeometricObject* hitLine = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Line, ObjectType::LineSegment});
    if (!hitLine) return false;
    lineObj = std::dynamic_pointer_cast<Line>(editor.findSharedPtr(hitLine));
  } else if (tool == ObjectType::ReflectAboutCircle) {
    GeometricObject* hitCircle = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Circle});
    if (!hitCircle) return false;
    circleObj = std::dynamic_pointer_cast<Circle>(editor.findSharedPtr(hitCircle));
  } else {
    GeometricObject* hitPoint = editor.lookForObjectAt(
        worldPos_sfml, tolerance, {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});
    if (!hitPoint) return false;
    pivotPoint = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hitPoint));
  }

  auto sourcePoint = std::dynamic_pointer_cast<Point>(sourceShared);
  if (sourcePoint) {
    auto newPoint = createTransformedPoint(editor, sourcePoint, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
    if (!newPoint) {
      editor.setGUIMessage("Error: Invalid transformation selection.");
      clearSelection();
      return false;
    }
    registerPoint(newPoint);
    if (tool == ObjectType::ReflectAboutCircle) {
      editor.setGUIMessage("Inverted point created.");
    } else if (tool == ObjectType::RotateAroundPoint) {
      editor.setGUIMessage("Rotated point created.");
    } else if (tool == ObjectType::DilateFromPoint) {
      editor.setGUIMessage("Dilated point created.");
    } else {
      editor.setGUIMessage("Reflected point created.");
    }
    clearSelection();
    return true;
  }

  if (sourceShared->getType() == ObjectType::Line || sourceShared->getType() == ObjectType::LineSegment) {
    auto line = std::dynamic_pointer_cast<Line>(sourceShared);
    if (!line) {
      editor.setGUIMessage("Error: Invalid line.");
      clearSelection();
      return false;
    }
    auto p1 = line->getStartPointObjectShared();
    auto p2 = line->getEndPointObjectShared();
    auto p1t = createTransformedPoint(editor, p1, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
    auto p2t = createTransformedPoint(editor, p2, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
    if (!p1t || !p2t) {
      editor.setGUIMessage("Error: Invalid transformation selection.");
      clearSelection();
      return false;
    }
    registerPoint(p1t);
    registerPoint(p2t);
    auto newLine = std::make_shared<Line>(p1t, p2t, line->isSegment(), line->getColor());
    editor.lines.push_back(newLine);
    editor.commandManager.pushHistoryOnly(
        std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));
    editor.setGUIMessage("Transformed line created.");
    clearSelection();
    return true;
  }

  if (sourceShared->getType() == ObjectType::Circle) {
    auto circle = std::dynamic_pointer_cast<Circle>(sourceShared);
    if (!circle) {
      editor.setGUIMessage("Error: Invalid circle.");
      clearSelection();
      return false;
    }
    if (tool == ObjectType::ReflectAboutCircle) {
      editor.setGUIMessage("Circle inversion not supported yet.");
      clearSelection();
      return false;
    }

    auto centerRaw = circle->getCenterPointObject();
    auto centerShared = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(centerRaw));
    auto newCenter = createTransformedPoint(editor, centerShared, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
    if (!newCenter) {
      editor.setGUIMessage("Error: Invalid transformation selection.");
      clearSelection();
      return false;
    }
    registerPoint(newCenter);
    double newRadius = circle->getRadius();
    if (tool == ObjectType::DilateFromPoint) {
      newRadius = circle->getRadius() * g_transformDilationFactor;
    }
    auto newCircle = std::make_shared<Circle>(newCenter.get(), nullptr, newRadius, circle->getColor());
    editor.circles.push_back(newCircle);
    editor.commandManager.pushHistoryOnly(
        std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newCircle)));
    editor.setGUIMessage("Transformed circle created.");
    clearSelection();
    return true;
  }

  if (sourceShared->getType() == ObjectType::Polygon) {
    auto poly = std::dynamic_pointer_cast<Polygon>(sourceShared);
    if (!poly) {
      editor.setGUIMessage("Error: Invalid polygon.");
      clearSelection();
      return false;
    }
    std::vector<std::shared_ptr<Point>> newVerts;
    newVerts.reserve(poly->getVertexCount());
    for (size_t i = 0; i < poly->getVertexCount(); ++i) {
      auto v = poly->getVertexPoint(i);
      auto vt = createTransformedPoint(editor, v, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      if (!vt) {
        editor.setGUIMessage("Error: Invalid transformation selection.");
        clearSelection();
        return false;
      }
      registerPoint(vt);
      newVerts.push_back(vt);
    }
    auto newPoly = std::make_shared<Polygon>(newVerts, poly->getColor());
    editor.polygons.push_back(newPoly);
    editor.commandManager.pushHistoryOnly(
        std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoly)));
    editor.setGUIMessage("Transformed polygon created.");
    clearSelection();
    return true;
  }

  editor.setGUIMessage("Error: Unsupported source object.");
  clearSelection();
  return false;
}
