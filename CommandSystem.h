#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "DynamicIntersection.h"
#include "ForwardDeclarations.h"
#include "GeometricObject.h"
#include "Point.h"
#include "ObjectPoint.h"
#include "Line.h"
#include "Circle.h"
#include "Rectangle.h"
#include "Polygon.h"
#include "RegularPolygon.h"
#include "Triangle.h"
#include "Angle.h"
#include "ObjectType.h"
#include "Types.h"

class GeometryEditor;

class Command {
 public:
  virtual ~Command() = default;
  virtual void execute() = 0;
  virtual void undo() = 0;
  virtual std::string getName() const = 0;
};

class CommandManager {
 public:
  void execute(const std::shared_ptr<Command> &cmd) {
    if (!cmd) return;
    cmd->execute();
    redoStack.clear();
    undoStack.push_back(cmd);
  }

  void pushHistoryOnly(const std::shared_ptr<Command> &cmd) {
    if (!cmd) return;
    redoStack.clear();
    undoStack.push_back(cmd);
  }

  void undo() {
    if (undoStack.empty()) return;
    auto cmd = undoStack.back();
    undoStack.pop_back();
    cmd->undo();
    redoStack.push_back(cmd);
  }

  void redo() {
    if (redoStack.empty()) return;
    auto cmd = redoStack.back();
    redoStack.pop_back();
    cmd->execute();
    undoStack.push_back(cmd);
  }

 private:
  std::vector<std::shared_ptr<Command>> undoStack;
  std::vector<std::shared_ptr<Command>> redoStack;
};

template <typename EditorT>
class CreateCommandT : public Command {
 public:
  CreateCommandT(EditorT &editor, std::shared_ptr<GeometricObject> object)
      : m_editor(editor), m_object(std::move(object)) {}

  void execute() override { addToEditor(m_editor, m_object); }
  void undo() override { removeFromEditor(m_editor, m_object); }
  std::string getName() const override { return "Create"; }

 private:
  EditorT &m_editor;
  std::shared_ptr<GeometricObject> m_object;

  template <typename T>
  static bool containsPtr(const std::vector<std::shared_ptr<T>> &vec,
                          const std::shared_ptr<T> &ptr) {
    return std::find(vec.begin(), vec.end(), ptr) != vec.end();
  }

  template <typename T>
  static void pushUnique(std::vector<std::shared_ptr<T>> &vec,
                         const std::shared_ptr<T> &ptr) {
    if (!ptr) return;
    if (!containsPtr(vec, ptr)) {
      vec.push_back(ptr);
    }
  }

  template <typename T>
  static void removePtr(std::vector<std::shared_ptr<T>> &vec,
                        const std::shared_ptr<T> &ptr) {
    if (!ptr) return;
    vec.erase(std::remove(vec.begin(), vec.end(), ptr), vec.end());
  }

  static void addToEditor(EditorT &editor, const std::shared_ptr<GeometricObject> &obj) {
    if (!obj) return;
    switch (obj->getType()) {
      case ObjectType::Point:
      case ObjectType::IntersectionPoint: {
        auto casted = std::dynamic_pointer_cast<Point>(obj);
        pushUnique(editor.points, casted);
        break;
      }
      case ObjectType::ObjectPoint: {
        auto casted = std::dynamic_pointer_cast<ObjectPoint>(obj);
        pushUnique(editor.ObjectPoints, casted);
        break;
      }
      case ObjectType::Line:
      case ObjectType::LineSegment: {
        auto casted = std::dynamic_pointer_cast<Line>(obj);
        pushUnique(editor.lines, casted);
        break;
      }
      case ObjectType::Circle: {
        auto casted = std::dynamic_pointer_cast<Circle>(obj);
        pushUnique(editor.circles, casted);
        break;
      }
      case ObjectType::Rectangle:
      case ObjectType::RectangleRotatable: {
        auto casted = std::dynamic_pointer_cast<Rectangle>(obj);
        pushUnique(editor.rectangles, casted);
        break;
      }
      case ObjectType::Polygon: {
        auto casted = std::dynamic_pointer_cast<Polygon>(obj);
        pushUnique(editor.polygons, casted);
        break;
      }
      case ObjectType::RegularPolygon: {
        auto casted = std::dynamic_pointer_cast<RegularPolygon>(obj);
        pushUnique(editor.regularPolygons, casted);
        break;
      }
      case ObjectType::Triangle: {
        auto casted = std::dynamic_pointer_cast<Triangle>(obj);
        pushUnique(editor.triangles, casted);
        break;
      }
      case ObjectType::Angle: {
        auto casted = std::dynamic_pointer_cast<Angle>(obj);
        pushUnique(editor.angles, casted);
        break;
      }
      default:
        break;
    }
  }

  static void removeFromEditor(EditorT &editor, const std::shared_ptr<GeometricObject> &obj) {
    if (!obj) return;
    editor.sanitizeReferences(obj.get());
    switch (obj->getType()) {
      case ObjectType::Point:
      case ObjectType::IntersectionPoint: {
        auto casted = std::dynamic_pointer_cast<Point>(obj);
        removePtr(editor.points, casted);
        break;
      }
      case ObjectType::ObjectPoint: {
        auto casted = std::dynamic_pointer_cast<ObjectPoint>(obj);
        removePtr(editor.ObjectPoints, casted);
        break;
      }
      case ObjectType::Line:
      case ObjectType::LineSegment: {
        auto casted = std::dynamic_pointer_cast<Line>(obj);
        if (casted) {
          try {
            DynamicIntersection::removeConstraintsInvolving(casted.get(), editor);
          } catch (...) {
          }
          try {
            casted->prepareForDestruction();
          } catch (...) {
          }
        }
        removePtr(editor.lines, casted);
        break;
      }
      case ObjectType::Circle: {
        auto casted = std::dynamic_pointer_cast<Circle>(obj);
        if (casted) {
          try {
            DynamicIntersection::removeConstraintsInvolving(casted.get(), editor);
          } catch (...) {
          }
        }
        removePtr(editor.circles, casted);
        break;
      }
      case ObjectType::Rectangle:
      case ObjectType::RectangleRotatable: {
        auto casted = std::dynamic_pointer_cast<Rectangle>(obj);
        if (casted) {
          try {
            DynamicIntersection::removeConstraintsInvolving(casted.get(), editor);
          } catch (...) {
          }
        }
        removePtr(editor.rectangles, casted);
        break;
      }
      case ObjectType::Polygon: {
        auto casted = std::dynamic_pointer_cast<Polygon>(obj);
        if (casted) {
          try {
            DynamicIntersection::removeConstraintsInvolving(casted.get(), editor);
          } catch (...) {
          }
        }
        removePtr(editor.polygons, casted);
        break;
      }
      case ObjectType::RegularPolygon: {
        auto casted = std::dynamic_pointer_cast<RegularPolygon>(obj);
        if (casted) {
          try {
            DynamicIntersection::removeConstraintsInvolving(casted.get(), editor);
          } catch (...) {
          }
        }
        removePtr(editor.regularPolygons, casted);
        break;
      }
      case ObjectType::Triangle: {
        auto casted = std::dynamic_pointer_cast<Triangle>(obj);
        if (casted) {
          try {
            DynamicIntersection::removeConstraintsInvolving(casted.get(), editor);
          } catch (...) {
          }
        }
        removePtr(editor.triangles, casted);
        break;
      }
      case ObjectType::Angle: {
        auto casted = std::dynamic_pointer_cast<Angle>(obj);
        removePtr(editor.angles, casted);
        break;
      }
      default:
        break;
    }
  }
};

template <typename EditorT>
class DeleteCommandT : public Command {
 public:
  DeleteCommandT(EditorT &editor, std::vector<std::shared_ptr<GeometricObject>> objects)
      : m_editor(editor), m_objects(std::move(objects)) {
    categorizeObjects();
  }

  void execute() override {
    if (m_objects.empty()) return;

    for (auto &obj : m_objects) {
      if (obj && !obj->isLocked()) {
        m_editor.sanitizeReferences(obj.get());
      }
    }

    for (auto &objPtr : m_objPoints) {
      removeObject(objPtr);
    }
    for (auto &linePtr : m_lines) {
      removeObject(linePtr);
    }
    for (auto &circlePtr : m_circles) {
      removeObject(circlePtr);
    }
    for (auto &rectPtr : m_rectangles) {
      removeObject(rectPtr);
    }
    for (auto &polyPtr : m_polygons) {
      removeObject(polyPtr);
    }
    for (auto &regPtr : m_regularPolygons) {
      removeObject(regPtr);
    }
    for (auto &triPtr : m_triangles) {
      removeObject(triPtr);
    }
    for (auto &anglePtr : m_angles) {
      removeObject(anglePtr);
    }
    for (auto &pointPtr : m_points) {
      removeObject(pointPtr);
    }
  }

  void undo() override {
    addObjects(m_points, m_editor.points);
    addObjects(m_lines, m_editor.lines, true);
    addObjects(m_circles, m_editor.circles);
    addObjects(m_rectangles, m_editor.rectangles);
    addObjects(m_polygons, m_editor.polygons);
    addObjects(m_regularPolygons, m_editor.regularPolygons);
    addObjects(m_triangles, m_editor.triangles);
    addObjects(m_angles, m_editor.angles);
    addObjects(m_objPoints, m_editor.ObjectPoints);
  }

  std::string getName() const override { return "Delete"; }

 private:
  EditorT &m_editor;
  std::vector<std::shared_ptr<GeometricObject>> m_objects;

  std::vector<std::shared_ptr<Point>> m_points;
  std::vector<std::shared_ptr<Line>> m_lines;
  std::vector<std::shared_ptr<Circle>> m_circles;
  std::vector<std::shared_ptr<ObjectPoint>> m_objPoints;
  std::vector<std::shared_ptr<Rectangle>> m_rectangles;
  std::vector<std::shared_ptr<Polygon>> m_polygons;
  std::vector<std::shared_ptr<RegularPolygon>> m_regularPolygons;
  std::vector<std::shared_ptr<Triangle>> m_triangles;
  std::vector<std::shared_ptr<Angle>> m_angles;

  template <typename T>
  static bool containsPtr(const std::vector<std::shared_ptr<T>> &vec,
                          const std::shared_ptr<T> &ptr) {
    return std::find(vec.begin(), vec.end(), ptr) != vec.end();
  }

  template <typename T>
  static void addUnique(std::vector<std::shared_ptr<T>> &vec,
                        const std::shared_ptr<T> &ptr) {
    if (!ptr) return;
    if (!containsPtr(vec, ptr)) {
      vec.push_back(ptr);
    }
  }

  template <typename T>
  static void removePtr(std::vector<std::shared_ptr<T>> &vec,
                        const std::shared_ptr<T> &ptr) {
    if (!ptr) return;
    vec.erase(std::remove(vec.begin(), vec.end(), ptr), vec.end());
  }

  void categorizeObjects() {
    for (auto &obj : m_objects) {
      if (!obj) continue;
      if (obj->isLocked()) continue;
      switch (obj->getType()) {
        case ObjectType::Point:
        case ObjectType::IntersectionPoint:
          addUnique(m_points, std::dynamic_pointer_cast<Point>(obj));
          break;
        case ObjectType::ObjectPoint:
          addUnique(m_objPoints, std::dynamic_pointer_cast<ObjectPoint>(obj));
          break;
        case ObjectType::Line:
        case ObjectType::LineSegment:
          addUnique(m_lines, std::dynamic_pointer_cast<Line>(obj));
          break;
        case ObjectType::Circle:
          addUnique(m_circles, std::dynamic_pointer_cast<Circle>(obj));
          break;
        case ObjectType::Rectangle:
        case ObjectType::RectangleRotatable:
          addUnique(m_rectangles, std::dynamic_pointer_cast<Rectangle>(obj));
          break;
        case ObjectType::Polygon:
          addUnique(m_polygons, std::dynamic_pointer_cast<Polygon>(obj));
          break;
        case ObjectType::RegularPolygon:
          addUnique(m_regularPolygons, std::dynamic_pointer_cast<RegularPolygon>(obj));
          break;
        case ObjectType::Triangle:
          addUnique(m_triangles, std::dynamic_pointer_cast<Triangle>(obj));
          break;
        case ObjectType::Angle:
          addUnique(m_angles, std::dynamic_pointer_cast<Angle>(obj));
          break;
        default:
          break;
      }
    }
  }

  template <typename T>
  void addObjects(const std::vector<std::shared_ptr<T>> &src,
                  std::vector<std::shared_ptr<T>> &dst,
                  bool registerLineEndpoints = false) {
    for (auto &ptr : src) {
      if (!ptr) continue;
      if (!containsPtr(dst, ptr)) {
        dst.push_back(ptr);
        if (registerLineEndpoints) {
          if (auto linePtr = std::dynamic_pointer_cast<Line>(ptr)) {
            try {
              linePtr->registerWithEndpoints();
            } catch (...) {
            }
          }
        }
      }
    }
  }

  template <typename T>
  void removeObject(const std::shared_ptr<T> &ptr) {
    if (!ptr) return;
    if (auto obj = std::dynamic_pointer_cast<GeometricObject>(ptr)) {
      try {
        DynamicIntersection::removeConstraintsInvolving(obj.get(), m_editor);
      } catch (...) {
      }
    }
    if (auto linePtr = std::dynamic_pointer_cast<Line>(ptr)) {
      try {
        linePtr->prepareForDestruction();
      } catch (...) {
      }
      removePtr(m_editor.lines, linePtr);
      return;
    }
    if (auto circlePtr = std::dynamic_pointer_cast<Circle>(ptr)) {
      removePtr(m_editor.circles, circlePtr);
      return;
    }
    if (auto rectPtr = std::dynamic_pointer_cast<Rectangle>(ptr)) {
      removePtr(m_editor.rectangles, rectPtr);
      return;
    }
    if (auto polyPtr = std::dynamic_pointer_cast<Polygon>(ptr)) {
      removePtr(m_editor.polygons, polyPtr);
      return;
    }
    if (auto regPtr = std::dynamic_pointer_cast<RegularPolygon>(ptr)) {
      removePtr(m_editor.regularPolygons, regPtr);
      return;
    }
    if (auto triPtr = std::dynamic_pointer_cast<Triangle>(ptr)) {
      removePtr(m_editor.triangles, triPtr);
      return;
    }
    if (auto objPointPtr = std::dynamic_pointer_cast<ObjectPoint>(ptr)) {
      removePtr(m_editor.ObjectPoints, objPointPtr);
      return;
    }
    if (auto pointPtr = std::dynamic_pointer_cast<Point>(ptr)) {
      removePtr(m_editor.points, pointPtr);
      return;
    }
    if (auto anglePtr = std::dynamic_pointer_cast<Angle>(ptr)) {
      removePtr(m_editor.angles, anglePtr);
      return;
    }
  }
};

class TranslateCommand : public Command {
 public:
  TranslateCommand(std::vector<GeometricObject *> objects, const Vector_2 &delta)
      : m_objects(std::move(objects)), m_delta(delta) {}

  void execute() override {
    for (auto *obj : m_objects) {
      if (obj && obj->isValid()) {
        obj->translate(m_delta);
      }
    }
  }

  void undo() override {
    Vector_2 negDelta(-m_delta.x(), -m_delta.y());
    for (auto *obj : m_objects) {
      if (obj && obj->isValid()) {
        obj->translate(negDelta);
      }
    }
  }

  std::string getName() const override { return "Translate"; }

 private:
  std::vector<GeometricObject *> m_objects;
  Vector_2 m_delta;
};

using CreateCommand = CreateCommandT<GeometryEditor>;
using DeleteCommand = DeleteCommandT<GeometryEditor>;
