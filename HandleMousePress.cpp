#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "Angle.h"
#include "CGALSafeUtils.h"
#include "Circle.h"
#include "CommandSystem.h"
#include "ConstraintUtils.h"
#include "Constants.h"
#include "ConstructionObjects.h"
#include "GUI.h"
#include "GeometricObject.h"
#include "GeometryEditor.h"
#include "Line.h"
#include "IntersectionPoint.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "PointUtils.h"
#include "Polygon.h"
#include "ProjectionUtils.h"
#include "Rectangle.h"
#include "RegularPolygon.h"
#include "TextLabel.h"
#include "TransformationObjects.h"
#include "Triangle.h"
#include "Types.h"
#include "VariantUtils.h"
#include "ForwardDeclarations.h"

// handleParallelLineCreation, handlePerpendicularLineCreation, handlePerpendicularBisectorCreation,
// handleAngleBisectorCreation, handleTangentCreation, handleObjectPointCreation,
// handleRectangleCreation, handleRotatableRectangleCreation, handlePolygonCreation,
// handleRegularPolygonCreation, handleTriangleCreation follow (migrated unchanged).
// (The full migrated implementations are appended below.)

extern float g_transformRotationDegrees;
extern float g_transformDilationFactor;
extern bool g_showAngleInputPopup;

// --- Helper: Midpoint & Compass State ---
static std::vector<GeometricObject*> tempSelectedObjects;
static std::optional<EdgeHitResult> g_transformEdgeSelection;
static size_t s_transformSourceCount = 0;

static void applyRectangleVertexLabels(GeometryEditor& editor, const std::shared_ptr<Rectangle>& rect, const std::vector<std::shared_ptr<Point>>& explicitPoints = {});
static void assignUnifiedLabels(GeometryEditor& editor, const std::vector<std::shared_ptr<Point>>& points);

// Forward declarations for helpers defined in HandleEvents.cpp
void deselectAllAndClearInteractionState(GeometryEditor& editor, bool preserveSelection = false);

static float screenPixelsToWorldUnits(const GeometryEditor& editor, float screenPixels) {
  if (editor.window.getSize().x > 0) {
    float scale = editor.drawingView.getSize().x / static_cast<float>(editor.window.getSize().x);
    return screenPixels * scale;
  }
  return 1e-12f;
}

static float getDynamicSelectionTolerance(const GeometryEditor& editor) {
  return screenPixelsToWorldUnits(editor, Constants::SELECTION_SCREEN_PIXELS);
}

static float getDynamicSnapTolerance(const GeometryEditor& editor) { return screenPixelsToWorldUnits(editor, Constants::SNAP_SCREEN_PIXELS); }

static GeometricObject* findClosestObject(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml, float tolerance);

static std::shared_ptr<Point> createSmartPointFromClick(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml, float tolerance) {
  // PASS 0: Priority Snapping (Restore regression)
  if (editor.m_snapTargetPoint && editor.m_snapTargetPoint->isValid()) {
      std::cout << "[SNAP] Using pre-calculated snap target: " << editor.m_snapTargetPoint->getID() << std::endl;
      return editor.m_snapTargetPoint;
  }

  size_t prePointsCount = editor.points.size();
  size_t preObjPointsCount = editor.ObjectPoints.size();
  Point_2 cgalPos = editor.toCGALPoint(worldPos_sfml);

  // 1. Check for EXISTING Objects (Points have highest priority)
  GeometricObject* hitObj = findClosestObject(editor, worldPos_sfml, tolerance);
  
  // PASS 1: Existing Objects (Topological Glue)
  if (hitObj) {
      if (hitObj->getType() == ObjectType::ObjectPoint) {
          // Explicitly search ObjectPoints vector
          for (auto& ptr : editor.ObjectPoints) {
              if (ptr.get() == hitObj) {
                  std::cout << "[SmartPoint] Snapped to existing ObjectPoint: " << ptr->getID() << std::endl;
                  return ptr; 
              }
          }
      } 
      else if (hitObj->getType() == ObjectType::Point || hitObj->getType() == ObjectType::IntersectionPoint) {
          // Standard search for Points/IntersectionPoints
          auto existingPoint = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hitObj));
          if (existingPoint) {
               std::cout << "[SmartPoint] Snapped to existing Point: " << existingPoint->getID() << std::endl;
               return existingPoint;
          }
      }
  }

  // PASS 1.5: Virtual Shape Vertices (RegularPolygon / Polygon / Rectangle)
  // For each shape vertex, if a Point already exists at that position, return it; otherwise, create a new Point.
  {
    const double tolSq = static_cast<double>(tolerance) * static_cast<double>(tolerance);
    auto trySmartVertexPoint = [&](const std::vector<Point_2>& vertices) -> std::shared_ptr<Point> {
      for (const auto& v : vertices) {
        double distSq = CGAL::to_double(CGAL::squared_distance(cgalPos, v));
        if (distSq <= tolSq) {
          // Check for existing Point at this vertex
          for (const auto& pt : editor.points) {
            if (pt && pt->isValid() && pt->isVisible()) {
              double ptDistSq = CGAL::to_double(CGAL::squared_distance(pt->getCGALPosition(), v));
              if (ptDistSq < 1e-12) {
                return pt;
              }
            }
          }
          // Otherwise, create a new Point at this vertex
          auto newPt = editor.createPoint(v);
          if (newPt) {
            editor.commandManager.pushHistoryOnly(
                std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPt)));
          }
          return newPt;
        }
      }
      return nullptr;
    };

    for (auto& regPoly : editor.regularPolygons) {
      if (!regPoly || !regPoly->isValid()) continue;
      const auto& verts = regPoly->getVertices();
      for (size_t i = 0; i < verts.size(); ++i) {
        double distSq = CGAL::to_double(CGAL::squared_distance(cgalPos, verts[i]));
        if (distSq <= tolSq) {
          // Reuse existing vertex-attached ObjectPoint if present
          for (auto& op : editor.ObjectPoints) {
            if (!op || op->getHostObject() != regPoly.get() || !op->isShapeEdgeAttachment()) continue;
            if (op->getEdgeIndex() == i && std::abs(op->getEdgeRelativePosition()) < 0.0001) {
              return op;
            }
          }

          // Create a vertex-anchored ObjectPoint at this vertex
          auto objPoint = ObjectPoint::createOnShapeEdge(regPoly, i, 0.0);
          if (objPoint && objPoint->isValid()) {
            objPoint->setIsVertexAnchor(true);
            objPoint->setRadius(editor.getCurrentPointSize());
            if (objPoint->getLabel().empty()) {
              objPoint->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
            }
            editor.ObjectPoints.push_back(objPoint);
            editor.commandManager.pushHistoryOnly(
                std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(objPoint)));
            return objPoint;
          }
        }
      }
    }

    for (auto& poly : editor.polygons) {
      if (poly && poly->isValid()) {
        if (auto smartPt = trySmartVertexPoint(poly->getVertices())) {
          return smartPt;
        }
      }
    }

    for (auto& rect : editor.rectangles) {
      if (rect && rect->isValid()) {
        if (auto smartPt = trySmartVertexPoint(rect->getVertices())) {
          return smartPt;
        }
      }
    }
  }

  // -----------------------------------------------------------------------
  // 2. Check for VIRTUAL Intersection (Ghost Point) BEFORE Lines
  // -----------------------------------------------------------------------
  if (auto hit = PointUtils::getHoveredIntersection(editor, worldPos_sfml, tolerance)) {
    // We found a virtual intersection! Create it now.
    // Use IntersectionPoint factory to ensure proper type and parent linkage
    auto pt = IntersectionPoint::create(hit->line1, hit->line2, hit->position);
    
    if (pt) {
      pt->setIntersectionPoint(true);
      pt->setFillColor(Constants::INTERSECTION_POINT_COLOR);
      pt->setLocked(true);  // Lock to prevent movement
      
      // FIX: MAKE IT DYNAMIC
      // Link the two objects that create this intersection
      if (hit->line1) hit->line1->addDependent(pt);
      if (hit->line2) hit->line2->addDependent(pt);
      
      pt->setDependent(true);
      
      // ADD TO EDITOR'S POINTS COLLECTION (IntersectionPoint inherits from Point)
      editor.points.push_back(pt);
      
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(pt)));
      
      std::cout << "Created IntersectionPoint at (" << CGAL::to_double(pt->getCGALPosition().x()) 
                << ", " << CGAL::to_double(pt->getCGALPosition().y()) << ")" << std::endl;
    }
    return pt; // Return immediately, DO NOT create an ObjectPoint on the line
  }

  // 3. NOW Check for Lines / Shapes (ObjectPoints)
  bool allowAutoObjectPoint = (editor.m_currentToolType != ObjectType::Rectangle && 
                               editor.m_currentToolType != ObjectType::RectangleRotatable);

  if (allowAutoObjectPoint && hitObj) {
    // B. Snap to Line/Segment/Ray
    if (hitObj->getType() == ObjectType::Line || hitObj->getType() == ObjectType::LineSegment || hitObj->getType() == ObjectType::Ray) {
      auto linePtr = std::dynamic_pointer_cast<Line>(editor.findSharedPtr(hitObj));
      if (linePtr && linePtr->isValid()) {
        Point_2 startPos = linePtr->getStartPoint();
        Point_2 endPos = linePtr->getEndPoint();
        double relativePos = ProjectionUtils::getRelativePositionOnLine(cgalPos, startPos, endPos, linePtr->isSegment());
        auto objPoint = ObjectPoint::create(linePtr, relativePos, Constants::OBJECT_POINT_DEFAULT_COLOR);
        if (objPoint && objPoint->isValid()) {
          // Assign label to ObjectPoint
          objPoint->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
          
          editor.ObjectPoints.push_back(objPoint);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(objPoint)));
          return objPoint;
        }
      }
    }

    // C. Snap to Circle
    if (hitObj->getType() == ObjectType::Circle) {
      auto circlePtr = std::dynamic_pointer_cast<Circle>(editor.findSharedPtr(hitObj));
      if (circlePtr && circlePtr->isValid()) {
        Point_2 center = circlePtr->getCenterPoint();
        Vector_2 toClick = cgalPos - center;
        double angleRad = std::atan2(CGAL::to_double(toClick.y()), CGAL::to_double(toClick.x()));
        auto objPoint = ObjectPoint::create(circlePtr, angleRad, Constants::OBJECT_POINT_DEFAULT_COLOR);
        if (objPoint && objPoint->isValid()) {
          // Assign label to ObjectPoint
          objPoint->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
          
          editor.ObjectPoints.push_back(objPoint);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(objPoint)));
          return objPoint;
        }
      }
    }

    // D. Snap to Shape Edge
    if (hitObj->getType() == ObjectType::Rectangle || hitObj->getType() == ObjectType::Triangle || 
        hitObj->getType() == ObjectType::Polygon || hitObj->getType() == ObjectType::RegularPolygon) {
      auto edgeHit = PointUtils::findNearestEdge(editor, worldPos_sfml, tolerance);
      if (edgeHit && edgeHit->host) {
        auto hostPtr = editor.findSharedPtr(edgeHit->host);
        if (hostPtr) {
          auto objPoint = ObjectPoint::createOnShapeEdge(hostPtr, edgeHit->edgeIndex, edgeHit->relativePosition);
          if (objPoint && objPoint->isValid()) {
            // Assign label to ObjectPoint
            objPoint->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
            
            editor.ObjectPoints.push_back(objPoint);
            editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(objPoint)));
            return objPoint;
          }
        }
      }
    }
  }

  // 4. Fallback: Create Free Point
  auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
  bool createdNew = (editor.points.size() > prePointsCount) || (editor.ObjectPoints.size() > preObjPointsCount);
  if (createdNew && smartPoint) {
    editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(smartPoint)));
  }
  return smartPoint;
}

static GeometricObject* findClosestObject(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml, float tolerance) {
  GeometricObject* closest = nullptr;
  double bestDistSq = static_cast<double>(tolerance) * static_cast<double>(tolerance);
  Point_2 cgalPos = editor.toCGALPoint(worldPos_sfml);

  // ============================================================
  // THREE-PASS SELECTION PRIORITY (Points > Lines > Shapes)
  // ============================================================

  // PASS 1: HIGH PRIORITY - Points (IntersectionPoint, Point, ObjectPoint)
  auto checkPoint = [&](GeometricObject* obj) -> bool {
    if (!obj || !obj->isValid() || !obj->isVisible()) return false;
    
    ObjectType type = obj->getType();
    if (type != ObjectType::Point && type != ObjectType::ObjectPoint && type != ObjectType::IntersectionPoint) {
      return false;
    }
    
    Point_2 objPos = static_cast<Point*>(obj)->getCGALPosition();
    double d = CGAL::to_double(CGAL::squared_distance(objPos, cgalPos));
    
    if (d <= bestDistSq) {
      bestDistSq = d;
      closest = obj;
      return true;
    }
    return false;
  };


  // Search all points in reverse (topmost first)
  for (auto it = editor.points.rbegin(); it != editor.points.rend(); ++it) {
    auto& pointPtr = *it;
    if (checkPoint(pointPtr.get()) && pointPtr->getType() == ObjectType::IntersectionPoint) {
      // IntersectionPoint found - highest priority, return immediately
      return pointPtr.get();
    }
  }
  for (auto it = editor.ObjectPoints.rbegin(); it != editor.ObjectPoints.rend(); ++it) {
    checkPoint(it->get());
  }

  // If we found any point, return it immediately (points have absolute priority)
  if (closest) return closest;


  // PASS 2: MEDIUM PRIORITY - Lines and Circles
  for (auto& linePtr : editor.lines) {
    if (!linePtr || !linePtr->isValid() || !linePtr->isVisible() || linePtr->isLocked()) continue;
    if (!linePtr->contains(worldPos_sfml, tolerance)) continue;
    double distSq = 0.0;
    if (linePtr->isSegment()) {
      Segment_2 seg(linePtr->getStartPoint(), linePtr->getEndPoint());
      distSq = CGAL::to_double(CGAL::squared_distance(seg, cgalPos));
    } else {
      distSq = CGAL::to_double(CGAL::squared_distance(linePtr->getCGALLine(), cgalPos));
    }
    if (distSq <= bestDistSq) {
      bestDistSq = distSq;
      closest = linePtr.get();
    }
  }

  for (auto& circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid() || !circlePtr->isVisible() || circlePtr->isLocked()) continue;

    if (circlePtr->contains(worldPos_sfml, tolerance)) {
      Point_2 center = circlePtr->getCenterPoint();
      double distToCenter = std::sqrt(CGAL::to_double(CGAL::squared_distance(center, cgalPos)));
      double delta = std::abs(distToCenter - circlePtr->getRadius());
      double distSq = delta * delta;

      if (distSq <= bestDistSq) {
        bestDistSq = distSq;
        closest = circlePtr.get();
      }
    }
  }
  
  // If we found a line or circle, return it (medium priority)
  if (closest) return closest;

  // PASS 3: LOW PRIORITY - Shapes (Rectangles, Triangles, Polygons)
  auto considerVertices = [&](GeometricObject* obj, const std::vector<Point_2>& verts) {
    if (!obj || !obj->isValid() || !obj->isVisible() || obj->isLocked()) return;
    for (const auto& v : verts) {
      double distSq = CGAL::to_double(CGAL::squared_distance(v, cgalPos));
      if (distSq <= bestDistSq) {
        bestDistSq = distSq;
        closest = obj;
      }
    }
  };
  
  auto considerEdges = [&](GeometricObject* obj, const std::vector<Segment_2>& edges) {
    if (!obj || !obj->isValid() || !obj->isVisible() || obj->isLocked()) return;
    double localBest = std::numeric_limits<double>::infinity();
    for (const auto& edge : edges) {
      double distSq = CGAL::to_double(CGAL::squared_distance(edge, cgalPos));
      if (distSq < localBest) localBest = distSq;
    }
    if (localBest <= bestDistSq) {
      bestDistSq = localBest;
      closest = obj;
    }
  };

  for (auto& rectPtr : editor.rectangles) {
    if (rectPtr) considerVertices(rectPtr.get(), rectPtr->getVertices());
  }
  for (auto& polyPtr : editor.polygons) {
    if (polyPtr) considerVertices(polyPtr.get(), polyPtr->getVertices());
  }
  for (auto& regPtr : editor.regularPolygons) {
    if (regPtr) considerVertices(regPtr.get(), regPtr->getVertices());
  }
  for (auto& triPtr : editor.triangles) {
    if (triPtr) considerVertices(triPtr.get(), triPtr->getVertices());
  }
  
  for (auto& rectPtr : editor.rectangles) {
    if (!rectPtr || !rectPtr->isValid() || !rectPtr->isVisible() || rectPtr->isLocked()) continue;
    considerEdges(rectPtr.get(), rectPtr->getEdges());
  }
  for (auto& triPtr : editor.triangles) {
    if (!triPtr || !triPtr->isValid() || !triPtr->isVisible() || triPtr->isLocked()) continue;
    considerEdges(triPtr.get(), triPtr->getEdges());
  }
  for (auto& polyPtr : editor.polygons) {
    if (!polyPtr || !polyPtr->isValid() || !polyPtr->isVisible() || polyPtr->isLocked()) continue;
    considerEdges(polyPtr.get(), polyPtr->getEdges());
  }
  for (auto& regPtr : editor.regularPolygons) {
    if (!regPtr || !regPtr->isValid() || !regPtr->isVisible() || regPtr->isLocked()) continue;
    considerEdges(regPtr.get(), regPtr->getEdges());
    if (regPtr->contains(worldPos_sfml, tolerance) && bestDistSq > 0.01) { // Small epsilon to prefer edges
        bestDistSq = 0.01;
        closest = regPtr.get();
    }
  }

  for (auto& textPtr : editor.textLabels) {
    if (!textPtr || !textPtr->isValid() || !textPtr->isVisible() || textPtr->isLocked()) continue;
    if (textPtr->contains(worldPos_sfml, tolerance)) {
        closest = textPtr.get();
        bestDistSq = 0.0;
        return closest; // Top-most text label wins immediately
    }
  }

  return closest;
}

static std::string getPrimeLabel(const std::string& oldLabel) { return oldLabel.empty() ? std::string() : (oldLabel + "'"); }

// --- Helper for Shape Edge Transformations ---
static std::shared_ptr<Point> getPointForShapeVertex(GeometryEditor& editor, std::shared_ptr<GeometricObject> shape, size_t index) {
  if (!shape) return nullptr;
  if (auto rect = std::dynamic_pointer_cast<Rectangle>(shape)) {
    if (index == 0) return rect->getCorner1Point();
    if (index == 2) return rect->getCorner2Point();

    // For auxiliary corners, ensure they exist
    if (index == 1) {
      if (rect->getCornerBPoint()) return rect->getCornerBPoint();
      // Force creation if missing
      auto p1 = rect->getCorner1Point();
      auto p2 = rect->getCorner2Point();
      if (!p1 || !p2) return nullptr;

      // We need to trigger syncDependentCorners to calculate positions, but we need points relative to that geometry
      // Create independent points first and let sync update them
      auto cb = std::make_shared<Point>(Point_2(0, 0), 1.0f);
      auto cd = rect->getCornerDPoint();
      if (!cd) cd = std::make_shared<Point>(Point_2(0, 0), 1.0f);

      rect->setDependentCornerPoints(cb, cd);
      cb->setVisible(false);
      cb->setDependent(true);
      cb->setShowLabel(false);
      if (!rect->getCornerDPoint()) {
        cd->setVisible(false);
        cd->setDependent(true);
        cd->setShowLabel(false);
      }

      return cb;
    }
    if (index == 3) {
      if (rect->getCornerDPoint()) return rect->getCornerDPoint();
      // Force creation
      auto p1 = rect->getCorner1Point();
      if (!p1) return nullptr;

      auto cd = std::make_shared<Point>(Point_2(0, 0), 1.0f);
      auto cb = rect->getCornerBPoint();
      if (!cb) cb = std::make_shared<Point>(Point_2(0, 0), 1.0f);

      rect->setDependentCornerPoints(cb, cd);
      cd->setVisible(false);
      cd->setDependent(true);
      cd->setShowLabel(false);
      if (!rect->getCornerBPoint()) {
        cb->setVisible(false);
        cb->setDependent(true);
        cb->setShowLabel(false);
      }

      return cd;
    }
  }
  if (auto tri = std::dynamic_pointer_cast<Triangle>(shape)) return tri->getVertexPoint(index);
  if (auto poly = std::dynamic_pointer_cast<Polygon>(shape)) return poly->getVertexPoint(index);

  // Fallback for types without Point vertices (e.g., RegularPolygon)
  auto verts = shape->getInteractableVertices();
  if (index >= verts.size()) return nullptr;

  // Search if an ObjectPoint already exists for this exact vertex
  for (auto& op : editor.ObjectPoints) {
    if (op && op->getHostObject() == shape.get() && op->isShapeEdgeAttachment() && op->getEdgeIndex() == index &&
        std::abs(op->getEdgeRelativePosition()) < 0.0001) {
      return op;
    }
  }

  // Create a new invisible dependent ObjectPoint at the vertex
  auto op = ObjectPoint::createOnShapeEdge(shape, index, 0.0);
  if (op) {
    op->setVisible(false);
    op->setShowLabel(false);
    op->setDependent(true);
    op->setCreatedWithShape(true);
    editor.ObjectPoints.push_back(op);
    editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(op)));
    return op;
  }
  return nullptr;
}

static std::shared_ptr<Line> getOrCreateHelperLineForEdge(GeometryEditor& editor, std::shared_ptr<GeometricObject> shape, size_t edgeIndex) {
  if (!shape) return nullptr;
  size_t n = shape->getEdges().size();
  if (n == 0) return nullptr;

  // For rectangles, use edge-attached ObjectPoints to avoid vertex-order mismatches
  if (shape->getType() == ObjectType::Rectangle || shape->getType() == ObjectType::RectangleRotatable) {
    std::shared_ptr<ObjectPoint> op1;
    std::shared_ptr<ObjectPoint> op2;
    for (auto& op : editor.ObjectPoints) {
      if (!op || op->getHostObject() != shape.get() || !op->isShapeEdgeAttachment()) continue;
      if (op->getEdgeIndex() == edgeIndex && std::abs(op->getEdgeRelativePosition()) < 0.0001) {
        op1 = op;
      } else if (op->getEdgeIndex() == edgeIndex && std::abs(op->getEdgeRelativePosition() - 1.0) < 0.0001) {
        op2 = op;
      }
    }

    if (!op1) {
      op1 = ObjectPoint::createOnShapeEdge(shape, edgeIndex, 0.0);
      if (op1) {
        op1->setVisible(false);
        op1->setDependent(true);
        op1->setShowLabel(false);
        editor.ObjectPoints.push_back(op1);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(op1)));
      }
    }
    if (!op2) {
      op2 = ObjectPoint::createOnShapeEdge(shape, edgeIndex, 1.0);
      if (op2) {
        op2->setVisible(false);
        op2->setDependent(true);
        op2->setShowLabel(false);
        editor.ObjectPoints.push_back(op2);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(op2)));
      }
    }

    auto p1 = std::static_pointer_cast<Point>(op1);
    auto p2 = std::static_pointer_cast<Point>(op2);
    if (!p1 || !p2) return nullptr;

    // Try to find an existing line representing this edge
    for (auto& ln : editor.lines) {
      if (!ln) continue;
      auto s = ln->getStartPointObjectShared();
      auto e = ln->getEndPointObjectShared();
      if ((s == p1 && e == p2) || (s == p2 && e == p1)) return ln;
    }

    auto newLine = std::make_shared<Line>(p1, p2, true, shape->getColor());
    newLine->setVisible(false);
    newLine->setLocked(true);
    newLine->setDependent(true);  // Prevent accidental cleanup
    newLine->setThickness(1.0f);
    editor.lines.push_back(newLine);
    editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));
    return newLine;
  }

  auto p1 = getPointForShapeVertex(editor, shape, edgeIndex);
  auto p2 = getPointForShapeVertex(editor, shape, (edgeIndex + 1) % n);
  if (!p1 || !p2) return nullptr;

  // Try to find an existing line representing this edge
  for (auto& ln : editor.lines) {
    if (!ln) continue;
    auto s = ln->getStartPointObjectShared();
    auto e = ln->getEndPointObjectShared();
    if ((s == p1 && e == p2) || (s == p2 && e == p1)) return ln;
  }

  // Create a hidden helper line
  auto newLine = std::make_shared<Line>(p1, p2, true, shape->getColor());
  newLine->setVisible(false);
  newLine->setLocked(true);
  newLine->setDependent(true);  // Prevent accidental cleanup
  newLine->setThickness(1.0f);
  editor.lines.push_back(newLine);
  editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));
  return newLine;
}

static std::shared_ptr<Point> createTransformedPoint(GeometryEditor& editor,
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

  TransformationType t = TransformationType::None;
  std::shared_ptr<GeometricObject> aux;
  switch (tool) {
    case ObjectType::ReflectAboutLine:
      t = TransformationType::Reflect;
      aux = kernelLine;
      break;
    case ObjectType::ReflectAboutPoint:
      t = TransformationType::ReflectPoint;
      aux = kernelPoint;
      break;
    case ObjectType::ReflectAboutCircle:
      t = TransformationType::ReflectCircle;
      aux = kernelCircle;
      break;
    case ObjectType::RotateAroundPoint:
      t = TransformationType::Rotate;
      aux = kernelPoint;
      break;
    case ObjectType::TranslateByVector:
      t = TransformationType::Translate;
      aux = vecEnd;
      break;
    case ObjectType::DilateFromPoint:
      t = TransformationType::Dilate;
      aux = kernelPoint;
      break;
    default:
      t = TransformationType::None;
      break;
  }

  if (t != TransformationType::None) {
    newPoint->restoreTransformation(source, aux, t);
  }

  // Inherit visibility from source
  newPoint->setVisible(source->isVisible());
  newPoint->setLabelOffset(source->getLabelOffset());

  std::string baseLabel = source->getLabel();

  // If source has a label, transform it (A -> A')
  if (!baseLabel.empty()) {
    std::string primeLabel = getPrimeLabel(baseLabel);
    // If getPrimeLabel fails or returns empty (it shouldn't if base is valid), fallback
    if (primeLabel.empty()) {
       // Just append reasonable suffix if getPrimeLabel is dumb
       primeLabel = baseLabel + "'"; 
    }
    newPoint->setLabel(primeLabel);
    
    // Often we want to show the label of a transformed point even if source was hidden (e.g. shape vertex),
    // but sticking to "inherit" behavior is safer unless it's a construction.
    // However, for explicit TRANSFORMATIONS, users usually want to see the result label.
    // If source was a shape vertex (hidden label), the new point is also a vertex of a new shape?
    // If this function is creating a point for a new shape, we might hide it later.
    // If it's a free point transform, we show it.
    newPoint->setShowLabel(true); 
  } else {
    // Source had no label.
    // Generate new label from sequence (A, B, C...)
    newPoint->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
    newPoint->setShowLabel(true);
  }

  return newPoint;
}

bool handleCompassCreation(GeometryEditor& editor,
                           std::vector<GeometricObject*>& tempSelectedObjects,
                           const sf::Vector2f& worldPos_sfml,
                           float tolerance,
                           GeometricObject* explicitCenter = nullptr) { // <--- Added Parameter

  std::shared_ptr<Point> centerPt = nullptr;

  // 1. Priority: Use the point provided by the click handler
  if (explicitCenter) {
      if (explicitCenter->getType() == ObjectType::Point || 
          explicitCenter->getType() == ObjectType::ObjectPoint || 
          explicitCenter->getType() == ObjectType::IntersectionPoint) {
          centerPt = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(explicitCenter));
      }
  }

  // 2. Fallback: Look under mouse (Legacy/Safety)
  if (!centerPt) {
      GeometricObject* hitPoint = editor.lookForObjectAt(worldPos_sfml, tolerance, 
        {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});
      if (hitPoint) {
        centerPt = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hitPoint));
      }
  }

  // 3. Final Fallback: Create Free Point (Allow placing anywhere)
  if (!centerPt) {
    centerPt = editor.createPoint(editor.toCGALPoint(worldPos_sfml));
    if (centerPt) {
      editor.commandManager.pushHistoryOnly(
          std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(centerPt)));
    }
  }

  // CRITICAL FIX: Ensure we clear selection even if we fail early
  if (!centerPt) {
    editor.setGUIMessage("Error: Could not create center point.");
    // Clear selection so user can try again
    for (auto* obj : tempSelectedObjects) { if (obj) obj->setSelected(false); }
    tempSelectedObjects.clear();
    return false;
  }

  std::shared_ptr<CompassCircle> newCircle;

  // Radius from Line
  if (tempSelectedObjects.size() == 1) {
    auto line = std::dynamic_pointer_cast<Line>(editor.findSharedPtr(tempSelectedObjects[0]));
    if (line) {
      newCircle = std::make_shared<CompassCircle>(centerPt, line, 0, editor.getCurrentColor());
    }
  } 
  // Radius from Two Points
  else if (tempSelectedObjects.size() == 2) {
    auto p1 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[0]));
    auto p2 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[1]));
    if (p1 && p2) {
      newCircle = std::make_shared<CompassCircle>(centerPt, p1, p2, 0, editor.getCurrentColor());
    }
  }

  if (newCircle) {
    newCircle->setThickness(editor.currentThickness);
    newCircle->update();
    editor.circles.push_back(newCircle);
    newCircle->setSelected(true);
    editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newCircle)));
    editor.setGUIMessage("Compass Circle constructed.");
  } else {
    editor.setGUIMessage("Error: Invalid compass configuration.");
  }

  // CRITICAL: Always clear selection to reset state for the next tool usage
  for (auto* obj : tempSelectedObjects) {
    if (obj) obj->setSelected(false);
  }
  tempSelectedObjects.clear();

  return newCircle != nullptr;
}
// --- Helper: Compass Tool Logic ---
static void handleCompassToolClick(GeometryEditor& editor, GeometricObject* clickedObj, const sf::Vector2f& worldPos_sfml, float tolerance) {
  // Use member variable for state so HandleEvents can see it (for preview)
  
  // 1. Smart Fallback
  if (!clickedObj) {
      auto smartPt = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
      if (smartPt) {
          clickedObj = smartPt.get();
      }
  }

  // --- PHASE 1: START SELECTION ---
  if (editor.m_compassSelection.empty()) {
    if (!clickedObj) return;

    if (clickedObj->getType() == ObjectType::LineSegment) {
      clickedObj->setSelected(true);
      editor.m_compassSelection.push_back(clickedObj);
      editor.setGUIMessage("Compass: Radius set from Line. Select Center Point.");
      return;
    }

    // Check for Point types
    if (clickedObj->getType() == ObjectType::Point || 
        clickedObj->getType() == ObjectType::ObjectPoint || 
        clickedObj->getType() == ObjectType::IntersectionPoint) {
      clickedObj->setSelected(true);
      editor.m_compassSelection.push_back(clickedObj);
      editor.setGUIMessage("Compass: Radius Pt 1 selected. Select Radius Pt 2.");
      return;
    }
  } 
  
  // --- PHASE 2: SECOND RADIUS POINT ---
  else if (editor.m_compassSelection.size() == 1 &&
           (editor.m_compassSelection[0]->getType() == ObjectType::Point || 
            editor.m_compassSelection[0]->getType() == ObjectType::ObjectPoint || 
            editor.m_compassSelection[0]->getType() == ObjectType::IntersectionPoint)) {
            
    if (!clickedObj) return;
    if (clickedObj == editor.m_compassSelection[0]) return; // Debounce same point

    if (clickedObj->getType() == ObjectType::Point || 
        clickedObj->getType() == ObjectType::ObjectPoint || 
        clickedObj->getType() == ObjectType::IntersectionPoint) {
      clickedObj->setSelected(true);
      editor.m_compassSelection.push_back(clickedObj);
      editor.setGUIMessage("Compass: Radius defined. Select Center Point.");
    }
    return;
  } 
  
  // --- PHASE 3: CREATE CENTER & CIRCLE ---
  // The logic here checks if we have enough items to create the compass
  else if ((editor.m_compassSelection.size() == 1 && editor.m_compassSelection[0]->getType() == ObjectType::LineSegment) ||
           (editor.m_compassSelection.size() == 2 && 
             (editor.m_compassSelection[0]->getType() == ObjectType::Point || 
              editor.m_compassSelection[0]->getType() == ObjectType::ObjectPoint || 
              editor.m_compassSelection[0]->getType() == ObjectType::IntersectionPoint))) {
    
    // Pass the member vector and the explicitly clicked center object
    handleCompassCreation(editor, editor.m_compassSelection, worldPos_sfml, tolerance, clickedObj);
  }
}

static GeometricObject* findTopNonDependentAt(GeometryEditor& editor, const sf::Vector2f& pos, float tol) {
  auto isCandidate = [&](GeometricObject* obj) {
    if (!obj || !obj->isValid() || !obj->isVisible()) return false;
    if (obj->isDependent()) return false;
    return true;
  };

  // ObjectPoints
  for (auto it = editor.ObjectPoints.rbegin(); it != editor.ObjectPoints.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }

  // Points
  for (auto it = editor.points.rbegin(); it != editor.points.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }

  // Shapes
  for (auto it = editor.triangles.rbegin(); it != editor.triangles.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }
  for (auto it = editor.regularPolygons.rbegin(); it != editor.regularPolygons.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }
  for (auto it = editor.polygons.rbegin(); it != editor.polygons.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }
  for (auto it = editor.rectangles.rbegin(); it != editor.rectangles.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }

  // Lines (including Ray/Vector)
  for (auto it = editor.lines.rbegin(); it != editor.lines.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }

  // Circles
  for (auto it = editor.circles.rbegin(); it != editor.circles.rend(); ++it) {
    if (isCandidate(it->get()) && (*it)->contains(pos, tol)) return it->get();
  }

  return nullptr;
}

static void clearTransformSelection(std::vector<GeometricObject*>& tempSelectedObjects) {
  for (auto* obj : tempSelectedObjects) {
    if (obj) obj->setSelected(false);
  }
  tempSelectedObjects.clear();
  g_transformEdgeSelection.reset();
  s_transformSourceCount = 0;
}

static void clearTemporarySelection(std::vector<GeometricObject*>& selection) {
  for (auto* obj : selection) {
    if (obj) obj->setSelected(false);
  }
  selection.clear();
}

void clearTempSelectedObjects() {
  clearTemporarySelection(tempSelectedObjects);
  g_transformEdgeSelection.reset();
}

static void registerTransformPoint(GeometryEditor& editor, const std::shared_ptr<Point>& newPoint) {
  if (!newPoint) return;
  editor.points.push_back(newPoint);
  editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoint)));
  if (newPoint->isVisible()) {
    newPoint->setSelected(true);
  }
}

static void registerHiddenTransformObjectPoint(GeometryEditor& editor, const std::shared_ptr<ObjectPoint>& newPoint) {
  if (!newPoint) return;
  newPoint->setVisible(false);
  newPoint->setShowLabel(false);
  newPoint->setDependent(true);
  newPoint->setCreatedWithShape(true);
  editor.ObjectPoints.push_back(newPoint);
  editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoint)));
}

static void attachTransformMetadata(const std::shared_ptr<GeometricObject>& source,
                                    const std::shared_ptr<GeometricObject>& created,
                                    ObjectType activeTool,
                                    const std::shared_ptr<GeometricObject>& aux,
                                    const std::shared_ptr<Point>& vStart,
                                    const std::shared_ptr<Point>& vEnd) {
  if (!source || !created) return;

  TransformationType t = TransformationType::None;
  switch (activeTool) {
    case ObjectType::ReflectAboutLine:
      t = TransformationType::Reflect;
      break;
    case ObjectType::ReflectAboutPoint:
      t = TransformationType::ReflectPoint;
      break;
    case ObjectType::ReflectAboutCircle:
      t = TransformationType::ReflectCircle;
      break;
    case ObjectType::RotateAroundPoint:
      t = TransformationType::Rotate;
      break;
    case ObjectType::TranslateByVector:
      t = TransformationType::Translate;
      break;
    case ObjectType::DilateFromPoint:
      t = TransformationType::Dilate;
      break;
    default:
      t = TransformationType::None;
      break;
  }

  created->setTransformType(t);
  created->restoreTransformation(source, aux, t);
  source->addDependent(created);

  if (t == TransformationType::Translate && vStart && vEnd) {
    Point_2 flatStart = flattenPoint(vStart->getCGALPosition());
    Point_2 flatEnd = flattenPoint(vEnd->getCGALPosition());
    created->setTranslationVector(flatEnd - flatStart);
  } else if (t == TransformationType::Rotate) {
    created->setTransformValue(g_transformRotationDegrees);
  } else if (t == TransformationType::Dilate) {
    created->setTransformValue(g_transformDilationFactor);
  }
}

static std::shared_ptr<Line> ensureHoverVector(GeometryEditor& editor, const std::shared_ptr<Point>& vStart, const std::shared_ptr<Point>& vEnd) {
  if (!vStart || !vEnd) return nullptr;

  for (const auto& ln : editor.lines) {
    if (!ln || !ln->isValid()) continue;
    // Look for any line/segment that connects the two points
    auto s = ln->getStartPointObjectShared();
    auto e = ln->getEndPointObjectShared();
    if ((s == vStart && e == vEnd) || (s == vEnd && e == vStart)) {
      return ln;
    }
  }

  auto vecLine = std::make_shared<Line>(vStart, vEnd, true, sf::Color(0, 0, 0, 0));
  vecLine->setLineType(Line::LineType::Vector);
  vecLine->setVisible(true);
  vecLine->setThickness(editor.currentThickness);
  editor.lines.push_back(vecLine);
  editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(vecLine)));
  return vecLine;
}

static bool tryTransformShapeEdge(GeometryEditor& editor,
                                  std::vector<GeometricObject*>& tempSelectedObjects,
                                  ObjectType activeTool,
                                  const std::shared_ptr<GeometricObject>& sourceShape,
                                  const std::shared_ptr<Point>& pivot,
                                  const std::shared_ptr<Line>& lineObj,
                                  const std::shared_ptr<Circle>& circleObj,
                                  const std::shared_ptr<Point>& vecStart,
                                  const std::shared_ptr<Point>& vecEnd) {
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::K)) return false;  // Force whole-shape mode
  if (!g_transformEdgeSelection || !sourceShape) return false;
  if (g_transformEdgeSelection->host != sourceShape.get()) return false;

  size_t edgeIndex = g_transformEdgeSelection->edgeIndex;
  auto edgeStart = ObjectPoint::createOnShapeEdge(sourceShape, edgeIndex, 0.0);
  auto edgeEnd = ObjectPoint::createOnShapeEdge(sourceShape, edgeIndex, 1.0);
  if (!edgeStart || !edgeEnd) return false;

  registerHiddenTransformObjectPoint(editor, edgeStart);
  registerHiddenTransformObjectPoint(editor, edgeEnd);

  auto p1t = createTransformedPoint(editor, edgeStart, activeTool, pivot, lineObj, circleObj, vecStart, vecEnd);
  if (p1t) {
    p1t->setColor(editor.getCurrentColor());
    if (p1t->getLabel().empty()) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      p1t->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
    }
    p1t->setShowLabel(true);
    registerTransformPoint(editor, p1t);
  }

  auto p2t = createTransformedPoint(editor, edgeEnd, activeTool, pivot, lineObj, circleObj, vecStart, vecEnd);
  if (!p1t || !p2t) {
    editor.setGUIMessage("Error: Invalid edge transformation selection.");
    clearTransformSelection(tempSelectedObjects);
    return true;
  }

  if (p2t) {
    p2t->setColor(editor.getCurrentColor());
    if (p2t->getLabel().empty()) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      p2t->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
    }
    p2t->setShowLabel(true);
    registerTransformPoint(editor, p2t);
  }

  auto newLine = std::make_shared<Line>(p1t, p2t, true, editor.getCurrentColor());
  newLine->setThickness(editor.currentThickness);
  newLine->setDependent(true);
  editor.lines.push_back(newLine);
  editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));
  editor.setGUIMessage("Transformed edge created.");
  clearTransformSelection(tempSelectedObjects);
  return true;
}

static GeometricObject* findClosestPointCandidate(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml, float tolerance) {
  GeometricObject* hit = findClosestObject(editor, worldPos_sfml, tolerance);
  if (!hit) return nullptr;
  if (hit->getType() == ObjectType::Point || hit->getType() == ObjectType::ObjectPoint ||
      hit->getType() == ObjectType::IntersectionPoint) {
    return hit;
  }
  return nullptr;
}

static std::shared_ptr<Point> getPointSharedFromHit(GeometryEditor& editor, GeometricObject* hit) {
  if (!hit) return nullptr;
  return std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hit));
}

static bool handleTranslationTool(GeometryEditor& editor,
                                  std::vector<GeometricObject*>& tempSelectedObjects,
                                  const sf::Vector2f& worldPos_sfml,
                                  float tolerance) {
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl)) return false;
  ObjectType tool = editor.m_currentToolType;

  auto applyVectorTranslation = [&](const std::shared_ptr<Point>& v1, const std::shared_ptr<Point>& v2) -> bool {
    std::shared_ptr<GeometricObject> vectorAux = ensureHoverVector(editor, v1, v2);
    int successCount = 0;
    for (size_t i = 0; i < s_transformSourceCount; ++i) {
      auto sourceShared = editor.findSharedPtr(tempSelectedObjects[i]);
      if (!sourceShared) continue;

      bool individualSuccess = false;
      // Edge transformation logic (only if we have exactly one source, for safety/existing behavior)
      if (s_transformSourceCount == 1 && (sourceShared->getType() == ObjectType::Rectangle || sourceShared->getType() == ObjectType::RectangleRotatable ||
          sourceShared->getType() == ObjectType::Triangle || sourceShared->getType() == ObjectType::Polygon ||
          sourceShared->getType() == ObjectType::RegularPolygon)) {
        if (tryTransformShapeEdge(editor, tempSelectedObjects, tool, sourceShared, nullptr, nullptr, nullptr, v1, v2)) {
          individualSuccess = true;
        }
      }

      if (!individualSuccess) {
        auto sourcePoint = std::dynamic_pointer_cast<Point>(sourceShared);
        if (sourcePoint) {
          auto newPoint = createTransformedPoint(editor, sourcePoint, tool, nullptr, nullptr, nullptr, v1, v2);
          if (newPoint) {
            registerTransformPoint(editor, newPoint);
            individualSuccess = true;
          }
        }
      }

      if (!individualSuccess && (sourceShared->getType() == ObjectType::Line || sourceShared->getType() == ObjectType::LineSegment)) {
        auto line = std::dynamic_pointer_cast<Line>(sourceShared);
        if (line) {
          auto p1 = line->getStartPointObjectShared();
          auto p2 = line->getEndPointObjectShared();
          auto p1t = createTransformedPoint(editor, p1, tool, nullptr, nullptr, nullptr, v1, v2);
          auto p2t = createTransformedPoint(editor, p2, tool, nullptr, nullptr, nullptr, v1, v2);
          if (p1t && p2t) {
            registerTransformPoint(editor, p1t);
            registerTransformPoint(editor, p2t);
            auto newLine = std::make_shared<Line>(p1t, p2t, line->isSegment(), editor.getCurrentColor());
            newLine->setThickness(line->getThickness());
            newLine->setDependent(true);
            attachTransformMetadata(sourceShared, newLine, tool, vectorAux, v1, v2);
            editor.lines.push_back(newLine);
            editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newLine));
            individualSuccess = true;
          }
        }
      }

      if (!individualSuccess && sourceShared->getType() == ObjectType::Circle) {
        auto circle = std::dynamic_pointer_cast<Circle>(sourceShared);
        if (circle) {
          auto centerRaw = circle->getCenterPointObject();
          auto centerShared = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(centerRaw));
          auto newCenter = createTransformedPoint(editor, centerShared, tool, nullptr, nullptr, nullptr, v1, v2);
          if (newCenter) {
            registerTransformPoint(editor, newCenter);
            auto newCircle = std::make_shared<Circle>(newCenter.get(), nullptr, circle->getRadius(), editor.getCurrentColor());
            newCircle->setThickness(circle->getThickness());
            newCircle->setDependent(true);
            attachTransformMetadata(sourceShared, newCircle, tool, vectorAux, v1, v2);
            editor.circles.push_back(newCircle);
            editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newCircle));
            individualSuccess = true;
          }
        }
      }

      if (!individualSuccess && (sourceShared->getType() == ObjectType::Rectangle || sourceShared->getType() == ObjectType::RectangleRotatable)) {
        auto rect = std::dynamic_pointer_cast<Rectangle>(sourceShared);
        auto V0 = getPointForShapeVertex(editor, rect, 0);
        auto V1 = getPointForShapeVertex(editor, rect, 1);
        auto V2 = getPointForShapeVertex(editor, rect, 2);
        auto V3 = getPointForShapeVertex(editor, rect, 3);
        auto V0t = createTransformedPoint(editor, V0, tool, nullptr, nullptr, nullptr, v1, v2);
        auto V1t = createTransformedPoint(editor, V1, tool, nullptr, nullptr, nullptr, v1, v2);
        auto V2t = createTransformedPoint(editor, V2, tool, nullptr, nullptr, nullptr, v1, v2);
        auto V3t = createTransformedPoint(editor, V3, tool, nullptr, nullptr, nullptr, v1, v2);
        if (V0t && V1t && V2t && V3t) {
          registerTransformPoint(editor, V0t);
          registerTransformPoint(editor, V1t);
          registerTransformPoint(editor, V2t);
          registerTransformPoint(editor, V3t);
          Point_2 p0 = flattenPoint(V0t->getCGALPosition());
          Point_2 p3 = flattenPoint(V3t->getCGALPosition());
          double h = std::sqrt(CGAL::to_double(CGAL::squared_distance(p0, p3)));
          auto newRect = std::make_shared<Rectangle>(V0t, V1t, h, editor.getCurrentColor(), editor.objectIdCounter++);
          newRect->setThickness(rect->getThickness());
          newRect->setDependent(true);
          attachTransformMetadata(sourceShared, newRect, tool, vectorAux, v1, v2);
          applyRectangleVertexLabels(editor, newRect, {V0t, V1t, V2t, V3t});
          editor.rectangles.push_back(newRect);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newRect));
          individualSuccess = true;
        }
      }

      if (!individualSuccess && sourceShared->getType() == ObjectType::Triangle) {
          auto tri = std::dynamic_pointer_cast<Triangle>(sourceShared);
          std::vector<std::shared_ptr<Point>> newPts;
          for (size_t k = 0; k < 3; ++k) {
              auto vt = createTransformedPoint(editor, tri->getVertexPoint(k), tool, nullptr, nullptr, nullptr, v1, v2);
              if (vt) {
                  registerTransformPoint(editor, vt);
                  newPts.push_back(vt);
              }
          }
          if (newPts.size() == 3) {
              auto newTri = std::make_shared<Triangle>(newPts[0], newPts[1], newPts[2], editor.getCurrentColor(), editor.objectIdCounter++);
              newTri->setThickness(tri->getThickness());
              newTri->setDependent(true);
              attachTransformMetadata(sourceShared, newTri, tool, vectorAux, v1, v2);
              editor.triangles.push_back(newTri);
              editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newTri));
              individualSuccess = true;
          }
      }

      if (!individualSuccess && sourceShared->getType() == ObjectType::RegularPolygon) {
          auto rpoly = std::dynamic_pointer_cast<RegularPolygon>(sourceShared);
          auto ct = createTransformedPoint(editor, rpoly->getCenterPoint(), tool, nullptr, nullptr, nullptr, v1, v2);
          auto vt = createTransformedPoint(editor, rpoly->getFirstVertexPoint(), tool, nullptr, nullptr, nullptr, v1, v2);
          if (ct && vt) {
              registerTransformPoint(editor, ct);
              registerTransformPoint(editor, vt);
              auto newRPoly = std::make_shared<RegularPolygon>(ct, vt, rpoly->getNumSides(), editor.getCurrentColor(), editor.objectIdCounter++);
              newRPoly->setThickness(rpoly->getThickness());
              newRPoly->setDependent(true);
              attachTransformMetadata(sourceShared, newRPoly, tool, vectorAux, v1, v2);
              editor.regularPolygons.push_back(newRPoly);
              editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newRPoly));
              individualSuccess = true;
          }
      }

      if (!individualSuccess && sourceShared->getType() == ObjectType::Polygon) {
          auto poly = std::dynamic_pointer_cast<Polygon>(sourceShared);
          if (poly) {
              std::vector<std::shared_ptr<Point>> newVerts;
              for (size_t k = 0; k < poly->getVertexCount(); ++k) {
                  auto v = poly->getVertexPoint(k);
                  auto vt = createTransformedPoint(editor, v, tool, nullptr, nullptr, nullptr, v1, v2);
                  if (vt) {
                      registerTransformPoint(editor, vt);
                      newVerts.push_back(vt);
                  }
              }
              if (newVerts.size() == poly->getVertexCount()) {
                  auto newPoly = std::make_shared<Polygon>(newVerts, editor.getCurrentColor());
                  newPoly->setThickness(poly->getThickness());
                  newPoly->setDependent(true);
                  attachTransformMetadata(sourceShared, newPoly, tool, vectorAux, v1, v2);
                  editor.polygons.push_back(newPoly);
                  editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newPoly));
                  individualSuccess = true;
              }
          }
      }

      if (individualSuccess) successCount++;
    }

    if (successCount > 0) {
      editor.setGUIMessage("Transformation completed.");
    } else {
      editor.setGUIMessage("Error: No objects were transformed.");
    }
    clearTransformSelection(tempSelectedObjects);
    return true;
  };

  if (tempSelectedObjects.size() == s_transformSourceCount) {
    // NEW: Check for EXISTING VECTOR OBJECT
    GeometricObject* vecObj = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Vector});
    if (vecObj && (vecObj->getType() == ObjectType::Vector || vecObj->getType() == ObjectType::Line || vecObj->getType() == ObjectType::LineSegment)) {
      auto vecLine = dynamic_cast<Line*>(vecObj);
      if (vecLine) {
        auto v1 = vecLine->getStartPointObjectShared();
        auto v2 = vecLine->getEndPointObjectShared();
        if (v1 && v2) {
          return applyVectorTranslation(v1, v2);
        }
      }
    }

    GeometricObject* vecStartRaw = findClosestPointCandidate(editor, worldPos_sfml, tolerance);
    if (!vecStartRaw) return false;
    tempSelectedObjects.push_back(vecStartRaw);
    vecStartRaw->setSelected(true);
    editor.setGUIMessage("Translate: Select vector end point.");
    return true;
  }

  GeometricObject* vecEndRaw = findClosestPointCandidate(editor, worldPos_sfml, tolerance);
  if (!vecEndRaw) return false;
  tempSelectedObjects.push_back(vecEndRaw);

  auto v1 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[s_transformSourceCount]));
  auto v2 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[s_transformSourceCount + 1]));
  if (!v1 || !v2) {
    clearTransformSelection(tempSelectedObjects);
    editor.setGUIMessage("Error: Invalid translation vector.");
    return false;
  }

  ensureHoverVector(editor, v1, v2);

  return applyVectorTranslation(v1, v2);
}

static bool handleLineReflectionSelection(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml, float tolerance,
                                          std::shared_ptr<Line>& lineObj) {
  GeometricObject* hitLine = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Line, ObjectType::LineSegment, ObjectType::Ray});
  if (hitLine) {
    lineObj = std::dynamic_pointer_cast<Line>(editor.findSharedPtr(hitLine));
    return (lineObj != nullptr);
  }

  std::vector<ObjectType> axisTypes = {ObjectType::Rectangle, ObjectType::RectangleRotatable, ObjectType::Triangle, ObjectType::Polygon,
                                       ObjectType::RegularPolygon};
  GeometricObject* hitAxis = editor.lookForObjectAt(worldPos_sfml, tolerance, axisTypes);
  if (!hitAxis) return false;
  auto edgeHit = PointUtils::findNearestEdge(editor, worldPos_sfml, tolerance);
  if (edgeHit && edgeHit->host == hitAxis) {
    lineObj = getOrCreateHelperLineForEdge(editor, editor.findSharedPtr(hitAxis), edgeHit->edgeIndex);
    return (lineObj != nullptr);
  }

  auto shapeShared = editor.findSharedPtr(hitAxis);
  if (shapeShared) {
    auto edges = shapeShared->getEdges();
    if (!edges.empty()) {
      Point_2 mousePos(FT(worldPos_sfml.x), FT(worldPos_sfml.y));
      double bestDist = std::numeric_limits<double>::max();
      size_t bestIndex = 0;
      for (size_t i = 0; i < edges.size(); ++i) {
        Point_2 proj;
        double rel;
        double dist = PointUtils::projectPointOntoSegment(mousePos, edges[i], proj, rel);
        if (dist < bestDist) {
          bestDist = dist;
          bestIndex = i;
        }
      }
      lineObj = getOrCreateHelperLineForEdge(editor, shapeShared, bestIndex);
      return (lineObj != nullptr);
    }
  }

  return false;
}

static bool handleCircleReflectionSelection(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml, float tolerance,
                                            std::shared_ptr<Circle>& circleObj) {
  GeometricObject* hitCircle = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Circle});
  if (!hitCircle) return false;
  circleObj = std::dynamic_pointer_cast<Circle>(editor.findSharedPtr(hitCircle));
  return (circleObj != nullptr);
}

static bool handlePointPivotSelection(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml, float tolerance,
                                      std::shared_ptr<Point>& pivotPoint) {
  GeometricObject* hitPoint = findClosestPointCandidate(editor, worldPos_sfml, tolerance);
  if (!hitPoint) return false;
  pivotPoint = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hitPoint));
  return (pivotPoint != nullptr);
}

bool handleTransformationCreation(GeometryEditor& editor,
                                  std::vector<GeometricObject*>& tempSelectedObjects,
                                  const sf::Vector2f& worldPos_sfml,
                                  float tolerance) {

  ObjectType tool = editor.m_currentToolType;
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl)) return false;

  // Step 1: Source object(s)
  if (tempSelectedObjects.empty()) {
    // If we already have selected objects (from move tool or box selection), use them
    if (!editor.selectedObjects.empty()) {
      for (auto* obj : editor.selectedObjects) {
        if (obj && obj->isValid() && obj->isVisible()) {
          tempSelectedObjects.push_back(obj);
          obj->setSelected(true);
        }
      }
    } else if (editor.selectedObject && editor.selectedObject->isValid() && editor.selectedObject->isVisible()) {
      tempSelectedObjects.push_back(editor.selectedObject);
      editor.selectedObject->setSelected(true);
    }

    if (!tempSelectedObjects.empty()) {
      s_transformSourceCount = tempSelectedObjects.size();
      if (tool == ObjectType::ReflectAboutLine) editor.setGUIMessage("Reflect: Select line to reflect about.");
      else if (tool == ObjectType::ReflectAboutPoint) editor.setGUIMessage("Reflect: Select center point.");
      else if (tool == ObjectType::ReflectAboutCircle) editor.setGUIMessage("Invert: Select circle for inversion.");
      else if (tool == ObjectType::RotateAroundPoint) editor.setGUIMessage("Rotate: Select pivot point.");
      else if (tool == ObjectType::TranslateByVector) editor.setGUIMessage("Translate: Select vector start point.");
      else if (tool == ObjectType::DilateFromPoint) editor.setGUIMessage("Dilate: Select center point.");
      // DO NOT return true here; allow the current click to be checked as an operator
    }

    if (tempSelectedObjects.empty()) {
      GeometricObject* hitObj = nullptr;
      if (editor.hoveredObject && editor.hoveredObject->isValid() && editor.hoveredObject->isVisible()) {
        ObjectType hoveredType = editor.hoveredObject->getType();
        const std::vector<ObjectType> allowedTypes = {ObjectType::Point,    ObjectType::ObjectPoint,   ObjectType::IntersectionPoint,
                                                      ObjectType::Line,     ObjectType::LineSegment,   ObjectType::Circle,
                                                      ObjectType::Polygon,  ObjectType::Rectangle,     ObjectType::RectangleRotatable,
                                                      ObjectType::Triangle, ObjectType::RegularPolygon};
        if (std::find(allowedTypes.begin(), allowedTypes.end(), hoveredType) != allowedTypes.end()) {
          if (editor.hoveredObject->contains(worldPos_sfml, tolerance)) {
            hitObj = editor.hoveredObject;
          }
        }
      }

      if (!hitObj) {
        GeometricObject* pointHit = findClosestPointCandidate(editor, worldPos_sfml, tolerance);
        if (pointHit) {
          hitObj = pointHit;
        } else {
          hitObj = editor.lookForObjectAt(
              worldPos_sfml, tolerance,
              {ObjectType::Line, ObjectType::LineSegment, ObjectType::Circle, ObjectType::Polygon, ObjectType::Rectangle,
               ObjectType::RectangleRotatable, ObjectType::Triangle, ObjectType::RegularPolygon});
        }
      }
      if (!hitObj) return false;

      // Removed forced preference for non-dependent (original) object.

      g_transformEdgeSelection.reset();
      // Prioritize edge selection for shapes that support it, unless Alt is held
      if (hitObj->getType() == ObjectType::Rectangle || hitObj->getType() == ObjectType::RectangleRotatable ||
          hitObj->getType() == ObjectType::Triangle || hitObj->getType() == ObjectType::Polygon ||
          hitObj->getType() == ObjectType::RegularPolygon) {
        bool forceWholeShape = sf::Keyboard::isKeyPressed(sf::Keyboard::K);
        if (!forceWholeShape) {
          if (auto edgeHit = PointUtils::findNearestEdge(editor, worldPos_sfml, tolerance)) {
            if (edgeHit->host == hitObj) {
              g_transformEdgeSelection = edgeHit;
              tempSelectedObjects.push_back(hitObj);
              hitObj->setSelected(true);
              s_transformSourceCount = 1;
              editor.setGUIMessage("Edge selected. Now select transformation operator. (Hold K for whole shape)");
              return true;
            }
          }
        }
      }
      // If no edge was hit, select the whole shape or object
      tempSelectedObjects.push_back(hitObj);
      hitObj->setSelected(true);
      s_transformSourceCount = 1;

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
  }

  if (tool == ObjectType::TranslateByVector) {
    // handleTranslationTool handles multi-selection source objects internally
    return handleTranslationTool(editor, tempSelectedObjects, worldPos_sfml, tolerance);
  }

  // Step 2: Operator object selection
  std::shared_ptr<Point> pivotPoint;
  std::shared_ptr<Line> lineObj;
  std::shared_ptr<Circle> circleObj;

  if (tool == ObjectType::ReflectAboutLine) {
    if (!handleLineReflectionSelection(editor, worldPos_sfml, tolerance, lineObj)) return false;
  } else if (tool == ObjectType::ReflectAboutCircle) {
    if (!handleCircleReflectionSelection(editor, worldPos_sfml, tolerance, circleObj)) return false;
  } else {
    if (!handlePointPivotSelection(editor, worldPos_sfml, tolerance, pivotPoint)) return false;
  }

  int successCount = 0;
  for (size_t i = 0; i < s_transformSourceCount; ++i) {
    auto sourceShared = editor.findSharedPtr(tempSelectedObjects[i]);
    if (!sourceShared) continue;

    if ((pivotPoint && sourceShared.get() == pivotPoint.get()) ||
        (lineObj && sourceShared.get() == lineObj.get()) ||
        (circleObj && sourceShared.get() == circleObj.get())) {
      continue; // Skip the operator itself
    }

    bool individualSuccess = false;

    // Edge-only transformation for shapes (only if single source, for clarity)
    if (s_transformSourceCount == 1 && (sourceShared->getType() == ObjectType::Rectangle || sourceShared->getType() == ObjectType::RectangleRotatable ||
        sourceShared->getType() == ObjectType::Triangle || sourceShared->getType() == ObjectType::Polygon ||
        sourceShared->getType() == ObjectType::RegularPolygon)) {
      if (tryTransformShapeEdge(editor, tempSelectedObjects, tool, sourceShared, pivotPoint, lineObj, circleObj, nullptr, nullptr)) {
        individualSuccess = true;
        // tryTransformShapeEdge calls clearTransformSelection, so we must exit
        return true; 
      }
    }

    // Point transformation
    auto sourcePoint = std::dynamic_pointer_cast<Point>(sourceShared);
    if (sourcePoint) {
      auto newPoint = createTransformedPoint(editor, sourcePoint, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      if (newPoint) {
        registerTransformPoint(editor, newPoint);
        individualSuccess = true;
      }
    }

    // Line transformation
    if (!individualSuccess && (sourceShared->getType() == ObjectType::Line || sourceShared->getType() == ObjectType::LineSegment)) {
      auto line = std::dynamic_pointer_cast<Line>(sourceShared);
      if (line) {
        auto p1 = line->getStartPointObjectShared();
        auto p2 = line->getEndPointObjectShared();
        auto p1t = createTransformedPoint(editor, p1, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
        auto p2t = createTransformedPoint(editor, p2, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
        if (p1t && p2t) {
          registerTransformPoint(editor, p1t);
          registerTransformPoint(editor, p2t);
          auto newLine = std::make_shared<Line>(p1t, p2t, line->isSegment(), line->getColor());
          newLine->setThickness(line->getThickness());
          newLine->setDependent(true);
          editor.lines.push_back(newLine);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newLine));
          individualSuccess = true;
        }
      }
    }

    // Circle transformation
    if (!individualSuccess && sourceShared->getType() == ObjectType::Circle) {
      auto circle = std::dynamic_pointer_cast<Circle>(sourceShared);
      if (circle) {
        if (tool == ObjectType::ReflectAboutCircle) {
          editor.setGUIMessage("Circle inversion not supported yet.");
        } else {
          auto centerRaw = circle->getCenterPointObject();
          auto centerShared = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(centerRaw));
          auto newCenter = createTransformedPoint(editor, centerShared, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
          if (newCenter) {
            registerTransformPoint(editor, newCenter);
            double newRadius = circle->getRadius();
            if (tool == ObjectType::DilateFromPoint) {
              newRadius = circle->getRadius() * g_transformDilationFactor;
            }
            auto newCircle = std::make_shared<Circle>(newCenter.get(), nullptr, newRadius, circle->getColor());
            newCircle->setDependent(true);
            std::shared_ptr<GeometricObject> genericAux = nullptr;
            if (tool == ObjectType::RotateAroundPoint || tool == ObjectType::DilateFromPoint) genericAux = pivotPoint;
            else if (tool == ObjectType::ReflectAboutLine) genericAux = lineObj;
            else if (tool == ObjectType::ReflectAboutCircle) genericAux = circleObj;
            attachTransformMetadata(sourceShared, newCircle, tool, genericAux, nullptr, nullptr);
            newCircle->updateDependentShape();
            editor.circles.push_back(newCircle);
            editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newCircle));
            individualSuccess = true;
          }
        }
      }
    }

    // Rectangle transformation
    if (!individualSuccess && (sourceShared->getType() == ObjectType::Rectangle || sourceShared->getType() == ObjectType::RectangleRotatable)) {
      auto rect = std::dynamic_pointer_cast<Rectangle>(sourceShared);
      auto V0 = getPointForShapeVertex(editor, rect, 0);
      auto V1 = getPointForShapeVertex(editor, rect, 1);
      auto V2 = getPointForShapeVertex(editor, rect, 2);
      auto V3 = getPointForShapeVertex(editor, rect, 3);
      auto V0t = createTransformedPoint(editor, V0, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      auto V1t = createTransformedPoint(editor, V1, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      auto V2t = createTransformedPoint(editor, V2, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      auto V3t = createTransformedPoint(editor, V3, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      if (V0t && V1t && V2t && V3t) {
        registerTransformPoint(editor, V0t);
        registerTransformPoint(editor, V1t);
        registerTransformPoint(editor, V2t);
        registerTransformPoint(editor, V3t);
        Point_2 p0 = flattenPoint(V0t->getCGALPosition());
        Point_2 p1 = flattenPoint(V1t->getCGALPosition());
        Point_2 p3 = flattenPoint(V3t->getCGALPosition());
        double dx = CGAL::to_double(p1.x() - p0.x());
        double dy = CGAL::to_double(p1.y() - p0.y());
        double hx = CGAL::to_double(p3.x() - p0.x());
        double hy = CGAL::to_double(p3.y() - p0.y());
        double det = dx * hy - dy * hx;
        double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(p0, p3)));
        double h = (det >= 0) ? dist : -dist;
        auto newRect = std::make_shared<Rectangle>(V0t, V1t, h, rect->getColor(), editor.objectIdCounter++);
        newRect->setThickness(rect->getThickness());
        newRect->setDependent(true);
        std::shared_ptr<GeometricObject> auxObj;
        if (tool == ObjectType::ReflectAboutLine) auxObj = lineObj;
        else if (tool == ObjectType::ReflectAboutCircle) auxObj = circleObj;
        else auxObj = pivotPoint;

        attachTransformMetadata(sourceShared, newRect, tool, auxObj, nullptr, nullptr);
        applyRectangleVertexLabels(editor, newRect, {V0t, V1t, V2t, V3t});
        editor.rectangles.push_back(newRect);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newRect));
        individualSuccess = true;
      }
    }

    // Triangle transformation
    if (!individualSuccess && sourceShared->getType() == ObjectType::Triangle) {
      auto tri = std::dynamic_pointer_cast<Triangle>(sourceShared);
      std::vector<std::shared_ptr<Point>> newPts;
      for (size_t k = 0; k < 3; ++k) {
        auto vt = createTransformedPoint(editor, tri->getVertexPoint(k), tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
        if (vt) {
          registerTransformPoint(editor, vt);
          newPts.push_back(vt);
        }
      }
      if (newPts.size() == 3) {
        auto newTri = std::make_shared<Triangle>(newPts[0], newPts[1], newPts[2], tri->getColor(), editor.objectIdCounter++);
        newTri->setThickness(tri->getThickness());
        newTri->setDependent(true);
        std::shared_ptr<GeometricObject> auxObj;
        if (tool == ObjectType::ReflectAboutLine) auxObj = lineObj;
        else if (tool == ObjectType::ReflectAboutCircle) auxObj = circleObj;
        else auxObj = pivotPoint;

        attachTransformMetadata(sourceShared, newTri, tool, auxObj, nullptr, nullptr);
        editor.triangles.push_back(newTri);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newTri));
        individualSuccess = true;
      }
    }

    // Regular Polygon transformation
    if (!individualSuccess && sourceShared->getType() == ObjectType::RegularPolygon) {
      auto rpoly = std::dynamic_pointer_cast<RegularPolygon>(sourceShared);
      auto ct = createTransformedPoint(editor, rpoly->getCenterPoint(), tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      auto vt = createTransformedPoint(editor, rpoly->getFirstVertexPoint(), tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      if (ct && vt) {
        registerTransformPoint(editor, ct);
        registerTransformPoint(editor, vt);
        auto newRPoly = std::make_shared<RegularPolygon>(ct, vt, rpoly->getNumSides(), rpoly->getColor(), editor.objectIdCounter++);
        newRPoly->setThickness(rpoly->getThickness());
        newRPoly->setDependent(true);
        std::shared_ptr<GeometricObject> auxObj;
        if (tool == ObjectType::ReflectAboutLine) auxObj = lineObj;
        else if (tool == ObjectType::ReflectAboutCircle) auxObj = circleObj;
        else auxObj = pivotPoint;

        attachTransformMetadata(sourceShared, newRPoly, tool, auxObj, nullptr, nullptr);
        editor.regularPolygons.push_back(newRPoly);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newRPoly));
        individualSuccess = true;
      }
    }

    // Polygon transformation
    if (!individualSuccess && sourceShared->getType() == ObjectType::Polygon) {
      auto poly = std::dynamic_pointer_cast<Polygon>(sourceShared);
      if (poly) {
        std::vector<std::shared_ptr<Point>> newVerts;
        for (size_t k = 0; k < poly->getVertexCount(); ++k) {
          auto v = poly->getVertexPoint(k);
          auto vt = createTransformedPoint(editor, v, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
          if (vt) {
            registerTransformPoint(editor, vt);
            newVerts.push_back(vt);
          }
        }
        if (newVerts.size() == poly->getVertexCount()) {
          auto newPoly = std::make_shared<Polygon>(newVerts, poly->getColor());
          newPoly->setThickness(poly->getThickness());
          newPoly->setDependent(true);
          std::shared_ptr<GeometricObject> auxObj;
          if (tool == ObjectType::ReflectAboutLine) auxObj = lineObj;
          else if (tool == ObjectType::ReflectAboutCircle) auxObj = circleObj;
          else auxObj = pivotPoint;

          attachTransformMetadata(sourceShared, newPoly, tool, auxObj, nullptr, nullptr);
          editor.polygons.push_back(newPoly);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newPoly));
          individualSuccess = true;
        }
      }
    }

    if (individualSuccess) successCount++;
  }

  if (successCount > 0) {
    editor.setGUIMessage("Transformation completed.");
  } else {
    editor.setGUIMessage("Error: No objects were transformed.");
  }

  clearTransformSelection(tempSelectedObjects);
  return true;
}

// --- Helper: Midpoint & Compass State ---

static void handleMidpointToolClick(GeometryEditor& editor, GeometricObject* clickedObj) {
  if (clickedObj && (clickedObj->getType() == ObjectType::Line || clickedObj->getType() == ObjectType::LineSegment)) {
    auto lineShared = std::dynamic_pointer_cast<Line>(editor.findSharedPtr(clickedObj));
    if (lineShared && lineShared->isValid()) {
      auto midpoint = std::make_shared<Midpoint>(lineShared, Constants::POINT_DEFAULT_COLOR);
      editor.points.push_back(midpoint);
      midpoint->setSelected(true);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(midpoint)));

      std::cout << "Created Dynamic Midpoint on Segment" << std::endl;
      editor.setGUIMessage("Midpoint constructed on segment.");
    }
    return;
  }

  if (clickedObj && (clickedObj->getType() == ObjectType::Point || clickedObj->getType() == ObjectType::ObjectPoint ||
                     clickedObj->getType() == ObjectType::IntersectionPoint)) {
    bool alreadySelected = false;
    for (auto* obj : tempSelectedObjects) {
      if (obj == clickedObj) alreadySelected = true;
    }
    if (alreadySelected) return;

    clickedObj->setSelected(true);
    tempSelectedObjects.push_back(clickedObj);

    if (tempSelectedObjects.size() == 2) {
      auto p1 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[0]));
      auto p2 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(tempSelectedObjects[1]));

      if (p1 && p2) {
        auto midpoint = std::make_shared<Midpoint>(p1, p2, Constants::POINT_DEFAULT_COLOR);
        editor.points.push_back(midpoint);
        midpoint->setSelected(true);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(midpoint)));

        std::cout << "Created Dynamic Midpoint between points" << std::endl;
        editor.setGUIMessage("Midpoint constructed.");
      }

      for (auto* obj : tempSelectedObjects) obj->setSelected(false);
      tempSelectedObjects.clear();
    } else {
      editor.setGUIMessage("Midpoint: Select second point.");
    }
  }
}


void createObjectPointOnLine(GeometryEditor& editor, Line* lineHost, const Point_2& cgalWorldPos) {
  std::cout << "Creating ObjectPoint on line" << std::endl;

  if (!lineHost->getStartPointObject() || !lineHost->getEndPointObject()) {
    std::cerr << "Cannot create ObjectPoint: Line has invalid endpoints" << std::endl;
    return;
  }

  Point_2 startPos, endPos;
  try {
    startPos = lineHost->getStartPoint();
    endPos = lineHost->getEndPoint();

    if (startPos == endPos) {
      std::cerr << "Cannot create ObjectPoint: Line endpoints are coincident" << std::endl;
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error getting line endpoints: " << e.what() << std::endl;
    return;
  }

  double relativePos;
  try {
    relativePos = ProjectionUtils::getRelativePositionOnLine(cgalWorldPos, startPos, endPos, lineHost->isSegment());
    if (lineHost->getType() == ObjectType::Ray && relativePos < 0.0) {
      relativePos = 0.0;
    }
    std::cout << "Calculated relative position: " << relativePos << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error calculating relative position: " << e.what() << std::endl;
    relativePos = 0.5;
  }

  auto hostLineShared = editor.getLineSharedPtr(lineHost);
  if (!hostLineShared) {
    std::cerr << "Error: Could not get shared_ptr for line host" << std::endl;
    return;
  }

  try {
    auto newObjPoint = ObjectPoint::create(hostLineShared, relativePos, Constants::OBJECT_POINT_DEFAULT_COLOR);

    if (newObjPoint && newObjPoint->isValid()) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      std::string label = LabelManager::instance().getNextLabel(editor.getAllPoints());
      newObjPoint->setLabel(label);
      newObjPoint->setShowLabel(true);
      editor.ObjectPoints.push_back(newObjPoint);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newObjPoint)));
      std::cout << "ObjectPoint created successfully at position " << relativePos << std::endl;
      std::cout << "Total ObjectPoints: " << editor.ObjectPoints.size() << std::endl;
      editor.setGUIMessage("ObjectPoint created on line");
    } else {
      std::cerr << "Failed to create valid ObjectPoint" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error creating ObjectPoint: " << e.what() << std::endl;
  }
}

void createObjectPointOnCircle(GeometryEditor& editor, std::shared_ptr<Circle> circlePtr, const Point_2& clickPos) {
  try {
    if (!circlePtr) {
      std::cerr << "ERROR: circlePtr is null" << std::endl;
      return;
    }

    Point_2 center = circlePtr->getCenterPoint();
    Vector_2 vectorToClick = clickPos - center;
    double angleRad = std::atan2(CGAL::to_double(vectorToClick.y()), CGAL::to_double(vectorToClick.x()));

    auto objectPoint = ObjectPoint::create(circlePtr, angleRad, Constants::OBJECT_POINT_DEFAULT_COLOR);

    if (objectPoint) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      std::string label = LabelManager::instance().getNextLabel(editor.getAllPoints());
      objectPoint->setLabel(label);
      objectPoint->setShowLabel(true);
      editor.ObjectPoints.push_back(objectPoint);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(objectPoint)));
      std::cout << "ObjectPoint created successfully on circle" << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "ERROR creating ObjectPoint: " << e.what() << std::endl;
  }
}

static void assignUnifiedLabels(GeometryEditor& editor, const std::vector<std::shared_ptr<Point>>& points) {
  std::vector<std::shared_ptr<Point>> pointsToLabel;
  for (const auto& p : points) {
    if (p && p->getLabel().empty()) {
      pointsToLabel.push_back(p);
    }
  }

  if (pointsToLabel.empty()) return;

  // Gather ALL existing points with labels to avoid collisions in LabelManager (though LabelManager already checks point pool)
  // Just use the enhanced getNextLabels
  auto labels = LabelManager::instance().getNextLabels(pointsToLabel.size(), editor.getAllPoints());
  for (size_t i = 0; i < pointsToLabel.size() && i < labels.size(); ++i) {
    pointsToLabel[i]->setLabel(labels[i]);
    pointsToLabel[i]->setShowLabel(false); // Hide point label, shape will draw it
  }
}

// Helper to get consistent CCW ordering for 4 points
static std::vector<std::shared_ptr<Point>> sortPointsCCW(const std::vector<std::shared_ptr<Point>>& pts) {
  if (pts.size() != 4) return pts;

  // Calculate centroid
  double cx = 0, cy = 0;
  int count = 0;
  for (const auto& p : pts) {
    if (!p) continue;
    Point_2 pos = p->getCGALPosition();
    cx += CGAL::to_double(pos.x());
    cy += CGAL::to_double(pos.y());
    count++;
  }
  if (count == 0) return pts;
  cx /= (double)count;
  cy /= (double)count;

  // Sort by angle around centroid
  // Standard Cartesian: atan2(y, x) gives angle from Positive X axis
  // Increasing angle = Counter-Clockwise
  std::vector<std::pair<double, std::shared_ptr<Point>>> sorted;
  for (const auto& p : pts) {
    if (!p) continue;
    Point_2 pos = p->getCGALPosition();
    double dx = CGAL::to_double(pos.x()) - cx;
    double dy = CGAL::to_double(pos.y()) - cy;
    double angle = std::atan2(dy, dx);
    sorted.push_back({angle, p});
  }

  // Sort: -PI to PI (Increasing angle = CCW)
  std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
  });

  std::vector<std::shared_ptr<Point>> result;
  for(auto& pair : sorted) result.push_back(pair.second);

  // Find "Top Left" vertex to be index 0
  int bestStart = 0;
  double bestScore = -1e18;
  for(int i = 0; i < 4; ++i) {
      Point_2 p = result[i]->getCGALPosition();
      // TL heuristic: Max Y, Min X. (Using Y - X score)
      double score = CGAL::to_double(p.y()) - CGAL::to_double(p.x());
      if(score > bestScore) {
          bestScore = score;
          bestStart = i;
      }
  }

  // Rotate result so bestStart (Top-Left) is at index 0
  std::rotate(result.begin(), result.begin() + bestStart, result.end());
  
  // CRITICAL: The angular sort (atan2 ascending) produced a specific winding.
  // If the user sees Clockwise, we REVERSE the direction of the cycle (keeping TL at 0).
  // TL, P1, P2, P3 -> TL, P3, P2, P1
  if (result.size() == 4) {
      std::swap(result[1], result[3]);
  }
  
  return result;
}

static void applyRectangleVertexLabels(GeometryEditor& editor, const std::shared_ptr<Rectangle>& rect, const std::vector<std::shared_ptr<Point>>& explicitPoints) {
  if (!rect) return;
  auto c1 = rect->getCorner1Point();
  auto c2 = rect->getCorner2Point();
  auto verts = rect->getVertices();
  if (verts.size() != 4) return;

  std::vector<std::shared_ptr<Point>> corners;

  if (!explicitPoints.empty() && explicitPoints.size() == 4) {
      // Use the points provided directly (common for transformations)
      corners = explicitPoints;
  } else {
      // Fallback: Build the list of 4 corner points by finding or creating them
      float tolerance = getDynamicSelectionTolerance(editor);
      auto findOrCreatePoint = [&](const Point_2& pos) -> std::shared_ptr<Point> {
          // 1. Look for existing point at this location
          sf::Vector2f worldPos = editor.toSFMLVector(pos);
          GeometricObject* hit = editor.lookForObjectAt(worldPos, tolerance, 
              {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});
          
          if (hit) {
              auto p = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hit));
              if (p) {
                  p->setDependent(true);
                  p->setCreatedWithShape(true);
                  return p;
              }
          }

          // 2. Create new if none found
          auto p = editor.createPoint(pos);
          if (!p) return nullptr;
          p->setShowLabel(false);
          p->setVisible(true);
          p->setDependent(true);
          p->setCreatedWithShape(true);
          return p;
      };

      for (const auto& vPos : verts) {
          if (CGAL::squared_distance(vPos, c1->getCGALPosition()) < 1e-6) {
              corners.push_back(c1);
          } else if (CGAL::squared_distance(vPos, c2->getCGALPosition()) < 1e-6) {
              corners.push_back(c2);
          } else {
              corners.push_back(findOrCreatePoint(vPos));
          }
      }
  }

  if (corners.size() != 4) return;

  // Identify pB and pD for the Rectangle's internal state
  std::vector<std::shared_ptr<Point>> dependents;
  for (auto& p : corners) {
      if (p != c1 && p != c2) dependents.push_back(p);
  }
  if (dependents.size() >= 2) {
      rect->setDependentCornerPoints(dependents[0], dependents[1]);
  }

  // --- LABELING LOGIC ---
  bool isTransform = (rect->getTransformType() != TransformationType::None);
  if (isTransform) {
      // For transformations, we WANT to respect the mapping from source to target.
      // createTransformedPoint already assigned A -> A', B -> B', etc.
      // We just need to ensure they are hidden (shape handles them) and have 4.
      std::vector<std::string> existing;
      for (auto& p : corners) {
          if (p) {
              p->setShowLabel(false);
              if (!p->getLabel().empty()) existing.push_back(p->getLabel());
          }
      }
      
      // If any labels are missing (e.g. source was unlabeled), fill them CCW
      if (existing.size() < 4) {
          auto sorted = sortPointsCCW(corners);
          for (auto& p : sorted) {
              if (p && p->getLabel().empty()) {
                  p->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
                  p->setShowLabel(false);
              }
          }
      }
      // DONE for transforms. Correspondence is maintained.
  } else {
      // Fresh Rectangle: Force A, B, C, D in CCW order from Top-Left
      auto sorted = sortPointsCCW(corners);
      
      // Clear temporary labels assigned during point creation so getNextLabels can reuse them
      for (auto& p : sorted) {
          if (p) p->setLabel(""); 
      }
      
      std::vector<std::string> labelsToUse = LabelManager::instance().getNextLabels(4, editor.getAllPoints());

      // Assign the labels in the CCW sorted order
      for (size_t i = 0; i < sorted.size() && i < 4 && i < labelsToUse.size(); ++i) {
          if (sorted[i]) {
              sorted[i]->setLabel(labelsToUse[i]);
              sorted[i]->setShowLabel(false); 
          }
      }
  }
}

void handlePointCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

  bool isAltPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) || sf::Keyboard::isKeyPressed(sf::Keyboard::RAlt);

  if (editor.m_snapState.kind != PointUtils::SnapState::Kind::None &&
      (isAltPressed || editor.m_snapState.kind != PointUtils::SnapState::Kind::ExistingPoint)) {
    cgalWorldPos = editor.m_snapState.position;
    worldPos_sfml = editor.toSFMLVector(cgalWorldPos);
    std::cout << "[SNAP] Using snapped position for point creation" << std::endl;
  }

  float selectionTolerance = getDynamicSelectionTolerance(editor);

  size_t prePointsCount = editor.points.size();
  size_t preObjPointsCount = editor.ObjectPoints.size();
  auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, selectionTolerance);
  bool createdNew = (editor.points.size() > prePointsCount) || (editor.ObjectPoints.size() > preObjPointsCount);
  bool isExistingPoint = false;
  auto itExisting = std::find(editor.points.begin(), editor.points.end(), smartPoint);
  if (itExisting != editor.points.end()) {
    isExistingPoint = true;
  }
  for (const auto& op : editor.ObjectPoints) {
    if (op == smartPoint) {
      isExistingPoint = true;
      break;
    }
  }

  if (isAltPressed && smartPoint && isExistingPoint && !createdNew) {
    auto newPoint = editor.createPoint(smartPoint->getCGALPosition());
    newPoint->setSelected(true);
    editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoint)));
    std::cout << "ALT+Click: Created point at snapped existing point." << std::endl;
    return;
  }

  if (!isAltPressed && smartPoint && isExistingPoint && !createdNew) {
    if (auto edgeHit = PointUtils::findNearestEdge(editor, worldPos_sfml, selectionTolerance)) {
      if (edgeHit->host) {
        if (auto circle = dynamic_cast<Circle*>(edgeHit->host)) {
          auto circlePtr = std::dynamic_pointer_cast<Circle>(editor.findSharedPtr(circle));
          if (circlePtr) {
            auto objPoint = ObjectPoint::create(circlePtr, edgeHit->relativePosition * 2.0 * M_PI, Constants::OBJECT_POINT_DEFAULT_COLOR);
            if (objPoint && objPoint->isValid()) {
              editor.ObjectPoints.push_back(objPoint);
              smartPoint = objPoint;
              createdNew = true;
            }
          }
        } else {
          auto hostPtr = editor.findSharedPtr(edgeHit->host);
          if (hostPtr) {
            auto objPoint = ObjectPoint::createOnShapeEdge(hostPtr, edgeHit->edgeIndex, edgeHit->relativePosition);
            if (objPoint && objPoint->isValid()) {
              objPoint->setIsVertexAnchor(false);
              editor.ObjectPoints.push_back(objPoint);
              smartPoint = objPoint;
              createdNew = true;
            }
          }
        }
      }
    }

    if (smartPoint && isExistingPoint && smartPoint == *itExisting) {
      Point_2 cursor = editor.toCGALPoint(worldPos_sfml);
      double bestDist = static_cast<double>(selectionTolerance);
      std::shared_ptr<Line> bestLine = nullptr;
      double bestRel = 0.0;
      for (auto& linePtr : editor.lines) {
        if (!linePtr || !linePtr->isValid()) continue;
        if (!linePtr->contains(worldPos_sfml, selectionTolerance)) continue;
        try {
          Point_2 startP = linePtr->getStartPoint();
          Point_2 endP = linePtr->getEndPoint();
          Line_2 hostLine = linePtr->getCGALLine();
          Point_2 proj = hostLine.projection(cursor);
          double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(proj, cursor)));
          if (dist < bestDist) {
            bestDist = dist;
            bestLine = linePtr;
            Vector_2 lineVec = endP - startP;
            double lineLenSq = CGAL::to_double(lineVec.squared_length());
            double t = 0.5;
            if (lineLenSq > 0.0) {
              Vector_2 toProj = proj - startP;
              t = CGAL::to_double(toProj * lineVec) / lineLenSq;
              if (linePtr->isSegment()) {
                t = std::max(0.0, std::min(1.0, t));
              }
            }
            bestRel = t;
          }
        } catch (...) {
        }
      }
      if (bestLine) {
        auto objPoint = ObjectPoint::create(bestLine, bestRel, Constants::OBJECT_POINT_DEFAULT_COLOR);
        if (objPoint && objPoint->isValid()) {
          editor.ObjectPoints.push_back(objPoint);
          smartPoint = objPoint;
          createdNew = true;
        }
      }
    }

    if (smartPoint && isExistingPoint && smartPoint == *itExisting) {
      smartPoint = editor.createPoint(editor.toCGALPoint(worldPos_sfml));
      createdNew = true;
    }
  }
  if (smartPoint && smartPoint->isValid()) {
    bool addedPoint = false;
    auto it = std::find(editor.points.begin(), editor.points.end(), smartPoint);

    bool isObjectPoint = false;
    for (const auto& op : editor.ObjectPoints) {
      if (op == smartPoint) {
        isObjectPoint = true;
        break;
      }
    }

    if (it == editor.points.end() && !isObjectPoint) {
      editor.points.push_back(smartPoint);
      addedPoint = true;
    }

    if (addedPoint || smartPoint->getLabel().empty()) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      std::string label = LabelManager::instance().getNextLabel(editor.getAllPoints());
      smartPoint->setLabel(label);
      smartPoint->setShowLabel(true);
    }

    // Only auto-select free points, not object points, per user request to avoid persistent highlighting
    if (!isObjectPoint) {
      smartPoint->setSelected(true);
      editor.selectedObject = smartPoint.get();
    }
    if (createdNew) {
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(smartPoint)));
    }
  }

  std::cout << "Point created at (" << cgalWorldPos.x() << ", " << cgalWorldPos.y() << ")" << std::endl;
}

void handleLineCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

    const float MAX_COORD = 1e25f;
    if (!std::isfinite(worldPos_sfml.x) || !std::isfinite(worldPos_sfml.y) || std::abs(worldPos_sfml.x) > MAX_COORD ||
        std::abs(worldPos_sfml.y) > MAX_COORD) {
      std::cerr << "handleLineCreation: Mouse coordinates out of bounds" << std::endl;
      return;
    }

    Point_2 cgalWorldPos;
    try {
      cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

      if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
        std::cerr << "handleLineCreation: CGAL point conversion failed" << std::endl;
        return;
      }

      double x_double = CGAL::to_double(cgalWorldPos.x());
      double y_double = CGAL::to_double(cgalWorldPos.y());

      if (!std::isfinite(x_double) || !std::isfinite(y_double) || std::abs(x_double) > MAX_COORD || std::abs(y_double) > MAX_COORD) {
        std::cerr << "handleLineCreation: CGAL coordinates out of reasonable range" << std::endl;
        return;
      }
    } catch (const std::exception& e) {
      std::cerr << "Error converting position to CGAL: " << e.what() << std::endl;
      return;
    }

    float tolerance = getDynamicSnapTolerance(editor);

    std::cout << "Line-like creation: Click at (" << CGAL::to_double(cgalWorldPos.x()) << ", " << CGAL::to_double(cgalWorldPos.y()) << ")" << std::endl;

    std::shared_ptr<Point> clickedPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
    if (!editor.lineCreationPoint1) {
      deselectAllAndClearInteractionState(editor);

      editor.lineCreationPoint1 = clickedPoint;

      if (!editor.lineCreationPoint1 || !editor.lineCreationPoint1->isValid()) {
        std::cerr << "Error: First point for line is invalid after creation/selection." << std::endl;
        editor.lineCreationPoint1 = nullptr;
        return;
      }

      editor.lineCreationPoint1->setSelected(true);
      editor.dragMode = DragMode::CreateLineP1;

      editor.previewLineOverlay.setPrimitiveType(sf::Lines);
      editor.previewLineOverlay.resize(2);
      editor.previewLineOverlay[0].position = editor.toSFMLVector(editor.lineCreationPoint1->getCGALPosition());
      editor.previewLineOverlay[1].position = worldPos_sfml;
      sf::Color previewColor(150, 150, 150, 150);
      editor.previewLineOverlay[0].color = previewColor;
      editor.previewLineOverlay[1].color = previewColor;
      editor.hasPreviewLineOverlay = true;

      std::cout << "First point selected for line creation." << std::endl;

    } else {
      if (!editor.lineCreationPoint1->isValid()) {
        std::cerr << "Error: First point for line creation is no longer valid." << std::endl;
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        editor.hasPreviewLineOverlay = false;
        return;
      }

      std::shared_ptr<Point> secondPoint = clickedPoint;

      if (secondPoint == editor.lineCreationPoint1) {
        std::cout << "Line creation canceled: same point selected for start and end." << std::endl;
        editor.lineCreationPoint1->setSelected(false);
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        editor.hasPreviewLineOverlay = false;
        return;
      }

      if (!editor.lineCreationPoint1 || !editor.lineCreationPoint1->isValid() || !secondPoint || !secondPoint->isValid()) {
        std::cerr << "Error: One or both points for line creation are invalid" << std::endl;
        if (editor.lineCreationPoint1) {
          editor.lineCreationPoint1->setSelected(false);
        }
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        editor.hasPreviewLineOverlay = false;
        return;
      }

      try {
        Point_2 p1_pos_final = editor.lineCreationPoint1->getCGALPosition();
        Point_2 p2_pos_final = secondPoint->getCGALPosition();

        if (!CGAL::is_finite(p1_pos_final.x()) || !CGAL::is_finite(p1_pos_final.y()) || !CGAL::is_finite(p2_pos_final.x()) ||
            !CGAL::is_finite(p2_pos_final.y())) {
          std::cerr << "Error: Line endpoint coordinates are not finite before creation" << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          editor.hasPreviewLineOverlay = false;
          return;
        }

        double p1_x_final, p1_y_final, p2_x_final, p2_y_final;
        try {
          p1_x_final = CGAL::to_double(p1_pos_final.x());
          p1_y_final = CGAL::to_double(p1_pos_final.y());
          p2_x_final = CGAL::to_double(p2_pos_final.x());
          p2_y_final = CGAL::to_double(p2_pos_final.y());
        } catch (const std::exception& e) {
          std::cerr << "Error: Cannot convert line endpoint coordinates to double: " << e.what() << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          editor.hasPreviewLineOverlay = false;
          return;
        }

        if (!std::isfinite(p1_x_final) || !std::isfinite(p1_y_final) || !std::isfinite(p2_x_final) || !std::isfinite(p2_y_final)) {
          std::cerr << "Error: Line endpoint coordinates are not finite doubles" << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          editor.hasPreviewLineOverlay = false;
          return;
        }

        const double MAX_COORD_SAFE = 1e15;
        if (std::abs(p1_x_final) > MAX_COORD_SAFE || std::abs(p1_y_final) > MAX_COORD_SAFE || std::abs(p2_x_final) > MAX_COORD_SAFE ||
            std::abs(p2_y_final) > MAX_COORD_SAFE) {
          std::cerr << "Error: Line coordinates exceed safe range" << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          editor.hasPreviewLineOverlay = false;
          return;
        }

        double dx_final = p2_x_final - p1_x_final;
        double dy_final = p2_y_final - p1_y_final;
        double distSquared_final = dx_final * dx_final + dy_final * dy_final;

        const double minDistSquared = Constants::MIN_DISTANCE_SQUARED_LINE_CREATION;

        if (!std::isfinite(distSquared_final) || distSquared_final < minDistSquared) {
          std::cerr << "Error: Final line endpoints are too close (distance² = " << distSquared_final << ", min required = " << minDistSquared << ")"
                    << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          editor.hasPreviewLineOverlay = false;
          return;
        }

        bool isSegment = (editor.m_currentToolType == ObjectType::LineSegment || editor.m_currentToolType == ObjectType::Vector);
        
        auto newLine = std::make_shared<Line>(editor.lineCreationPoint1, secondPoint, isSegment, editor.getCurrentColor(), editor.objectIdCounter++);
        
        // Automatic label assignment (a, b, c...)
        if (editor.showGlobalLabels) {
          newLine->setLabel(LabelManager::instance().getNextLineLabel(editor.getAllObjects()));
          newLine->setLabelMode(LabelMode::Name);
        }
        
        if (editor.m_currentToolType == ObjectType::Ray) {
           newLine->setLineType(Line::LineType::Ray);
        } else if (editor.m_currentToolType == ObjectType::Vector) {
           newLine->setLineType(Line::LineType::Vector);
        }

        if (!newLine || !newLine->isValid()) {
          std::cerr << "Error: Failed to create valid geometric line object." << std::endl;
        } else {
          newLine->registerWithEndpoints();
          newLine->setThickness(editor.currentThickness);
          editor.lines.push_back(newLine);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));
          std::cout << "Line-like object created successfully." << std::endl;
        }

      } catch (const std::exception& e) {
        std::cerr << "Error during final line validation/creation: " << e.what() << std::endl;
        editor.lineCreationPoint1->setSelected(false);
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        editor.hasPreviewLineOverlay = false;
        return;
      }

      editor.lineCreationPoint1->setSelected(false);
      editor.lineCreationPoint1 = nullptr;
      editor.dragMode = DragMode::None;
      editor.hasPreviewLineOverlay = false;
    }

  } catch (const std::exception& e) {
    std::cerr << "Critical error in handleLineCreation: " << e.what() << std::endl;
    if (editor.lineCreationPoint1) {
      try {
        editor.lineCreationPoint1->setSelected(false);
      } catch (...) {
      }
    }
    editor.lineCreationPoint1 = nullptr;
    editor.dragMode = DragMode::None;
  }
}

void handleLineSegmentCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) { handleLineCreation(editor, mouseEvent); }
// --- Ray Tool Handler ---
static void handleRayCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  handleLineCreation(editor, mouseEvent);
}

void handleParallelLineCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos;
  try {
    cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
    if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
      std::cerr << "ParallelLineCreation: Initial cgalWorldPos from click is not finite." << std::endl;
      editor.resetParallelLineToolState();
      clearTemporarySelection(tempSelectedObjects);
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error converting click to CGAL point in ParallelLineCreation: " << e.what() << std::endl;
    editor.resetParallelLineToolState();
    clearTemporarySelection(tempSelectedObjects);
    return;
  }
  float tolerance = getDynamicSelectionTolerance(editor);

  try {
    if (!editor.m_isPlacingParallel) {
      editor.resetParallelLineToolState();

      // STEP 1: SELECT REFERENCE LINE - Strict Filter (only lines, no points)
      GeometricObject* refObj = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Line, ObjectType::LineSegment, ObjectType::Ray});

      if (refObj) {
        std::shared_ptr<GeometricObject> refObjSP;
        int edgeIndex = -1;
        Vector_2 refDirection;
        bool validReferenceFound = false;

        if (refObj->getType() == ObjectType::Line || refObj->getType() == ObjectType::LineSegment || refObj->getType() == ObjectType::Ray) {
          Line* rawLinePtr = static_cast<Line*>(refObj);
          refObjSP = editor.getLineSharedPtr(rawLinePtr);
          if (refObjSP) {
            if (auto linePtr = std::dynamic_pointer_cast<Line>(refObjSP)) {
              if (linePtr->isValid()) {
                Point_2 p1 = linePtr->getStartPoint();
                Point_2 p2 = linePtr->getEndPoint();
                if (p1 != p2) {
                  refDirection = Vector_2(p2.x() - p1.x(), p2.y() - p1.y());
                  validReferenceFound = true;
                }
              }
            }
          }
        } else {
          std::vector<Segment_2> edges = refObj->getEdges();
          double minDistance = std::numeric_limits<double>::max();
          int bestEdgeIndex = -1;

          for (size_t i = 0; i < edges.size(); ++i) {
            double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(edges[i], cgalWorldPos)));
            if (dist < minDistance) {
              minDistance = dist;
              bestEdgeIndex = static_cast<int>(i);
            }
          }

          if (bestEdgeIndex != -1) {
            if (refObj->getType() == ObjectType::Rectangle) {
              for (auto& r : editor.rectangles)
                if (r.get() == refObj) refObjSP = r;
            } else if (refObj->getType() == ObjectType::Triangle) {
              for (auto& t : editor.triangles)
                if (t.get() == refObj) refObjSP = t;
            } else if (refObj->getType() == ObjectType::Polygon) {
              for (auto& p : editor.polygons)
                if (p.get() == refObj) refObjSP = p;
            } else if (refObj->getType() == ObjectType::RegularPolygon) {
              for (auto& rp : editor.regularPolygons)
                if (rp.get() == refObj) refObjSP = rp;
            }

            if (refObjSP) {
              refDirection = edges[bestEdgeIndex].to_vector();
              edgeIndex = bestEdgeIndex;
              validReferenceFound = true;
            }
          }
        }

        if (validReferenceFound) {
          if (refDirection.squared_length() < Kernel::FT(Constants::CGAL_EPSILON_SQUARED)) {
            editor.setGUIMessage("Parallel: Edge too short.");
            editor.resetParallelLineToolState();
            clearTemporarySelection(tempSelectedObjects);
            return;
          }

          editor.m_parallelReference.object = refObjSP;
          editor.m_parallelReference.edgeIndex = edgeIndex;
          editor.m_parallelReferenceDirection = refDirection;

          editor.m_isPlacingParallel = true;
          editor.setGUIMessage("Parallel: Ref selected. Click to place line.");
          return;
        }
      }

      bool isHorizontalAxis = false;
      bool isVerticalAxis = false;
      if (std::abs(worldPos_sfml.y) <= tolerance)
        isHorizontalAxis = true;
      else if (std::abs(worldPos_sfml.x) <= tolerance)
        isVerticalAxis = true;

      if (isHorizontalAxis) {
        editor.m_parallelReference.reset();
        editor.m_parallelReferenceDirection = Vector_2(1, 0);
        editor.m_isPlacingParallel = true;
        editor.setGUIMessage("Parallel: X-Axis selected. Click to place line.");
      } else if (isVerticalAxis) {
        editor.m_parallelReference.reset();
        editor.m_parallelReferenceDirection = Vector_2(0, 1);
        editor.m_isPlacingParallel = true;
        editor.setGUIMessage("Parallel: Y-Axis selected. Click to place line.");
      } else {
        editor.setGUIMessage("Parallel: Click on a line, shape edge, or axis to select reference.");
      }

    } else {
      if (editor.m_parallelReferenceDirection == Vector_2(0, 0)) {
        std::cerr << "CRITICAL_ERROR (Parallel): No reference direction set for placement." << std::endl;
        editor.resetParallelLineToolState();
        clearTemporarySelection(tempSelectedObjects);
        return;
      }

      // --- Standardized Point Selection (Step 2) ---
      Point_2 anchorPos_cgal = cgalWorldPos;
      std::shared_ptr<Point> clickedExistingPt = nullptr;

      std::shared_ptr<Point> finalPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
      if (!finalPoint) {
        editor.resetParallelLineToolState();
        clearTemporarySelection(tempSelectedObjects);
        return;
      }

      clickedExistingPt = finalPoint;
      anchorPos_cgal = clickedExistingPt->getCGALPosition();

      Vector_2 unit_construction_dir;
      try {
        unit_construction_dir = CGALSafeUtils::normalize_vector_robust(editor.m_parallelReferenceDirection, "ParallelCreate_normalize_ref_dir");
      } catch (const std::runtime_error& e) {
        std::cerr << "CRITICAL (Parallel): Normalizing dir: " << e.what() << std::endl;
        editor.resetParallelLineToolState();
        clearTemporarySelection(tempSelectedObjects);
        return;
      }
      Kernel::FT constructionLength_ft = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);

      // Proceed to create P2 and Line
      std::shared_ptr<Point> finalStartPoint = clickedExistingPt;
      Point_2 p2_final_pos = finalStartPoint->getCGALPosition() + (unit_construction_dir * constructionLength_ft);
      std::shared_ptr<Point> finalEndPoint = nullptr;

      std::shared_ptr<Point> newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);
      if (newP2 && newP2->isValid()) {
        newP2->setVisible(false);
        editor.points.push_back(newP2);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP2)));
        finalEndPoint = newP2;
      }

      if (finalStartPoint && finalEndPoint && finalStartPoint->isValid() && finalEndPoint->isValid()) {
        auto newLine = std::make_shared<Line>(finalStartPoint, finalEndPoint, false, Constants::CONSTRUCTION_LINE_COLOR, editor.objectIdCounter++);
        if (newLine && newLine->isValid()) {
          newLine->registerWithEndpoints();
          newLine->setThickness(editor.currentThickness);
          editor.lines.push_back(newLine);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));

          if (auto refObj = editor.m_parallelReference.lock()) {
            newLine->setAsParallelLine(refObj, editor.m_parallelReference.edgeIndex, editor.m_parallelReferenceDirection);
          } else {
            newLine->setAsParallelLine(nullptr, -1, editor.m_parallelReferenceDirection);
            std::cout << "Parallel line created relative to axis" << std::endl;
          }

          editor.setGUIMessage("Parallel: Line placed. Select new reference.");
        } else {
          std::cerr << "Failed to create valid parallel line object." << std::endl;
        }
      }
      editor.resetParallelLineToolState();
      clearTemporarySelection(tempSelectedObjects);
    }
  } catch (const std::exception& e) {
    std::cerr << "CRITICAL EXCEPTION in handleParallelLineCreation: " << e.what() << std::endl;
    editor.resetParallelLineToolState();
    clearTemporarySelection(tempSelectedObjects);
  }
}

void handlePerpendicularLineCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos;
  try {
    cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
    if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
      std::cerr << "PerpLineCreation: Initial cgalWorldPos from click is not finite." << std::endl;
      editor.resetPerpendicularLineToolState();
      clearTemporarySelection(tempSelectedObjects);
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error converting click to CGAL point in PerpendicularLineCreation: " << e.what() << std::endl;
    editor.resetPerpendicularLineToolState();
    clearTemporarySelection(tempSelectedObjects);
    return;
  }
  float tolerance = getDynamicSelectionTolerance(editor);

  try {
    if (!editor.m_isPlacingPerpendicular) {
      editor.resetPerpendicularLineToolState();

      // STEP 1: SELECT REFERENCE LINE - Strict Filter (only lines, no points)
      GeometricObject* refObj = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Line, ObjectType::LineSegment, ObjectType::Ray});

      if (refObj) {
        std::shared_ptr<GeometricObject> refObjSP;
        int edgeIndex = -1;
        Vector_2 refDirection;
        bool validReferenceFound = false;

        if (refObj->getType() == ObjectType::Line || refObj->getType() == ObjectType::LineSegment || refObj->getType() == ObjectType::Ray) {
          Line* rawLinePtr = static_cast<Line*>(refObj);
          refObjSP = editor.getLineSharedPtr(rawLinePtr);
          if (refObjSP) {
            if (auto linePtr = std::dynamic_pointer_cast<Line>(refObjSP)) {
              if (linePtr->isValid()) {
                Point_2 p1 = linePtr->getStartPoint();
                Point_2 p2 = linePtr->getEndPoint();
                if (p1 != p2) {
                  refDirection = Vector_2(p2.x() - p1.x(), p2.y() - p1.y());
                  validReferenceFound = true;
                }
              }
            }
          }
        } else {
          std::vector<Segment_2> edges = refObj->getEdges();
          double minDistance = std::numeric_limits<double>::max();
          int bestEdgeIndex = -1;

          for (size_t i = 0; i < edges.size(); ++i) {
            double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(edges[i], cgalWorldPos)));
            if (dist < minDistance) {
              minDistance = dist;
              bestEdgeIndex = static_cast<int>(i);
            }
          }

          if (bestEdgeIndex != -1) {
            if (refObj->getType() == ObjectType::Rectangle) {
              for (auto& r : editor.rectangles)
                if (r.get() == refObj) refObjSP = r;
            } else if (refObj->getType() == ObjectType::Triangle) {
              for (auto& t : editor.triangles)
                if (t.get() == refObj) refObjSP = t;
            } else if (refObj->getType() == ObjectType::Polygon) {
              for (auto& p : editor.polygons)
                if (p.get() == refObj) refObjSP = p;
            } else if (refObj->getType() == ObjectType::RegularPolygon) {
              for (auto& rp : editor.regularPolygons)
                if (rp.get() == refObj) refObjSP = rp;
            }

            if (refObjSP) {
              refDirection = edges[bestEdgeIndex].to_vector();
              edgeIndex = bestEdgeIndex;
              validReferenceFound = true;
            }
          }
        }

        if (validReferenceFound) {
          if (refDirection.squared_length() < Kernel::FT(Constants::CGAL_EPSILON_SQUARED)) {
            editor.setGUIMessage("Perp: Edge too short.");
            editor.resetPerpendicularLineToolState();
            clearTemporarySelection(tempSelectedObjects);
            return;
          }

          editor.m_perpendicularReference.object = refObjSP;
          editor.m_perpendicularReference.edgeIndex = edgeIndex;
          editor.m_perpendicularReferenceDirection = refDirection;

          editor.m_isPlacingPerpendicular = true;
          editor.setGUIMessage("Perp: Ref selected. Click to place line.");
          return;
        }
      }

      bool isHorizontalAxis = false;
      bool isVerticalAxis = false;
      if (std::abs(worldPos_sfml.y) <= tolerance)
        isHorizontalAxis = true;
      else if (std::abs(worldPos_sfml.x) <= tolerance)
        isVerticalAxis = true;

      if (isHorizontalAxis) {
        editor.m_perpendicularReference.reset();
        editor.m_perpendicularReferenceDirection = Vector_2(1, 0);
        editor.m_isPlacingPerpendicular = true;
        editor.setGUIMessage("Perp: X-Axis selected. Click to place line.");
      } else if (isVerticalAxis) {
        editor.m_perpendicularReference.reset();
        editor.m_perpendicularReferenceDirection = Vector_2(0, 1);
        editor.m_isPlacingPerpendicular = true;
        editor.setGUIMessage("Perp: Y-Axis selected. Click to place line.");
      } else {
        editor.setGUIMessage("Select a line or edge to invoke Perpendicular Tool.");
      }
    } else {
      if (editor.m_perpendicularReferenceDirection == Vector_2(0, 0)) {
        std::cerr << "CRITICAL_ERROR (Perp): No reference direction set for placement." << std::endl;
        editor.resetPerpendicularLineToolState();
        clearTemporarySelection(tempSelectedObjects);
        return;
      }

      // --- Standardized Point Selection (Step 2) ---
      Point_2 anchorPos_cgal = cgalWorldPos;
      std::shared_ptr<Point> clickedExistingPt = nullptr;

      std::shared_ptr<Point> finalPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
      if (!finalPoint) {
        editor.resetPerpendicularLineToolState();
        clearTemporarySelection(tempSelectedObjects);
        return;
      }

      clickedExistingPt = finalPoint;
      anchorPos_cgal = clickedExistingPt->getCGALPosition();

      Vector_2 perp_to_ref_dir(-editor.m_perpendicularReferenceDirection.y(), editor.m_perpendicularReferenceDirection.x());

      Vector_2 unit_construction_dir;
      try {
        unit_construction_dir = CGALSafeUtils::normalize_vector_robust(perp_to_ref_dir, "PerpCreate_normalize_perp_dir");
      } catch (const std::runtime_error& e) {
        std::cerr << "CRITICAL_ERROR (Perp): Normalizing construction direction: " << e.what() << std::endl;
        editor.resetPerpendicularLineToolState();
        clearTemporarySelection(tempSelectedObjects);
        return;
      }

      Kernel::FT constructionLength_ft = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
      Point_2 p2_final_pos = anchorPos_cgal + (unit_construction_dir * constructionLength_ft);

      std::shared_ptr<Point> finalStartPoint = clickedExistingPt;
      std::shared_ptr<Point> finalEndPoint = nullptr;

      std::shared_ptr<Point> newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);
      if (newP2 && newP2->isValid()) {
        newP2->setVisible(false);
        editor.points.push_back(newP2);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP2)));
        finalEndPoint = newP2;
      }

      if (finalStartPoint && finalEndPoint && finalStartPoint->isValid() && finalEndPoint->isValid()) {
        auto newLine = std::make_shared<Line>(finalStartPoint, finalEndPoint, false, Constants::CONSTRUCTION_LINE_COLOR, editor.objectIdCounter++);
        if (newLine && newLine->isValid()) {
          newLine->registerWithEndpoints();
          newLine->setThickness(editor.currentThickness);
          editor.lines.push_back(newLine);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));

          if (auto refObj = editor.m_perpendicularReference.lock()) {
            newLine->setAsPerpendicularLine(refObj, editor.m_perpendicularReference.edgeIndex, editor.m_perpendicularReferenceDirection);
          } else {
            newLine->setAsPerpendicularLine(nullptr, -1, editor.m_perpendicularReferenceDirection);
            std::cout << "Perpendicular line created relative to axis" << std::endl;
          }

          editor.setGUIMessage("Perp: Line placed. Select new reference.");
        } else {
          std::cerr << "Failed to create valid perpendicular line object." << std::endl;
        }
      }
      editor.resetPerpendicularLineToolState();
      clearTemporarySelection(tempSelectedObjects);
    }
  } catch (const std::exception& e) {
    std::cerr << "CRITICAL EXCEPTION in handlePerpendicularLineCreation: " << e.what() << std::endl;
    editor.resetPerpendicularLineToolState();
    clearTemporarySelection(tempSelectedObjects);
  }
}

void handlePerpendicularBisectorCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), editor.drawingView);
  float tolerance = getDynamicSelectionTolerance(editor);

  auto resetState = [&]() {
    editor.isCreatingPerpendicularBisector = false;
    editor.perpBisectorP1 = nullptr;
    editor.perpBisectorP2 = nullptr;
    editor.perpBisectorLineRef = nullptr;
  };

  auto createBisectorFromPoints = [&](std::shared_ptr<Point> a, std::shared_ptr<Point> b) {
    if (!a || !b) {
      editor.setGUIMessage("Error: Missing points for bisector.");
      resetState();
      return;
    }

    Vector_2 v = b->getCGALPosition() - a->getCGALPosition();
    double lenSq = CGAL::to_double(v.squared_length());
    if (lenSq < 1e-12) {
      editor.setGUIMessage("Error: Cannot build bisector of zero-length segment.");
      resetState();
      return;
    }

    auto bisector = std::make_shared<PerpendicularBisector>(a, b, editor.objectIdCounter++);
    if (bisector && bisector->isValid()) {
      bisector->setThickness(editor.currentThickness);
      editor.lines.push_back(bisector);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(bisector)));
      editor.setGUIMessage("Perpendicular bisector created.");
    } else {
      editor.setGUIMessage("Error: Failed to create bisector line.");
    }
    resetState();
  };

  if (!editor.isCreatingPerpendicularBisector) {
    // 1. PRIORITY CHECK: Is there a specific POINT under the mouse?
    // We must check this FIRST to avoid the Line "hijacking" the click at endpoints.
    GeometricObject* hitPt = editor.lookForObjectAt(worldPos_sfml, tolerance, 
        {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});

    if (hitPt) {
        // Found a point - Start Two-Point Mode
        editor.perpBisectorP1 = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(hitPt));
        editor.isCreatingPerpendicularBisector = true;
        editor.setGUIMessage("Select second point for bisector.");
        return;
    }

    // 2. SECONDARY CHECK: Is there a LINE? (Clicked "Empty" part of line)
    GeometricObject* hitLine = editor.lookForObjectAt(worldPos_sfml, tolerance, 
        {ObjectType::Line, ObjectType::LineSegment});
    
    if (hitLine) {
        // Found a line body - Immediate Creation
        auto* lineRaw = static_cast<Line*>(hitLine);
        createBisectorFromPoints(lineRaw->getStartPointObjectShared(), lineRaw->getEndPointObjectShared());
        return;
    }

    // 3. FALLBACK: Create/Snap Smart Point (Edges, Intersections, Free)
    // Use the unified function we fixed earlier
    auto smartPt = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
    if (smartPt && smartPt->isValid()) {
        editor.perpBisectorP1 = smartPt;
        editor.isCreatingPerpendicularBisector = true;
        editor.setGUIMessage("Select second point for bisector.");
    } else {
        editor.setGUIMessage("Select a point or segment for bisector.");
    }
    return;
  }

  // STEP 2: Second Point Selection
  auto second = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
  if (!second || !second->isValid()) {
    editor.setGUIMessage("Select a valid second point.");
    // Do not reset state; allow retry
    return;
  }
  
  // Prevent picking the same point twice
  if (second == editor.perpBisectorP1) return;

  editor.perpBisectorP2 = second;
  createBisectorFromPoints(editor.perpBisectorP1, editor.perpBisectorP2);
}

void handleAngleBisectorCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), editor.drawingView);
  float tolerance = getDynamicSelectionTolerance(editor);

  auto resetState = [&]() {
    editor.isCreatingAngleBisector = false;
    editor.angleBisectorPoints.clear();
    editor.angleBisectorLine1 = nullptr;
    editor.angleBisectorLine2 = nullptr;
  };

  // 1. PRIORITY CHECK: Points
  GeometricObject* hitPt = editor.lookForObjectAt(worldPos_sfml, tolerance, 
      {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::IntersectionPoint});

  // 2. SECONDARY CHECK: Lines (Only if NO point found)
  GeometricObject* obj = nullptr;
  if (!hitPt) {
      obj = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Line, ObjectType::LineSegment});
  }

  if (!editor.isCreatingAngleBisector && obj && (obj->getType() == ObjectType::Line || obj->getType() == ObjectType::LineSegment)) {
    editor.angleBisectorLine1 = editor.getLineSharedPtr(static_cast<Line*>(obj));
    editor.isCreatingAngleBisector = true;
    editor.setGUIMessage("AngleBis: Select second line.");
    return;
  }

  if (editor.angleBisectorLine1 && obj && (obj->getType() == ObjectType::Line || obj->getType() == ObjectType::LineSegment)) {
    editor.angleBisectorLine2 = editor.getLineSharedPtr(static_cast<Line*>(obj));
    if (!editor.angleBisectorLine2 || editor.angleBisectorLine1.get() == obj) {
      editor.setGUIMessage("AngleBis: Invalid second line.");
      resetState();
      return;
    }

    auto result = CGAL::intersection(editor.angleBisectorLine1->getCGALLine(), editor.angleBisectorLine2->getCGALLine());
    if (!result) {
      editor.setGUIMessage("AngleBis: Lines do not intersect.");
      resetState();
      return;
    }
    const Point_2* I = safe_get_point<Point_2>(&(*result));
    if (!I) {
      editor.setGUIMessage("AngleBis: Intersection not a point.");
      resetState();
      return;
    }
    Vector_2 d1 = editor.angleBisectorLine1->getCGALLine().direction().to_vector();
    Vector_2 d2 = editor.angleBisectorLine2->getCGALLine().direction().to_vector();
    double l1 = std::sqrt(CGAL::to_double(d1.squared_length()));
    double l2 = std::sqrt(CGAL::to_double(d2.squared_length()));
    if (l1 < 1e-9 || l2 < 1e-9) {
      editor.setGUIMessage("AngleBis: Invalid line directions.");
      resetState();
      return;
    }
    Vector_2 u = d1 / FT(l1);
    Vector_2 v = d2 / FT(l2);
    Vector_2 w = u + v;
    if (CGAL::to_double(w.squared_length()) < 1e-12) {
      editor.setGUIMessage("AngleBis: Lines are opposite; bisector undefined.");
      resetState();
      return;
    }
    auto bisector = std::make_shared<AngleBisector>(editor.angleBisectorLine1, editor.angleBisectorLine2, editor.objectIdCounter++);
    if (bisector && bisector->isValid()) {
      bisector->setThickness(editor.currentThickness);
      
      // REGISTER DEPENDENCIES (Internal Bisector)
      bisector->setDependent(true);
      editor.angleBisectorLine1->addDependent(bisector);
      editor.angleBisectorLine2->addDependent(bisector);

      editor.lines.push_back(bisector);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(bisector)));
      // DYNAMIC EXTERNAL BISECTOR
      auto extBisector = std::make_shared<AngleBisector>(editor.angleBisectorLine1, editor.angleBisectorLine2, editor.objectIdCounter++, true);
      if (extBisector && extBisector->isValid()) {
          extBisector->setThickness(editor.currentThickness);
          extBisector->setAsConstructionLine();
          
          // REGISTER DEPENDENCIES (External Bisector)
          extBisector->setDependent(true);
          editor.angleBisectorLine1->addDependent(extBisector);
          editor.angleBisectorLine2->addDependent(extBisector);

          editor.lines.push_back(extBisector);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(extBisector)));
      }

      editor.setGUIMessage("Angle bisectors created.");
    } else {
      editor.setGUIMessage("AngleBis: Failed to create line.");
    }
    resetState();
    return;
  }

  auto pt = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
  if (!pt || !pt->isValid()) {
    editor.setGUIMessage("AngleBis: Select valid points.");
    resetState();
    return;
  }
  editor.angleBisectorPoints.push_back(pt);
  editor.isCreatingAngleBisector = true;
  if (editor.angleBisectorPoints.size() < 3) {
    editor.setGUIMessage("AngleBis: Select more points (need 3).");
    return;
  }

  auto A = editor.angleBisectorPoints[0]->getCGALPosition();
  auto B = editor.angleBisectorPoints[1]->getCGALPosition();
  auto C = editor.angleBisectorPoints[2]->getCGALPosition();
  Vector_2 u = A - B;
  Vector_2 v = C - B;
  double lu = std::sqrt(CGAL::to_double(u.squared_length()));
  double lv = std::sqrt(CGAL::to_double(v.squared_length()));
  if (lu < 1e-9 || lv < 1e-9) {
    editor.setGUIMessage("AngleBis: Points too close.");
    resetState();
    return;
  }
  Vector_2 uN = u / FT(lu);
  Vector_2 vN = v / FT(lv);
  Vector_2 w = uN + vN;
  if (CGAL::to_double(w.squared_length()) < 1e-12) {
    editor.setGUIMessage("AngleBis: 180-degree angle; bisector undefined.");
    resetState();
    return;
  }
  auto bisector = std::make_shared<AngleBisector>(editor.angleBisectorPoints[1], editor.angleBisectorPoints[0], editor.angleBisectorPoints[2],
                                                  editor.objectIdCounter++);
  if (bisector && bisector->isValid()) {
    bisector->setThickness(editor.currentThickness);

    // REGISTER DEPENDENCIES (Internal Bisector)
    bisector->setDependent(true);
    editor.angleBisectorPoints[0]->addDependent(bisector);
    editor.angleBisectorPoints[1]->addDependent(bisector);
    editor.angleBisectorPoints[2]->addDependent(bisector);

    editor.lines.push_back(bisector);
    editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(bisector)));
    editor.setGUIMessage("Angle bisector created.");
  } else {
    editor.setGUIMessage("AngleBis: Failed to create line.");
  }
  resetState();
}

void handleTangentCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), editor.drawingView);
  float tolerance = getDynamicSelectionTolerance(editor);

  auto resetState = [&]() {
    editor.isCreatingTangent = false;
    editor.tangentAnchorPoint = nullptr;
    editor.tangentCircle = nullptr;
  };

  auto addTangentLine = [&](int solutionIndex, const std::string& msg) {
    auto tangent = std::make_shared<TangentLine>(editor.tangentAnchorPoint, editor.tangentCircle, solutionIndex, editor.objectIdCounter++);
    if (tangent && tangent->isValid()) {
      tangent->setThickness(editor.currentThickness);
      editor.lines.push_back(tangent);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(tangent)));
      editor.setGUIMessage(msg);
    } else {
      editor.setGUIMessage("Error: Failed to create tangent line.");
    }
  };

  auto findCircleAt = [&]() -> GeometricObject* {
    GeometricObject* hit = nullptr;
    double bestDist = std::numeric_limits<double>::infinity();
    Point_2 cgalPos = editor.toCGALPoint(worldPos_sfml);
    for (auto& c : editor.circles) {
      if (!c || !c->isValid() || !c->isVisible() || c->isLocked()) continue;
      Point_2 center = c->getCenterPoint();
      double r = c->getRadius();
      double d = std::sqrt(CGAL::to_double(CGAL::squared_distance(center, cgalPos)));
      if (d <= r + tolerance && d < bestDist) {
        bestDist = d;
        hit = c.get();
      }
    }
    return hit;
  };

  auto findPointAt = [&]() -> std::shared_ptr<Point> {
    std::shared_ptr<Point> best;
    float bestDist2 = tolerance * tolerance;

    auto considerPoint = [&](const std::shared_ptr<Point>& p, bool requireVisible) {
      if (!p || !p->isValid()) return;
      if (requireVisible && !p->isVisible()) return;
      sf::Vector2f pos = p->getSFMLPosition();
      float dx = pos.x - worldPos_sfml.x;
      float dy = pos.y - worldPos_sfml.y;
      float d2 = dx * dx + dy * dy;
      if (d2 <= bestDist2) {
        bestDist2 = d2;
        best = p;
      }
    };

    for (auto& pt : editor.points) considerPoint(pt, true);
    for (auto& op : editor.ObjectPoints) considerPoint(std::static_pointer_cast<Point>(op), true);

    // Allow selecting circle radius points even if they are hidden
    for (auto& c : editor.circles) {
      if (!c || !c->isValid()) continue;
      Point* radiusRaw = c->getRadiusPointObject();
      if (!radiusRaw) continue;

      std::shared_ptr<Point> radiusShared;
      for (auto& pt : editor.points) {
        if (pt.get() == radiusRaw) {
          radiusShared = pt;
          break;
        }
      }
      if (!radiusShared) {
        for (auto& op : editor.ObjectPoints) {
          if (op.get() == radiusRaw) {
            radiusShared = std::static_pointer_cast<Point>(op);
            break;
          }
        }
      }
      if (radiusShared) considerPoint(radiusShared, false);
    }

    return best;
  };

  GeometricObject* circleHit = findCircleAt();
  std::shared_ptr<Point> pointHit = findPointAt();

  if (!editor.tangentAnchorPoint && !editor.tangentCircle) {
    if (pointHit) {
      editor.tangentAnchorPoint = pointHit;
      editor.setGUIMessage("Tangent: select a circle.");
      return;
    }
    if (circleHit) {
      for (auto& c : editor.circles) {
        if (c.get() == circleHit) {
          editor.tangentCircle = c;
          break;
        }
      }
      editor.setGUIMessage("Tangent: now pick a point.");
      return;
    }
    return;
  }

  if (editor.tangentAnchorPoint && !editor.tangentCircle) {
    if (!circleHit) {
      editor.setGUIMessage("Tangent: select a circle.");
      return;
    }
    for (auto& c : editor.circles) {
      if (c.get() == circleHit) {
        editor.tangentCircle = c;
        break;
      }
    }
  }

  if (!editor.tangentAnchorPoint && editor.tangentCircle) {
    if (!pointHit) {
      editor.setGUIMessage("Tangent: select a point.");
      return;
    }
    editor.tangentAnchorPoint = pointHit;
  }

  if (!editor.tangentAnchorPoint || !editor.tangentCircle) return;

  Point_2 P = editor.tangentAnchorPoint->getCGALPosition();
  Point_2 O = editor.tangentCircle->getCenterPoint();
  double r = editor.tangentCircle->getRadius();
  Vector_2 OP = P - O;
  double d = std::sqrt(CGAL::to_double(OP.squared_length()));
  if (d < 1e-9) {
    editor.setGUIMessage("Tangent: point coincides with center.");
    resetState();
    return;
  }

  if (d < r - 1e-6) {
    editor.setGUIMessage("Tangent: Point inside circle.");
    resetState();
    return;
  }

  if (std::abs(d - r) < 1e-6) {
    addTangentLine(0, "Tangent created (point on circle).");
    resetState();
    return;
  }

  addTangentLine(0, "Tangent created.");
  addTangentLine(1, "Tangent created.");
  resetState();
}

// --- Vector Tool Handler ---
static void handleVectorCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  handleLineCreation(editor, mouseEvent);
}

void handleObjectPointCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  std::cout << "handleObjectPointCreation: ENTERED" << std::endl;

  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos = editor.toCGALPoint(worldPos);
  float tolerance = getDynamicSelectionTolerance(editor);

  std::cout << "Looking for host at (" << worldPos.x << ", " << worldPos.y << ") with tolerance " << tolerance << std::endl;

  std::shared_ptr<Line> hostLine = nullptr;
  std::shared_ptr<Circle> hostCircle = nullptr;

  for (auto& line : editor.lines) {
    if (line && line->contains(worldPos, tolerance)) {
      hostLine = line;
      std::cout << "Found line host: " << line->getID() << std::endl;
      break;
    }
  }

  if (!hostLine) {
    std::cout << "getCircleAtPosition: Checking " << editor.circles.size() << " circles" << std::endl;

    for (size_t i = 0; i < editor.circles.size(); ++i) {
      auto& circle_sp_in_list = editor.circles[i];

      std::cout << "  Circle[" << i << "]: ptr=" << circle_sp_in_list.get() << ", use_count=" << circle_sp_in_list.use_count() << std::endl;

      if (circle_sp_in_list && circle_sp_in_list->isValid() && circle_sp_in_list->contains(worldPos, tolerance)) {
        std::cout << "  Found matching circle at index " << i << std::endl;

        try {
          auto shared_test = circle_sp_in_list->shared_from_this();
          std::cout << "  Circle shared_from_this() test: use_count=" << shared_test.use_count() << std::endl;

          if (!shared_test) {
            std::cout << "  ERROR: Circle's shared_from_this() returned null!" << std::endl;
            std::cout << "  This indicates enable_shared_from_this was never initialized" << std::endl;
            std::cout << "  Skipping this circle to avoid crash" << std::endl;
            continue;
          }
        } catch (const std::exception& e) {
          std::cout << "  ERROR: Exception testing shared_from_this(): " << e.what() << std::endl;
          std::cout << "  Skipping this circle to avoid crash" << std::endl;
          continue;
        }

        hostCircle = circle_sp_in_list;
        if (hostCircle) {
          std::cout << "Found circle host: " << hostCircle->getID() << ", current use_count: " << hostCircle.use_count() << std::endl;
        }
        break;
      }
    }

    if (!hostCircle) {
      std::cout << "No circle found at position" << std::endl;
    }
  }

  if (!hostLine && !hostCircle) {
    // Pass 3: Shape edges (Rectangle/Polygon/RegularPolygon/Triangle)
    auto edgeHit = PointUtils::findNearestEdge(editor, worldPos, tolerance);
    if (edgeHit && edgeHit->host) {
      auto hostPtr = editor.findSharedPtr(edgeHit->host);
      if (hostPtr) {
        auto objPoint = ObjectPoint::createOnShapeEdge(hostPtr, edgeHit->edgeIndex, edgeHit->relativePosition);
        if (objPoint && objPoint->isValid()) {
          objPoint->setLabel(LabelManager::instance().getNextLabel(editor.getAllPoints()));
          editor.ObjectPoints.push_back(objPoint);
          editor.commandManager.pushHistoryOnly(
              std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(objPoint)));
          editor.setGUIMessage("ObjectPoint created on shape edge");
          return;
        }
      }
    }

    std::cout << "No suitable host object found for ObjectPoint creation" << std::endl;
    editor.setGUIMessage("ObjectPoint: Click on a line, circle, or shape edge");
    return;
  }

  try {
    if (hostLine) {
      createObjectPointOnLine(editor, hostLine.get(), cgalWorldPos);
    } else if (hostCircle) {
      if (!hostCircle || hostCircle.use_count() == 0) {
        std::cerr << "ERROR: hostCircle for ObjectPoint creation is null or has zero use_count before calling createObjectPointOnCircle."
                  << std::endl;
        editor.setGUIMessage("Error: Invalid circle host for ObjectPoint");
        return;
      }
      std::cout << "Passing hostCircle to createObjectPointOnCircle, use_count: " << hostCircle.use_count() << std::endl;
      createObjectPointOnCircle(editor, hostCircle, cgalWorldPos);
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception creating ObjectPoint: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to create ObjectPoint");
  }
}

void handleRectangleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSnapTolerance(editor);  // Use SNAP tolerance for consistency

    std::shared_ptr<Point> smartPoint = nullptr;
    Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
    Point_2 originalMousePos = cgalWorldPos;  // Store original position BEFORE snapping

    if (!editor.isCreatingRotatableRectangle || editor.dragMode == DragMode::RotatedRectP2) {
      smartPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);
      
      // CRITICAL FIX: Double-Check Pattern - Validate snap target distance
      if (smartPoint && smartPoint->isValid()) {
        sf::Vector2f snapPos = smartPoint->getSFMLPosition();
        float dx = worldPos_sfml.x - snapPos.x;
        float dy = worldPos_sfml.y - snapPos.y;
        float currentDist = std::sqrt(dx * dx + dy * dy);
        
        if (currentDist > tolerance) {
          // User moved mouse too fast; snap is stale. Ignore it.
          std::cout << "[Rectangle] Snap REJECTED: distance " << currentDist << " > tolerance " << tolerance << std::endl;
          smartPoint = nullptr;
          editor.m_snapTargetPoint = nullptr;
          cgalWorldPos = originalMousePos;
        } else {
          std::cout << "[Rectangle] Snap VALIDATED: distance " << currentDist << " <= tolerance " << tolerance << std::endl;
          cgalWorldPos = smartPoint->getCGALPosition();
        }
      }
    } else if (editor.dragMode == DragMode::RotatedRectHeight) {
      auto anchor = PointUtils::findAnchorPoint(editor, worldPos_sfml, tolerance * 1.5f);
      if (anchor) {
        cgalWorldPos = anchor->getCGALPosition();
      }
    }

    if (!editor.isCreatingRectangle) {
      // --- FIX: The Magic Shield ---
      // Clear zombie selections (like parallel line points) before starting
      deselectAllAndClearInteractionState(editor);
      // -----------------------------

      // FIX: Don't snap to points that belong to constrained lines (parallel/perpendicular)  
      // This prevents breaking geometric constraints and creating duplicate points
      std::cout << "[Rectangle] smartPoint = " << smartPoint.get() << std::endl;
      if (smartPoint && smartPoint->getType() != ObjectType::ObjectPoint &&
          ConstraintUtils::isPointConstrainedByLine(editor, smartPoint)) {
        std::cout << "[Rectangle] *** Point IS constrained, rejecting snap and using raw mouse position ***" << std::endl;
        editor.rectangleCorner1 = originalMousePos;  // Use original mouse position, NOT snapped position
        editor.rectangleCorner1Point = nullptr;      // Don't reuse the constrained point
      } else {
        std::cout << "[Rectangle] Point is NOT constrained, using snap" << std::endl;
        editor.rectangleCorner1 = cgalWorldPos;      // Use snapped position
        editor.rectangleCorner1Point = smartPoint;    // Safe to reuse
      }
      editor.isCreatingRectangle = true;
      
      // Optional: Set a specific drag mode to prevent object dragging
      // editor.dragMode = DragMode::CreatingRectangle; 

      try {
        editor.previewRectangle =
            std::make_shared<Rectangle>(editor.rectangleCorner1, editor.rectangleCorner1, false, editor.getCurrentColor(), editor.objectIdCounter);
      } catch (...) {
        editor.previewRectangle.reset();
      }
      editor.setGUIMessage("Rectangle: Click 2nd corner (Snapping Enabled)");
    } else {
      // FIX: Don't snap to points that belong to constrained lines
      if (smartPoint && smartPoint->getType() != ObjectType::ObjectPoint &&
          ConstraintUtils::isPointConstrainedByLine(editor, smartPoint)) {
        std::cout << "[Rectangle] *** Corner2 Point IS constrained, rejecting snap ***" << std::endl;
        editor.rectangleCorner2 = originalMousePos;  // Use original mouse position, NOT snapped position
        editor.rectangleCorner2Point = nullptr;      // Don't reuse the constrained point
      } else {
        std::cout << "[Rectangle] Corner2 Point is NOT constrained" << std::endl;
        editor.rectangleCorner2 = cgalWorldPos;      // Use snapped position
        editor.rectangleCorner2Point = smartPoint;    // Safe to reuse
      }
      if (editor.rectangleCorner1 != editor.rectangleCorner2) {
        auto corner1 = editor.rectangleCorner1Point ? editor.rectangleCorner1Point : editor.createPoint(editor.rectangleCorner1);
        auto corner2 = editor.rectangleCorner2Point ? editor.rectangleCorner2Point : editor.createPoint(editor.rectangleCorner2);
        if (corner1 && !editor.rectangleCorner1Point) {
          corner1->setCreatedWithShape(true);
        }
        if (corner2 && !editor.rectangleCorner2Point) {
          corner2->setCreatedWithShape(true);
        }
        auto newRectangle = std::make_shared<Rectangle>(corner1, corner2, false, editor.getCurrentColor(), editor.objectIdCounter++);
        applyRectangleVertexLabels(editor, newRectangle);
        
        // Automatic label for rectangle
        if (editor.showGlobalLabels) {
          newRectangle->setLabel(LabelManager::instance().getNextPolygonLabel(editor.getAllObjects()));
          newRectangle->setLabelMode(LabelMode::Name);
        }

        editor.rectangles.push_back(newRectangle);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newRectangle)));
        editor.setGUIMessage("Rectangle created");
      }
      editor.isCreatingRectangle = false;
      editor.rectangleCorner1Point.reset();
      editor.rectangleCorner2Point.reset();
      editor.previewRectangle.reset();
      
      // --- FIX: Ensure DragMode is reset ---
      editor.dragMode = DragMode::None;
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleRectangleCreation: " << e.what() << std::endl;
    editor.isCreatingRectangle = false;
    // --- FIX: Safety Reset ---
    editor.dragMode = DragMode::None; 
  }
}

void handleRotatableRectangleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSnapTolerance(editor);  // Use SNAP tolerance for consistency

    size_t pointsCountBefore = editor.points.size();
    size_t objectPointsCountBefore = editor.ObjectPoints.size();
    
    Point_2 originalMousePos = editor.toCGALPoint(worldPos_sfml);
    std::shared_ptr<Point> smartPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);

    // CRITICAL FIX: Double-Check Pattern - Validate snap target distance
    if (smartPoint && smartPoint->isValid()) {
      sf::Vector2f snapPos = smartPoint->getSFMLPosition();
      float dx = worldPos_sfml.x - snapPos.x;
      float dy = worldPos_sfml.y - snapPos.y;
      float currentDist = std::sqrt(dx * dx + dy * dy);
      
      if (currentDist > tolerance) {
        // User moved mouse too fast; snap is stale. Ignore it.
        std::cout << "[RotRect] Snap REJECTED: distance " << currentDist << " > tolerance " << tolerance << std::endl;
        smartPoint = nullptr;
        editor.m_snapTargetPoint = nullptr;
      } else {
        std::cout << "[RotRect] Snap VALIDATED: distance " << currentDist << " <= tolerance " << tolerance << std::endl;
      }
    }

    // REJECT CONSTRAINED POINTS for Rotatable Rectangle (Magic Shield)
    if (smartPoint && smartPoint->getType() != ObjectType::ObjectPoint &&
      ConstraintUtils::isPointConstrainedByLine(editor, smartPoint)) {
        std::cout << "[RotRect] *** Point IS constrained, rejecting snap ***" << std::endl;
        smartPoint = nullptr;
        editor.m_snapTargetPoint = nullptr;
    }

    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : originalMousePos;

    if (editor.isCreatingRotatableRectangle && editor.dragMode == DragMode::RotatedRectHeight && smartPoint) {
      bool createdNewPoint = false;
      if (editor.points.size() > pointsCountBefore && !editor.points.empty()) {
        createdNewPoint = (editor.points.back() == smartPoint);
      } else if (editor.ObjectPoints.size() > objectPointsCountBefore && !editor.ObjectPoints.empty()) {
        createdNewPoint = (editor.ObjectPoints.back() == smartPoint);
      }

      if (createdNewPoint) {
        smartPoint->setShowLabel(false);
        smartPoint->setVisible(false);
        smartPoint->setDependent(true);
        smartPoint->setCreatedWithShape(true);
      }
    }

    if (!editor.isCreatingRotatableRectangle) {
      
      // --- FIX: The Magic Shield ---
      // This wipes the Parallel Line selection so it doesn't get dragged
      deselectAllAndClearInteractionState(editor); 
      // -----------------------------

      editor.rectangleCorner1 = cgalWorldPos;
      editor.rectangleCorner1Point = smartPoint;
      editor.rectangleCorner2Point.reset();
      editor.isCreatingRotatableRectangle = true;
      editor.dragMode = DragMode::RotatedRectP2;
      try {
        editor.previewRectangle =
            std::make_shared<Rectangle>(editor.rectangleCorner1, editor.rectangleCorner1, 0.0, editor.getCurrentColor(), editor.objectIdCounter);
      } catch (...) {
        editor.previewRectangle.reset();
      }
      editor.setGUIMessage("RotRect: Click second corner (base edge)");
      return;
    }

    if (editor.isCreatingRotatableRectangle && editor.dragMode == DragMode::RotatedRectP2) {
      editor.rectangleCorner2 = cgalWorldPos;
      editor.rectangleCorner2Point = smartPoint;
      editor.dragMode = DragMode::RotatedRectHeight;
      editor.setGUIMessage("RotRect: Drag height, click to finish");
      return;
    }

    if (editor.isCreatingRotatableRectangle && editor.dragMode == DragMode::RotatedRectHeight) {
      Point_2 baseStart = editor.rectangleCorner1;
      Point_2 baseEnd = editor.rectangleCorner2;
      double dx = CGAL::to_double(baseEnd.x() - baseStart.x());
      double dy = CGAL::to_double(baseEnd.y() - baseStart.y());
      double baseLen = std::sqrt(dx * dx + dy * dy);
      if (baseLen <= Constants::MIN_CIRCLE_RADIUS) {
        editor.setGUIMessage("RotRect: Base too small");
        editor.isCreatingRotatableRectangle = false;
        editor.rectangleCorner1Point.reset();
        editor.rectangleCorner2Point.reset();
        editor.previewRectangle.reset();
        
        // --- FIX: Reset Drag Mode ---
        editor.dragMode = DragMode::None;
        return;
      }

      double ux = -dy / baseLen;
      double uy = dx / baseLen;
      double vx = CGAL::to_double(cgalWorldPos.x() - baseStart.x());
      double vy = CGAL::to_double(cgalWorldPos.y() - baseStart.y());
      double signedHeight = vx * ux + vy * uy;
      double heightAbs = std::abs(signedHeight);

      if (heightAbs > Constants::MIN_CIRCLE_RADIUS) {
        auto corner1 = editor.rectangleCorner1Point ? editor.rectangleCorner1Point : editor.createPoint(editor.rectangleCorner1);
        auto corner2 = editor.rectangleCorner2Point ? editor.rectangleCorner2Point : editor.createPoint(editor.rectangleCorner2);
        if (corner1 && !editor.rectangleCorner1Point) {
          corner1->setCreatedWithShape(true);
        }
        if (corner2 && !editor.rectangleCorner2Point) {
          corner2->setCreatedWithShape(true);
        }
        auto newRectangle = std::make_shared<Rectangle>(corner1, corner2, baseLen, editor.getCurrentColor(), editor.objectIdCounter++);
        newRectangle->setHeight(static_cast<float>(signedHeight));
        applyRectangleVertexLabels(editor, newRectangle);
        newRectangle->setThickness(editor.currentThickness);
        editor.rectangles.push_back(newRectangle);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newRectangle)));
        editor.setGUIMessage("Rotatable rectangle created");
      }

      editor.isCreatingRotatableRectangle = false;
      editor.rectangleCorner1Point.reset();
      editor.rectangleCorner2Point.reset();
      editor.previewRectangle.reset();
      
      // --- FIX: Reset Drag Mode ---
      editor.dragMode = DragMode::None;
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleRotatableRectangleCreation: " << e.what() << std::endl;
    editor.isCreatingRotatableRectangle = false;
    // --- FIX: Safety Reset ---
    editor.dragMode = DragMode::None;
  }
}

void handlePolygonCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    std::shared_ptr<Point> smartPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);

    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);

    if (!editor.isCreatingPolygon) {
      editor.isCreatingPolygon = true;
      editor.polygonVertices.clear();
      editor.polygonVertexPoints.clear();
      editor.polygonVertices.push_back(cgalWorldPos);
      editor.polygonVertexPoints.push_back(smartPoint);
    } else {
      if (editor.polygonVertices.size() >= 3 && cgalWorldPos == editor.polygonVertices[0]) {
        std::vector<std::shared_ptr<Point>> finalPoints;
        for (size_t i = 0; i < editor.polygonVertexPoints.size(); ++i) {
          if (editor.polygonVertexPoints[i])
            finalPoints.push_back(editor.polygonVertexPoints[i]);
          else
            finalPoints.push_back(editor.createPoint(editor.polygonVertices[i]));
        }

        auto newPolygon = std::make_shared<Polygon>(finalPoints, editor.getCurrentColor(), editor.objectIdCounter++);
        assignUnifiedLabels(editor, finalPoints);
        
        // Automatic label for polygon
        if (editor.showGlobalLabels) {
          newPolygon->setLabel(LabelManager::instance().getNextPolygonLabel(editor.getAllObjects()));
          newPolygon->setLabelMode(LabelMode::Name);
        }

        newPolygon->setThickness(editor.currentThickness);
        editor.polygons.push_back(newPolygon);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPolygon)));
        editor.isCreatingPolygon = false;
        editor.polygonVertices.clear();
        editor.polygonVertexPoints.clear();
        editor.previewPolygon.reset();
        editor.setGUIMessage("Polygon closed.");
        return;
      }
      editor.polygonVertices.push_back(cgalWorldPos);
      editor.polygonVertexPoints.push_back(smartPoint);
    }

    if (!editor.polygonVertices.empty()) {
      editor.previewPolygon = std::make_shared<Polygon>(editor.polygonVertices, editor.getCurrentColor(), editor.objectIdCounter);
    }
    editor.setGUIMessage("Polygon: Add vertex or click start to close");
  } catch (const std::exception& e) {
    std::cerr << "Exception in handlePolygonCreation: " << e.what() << std::endl;
  }
}

void handleRegularPolygonCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    std::shared_ptr<Point> smartPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);

    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);

    if (editor.regularPolygonPhase == 0) {
      editor.regularPolygonCenter = cgalWorldPos;
      editor.regularPolygonCenterPoint = smartPoint;
      editor.regularPolygonPhase = 1;
      editor.setGUIMessage("RegPoly: Define radius (Snapping Enabled)");
    } else if (editor.regularPolygonPhase == 1) {
      editor.regularPolygonFirstVertex = cgalWorldPos;
      editor.regularPolygonFirstVertexPoint = smartPoint;

      double dx = CGAL::to_double(editor.regularPolygonFirstVertex.x() - editor.regularPolygonCenter.x());
      double dy = CGAL::to_double(editor.regularPolygonFirstVertex.y() - editor.regularPolygonCenter.y());
      if (std::sqrt(dx * dx + dy * dy) > Constants::MIN_CIRCLE_RADIUS) {
        auto newRegPoly = std::make_shared<RegularPolygon>(
            editor.regularPolygonCenterPoint ? editor.regularPolygonCenterPoint : editor.createPoint(editor.regularPolygonCenter),
            editor.regularPolygonFirstVertexPoint ? editor.regularPolygonFirstVertexPoint : editor.createPoint(editor.regularPolygonFirstVertex),
            editor.regularPolygonNumSides, editor.getCurrentColor(), editor.objectIdCounter++);
        
        // Automatic label for regular polygon
        if (editor.showGlobalLabels) {
          newRegPoly->setLabel(LabelManager::instance().getNextPolygonLabel(editor.getAllObjects()));
          newRegPoly->setLabelMode(LabelMode::Name);
        }

        newRegPoly->setThickness(editor.currentThickness);
        editor.regularPolygons.push_back(newRegPoly);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newRegPoly)));
        editor.setGUIMessage("Regular polygon created");
        editor.regularPolygonPhase = 0;
        editor.regularPolygonCenterPoint.reset();
        editor.regularPolygonFirstVertexPoint.reset();
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleRegularPolygonCreation: " << e.what() << std::endl;
    editor.regularPolygonPhase = 0;
  }
}

void handleTriangleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    std::shared_ptr<Point> smartPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);

    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);

    editor.triangleVertices.push_back(cgalWorldPos);
    editor.triangleVertexPoints.push_back(smartPoint);

    if (editor.triangleVertices.size() == 1) {
      editor.setGUIMessage("Triangle: Click 2nd vertex");
      editor.isCreatingTriangle = true;
    } else if (editor.triangleVertices.size() == 2) {
      editor.setGUIMessage("Triangle: Click 3rd vertex");
      editor.previewTriangle = nullptr;
    } else if (editor.triangleVertices.size() == 3) {
      if (!CGAL::collinear(editor.triangleVertices[0], editor.triangleVertices[1], editor.triangleVertices[2])) {
        auto p1 = editor.triangleVertexPoints[0] ? editor.triangleVertexPoints[0] : editor.createPoint(editor.triangleVertices[0]);
        auto p2 = editor.triangleVertexPoints[1] ? editor.triangleVertexPoints[1] : editor.createPoint(editor.triangleVertices[1]);
        auto p3 = editor.triangleVertexPoints[2] ? editor.triangleVertexPoints[2] : editor.createPoint(editor.triangleVertices[2]);

        auto newTriangle = std::make_shared<Triangle>(p1, p2, p3, editor.getCurrentColor(), editor.objectIdCounter++);
        
        std::vector<std::shared_ptr<Point>> triPoints = {p1, p2, p3};
        assignUnifiedLabels(editor, triPoints);
        
        // Automatic label for triangle
        if (editor.showGlobalLabels) {
          newTriangle->setLabel(LabelManager::instance().getNextPolygonLabel(editor.getAllObjects()));
          newTriangle->setLabelMode(LabelMode::Name);
        }

        newTriangle->setThickness(editor.currentThickness);
        editor.triangles.push_back(newTriangle);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newTriangle)));
        editor.setGUIMessage("Triangle created");
      } else {
        editor.setGUIMessage("Error: Points are collinear");
      }
      editor.triangleVertices.clear();
      editor.triangleVertexPoints.clear();
      editor.isCreatingTriangle = false;
      editor.previewTriangle = nullptr;
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleTriangleCreation: " << e.what() << std::endl;
    editor.triangleVertices.clear();
    editor.triangleVertexPoints.clear();
    editor.isCreatingTriangle = false;
  }
}

static void handleIntersectionCreation(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml) {
  float tolerance = getDynamicSelectionTolerance(editor);
  auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
  if (smartPoint && smartPoint->isValid()) {
    // Only auto-select free points, not object points (intersections), per user request
    if (smartPoint->getType() != ObjectType::ObjectPoint) {
      smartPoint->setSelected(true);
      editor.selectedObject = smartPoint.get();
    }
  }
}

static void handleCircleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent, const sf::Vector2f& worldPos_sfml) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  if (!editor.isCreatingCircle) {
    sf::Vector2i mousePos = sf::Mouse::getPosition(editor.window);
    sf::Vector2f worldPos = editor.window.mapPixelToCoords(mousePos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    std::shared_ptr<Point> centerPoint = createSmartPointFromClick(editor, worldPos, tolerance);

    if (!centerPoint || !centerPoint->isValid()) {
      return;
    }

    Point_2 center = centerPoint->getCGALPosition();
    editor.isCreatingCircle = true;
    editor.createStart_cgal = center;

    sf::Color selectedColor = editor.getCurrentColor();
    editor.previewCircle = Circle::create(centerPoint.get(), nullptr, 0.0, selectedColor);
    if (editor.previewCircle) editor.previewCircle->setThickness(editor.currentThickness);
    std::cout << "Preview circle created at address: " << editor.previewCircle.get() << std::endl;
    std::cout << "Creating circle with center at (" << CGAL::to_double(center.x()) << ", " << CGAL::to_double(center.y()) << ") and radius 0"
              << std::endl;
  } else {
    try {
      float tolerance = getDynamicSelectionTolerance(editor);

      std::shared_ptr<Point> radiusPoint = createSmartPointFromClick(editor, worldPos_sfml, tolerance);

      if (!radiusPoint || !radiusPoint->isValid()) {
        return;
      }

      Point* centerPoint = editor.previewCircle ? editor.previewCircle->getCenterPointObject() : nullptr;
      if (!centerPoint) {
        return;
      }

      Point_2 center = centerPoint->getCGALPosition();
      double radius_sq = CGAL::to_double(CGAL::squared_distance(center, radiusPoint->getCGALPosition()));
      double radius = std::sqrt(radius_sq);

      if (radius >= Constants::MIN_CIRCLE_RADIUS) {
        sf::Color selectedColor = editor.previewCircle ? editor.previewCircle->getColor() : editor.getCurrentColor();
        auto finalCircle = Circle::create(centerPoint, radiusPoint, radius, selectedColor);
        if (finalCircle && finalCircle->isValid()) {
          finalCircle->setThickness(editor.currentThickness);
          
          // Automatic label for circle
          if (editor.showGlobalLabels) {
            finalCircle->setLabel(LabelManager::instance().getNextLineLabel(editor.getAllObjects()));
            finalCircle->setLabelMode(LabelMode::Name);
          }

          editor.circles.push_back(finalCircle);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(finalCircle)));
          std::cout << "Circle Created with radius: " << radius << std::endl;
        } else {
          std::cerr << "Preview circle is invalid" << std::endl;
        }
      } else {
        std::cout << "Circle not created: radius too small." << std::endl;
      }
    } catch (const std::exception& e) {
      std::cerr << "Error creating circle: " << e.what() << std::endl;
    }

    editor.isCreatingCircle = false;
    editor.previewCircle.reset();
    editor.dragMode = DragMode::None;
  }
}

static void handleAngleCreation(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml) {
  float tolerance = getDynamicSelectionTolerance(editor);

  auto anchor = PointUtils::findAnchorPoint(editor, worldPos_sfml, tolerance * 1.5f);

  if (anchor && anchor->isValid()) {
    auto ensurePointStored = [&](const std::shared_ptr<Point>& pt) {
      if (!pt) return;
      for (auto& p : editor.points)
        if (p == pt) return;
      for (auto& op : editor.ObjectPoints)
        if (op == pt) return;
      if (std::find(editor.points.begin(), editor.points.end(), pt) == editor.points.end() &&
          std::find(editor.ObjectPoints.begin(), editor.ObjectPoints.end(), pt) == editor.ObjectPoints.end()) {
        pt->setVisible(true);
        pt->update();
        editor.points.push_back(pt);
      }
    };

    ensurePointStored(anchor);

    if (editor.angleLine1) {
      editor.angleLine1->setSelected(false);
      editor.angleLine1 = nullptr;
      editor.setGUIMessage("Angle: Switched to Point Selection");
    }

    if (!editor.anglePointA) {
      editor.anglePointA = anchor;
      editor.setGUIMessage("Angle: Point A selected. Click Vertex.");
      return;
    }
    if (!editor.angleVertex) {
      editor.angleVertex = anchor;
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift)) {
        g_showAngleInputPopup = true;
        editor.setGUIMessage("Angle: Enter degrees.");
      } else {
        editor.setGUIMessage("Angle: Vertex selected. Click Point B.");
      }
      return;
    }
    if (!editor.anglePointB) {
      editor.anglePointB = anchor;
    }

    if (editor.anglePointA && editor.angleVertex && editor.anglePointB) {
      auto angle = std::make_shared<Angle>(editor.anglePointA, editor.angleVertex, editor.anglePointB, false, editor.getCurrentColor());
      sf::Vector2u winSize = editor.window.getSize();
      sf::Vector2f viewSize = editor.drawingView.getSize();
      float zoomScale = (winSize.x > 0) ? (viewSize.x / static_cast<float>(winSize.x)) : 1.0f;
      float idealRadius = 50.0f * zoomScale;
      angle->setRadius(idealRadius);
      
      // Automatic Greek label assignment
      angle->setLabel(LabelManager::instance().getNextGreekLabel(editor.getAllObjects()));
      angle->setLabelMode(LabelMode::Name); // Default to showing name

      angle->setVisible(true);
      angle->update();
      editor.angles.push_back(angle);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(angle)));
      std::cout << "Angle created via 3 points." << std::endl;
      editor.setGUIMessage("Angle created.");

      editor.anglePointA = nullptr;
      editor.angleVertex = nullptr;
      editor.anglePointB = nullptr;
    }
    return;
  }

  GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Line, ObjectType::LineSegment});
  if (hitObj) {
    if (editor.anglePointA) editor.anglePointA = nullptr;
    if (editor.angleVertex) editor.angleVertex = nullptr;

    Line* lineRaw = static_cast<Line*>(hitObj);
    auto linePtr = editor.getLineSharedPtr(lineRaw);

    if (linePtr) {
      if (!editor.angleLine1) {
        editor.angleLine1 = linePtr;
        editor.angleLine1->setSelected(true);
        editor.setGUIMessage("Angle: Line 1 selected. Click Line 2.");
      } else if (!editor.angleLine2 && linePtr != editor.angleLine1) {
        editor.angleLine2 = linePtr;

        Vector_2 v1 = editor.angleLine1->getCGALLine().direction().to_vector();
        Vector_2 v2 = editor.angleLine2->getCGALLine().direction().to_vector();
        double len1 = std::sqrt(CGAL::to_double(v1.squared_length()));
        double len2 = std::sqrt(CGAL::to_double(v2.squared_length()));

        constexpr double kEpsLen = 1e-9;
        if (len1 < kEpsLen || len2 < kEpsLen) {
          editor.setGUIMessage("Error: Cannot measure angle of zero-length segment.");
          editor.angleLine1->setSelected(false);
          editor.angleLine1 = nullptr;
          editor.angleLine2 = nullptr;
          return;
        }

        double dot = CGAL::to_double(v1 * v2) / (len1 * len2);

        if (std::abs(std::abs(dot) - 1.0) < 1e-9) {
          editor.setGUIMessage("Error: Lines are parallel, cannot form angle.");
          editor.angleLine1->setSelected(false);
          editor.angleLine1 = nullptr;
          editor.angleLine2 = nullptr;
          return;
        }

        auto result = CGAL::intersection(editor.angleLine1->getCGALLine(), editor.angleLine2->getCGALLine());
        if (result) {
          const Point_2* intersectionPt = nullptr;

          if (const Point_2* p = safe_get_point<Point_2>(&(*result))) {
            intersectionPt = p;
          } else if (const auto* p = safe_get_point<Point_2>(&(*result))) {
            intersectionPt = p;
          }

          if (intersectionPt) {
            auto vertex = PointUtils::createSmartPoint(editor, editor.toSFMLVector(*intersectionPt), tolerance);

            if (std::find(editor.points.begin(), editor.points.end(), vertex) == editor.points.end() &&
                std::find(editor.ObjectPoints.begin(), editor.ObjectPoints.end(), vertex) == editor.ObjectPoints.end()) {
              vertex->setVisible(true);
              editor.points.push_back(vertex);
            }

            Point_2 vPos = vertex->getCGALPosition();

            Point_2 p1_cgal;
            double d1_start = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine1->getStartPoint()));
            double d1_end = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine1->getEndPoint()));

            if (d1_start > d1_end) {
              p1_cgal = editor.angleLine1->getStartPoint();
            } else {
              p1_cgal = editor.angleLine1->getEndPoint();
            }

            Point_2 p2_cgal;
            double d2_start = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine2->getStartPoint()));
            double d2_end = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine2->getEndPoint()));

            if (d2_start > d2_end) {
              p2_cgal = editor.angleLine2->getStartPoint();
            } else {
              p2_cgal = editor.angleLine2->getEndPoint();
            }

            if (std::max(d1_start, d1_end) < 1e-9 || std::max(d2_start, d2_end) < 1e-9) {
              editor.setGUIMessage("Error: Cannot measure angle of zero-length segment.");
              editor.angleLine1->setSelected(false);
              editor.angleLine1 = nullptr;
              editor.angleLine2 = nullptr;
              return;
            }

            auto ensureArmPoint = [&](const Point_2& target) -> std::shared_ptr<Point> {
              for (auto& p : editor.points) {
                if (p && p->isValid() && p->getCGALPosition() == target) return p;
              }
              for (auto& p : editor.ObjectPoints) {
                if (p && p->isValid() && p->getCGALPosition() == target) return p;
              }
              auto newP = std::make_shared<Point>(target, 1.0f, sf::Color::Transparent);
              newP->setVisible(false);
              editor.points.push_back(newP);
              return newP;
            };

            auto p1 = ensureArmPoint(p1_cgal);
            auto p2 = ensureArmPoint(p2_cgal);

            if (p1 && vertex && p2) {
              auto angle = std::make_shared<Angle>(p1, vertex, p2, false, editor.getCurrentColor());
              sf::Vector2u winSize = editor.window.getSize();
              sf::Vector2f viewSize = editor.drawingView.getSize();
              float zoomScale = (winSize.x > 0) ? (viewSize.x / static_cast<float>(winSize.x)) : 1.0f;
              float idealRadius = 50.0f * zoomScale;
              angle->setRadius(idealRadius);
              
              // Automatic Greek label assignment
              std::vector<std::shared_ptr<GeometricObject>> allObjects;
              for (auto& p : editor.points) allObjects.push_back(p);
              for (auto& l : editor.lines) allObjects.push_back(l);
              for (auto& c : editor.circles) allObjects.push_back(c);
              for (auto& a : editor.angles) allObjects.push_back(a);
              angle->setLabel(LabelManager::instance().getNextGreekLabel(allObjects));
              angle->setLabelMode(LabelMode::Name); // Default to showing name

              angle->setVisible(true);
              angle->update();
              editor.angles.push_back(angle);
              editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(angle)));
              std::cout << "Angle created via 2 lines." << std::endl;
              editor.setGUIMessage("Angle created.");
            } else {
              editor.setGUIMessage("Error: Failed to determine angle vertices.");
            }
          } else {
            editor.setGUIMessage("Error: Intersection is not a point (Collinear/Overlapping?).");
          }
        } else {
          editor.setGUIMessage("Error: Lines are parallel.");
        }

        editor.angleLine1->setSelected(false);
        editor.angleLine1 = nullptr;
        editor.angleLine2 = nullptr;
      }
    }
  }
}
static void handleSemicircleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
      // Implement 2-point diameter creation for Semicircle
      static bool isCreatingSemi = false;
      static std::shared_ptr<Point> p1 = nullptr;

      sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
      sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
      float tolerance = getDynamicSnapTolerance(editor);

      std::shared_ptr<Point> clickedPoint = createSmartPointFromClick(editor, worldPos, tolerance);
      if (!clickedPoint) return;

      if (!isCreatingSemi) {
        p1 = clickedPoint;
        isCreatingSemi = true;
        editor.setGUIMessage("Semicircle Start selected. Click End point (Diameter).");
      } else {
        if (clickedPoint != p1) {
          // Calculate center
          Point_2 start = p1->getCGALPosition();
          Point_2 end = clickedPoint->getCGALPosition();
          Point_2 centerCgal = CGAL::midpoint(start, end);

          auto centerPt = editor.createPoint(centerCgal);  // Create explicit center point?
          // For Semicircle, usually center is implicit or explicit.
          // Let's create it explicitly so it can be moved.
          centerPt->setVisible(true);  // Or hidden?
          editor.points.push_back(centerPt);

          // Radius is dist(center, p1)
          double r = std::sqrt(CGAL::to_double(CGAL::squared_distance(centerCgal, start)));

          auto newSemi = std::make_shared<Circle>(centerPt.get(), p1, r, editor.getCurrentColor());
          newSemi->setSemicircle(true);
          newSemi->setSemicircleDiameterPoints(p1, clickedPoint);
          newSemi->setSemicircleBasis(start, end);

          // Automatic label for semicircle
          if (editor.showGlobalLabels) {
            newSemi->setLabel(LabelManager::instance().getNextLineLabel(editor.getAllObjects()));
            newSemi->setLabelMode(LabelMode::Name);
          }

          newSemi->setVisible(true);
          newSemi->setThickness(editor.currentThickness);
          editor.circles.push_back(newSemi);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newSemi)));

          isCreatingSemi = false;
          p1 = nullptr;
          editor.setGUIMessage("Semicircle created.");
        }
      }
    }

    static void handleCircle3pCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
      // Implements 3-point circle creation using editor creation buffers (no static state)
      sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
      sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
      float tolerance = getDynamicSnapTolerance(editor);

      std::shared_ptr<Point> clickedPoint = createSmartPointFromClick(editor, worldPos, tolerance);
      if (!clickedPoint) return;

      // Check for duplicates
      for (const auto& p : editor.circle3PPointObjects) {
          if (p == clickedPoint) {
              editor.setGUIMessage("Circle3P: Point already selected. Select a different point.");
              return;
          }
      }

      editor.circle3PPointObjects.push_back(clickedPoint);
      clickedPoint->setSelected(true);
      editor.isCreatingCircle3P = true;

      if (editor.circle3PPointObjects.size() == 1) {
        editor.setGUIMessage("Circle3P: First point selected. Select second point.");
      } else if (editor.circle3PPointObjects.size() == 2) {
        editor.setGUIMessage("Circle3P: Second point selected. Select third point.");
      } else if (editor.circle3PPointObjects.size() == 3) {
        std::shared_ptr<Point> p1 = editor.circle3PPointObjects[0];
        std::shared_ptr<Point> p2 = editor.circle3PPointObjects[1];
        std::shared_ptr<Point> p3 = editor.circle3PPointObjects[2];

        // Check for collinearity
        Point_2 a = p1->getCGALPosition();
        Point_2 b = p2->getCGALPosition();
        Point_2 c = p3->getCGALPosition();
        if (CGAL::collinear(a, b, c)) {
          editor.setGUIMessage("Points are collinear. Cannot create a circle.");
          editor.resetCreationStates();
          return;
        }

        // Compute circumcenter
        Point_2 center = CGAL::circumcenter(a, b, c);

        // Create the center point (managed via editor)
        auto centerPt = editor.createPoint(center);
        centerPt->setVisible(true);
        // centerPt is automatically added to editor.points by the factory

        double r = std::sqrt(CGAL::to_double(CGAL::squared_distance(center, a)));
        auto newCircle = std::make_shared<Circle>(centerPt.get(), nullptr, r, editor.getCurrentColor());
        
        // Automatic label for circle (3 points)
        if (editor.showGlobalLabels) {
          newCircle->setLabel(LabelManager::instance().getNextLineLabel(editor.getAllObjects()));
          newCircle->setLabelMode(LabelMode::Name);
        }
        
        // Link points and enable dynamic updates
        newCircle->set3PointDefinition(p1, p2, p3);
        newCircle->setVisible(true);
        newCircle->setThickness(editor.currentThickness);
        
        editor.circles.push_back(newCircle);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newCircle)));

        editor.setGUIMessage("Circle through 3 points created.");
        
        // Cleanup selection visual on control points
        for (auto& p : editor.circle3PPointObjects) {
            if (p) p->setSelected(false);
        }
        
        editor.resetCreationStates();
      }
    }

    static void beginTextEdit(GeometryEditor& editor, const std::shared_ptr<TextLabel>& label) {
      if (!label) return;
      editor.isTextEditing = true;
      editor.textEditingLabel = label;
      editor.textEditBuffer = label->getRawContent();
      editor.textEditIsRich = label->isRichText();
      editor.textEditFontSize = label->getFontSize();
      editor.textCursorIndex = editor.textEditBuffer.size();
      editor.textCursorBlinkClock.restart();
      editor.showSymbolPalette = true;
      {
        sf::Vector2f worldPos(
          static_cast<float>(CGAL::to_double(label->getCGALPosition().x())),
          static_cast<float>(CGAL::to_double(label->getCGALPosition().y())));
        sf::Vector2i screenPos = editor.window.mapCoordsToPixel(worldPos, editor.drawingView);
        sf::Vector2f uiPos = editor.window.mapPixelToCoords(screenPos, editor.guiView);
        editor.textPalettePosition = uiPos + sf::Vector2f(0.f, 40.f);
      }

      editor.clearSelection();
      label->setSelected(true);
      editor.selectedObject = label.get();
    }

    static void handleTextTool(GeometryEditor& editor, const sf::Vector2f& worldPos_sfml) {
      float tolerance = getDynamicSelectionTolerance(editor);
      GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::TextLabel});
      if (hitObj) {
        auto label = std::dynamic_pointer_cast<TextLabel>(editor.findSharedPtr(hitObj));
        if (label) {
          editor.textEditingLabel = label;
          editor.textEditorDialog.open(label->getRawContent(), label->isRichText(), label->getFontSize());
          editor.setGUIMessage("Text: Editing existing label (OK to confirm, Cancel to abort).");
          return;
        }
      }

      // Begin drag-to-size text box
      editor.isCreatingTextBox = true;
      editor.textBoxStart_sfml = worldPos_sfml;
      editor.textBoxCurrent_sfml = worldPos_sfml;
      editor.textBoxStartScale = editor.drawingView.getSize().y / static_cast<float>(editor.window.getSize().y);
      editor.textBoxPreviewShape.setPosition(worldPos_sfml);
      editor.textBoxPreviewShape.setSize(sf::Vector2f(0.f, 0.f));
      editor.setGUIMessage("Text: Drag to set box width (release to edit).");
    }
    
    
void handleMousePress(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  editor.isResizingAngle = false; // Reset resizing flag
  // --- Angle Arc Resize Handle Hit Test ---
  // (Moved after worldPos_sfml is defined)
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);

    sf::Vector2f guiPos = editor.window.mapPixelToCoords(pixelPos, editor.guiView);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

    // --- Angle Arc Resize Handle Hit Test (now after worldPos_sfml is defined) ---
    // Only run arc handle hit test if not clicking on a vertex/point or vertex label first
    if (mouseEvent.button == sf::Mouse::Left) {
      // Check for vertex/point hit first (priority)
      float tolerance = getDynamicSelectionTolerance(editor);
      for (const auto& point : editor.points) {
        if (!point || !point->isVisible()) continue;
        sf::Vector2f pos = point->getSFMLPosition();
        float dx = worldPos_sfml.x - pos.x;
        float dy = worldPos_sfml.y - pos.y;
        if ((dx * dx + dy * dy) <= tolerance * tolerance) {
          goto skipAngleArcHandle;
        }
      }
      for (const auto& objPt : editor.ObjectPoints) {
        if (!objPt || !objPt->isVisible()) continue;
        sf::Vector2f pos = objPt->getSFMLPosition();
        float dx = worldPos_sfml.x - pos.x;
        float dy = worldPos_sfml.y - pos.y;
        if ((dx * dx + dy * dy) <= tolerance * tolerance) {
          goto skipAngleArcHandle;
        }
      }
      // Check for vertex label hit (precedence)
      sf::Vector2f mouseScreenPos(static_cast<float>(pixelPos.x), static_cast<float>(pixelPos.y));
      for (const auto& rect : editor.rectangles) {
        if (!rect || !rect->isVisible()) continue;
        auto verts = rect->getVerticesSFML();
        const char* labels[] = {"A", "B", "C", "D"};
        for (size_t i = 0; i < verts.size() && i < 4; ++i) {
          sf::Vector2f worldPos = verts[i];
          sf::Vector2i screenPos = editor.window.mapCoordsToPixel(worldPos, editor.drawingView);
          sf::Vector2f labelPos(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
          labelPos += rect->getVertexLabelOffset(i);
          sf::Text text;
          text.setFont(Button::getFont());
          text.setString(labels[i]);
          text.setCharacterSize(LabelManager::instance().getFontSize());
          text.setPosition(labelPos);
          sf::FloatRect bounds = text.getGlobalBounds();
          float dynamicTol = getDynamicSelectionTolerance(editor);
          bounds.left -= dynamicTol;
          bounds.top -= dynamicTol;
          bounds.width += dynamicTol * 2.0f;
          bounds.height += dynamicTol * 2.0f;
          if (bounds.contains(mouseScreenPos)) {
            goto skipAngleArcHandle;
          }
        }
      }
      // Now check for angle arc handle
      for (const auto& angle : editor.angles) {
        if (!angle || !angle->isValid() || !angle->isVisible()) continue;
        if (!(angle->isSelected() || angle->isHovered())) continue;

        // Recompute midAngle as in label/arc logic
        auto pointA = angle->getPointA().lock();
        auto vertex = angle->getVertex().lock();
        auto pointB = angle->getPointB().lock();
        if (!pointA || !vertex || !pointB) continue;
        Point_2 a = pointA->getCGALPosition();
        Point_2 v = vertex->getCGALPosition();
        Point_2 b = pointB->getCGALPosition();
        double vx = CGAL::to_double(v.x());
        double vy = CGAL::to_double(v.y());
        double ax = CGAL::to_double(a.x());
        double ay = CGAL::to_double(a.y());
        double bx = CGAL::to_double(b.x());
        double by = CGAL::to_double(b.y());
        double angle1 = std::atan2(ay - vy, ax - vx);
        double angle2 = std::atan2(by - vy, bx - vx);
        double sweep = angle2 - angle1;
        if (angle->isReflex()) {
          if (sweep >= 0)
            sweep -= 2.0 * 3.14159265358979323846;
          else
            sweep += 2.0 * 3.14159265358979323846;
        } else {
          if (sweep < 0) sweep += 2.0 * 3.14159265358979323846;
        }
        double midAngle = angle1 + sweep * 0.5;
        float hx = static_cast<float>(vx + angle->getRadius() * std::cos(midAngle));
        float hy = static_cast<float>(vy + angle->getRadius() * std::sin(midAngle));
        sf::Vector2f handlePos(hx, hy);
        float handleRadius = 8.0f; // Make handle easier to hit
        float dx = worldPos_sfml.x - handlePos.x;
        float dy = worldPos_sfml.y - handlePos.y;
        if ((dx * dx + dy * dy) <= handleRadius * handleRadius) {
          deselectAllAndClearInteractionState(editor);
          editor.selectedObject = angle.get();
          editor.selectedObjects.clear();
          editor.selectedObjects.push_back(angle.get());
          angle->setSelected(true);
          editor.isResizingAngle = true;
          editor.isDragging = true;
          editor.dragMode = DragMode::None;
          editor.lastMousePos_sfml = worldPos_sfml;
          editor.dragStart_sfml = worldPos_sfml;
          return;
        }
      }
    }
  skipAngleArcHandle:

  if (editor.isTextEditing && editor.showSymbolPalette && !editor.useImGuiSymbolPalette) {
    std::string insert;
    if (editor.textSymbolPalette.handleClick(guiPos, insert)) {
      editor.textEditBuffer.insert(editor.textCursorIndex, insert);
      editor.textCursorIndex += insert.size();
      editor.textCursorBlinkClock.restart();
      if (editor.textEditingLabel) {
        editor.textEditingLabel->setRawContent(editor.textEditBuffer, editor.textEditIsRich);
        editor.textEditingLabel->setFontSize(editor.textEditFontSize);
      }
      return;
    }
  }

  if (editor.gui.isMouseOverPalette(guiPos)) {
    sf::Event eventToForward;
    eventToForward.type = sf::Event::MouseButtonPressed;
    eventToForward.mouseButton = mouseEvent;
    editor.gui.handleEvent(editor.window, eventToForward, editor);
    return;
  }

  bool gridSnapActive = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
  if (gridSnapActive) {
    float g = editor.getCurrentGridSpacing();
    if (g <= 0.0f) {
      g = Constants::GRID_SIZE;
    }
    worldPos_sfml.x = std::round(worldPos_sfml.x / g) * g;
    worldPos_sfml.y = std::round(worldPos_sfml.y / g) * g;
  }

  // std::cout << "Mouse press at pixel: (" << pixelPos.x << ", " << pixelPos.y << "), world: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")"
  //           << std::endl;

  Point_2 worldPos_cgal;
  try {
    worldPos_cgal = editor.toCGALPoint(worldPos_sfml);
    // std::cout << "Converted to CGAL: (" << CGAL::to_double(worldPos_cgal.x()) << ", " << CGAL::to_double(worldPos_cgal.y()) << ")" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error converting to CGAL point: " << e.what() << std::endl;
  }

  float tolerance = getDynamicSelectionTolerance(editor);
  // std::cout << "Dynamic tolerance: " << tolerance << std::endl;
  // std::cout << "Current tolerance: " << tolerance << std::endl;

  editor.showHoverMessage = false;
  if (editor.debugMode) {
    editor.debugObjectDetection(worldPos_sfml, tolerance);
  }

  bool eventHandledByGui = false;
  // std::cout << "GUI check: y=" << guiPos.y << ", threshold=" << (Constants::BUTTON_SIZE.y + 0.7f) << std::endl;
  if (guiPos.y <= Constants::BUTTON_SIZE.y + 0.7f) {
    // std::cout << "In GUI button area, forwarding to GUI handler..." << std::endl;
    sf::Event eventToForward;
    eventToForward.type = sf::Event::MouseButtonPressed;
    eventToForward.mouseButton = mouseEvent;
    if (editor.gui.handleEvent(editor.window, eventToForward, editor)) {
      // std::cout << "GUI handled the event!" << std::endl;
      eventHandledByGui = true;
    } else {
      std::cout << "GUI did not handle the event" << std::endl;
    }
  }
  if (eventHandledByGui) {
    return;
  }

  // Label dragging check - runs early to allow dragging labels
  // Note: Visibility is checked inside hitLabel via m_labelVisibility
  // Skip label dragging if we're actively creating objects (to avoid blocking point selection)
  bool isInCreationMode = (
    editor.m_currentToolType == ObjectType::Line ||
    editor.m_currentToolType == ObjectType::LineSegment ||
    editor.m_currentToolType == ObjectType::Ray ||
    editor.m_currentToolType == ObjectType::Vector ||
    editor.m_currentToolType == ObjectType::Circle ||
    editor.m_currentToolType == ObjectType::Semicircle ||
    editor.m_currentToolType == ObjectType::Circle3P ||
    editor.m_currentToolType == ObjectType::Rectangle ||
    editor.m_currentToolType == ObjectType::RectangleRotatable ||
    editor.m_currentToolType == ObjectType::Polygon ||
    editor.m_currentToolType == ObjectType::RegularPolygon ||
    editor.m_currentToolType == ObjectType::Triangle ||
    editor.m_currentToolType == ObjectType::Angle ||
    editor.m_currentToolType == ObjectType::ParallelLine ||
    editor.m_currentToolType == ObjectType::PerpendicularLine ||
    editor.m_currentToolType == ObjectType::PerpendicularBisector ||
    editor.m_currentToolType == ObjectType::AngleBisector ||
    editor.m_currentToolType == ObjectType::TangentLine ||
    editor.m_currentToolType == ObjectType::Compass
  );
  
  if (mouseEvent.button == sf::Mouse::Left && !isInCreationMode) {
    sf::Vector2f mouseScreenPos(static_cast<float>(pixelPos.x), static_cast<float>(pixelPos.y));

    // --- SELECTION STRATIFICATION: Points > Labels ---
    // Rule: If a point is under the mouse, we skip label selection/dragging entirely.
    // This ensures Points (and vertices) take priority over overlapping Labels.
    // Transformation tools also ignore labels to prevent accidental dragging.
    bool tToolActive = (editor.m_currentToolType == ObjectType::ReflectAboutLine || 
                         editor.m_currentToolType == ObjectType::ReflectAboutPoint ||
                         editor.m_currentToolType == ObjectType::ReflectAboutCircle || 
                         editor.m_currentToolType == ObjectType::RotateAroundPoint ||
                         editor.m_currentToolType == ObjectType::TranslateByVector || 
                         editor.m_currentToolType == ObjectType::DilateFromPoint);

    float ptPriorityTol = getDynamicSelectionTolerance(editor);
    bool hitPointPriority = false;
    for (auto it = editor.points.rbegin(); it != editor.points.rend(); ++it) {
        if ((*it) && (*it)->isVisible() && (*it)->contains(worldPos_sfml, ptPriorityTol)) { hitPointPriority = true; break; }
    }
    if (!hitPointPriority) {
        for (auto it = editor.ObjectPoints.rbegin(); it != editor.ObjectPoints.rend(); ++it) {
            if ((*it) && (*it)->isVisible() && (*it)->contains(worldPos_sfml, ptPriorityTol)) { hitPointPriority = true; break; }
        }
    }

    // Skip label dragging if a point is hit (priority) or if a transform tool is active.
    bool skipLabels = hitPointPriority || tToolActive;

    auto hitLabel = [&](GeometricObject* obj, sf::Vector2f& outLabelPos) -> bool {
      if (!obj) return false;
      
      // Global Visibility Check (mirrors render logic)
        if (editor.m_labelVisibility == GeometryEditor::LabelVisibilityMode::None) return false;
        if (editor.m_labelVisibility == GeometryEditor::LabelVisibilityMode::PointsOnly) {
          ObjectType t = obj->getType();
          bool isPointOrVertex = (t == ObjectType::Point || t == ObjectType::ObjectPoint || 
                     t == ObjectType::IntersectionPoint || t == ObjectType::Midpoint ||
                     t == ObjectType::Rectangle || t == ObjectType::Triangle ||
                     t == ObjectType::Polygon || t == ObjectType::RegularPolygon);
          if (!isPointOrVertex) return false;
        }

      if (!obj->getShowLabel()) return false;
      if (obj->getLabelMode() == LabelMode::Hidden) return false;
      if (!obj->isVisible()) return false;

      std::string labelStr = "";
      // Construct the label string exactly as rendered
      std::string label = obj->getLabel();
      std::string valueStr = "";
      
      // Get value string based on type
      if (obj->getType() == ObjectType::Angle) {
          if (auto* ang = dynamic_cast<Angle*>(obj)) {
               valueStr = std::to_string(static_cast<int>(std::round(ang->getCurrentDegrees()))) + "\xB0";
          }
      } 
      // Add other value types (Area, Length) here if needed for hit testing
      
      switch (obj->getLabelMode()) {
          case LabelMode::Name: labelStr = label; break;
          case LabelMode::Value: labelStr = valueStr; break;
          case LabelMode::NameAndValue: labelStr = label + (label.empty() ? "" : " = ") + valueStr; break;
          case LabelMode::Caption: labelStr = obj->getCaption(); break;
          default: break;
      }
      
      if (labelStr.empty()) return false;

      const sf::Font* labelFont = Point::commonFont ? Point::commonFont : (Button::getFontLoaded() ? &Button::getFont() : nullptr);
      if (!labelFont) return false;

      // Use the generic anchor from the object
      sf::Vector2f anchor = obj->getLabelAnchor(editor.drawingView);
      sf::Vector2i screenPos = editor.window.mapCoordsToPixel(anchor, editor.drawingView);
      
      sf::Vector2f labelPos(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
      labelPos += obj->getLabelOffset();
      
      // Default offset logic for Angle match rendering
      if (obj->getType() == ObjectType::Angle && obj->getLabelOffset() == sf::Vector2f(0.f, 0.f)) {
         labelPos += sf::Vector2f(10.f, -10.f);
      }

      labelPos.x = std::round(labelPos.x);
      labelPos.y = std::round(labelPos.y);

      sf::Text text;
      text.setFont(*labelFont);
      text.setString(sf::String::fromUtf8(labelStr.begin(), labelStr.end()));
      text.setCharacterSize(LabelManager::instance().getFontSize());
      text.setPosition(labelPos);
      
      // Center origin logic matches render
      sf::FloatRect bounds = text.getLocalBounds();
      text.setOrigin(bounds.width / 2.0f, bounds.height / 2.0f);
      
      // Recalculate global bounds after origin set

      bounds = text.getGlobalBounds();
      float dynamicTol = getDynamicSelectionTolerance(editor);
      bounds.left -= dynamicTol;
      bounds.top -= dynamicTol;
      bounds.width += dynamicTol * 2.0f;
      bounds.height += dynamicTol * 2.0f;

      if (bounds.contains(mouseScreenPos)) {
        outLabelPos = labelPos;
        return true;
      }
      return false;
    };

    auto tryLabelDrag = [&](const auto& container) -> bool {
      for (const auto& obj : container) {
        if (!obj) continue;
        sf::Vector2f labelPos;
        if (hitLabel(obj.get(), labelPos)) {
          deselectAllAndClearInteractionState(editor);
          editor.selectedObject = obj.get();
          editor.selectedObjects.clear();
          editor.selectedObjects.push_back(obj.get());
          obj->setSelected(true);

          editor.isDraggingLabel = true;
          editor.labelDragObject = obj.get();
          editor.labelDragGrabOffset = mouseScreenPos - labelPos;
          editor.isDragging = false;
          editor.dragMode = DragMode::None;
          editor.m_selectedEndpoint = EndpointSelection::None;
          return true;
        }
      }
      return false;
    };

    if (!skipLabels) {
      if (tryLabelDrag(editor.points)) return;
      if (tryLabelDrag(editor.ObjectPoints)) return;
      if (tryLabelDrag(editor.lines)) return;
      if (tryLabelDrag(editor.circles)) return;
      if (tryLabelDrag(editor.angles)) return;
      if (tryLabelDrag(editor.rectangles)) return;
      if (tryLabelDrag(editor.polygons)) return;
      if (tryLabelDrag(editor.regularPolygons)) return;
      if (tryLabelDrag(editor.triangles)) return;

      // Rectangle vertex label dragging
      for (const auto& rect : editor.rectangles) {
        if (!rect || !rect->isVisible()) continue;
        auto verts = rect->getVerticesSFML();
        const char* labels[] = {"A", "B", "C", "D"};

        for (size_t i = 0; i < verts.size() && i < 4; ++i) {
          sf::Vector2f worldPos = verts[i];
          sf::Vector2i screenPos = editor.window.mapCoordsToPixel(worldPos, editor.drawingView);
          sf::Vector2f labelPos(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
          labelPos += rect->getVertexLabelOffset(i);

          sf::Text text;
          text.setFont(Button::getFont());
          text.setString(labels[i]);
          text.setCharacterSize(LabelManager::instance().getFontSize());
          text.setPosition(labelPos);

          sf::FloatRect bounds = text.getGlobalBounds();
          float dynamicTol = getDynamicSelectionTolerance(editor);
          bounds.left -= dynamicTol;
          bounds.top -= dynamicTol;
          bounds.width += dynamicTol * 2.0f;
          bounds.height += dynamicTol * 2.0f;
          if (bounds.contains(mouseScreenPos)) {
            deselectAllAndClearInteractionState(editor);
            editor.selectedObject = rect.get();
            editor.selectedObjects.clear();
            editor.selectedObjects.push_back(rect.get());
            rect->setSelected(true);

            editor.isDraggingLabel = true;
            editor.labelDragObject = rect.get();
            editor.labelDragVertexIndex = static_cast<int>(i);
            editor.labelDragGrabOffset = mouseScreenPos - labelPos;
            editor.isDragging = false;
            editor.dragMode = DragMode::None;
            editor.m_selectedEndpoint = EndpointSelection::None;
            return;
          }
        }
      }

      // Angle label dragging
      for (const auto& angle : editor.angles) {
        if (!angle || !angle->isValid() || !angle->isVisible()) continue;

        // Calculate label position (similar to Angle::draw() updateSFMLShape())
        auto pointA = angle->getPointA().lock();
        auto vertex = angle->getVertex().lock();
        auto pointB = angle->getPointB().lock();

        if (!pointA || !vertex || !pointB) continue;

        Point_2 vertexCGAL = vertex->getCGALPosition();

        // Get angle arc parameters (need to calculate midAngle)
        Point_2 a = pointA->getCGALPosition();
        Point_2 v = vertexCGAL;
        Point_2 b = pointB->getCGALPosition();

        double vx = CGAL::to_double(v.x());
        double vy = CGAL::to_double(v.y());
        double ax = CGAL::to_double(a.x());
        double ay = CGAL::to_double(a.y());
        double bx = CGAL::to_double(b.x());
        double by = CGAL::to_double(b.y());

        double angle1 = std::atan2(ay - vy, ax - vx);
        double angle2 = std::atan2(by - vy, bx - vx);

        double sweep = angle2 - angle1;
        if (angle->isReflex()) {
          if (sweep >= 0)
            sweep -= 2.0 * 3.14159265358979323846;
          else
            sweep += 2.0 * 3.14159265358979323846;
        } else {
          if (sweep < 0) sweep += 2.0 * 3.14159265358979323846;
        }

        double midAngle = angle1 + sweep * 0.5;
        double textRadius = angle->getRadius() + 12.0;

        // Calculate label position in world coordinates
        sf::Vector2f labelWorldPos(static_cast<float>(vx + textRadius * std::cos(midAngle)), static_cast<float>(vy + textRadius * std::sin(midAngle)));

        // Convert to screen coordinates
        sf::Vector2i labelScreen = editor.window.mapCoordsToPixel(labelWorldPos, editor.drawingView);
        sf::Vector2f labelPos(static_cast<float>(labelScreen.x), static_cast<float>(labelScreen.y));

        // Apply label offset
        labelPos += angle->getLabelOffset();

        // Create text to get bounds
        std::string degreeStr = std::to_string(static_cast<int>(std::round(angle->getCurrentDegrees()))) + "°";
        sf::Text text;
        text.setFont(Button::getFont());
        text.setString(degreeStr);
        text.setCharacterSize(LabelManager::instance().getFontSize());
        text.setPosition(labelPos);

        sf::FloatRect bounds = text.getGlobalBounds();
        float dynamicTol = getDynamicSelectionTolerance(editor);
        bounds.left -= dynamicTol;
        bounds.top -= dynamicTol;
        bounds.width += dynamicTol * 2.0f;
        bounds.height += dynamicTol * 2.0f;
        if (bounds.contains(mouseScreenPos)) {
          deselectAllAndClearInteractionState(editor);
          editor.selectedObject = angle.get();
          editor.selectedObjects.clear();
          editor.selectedObjects.push_back(angle.get());
          angle->setSelected(true);

          editor.isDraggingLabel = true;
          editor.labelDragObject = angle.get();
          editor.labelDragGrabOffset = mouseScreenPos - labelPos;
          editor.isDragging = false;
          editor.dragMode = DragMode::None;
          editor.m_selectedEndpoint = EndpointSelection::None;
          return;
        }
      }
    }
  }

  if (mouseEvent.button == sf::Mouse::Right && sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Right+Ctrl at: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Space+Left at: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt)) {
    if (editor.m_currentToolType == ObjectType::None) {
      editor.isPanning = true;
      editor.lastMousePos_sfml = worldPos_sfml;
      std::cout << "Panning started with Alt+Left at: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")" << std::endl;
      return;
    }
  }

  if (mouseEvent.button == sf::Mouse::Right) {
    static sf::Clock lastRightClickClock;
    static int rightClickCount = 0;

    float timeSinceLast = lastRightClickClock.getElapsedTime().asSeconds();
    if (timeSinceLast < 0.4f) {
      rightClickCount++;
    } else {
      rightClickCount = 1;
    }
    lastRightClickClock.restart();


    // Open context menu on SINGLE right-click (more user-friendly)
    if (rightClickCount == 1) {
      float tolerance = getDynamicSelectionTolerance(editor);
      GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance);

      if (hitObj) {
        // Select the object for context menu actions
        editor.selectedObject = hitObj;
        
        // Add to selectedObjects if not already there (for multi-selection support)
        if (std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), hitObj) == editor.selectedObjects.end()) {
          editor.selectedObjects.push_back(hitObj);
        }
        
        std::cout << "Opening context menu for object type: " << static_cast<int>(hitObj->getType()) << std::endl;
        
        // Open context menu at GUI coordinates
        sf::View DefaultView = editor.window.getDefaultView();
        sf::Vector2f guiPos = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), DefaultView);
        
        auto& menu = editor.getGUI().getContextMenu();
        menu.clear();
        
        // --- 1. Header (Object Name/Type) ---
        // Maybe useful, or just skip to actions
        
        // --- 2. Visibility / Labeling ---
        bool showLabel = hitObj->getShowLabel();
        ObjectType type = hitObj->getType();
        
        // RE-INTRODUCE separate context menu toggles for "Vertex label" and "Point label"
        if (type == ObjectType::Point || type == ObjectType::ObjectPoint || 
            type == ObjectType::IntersectionPoint || type == ObjectType::Midpoint) {
            
            bool isVertex = false;
            if (auto* p = dynamic_cast<Point*>(hitObj)) {
                isVertex = p->isCreatedWithShape();
            }

            menu.addItem(showLabel ? (isVertex ? "Hide Vertex Label" : "Hide Point Label") 
                                   : (isVertex ? "Show Vertex Label" : "Show Point Label"), 
                         [hitObj](GeometryEditor& /*ed*/) {
                hitObj->setShowLabel(!hitObj->getShowLabel());
            });
        } else {
            menu.addItem(showLabel ? "Hide Label" : "Show Label", [hitObj](GeometryEditor& /*ed*/) {
                hitObj->setShowLabel(!hitObj->getShowLabel());
            });
        }

        // Shape-level multi-vertex toggle
        if (type == ObjectType::Rectangle || type == ObjectType::RectangleRotatable || 
            type == ObjectType::Triangle || type == ObjectType::Polygon || type == ObjectType::RegularPolygon) {
            
            menu.addItem("Toggle Vertex Labels", [hitObj](GeometryEditor& /*ed*/) {
                if (auto* rect = dynamic_cast<Rectangle*>(hitObj)) {
                    auto p1 = rect->getCorner1Point();
                    if (p1) {
                        bool state = !p1->getShowLabel();
                        if (auto p = rect->getCorner1Point()) p->setShowLabel(state);
                        if (auto p = rect->getCornerBPoint()) p->setShowLabel(state);
                        if (auto p = rect->getCorner2Point()) p->setShowLabel(state);
                        if (auto p = rect->getCornerDPoint()) p->setShowLabel(state);
                    }
                } else if (auto* tri = dynamic_cast<Triangle*>(hitObj)) {
                    if (tri->getVertexPoint(0)) {
                        bool state = !tri->getVertexPoint(0)->getShowLabel();
                        for (size_t i = 0; i < 3; ++i) if (auto p = tri->getVertexPoint(i)) p->setShowLabel(state);
                    }
                } else if (auto* poly = dynamic_cast<Polygon*>(hitObj)) {
                    if (poly->getVertexCount() > 0 && poly->getVertexPoint(0)) {
                        bool state = !poly->getVertexPoint(0)->getShowLabel();
                        for (size_t i = 0; i < poly->getVertexCount(); ++i) if (auto p = poly->getVertexPoint(i)) p->setShowLabel(state);
                    }
                }
            });
        }
        
        menu.addItem("Label Mode: Name", [hitObj](GeometryEditor&) { hitObj->setLabelMode(LabelMode::Name); });
        menu.addItem("Label Mode: Value", [hitObj](GeometryEditor&) { hitObj->setLabelMode(LabelMode::Value); });
        menu.addItem("Label Mode: Name & Value", [hitObj](GeometryEditor&) { hitObj->setLabelMode(LabelMode::NameAndValue); });
        menu.addItem("Label Mode: Caption", [hitObj](GeometryEditor&) { hitObj->setLabelMode(LabelMode::Caption); });

        auto collectTransformPoints = [](const std::vector<GeometricObject*>& objects) {
          std::set<Point*> uniquePoints;
          for (auto* obj : objects) {
            if (!obj) continue;
            if (auto* p = dynamic_cast<Point*>(obj)) {
              uniquePoints.insert(p);
              continue;
            }
            if (auto* line = dynamic_cast<Line*>(obj)) {
              if (line->getStartPointObject()) uniquePoints.insert(line->getStartPointObject());
              if (line->getEndPointObject()) uniquePoints.insert(line->getEndPointObject());
              continue;
            }
            if (auto* circle = dynamic_cast<Circle*>(obj)) {
              if (circle->getCenterPointObject()) uniquePoints.insert(circle->getCenterPointObject());
              if (circle->getRadiusPointObject()) uniquePoints.insert(circle->getRadiusPointObject());
              if (circle->isSemicircle()) {
                if (auto p1 = circle->getDiameterP1()) uniquePoints.insert(p1.get());
                if (auto p2 = circle->getDiameterP2()) uniquePoints.insert(p2.get());
              }
              continue;
            }
            if (auto* rect = dynamic_cast<Rectangle*>(obj)) {
              if (auto p1 = rect->getCorner1Point()) uniquePoints.insert(p1.get());
              if (auto p2 = rect->getCorner2Point()) uniquePoints.insert(p2.get());
              if (auto p3 = rect->getCornerBPoint()) uniquePoints.insert(p3.get());
              if (auto p4 = rect->getCornerDPoint()) uniquePoints.insert(p4.get());
              continue;
            }
            if (auto* tri = dynamic_cast<Triangle*>(obj)) {
              for (size_t i = 0; i < 3; ++i) {
                if (auto p = tri->getVertexPoint(i)) uniquePoints.insert(p.get());
              }
              continue;
            }
            if (auto* poly = dynamic_cast<Polygon*>(obj)) {
              for (size_t i = 0; i < poly->getVertexCount(); ++i) {
                if (auto p = poly->getVertexPoint(i)) uniquePoints.insert(p.get());
              }
              continue;
            }
            if (auto* reg = dynamic_cast<RegularPolygon*>(obj)) {
              if (auto p = reg->getCenterPoint()) uniquePoints.insert(p.get());
              if (auto p = reg->getFirstVertexPoint()) uniquePoints.insert(p.get());
              continue;
            }
          }
          return uniquePoints;
        };

        auto buildTransformTargets = [&]() {
          return !editor.selectedObjects.empty() ? editor.selectedObjects
                               : std::vector<GeometricObject*>{editor.selectedObject};
        };

        // --- 3. Quick Transforms ---
        menu.addItem("Rotate 90 deg CW", [collectTransformPoints, buildTransformTargets](GeometryEditor& /*ed*/) {
          auto objects = buildTransformTargets();
          auto uniquePoints = collectTransformPoints(objects);
          if (uniquePoints.empty()) return;

          sf::Vector2f center(0, 0);
          for (auto* p : uniquePoints) {
            center += p->getSFMLPosition();
          }
          if (uniquePoints.size() > 1) {
            center /= static_cast<float>(uniquePoints.size());
          }

          for (auto* p : uniquePoints) {
            sf::Vector2f pos = p->getSFMLPosition();
            sf::Vector2f relative = pos - center;
            sf::Vector2f rotated(relative.y, -relative.x);
            p->setPosition(center + rotated);
          }
        });

        menu.addItem("Rotate 90 deg CCW", [collectTransformPoints, buildTransformTargets](GeometryEditor& /*ed*/) {
          auto objects = buildTransformTargets();
          auto uniquePoints = collectTransformPoints(objects);
          if (uniquePoints.empty()) return;

          sf::Vector2f center(0, 0);
          for (auto* p : uniquePoints) {
            center += p->getSFMLPosition();
          }
          if (uniquePoints.size() > 1) {
            center /= static_cast<float>(uniquePoints.size());
          }

          for (auto* p : uniquePoints) {
            sf::Vector2f pos = p->getSFMLPosition();
            sf::Vector2f relative = pos - center;
            sf::Vector2f rotated(-relative.y, relative.x);
            p->setPosition(center + rotated);
          }
        });

        menu.addItem("Flip Horizontal", [collectTransformPoints, buildTransformTargets](GeometryEditor& /*ed*/) {
          auto objects = buildTransformTargets();
          auto uniquePoints = collectTransformPoints(objects);
          if (uniquePoints.empty()) return;

          sf::Vector2f center(0, 0);
          for (auto* p : uniquePoints) {
            center += p->getSFMLPosition();
          }
          if (uniquePoints.size() > 1) {
            center /= static_cast<float>(uniquePoints.size());
          }

          for (auto* p : uniquePoints) {
            sf::Vector2f pos = p->getSFMLPosition();
            sf::Vector2f relative = pos - center;
            sf::Vector2f flipped(-relative.x, relative.y);
            p->setPosition(center + flipped);
          }
        });

        menu.addItem("Flip Vertical", [collectTransformPoints, buildTransformTargets](GeometryEditor& /*ed*/) {
          auto objects = buildTransformTargets();
          auto uniquePoints = collectTransformPoints(objects);
          if (uniquePoints.empty()) return;

          sf::Vector2f center(0, 0);
          for (auto* p : uniquePoints) {
            center += p->getSFMLPosition();
          }
          if (uniquePoints.size() > 1) {
            center /= static_cast<float>(uniquePoints.size());
          }

          for (auto* p : uniquePoints) {
            sf::Vector2f pos = p->getSFMLPosition();
            sf::Vector2f relative = pos - center;
            sf::Vector2f flipped(relative.x, -relative.y);
            p->setPosition(center + flipped);
          }
        });
        
        // --- 3. Object Specifics ---
        
        
        

        
        if (type == ObjectType::Angle) {
            auto* ang = dynamic_cast<Angle*>(hitObj);
            if (ang) {
                 menu.addItem(ang->isReflex() ? "Show Normal Angle" : "Show Reflex Angle", [ang](GeometryEditor& /*ed*/) {
                     ang->setReflex(!ang->isReflex());
                 });
            }
        }
        
        // --- 4. Transform Tools ---
        menu.addItem("Reflect about Line", [](GeometryEditor& ed) {
          ed.setCurrentTool(ObjectType::ReflectAboutLine);
          ed.setToolHint("Reflect: Select line to reflect about.");
        });
        menu.addItem("Reflect about Point", [](GeometryEditor& ed) {
          ed.setCurrentTool(ObjectType::ReflectAboutPoint);
          ed.setToolHint("Reflect: Select center point.");
        });
        menu.addItem("Invert (Circle)", [](GeometryEditor& ed) {
          ed.setCurrentTool(ObjectType::ReflectAboutCircle);
          ed.setToolHint("Invert: Select circle for inversion.");
        });
        menu.addItem("Rotate around Point", [](GeometryEditor& ed) {
          ed.setCurrentTool(ObjectType::RotateAroundPoint);
          ed.setToolHint("Rotate: Select pivot point.");
        });
        menu.addItem("Translate by Vector", [](GeometryEditor& ed) {
          ed.setCurrentTool(ObjectType::TranslateByVector);
          ed.setToolHint("Translate: Select vector end point.");
        });
        menu.addItem("Dilate from Point", [](GeometryEditor& ed) {
          ed.setCurrentTool(ObjectType::DilateFromPoint);
          ed.setToolHint("Dilate: Select center point.");
        });

        // --- 5. General Actions ---
        menu.addItem("Delete", [hitObj](GeometryEditor& ed) {
            std::vector<std::shared_ptr<GeometricObject>> toDelete;
            // Find shared ptr
            std::shared_ptr<GeometricObject> ptr = ed.findSharedPtr(hitObj);
            if (ptr) {
                toDelete.push_back(ptr);
                auto cmd = std::make_shared<DeleteCommand>(ed, toDelete);
                ed.commandManager.execute(cmd);
            }
        });

        menu.open(guiPos, editor);

        editor.isPanning = false;
        rightClickCount = 0;
        return;
      }
    }

    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Right click" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && !eventHandledByGui) {
    editor.isPanning = false;
  }

  if (eventHandledByGui) {
    return;
  }

  switch (editor.m_currentToolType) {
    case ObjectType::Hide: {
      if (mouseEvent.button == sf::Mouse::Left) {
        float tolerance = getDynamicSelectionTolerance(editor);
        GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance);
        if (hitObj) {
          hitObj->setVisible(!hitObj->isVisible());
          hitObj->setSelected(false);
          hitObj->setHovered(false);
          if (editor.selectedObject == hitObj) {
            editor.selectedObject = nullptr;
          }
        }
        return;
      }
      break;
    }
    case ObjectType::Detach: {
      if (mouseEvent.button == sf::Mouse::Left) {
        float tolerance = getDynamicSelectionTolerance(editor);
        double bestDistSq = static_cast<double>(tolerance) * static_cast<double>(tolerance);

        GeometricObject* bestObj = nullptr;
        std::shared_ptr<Point> pointToDetach = nullptr;
        enum class PointRole { None, LineStart, LineEnd, CircleCenter, CircleRadius };
        PointRole role = PointRole::None;

        Point_2 clickPos = editor.toCGALPoint(worldPos_sfml);

        for (auto& linePtr : editor.lines) {
          if (!linePtr || !linePtr->isValid() || !linePtr->isVisible()) continue;
          auto start = linePtr->getStartPointObjectShared();
          auto end = linePtr->getEndPointObjectShared();

          if (start) {
            double d1 = CGAL::to_double(CGAL::squared_distance(clickPos, start->getCGALPosition()));
            if (d1 <= bestDistSq) {
              bestDistSq = d1;
              bestObj = linePtr.get();
              pointToDetach = start;
              role = PointRole::LineStart;
            }
          }

          if (end) {
            double d2 = CGAL::to_double(CGAL::squared_distance(clickPos, end->getCGALPosition()));
            if (d2 <= bestDistSq) {
              bestDistSq = d2;
              bestObj = linePtr.get();
              pointToDetach = end;
              role = PointRole::LineEnd;
            }
          }
        }

        for (auto& circlePtr : editor.circles) {
          if (!circlePtr || !circlePtr->isValid() || !circlePtr->isVisible()) continue;

          Point* centerRaw = circlePtr->getCenterPointObject();
          std::shared_ptr<Point> centerShared;

          if (centerRaw) {
            for (auto& p : editor.points) {
              if (p.get() == centerRaw) {
                centerShared = p;
                break;
              }
            }
            if (!centerShared) {
              for (auto& p : editor.ObjectPoints) {
                if (p.get() == centerRaw) {
                  centerShared = std::static_pointer_cast<Point>(p);
                  break;
                }
              }
            }
          }

          if (centerShared) {
            double dCenter = CGAL::to_double(CGAL::squared_distance(clickPos, centerShared->getCGALPosition()));
            if (dCenter <= bestDistSq) {
              bestDistSq = dCenter;
              bestObj = circlePtr.get();
              pointToDetach = centerShared;
              role = PointRole::CircleCenter;
            }
          }

          Point* radiusRaw = circlePtr->getRadiusPointObject();
          std::shared_ptr<Point> radiusShared;
          if (radiusRaw) {
            for (auto& p : editor.points) {
              if (p.get() == radiusRaw) {
                radiusShared = p;
                break;
              }
            }
            if (!radiusShared) {
              for (auto& p : editor.ObjectPoints) {
                if (p.get() == radiusRaw) {
                  radiusShared = std::static_pointer_cast<Point>(p);
                  break;
                }
              }
            }
          }

          if (radiusShared) {
            double dRadius = CGAL::to_double(CGAL::squared_distance(clickPos, radiusShared->getCGALPosition()));
            if (dRadius <= bestDistSq) {
              bestDistSq = dRadius;
              bestObj = circlePtr.get();
              pointToDetach = radiusShared;
              role = PointRole::CircleRadius;
            }
          }
        }

        if (bestObj && pointToDetach) {
          int usageCount = 0;

          for (auto& linePtr : editor.lines) {
            if (!linePtr || !linePtr->isValid()) continue;
            if (linePtr->getStartPointObjectShared() == pointToDetach || linePtr->getEndPointObjectShared() == pointToDetach) {
              ++usageCount;
            }
          }

          for (auto& circlePtr : editor.circles) {
            if (!circlePtr || !circlePtr->isValid()) continue;
            Point* c = circlePtr->getCenterPointObject();
            Point* r = circlePtr->getRadiusPointObject();
            if (c == pointToDetach.get()) ++usageCount;
            if (r == pointToDetach.get()) ++usageCount;
          }

          if (usageCount > 1) {
            auto newPoint =
                std::make_shared<Point>(pointToDetach->getCGALPosition(), 1.0f, pointToDetach->getFillColor(), pointToDetach->getOutlineColor());
            newPoint->setVisible(true);
            newPoint->update();
            editor.points.push_back(newPoint);

            if (role == PointRole::LineStart) {
              static_cast<Line*>(bestObj)->setPoints(newPoint, static_cast<Line*>(bestObj)->getEndPointObjectShared());
            } else if (role == PointRole::LineEnd) {
              static_cast<Line*>(bestObj)->setPoints(static_cast<Line*>(bestObj)->getStartPointObjectShared(), newPoint);
            } else if (role == PointRole::CircleCenter) {
              static_cast<Circle*>(bestObj)->setCenterPointObject(newPoint.get());
            } else if (role == PointRole::CircleRadius) {
              static_cast<Circle*>(bestObj)->setRadiusPoint(newPoint);
            }

            editor.selectedObject = newPoint.get();
            editor.selectedObject->setSelected(true);
            editor.setGUIMessage("Endpoint/Center detached.");
          } else {
            editor.setGUIMessage("Point is not shared.");
          }
        }
        return;
      }
      break;
    }
    case ObjectType::Midpoint: {
      if (mouseEvent.button == sf::Mouse::Left) {
        sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
        sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
        float tolerance = std::max(getDynamicSelectionTolerance(editor), getDynamicSnapTolerance(editor));
        GeometricObject* clickedObj = findClosestObject(editor, worldPos, tolerance);
        handleMidpointToolClick(editor, clickedObj);
        return;
      }
      break;
    }
    case ObjectType::Compass: {
      if (mouseEvent.button == sf::Mouse::Left) {
        sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
        sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
        float tolerance = std::max(getDynamicSelectionTolerance(editor), getDynamicSnapTolerance(editor));
        GeometricObject* clickedObj = findClosestObject(editor, worldPos, tolerance);
        handleCompassToolClick(editor, clickedObj, worldPos, tolerance);
        return;
      }
      break;
    }
    case ObjectType::ReflectAboutLine:
    case ObjectType::ReflectAboutPoint:
    case ObjectType::ReflectAboutCircle:
    case ObjectType::RotateAroundPoint:
    case ObjectType::TranslateByVector:
    case ObjectType::DilateFromPoint: {
      if (mouseEvent.button == sf::Mouse::Left) {
        sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
        sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
        float tolerance = getDynamicSelectionTolerance(editor);
        if (handleTransformationCreation(editor, tempSelectedObjects, worldPos, tolerance)) return;
      }
      break;
    }
    case ObjectType::Angle: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleAngleCreation(editor, worldPos_sfml);
        return;
      }
      break;
    }
    case ObjectType::Point: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handlePointCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::Intersection: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleIntersectionCreation(editor, worldPos_sfml);
        return;
      }
      break;
    }
    case ObjectType::ParallelLine: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleParallelLineCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::PerpendicularLine: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handlePerpendicularLineCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::PerpendicularBisector: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handlePerpendicularBisectorCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::AngleBisector: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleAngleBisectorCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::TangentLine: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleTangentCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::Circle: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleCircleCreation(editor, mouseEvent, worldPos_sfml);
        return;
      }
      break;
    }
    case ObjectType::Line: {
      if (mouseEvent.button == sf::Mouse::Left) {
        try {
          handleLineCreation(editor, mouseEvent);
        } catch (const std::exception& e) {
          std::cerr << "handleLineCreation threw exception: " << e.what() << std::endl;
        } catch (...) {
          std::cerr << "handleLineCreation threw unknown exception" << std::endl;
        }
        return;
      }
      break;
    }
    case ObjectType::LineSegment: {
      if (mouseEvent.button == sf::Mouse::Left) {
        try {
          handleLineSegmentCreation(editor, mouseEvent);
        } catch (const std::exception& e) {
          std::cerr << "handleLineCreation threw exception: " << e.what() << std::endl;
        } catch (...) {
          std::cerr << "handleLineCreation threw unknown exception" << std::endl;
        }
        return;
      }
      break;
    }
    case ObjectType::ObjectPoint: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleObjectPointCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::Rectangle: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleRectangleCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::RectangleRotatable: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleRotatableRectangleCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::Polygon: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handlePolygonCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::RegularPolygon: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleRegularPolygonCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::Triangle: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleTriangleCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::Ray: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleRayCreation(editor, mouseEvent);
        return;
      }
      break;
    }

    case ObjectType::Vector: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleVectorCreation(editor, mouseEvent);
        return;
      }
      break;
    }

    case ObjectType::Semicircle: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleSemicircleCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::Circle3P: {
      if (mouseEvent.button == sf::Mouse::Left) {
        handleCircle3pCreation(editor, mouseEvent);
        return;
      }
      break;
    }
    case ObjectType::TextLabel: {
      static sf::Clock doubleClickClock;
      static GeometricObject* lastClickedObj = nullptr;

      if (mouseEvent.button == sf::Mouse::Left) {
        sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
        sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
        float tolerance = std::max(getDynamicSelectionTolerance(editor), editor.getScaledTolerance(editor.drawingView) * 10.0f); // Allow generous hit for text
        
        // Strict check for text labels
        std::vector<ObjectType> allowed = {ObjectType::TextLabel};
        GeometricObject* clickedObj = editor.lookForObjectAt(worldPos, tolerance, allowed);

        if (clickedObj && clickedObj->getType() == ObjectType::TextLabel) {
            // Check for Double Click
            if (lastClickedObj == clickedObj && doubleClickClock.getElapsedTime().asMilliseconds() < 500) {
                 std::cout << "Double Click detected on TextLabel!" << std::endl;
                 auto textLabel = std::dynamic_pointer_cast<TextLabel>(editor.findSharedPtr(clickedObj));
                 if (textLabel) {
                     editor.textEditingLabel = textLabel;
                     editor.isEditingExistingText = true;
                     editor.textEditorDialog.open(textLabel->getRawContent(), textLabel->isRichText(), textLabel->getFontSize());
                     
                     // Reset Double Click state
                     lastClickedObj = nullptr;
                     return;
                 }
            }
            lastClickedObj = clickedObj;
            doubleClickClock.restart();
        } else {
            lastClickedObj = nullptr;
        }

        // Ensure new text creation doesn't inherit "editing" state
        editor.isEditingExistingText = false;
        handleTextTool(editor, worldPos);
        return;
      }
      break;
    }
  
    case ObjectType::AngleGiven: {
      if (mouseEvent.button == sf::Mouse::Left) {
        sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
        sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
        float tolerance = getDynamicSnapTolerance(editor);

        std::shared_ptr<Point> clickedPoint = createSmartPointFromClick(editor, worldPos, tolerance);
        if (!clickedPoint) return;

        if (!editor.anglePointA) {
          editor.anglePointA = clickedPoint;
          editor.setGUIMessage("Select Vertex Point.");
        } else if (!editor.angleVertex) {
          editor.angleVertex = clickedPoint;
          if (editor.angleVertex == editor.anglePointA) {
            editor.setGUIMessage("Vertex cannot be same as Point A.");
            editor.angleVertex = nullptr;
            return;
          }
          // Trigger Popup
          g_showAngleInputPopup = true;
          editor.setGUIMessage("Enter angle in popup.");
        }
        return;
      }
      break;
    }
    case ObjectType::None:
    default:
      break;
  }

  bool isTransformToolActive = (editor.m_currentToolType == ObjectType::ReflectAboutLine || 
                                editor.m_currentToolType == ObjectType::ReflectAboutPoint ||
                                editor.m_currentToolType == ObjectType::ReflectAboutCircle || 
                                editor.m_currentToolType == ObjectType::RotateAroundPoint ||
                                editor.m_currentToolType == ObjectType::TranslateByVector || 
                                editor.m_currentToolType == ObjectType::DilateFromPoint);

  if (mouseEvent.button == sf::Mouse::Left && (editor.m_currentToolType == ObjectType::None || isTransformToolActive)) {

    // Always update the potential selection box start position for this click
    // This prevents stale coordinates from previous clicks causing false selection box triggers
    editor.potentialSelectionBoxStart_sfml = worldPos_sfml;

    GeometricObject* potentialSelection = nullptr;
    DragMode potentialDragMode = DragMode::None;
    EndpointSelection potentialEndpointSelection = EndpointSelection::None;
    int potentialVertexIndex = -1;

    editor.activeVertexIndex = -1;
    editor.activeVertexShape = nullptr;

    std::cout << "Looking for object at (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ") with tolerance " << tolerance << std::endl;


    GeometricObject* closestObject = findClosestObject(editor, worldPos_sfml, tolerance);
    // --- Label for goto ---

    if (closestObject && !potentialSelection) {
      potentialSelection = closestObject;
      ObjectType type = closestObject->getType();
      // Map Object to DragMode
      if (type == ObjectType::Point || type == ObjectType::IntersectionPoint) {
        potentialDragMode = closestObject->isLocked() ? DragMode::None : DragMode::MoveFreePoint;
      } else if (type == ObjectType::ObjectPoint) {
        potentialDragMode = closestObject->isLocked() ? DragMode::None : DragMode::DragObjectPoint;
      } else if (type == ObjectType::Line || type == ObjectType::LineSegment || type == ObjectType::Ray || type == ObjectType::Vector) {
        auto* line = static_cast<Line*>(closestObject);
        // check for constrained endpoints to decide if we can translate
        Point* startPt = line->getStartPointObject();
        Point* endPt = line->getEndPointObject();
        bool startConstrained = (dynamic_cast<ObjectPoint*>(startPt) != nullptr) || (startPt && startPt->isLocked());
        bool endConstrained = (dynamic_cast<ObjectPoint*>(endPt) != nullptr) || (endPt && endPt->isLocked());
        if (startConstrained || endConstrained) {
          potentialDragMode = DragMode::None;
        } else {
          potentialDragMode = DragMode::TranslateLine;
        }
      } else if (type == ObjectType::Circle || type == ObjectType::Semicircle) {
        potentialDragMode = DragMode::InteractWithCircle;
      } else if (type == ObjectType::Angle) {
        potentialDragMode = DragMode::None; // Angles usually not dragged directly
        if (static_cast<Angle*>(closestObject)->isMouseOverArc(worldPos_sfml, tolerance)) {
          editor.isResizingAngle = true;
        }
      } else if (type == ObjectType::TextLabel) {
        potentialDragMode = DragMode::TranslateShape;
      } else {
        // Shapes (Rectangle, Polygon, etc.)
        potentialDragMode = DragMode::TranslateShape;
      }

      // Check for vertex hits if it's a shape
      auto checkVertexHit = [&](GeometricObject* obj) -> int {
        if (!obj) return -1;
        auto verts = obj->getInteractableVertices();
        for (size_t i = 0; i < verts.size(); ++i) {
          float vx = static_cast<float>(CGAL::to_double(verts[i].x()));
          float vy = static_cast<float>(CGAL::to_double(verts[i].y()));
          float dx = vx - worldPos_sfml.x;
          float dy = vy - worldPos_sfml.y;
          if (std::sqrt(dx*dx + dy*dy) <= tolerance * 1.5f) return static_cast<int>(i);
        }
        return -1;
      };

      int vIdx = checkVertexHit(closestObject);
      if (vIdx != -1) {
        potentialDragMode = DragMode::MoveShapeVertex;
        potentialVertexIndex = vIdx;
      }
    }

    // --- Restore edge hit detection for transformation tools ---
    if (isTransformToolActive && !potentialSelection) {
      auto checkShapeEdges = [&](auto& container) -> bool {
        for (auto& shape : container) {
          if (!shape || !shape->isValid() || !shape->isVisible() || shape->isLocked()) continue;
          auto edges = shape->getEdges();
          for (const auto& edge : edges) {
            Point_2 proj;
            double relPos;
            double dist = PointUtils::projectPointOntoSegment(editor.toCGALPoint(worldPos_sfml), edge, proj, relPos);
            if (dist < tolerance) {
              std::cout << "Found Shape Edge for Transform" << std::endl;
              potentialSelection = shape.get();
              potentialDragMode = DragMode::TranslateShape;
              break;
            }
          }
        }
        return false;
      };
      checkShapeEdges(editor.rectangles);
      checkShapeEdges(editor.polygons);
      checkShapeEdges(editor.regularPolygons);
      checkShapeEdges(editor.triangles);
    }

    if (potentialSelection) {
      std::cout << "Object found! Type: " << static_cast<int>(potentialSelection->getType()) << std::endl;

      bool isCtrlHeld = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl);

      if (isCtrlHeld) {
        auto it = std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), potentialSelection);
        if (it != editor.selectedObjects.end()) {
          potentialSelection->setSelected(false);
          editor.selectedObjects.erase(it);
          if (editor.selectedObject == potentialSelection) {
            editor.selectedObject = (editor.selectedObjects.empty() ? nullptr : editor.selectedObjects.back());
          }
        } else {
          potentialSelection->setSelected(true);
          editor.selectedObjects.push_back(potentialSelection);
          editor.selectedObject = potentialSelection;
        }
      } else {
        auto it = std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), potentialSelection);
        if (it == editor.selectedObjects.end()) {
          deselectAllAndClearInteractionState(editor);
          editor.selectedObject = potentialSelection;
          editor.selectedObjects.push_back(potentialSelection);
          potentialSelection->setSelected(true);
        } else {
          editor.selectedObject = potentialSelection;
        }
      }

      editor.dragMode = potentialDragMode;
      editor.m_selectedEndpoint = potentialEndpointSelection;
      editor.isDragging = true;
      editor.lastMousePos_sfml = worldPos_sfml;
      editor.dragStart_sfml = worldPos_sfml;

      if (editor.selectedObject && editor.selectedObject->isLocked()) {
        editor.dragMode = DragMode::None;
        editor.m_selectedEndpoint = EndpointSelection::None;
        editor.isDragging = true; // KEEP TRUE to block selection box
        editor.isDrawingSelectionBox = false; // Explicitly prevent selection box
        editor.potentialSelectionBoxStart_sfml = worldPos_sfml; // Update to current position to prevent stale delta
        
        // Debug: Check if it's an intersection point
        if (editor.selectedObject->getType() == ObjectType::IntersectionPoint) {
          auto* pt = static_cast<Point*>(editor.selectedObject);
          std::cout << "IntersectionPoint selected! m_isIntersectionPoint=" << pt->isIntersectionPoint() 
                    << ", isSelected=" << pt->isSelected() << std::endl;
        } else {
          std::cout << "Locked object selected. Interaction captured." << std::endl;
        }
        return;
      }

      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        if (auto* rect = dynamic_cast<Rectangle*>(potentialSelection)) rect->setActiveVertex(potentialVertexIndex);
        else if (auto* poly = dynamic_cast<Polygon*>(potentialSelection)) poly->setActiveVertex(potentialVertexIndex);
        else if (auto* reg = dynamic_cast<RegularPolygon*>(potentialSelection)) reg->setActiveVertex(potentialVertexIndex);
        else if (auto* tri = dynamic_cast<Triangle*>(potentialSelection)) tri->setActiveVertex(potentialVertexIndex);
      }

    } else {
      deselectAllAndClearInteractionState(editor);
      editor.selectedObject = nullptr;
      editor.potentialSelectionBoxStart_sfml = worldPos_sfml;
      editor.isDrawingSelectionBox = false;
      return;
    }
  }
}
//