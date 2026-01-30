#include "CharTraitsFix.h"
#include <string>
#include "PointUtils.h"
#include "GeometryEditor.h"
#include "Point.h"
#include "ObjectPoint.h"
#include "Line.h"
#include "Circle.h"
#include "Rectangle.h"
#include "Triangle.h"
#include "Polygon.h"
#include "RegularPolygon.h"
#include "Intersection.h"
#include "IntersectionSystem.h"
#include "Constants.h"
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>
#include <unordered_set>

std::shared_ptr<Point> PointUtils::findAnchorPoint(
    GeometryEditor& editor,
    const sf::Vector2f& worldPos_sfml,
    float tolerance) {
  
  std::shared_ptr<Point> best = nullptr;
  float bestDist2 = std::numeric_limits<float>::max();
  const float tolerance2 = tolerance * tolerance;
  
  // Priority 1: Search free points
  for (auto& pt : editor.points) {
    if (pt && pt->isValid() && pt->isVisible()) {
      sf::Vector2f ptPos = pt->getSFMLPosition();
      float dx = ptPos.x - worldPos_sfml.x;
      float dy = ptPos.y - worldPos_sfml.y;
      float d2 = dx * dx + dy * dy;
      if (d2 <= tolerance2 && d2 < bestDist2) {
        best = pt;
        bestDist2 = d2;
      }
    }
  }
  
  // Priority 2: Search ObjectPoints
  for (auto& op : editor.ObjectPoints) {
    if (op && op->isValid() && op->isVisible()) {
      sf::Vector2f opPos = op->getSFMLPosition();
      float dx = opPos.x - worldPos_sfml.x;
      float dy = opPos.y - worldPos_sfml.y;
      float d2 = dx * dx + dy * dy;
      if (d2 <= tolerance2 && d2 < bestDist2) {
        best = std::static_pointer_cast<Point>(op);
        bestDist2 = d2;
      }
    }
  }
  
  // If we found a free point or ObjectPoint, return it (highest priority)
  if (best) {
    editor.setGUIMessage("Snapped to Vertex"); // Feedback
    return best;
  }
  
  // Priority 3: Search line endpoints (even if hidden, since lines hide their endpoint Points)
  for (auto& linePtr : editor.lines) {
    if (!linePtr || !linePtr->isValid() || !linePtr->isVisible()) continue;
    
    // Check start endpoint
    std::shared_ptr<Point> startPt = linePtr->getStartPointObjectShared();
    if (startPt && startPt->isValid()) {
      sf::Vector2f ptPos = startPt->getSFMLPosition();
      float dx = ptPos.x - worldPos_sfml.x;
      float dy = ptPos.y - worldPos_sfml.y;
      float d2 = dx * dx + dy * dy;
      if (d2 <= tolerance2 && d2 < bestDist2) {
        best = startPt;
        bestDist2 = d2;
      }
    }
    
    // Check end endpoint
    std::shared_ptr<Point> endPt = linePtr->getEndPointObjectShared();
    if (endPt && endPt->isValid()) {
      sf::Vector2f ptPos = endPt->getSFMLPosition();
      float dx = ptPos.x - worldPos_sfml.x;
      float dy = ptPos.y - worldPos_sfml.y;
      float d2 = dx * dx + dy * dy;
      if (d2 <= tolerance2 && d2 < bestDist2) {
        best = endPt;
        bestDist2 = d2;
      }
    }
  }
  
  // If we found a line endpoint, return it
  if (best) {
    editor.setGUIMessage("Snapped to Line Endpoint"); // Feedback
    return best;
  }
  
  // Priority 4: Search shape vertices via generic getInteractableVertices()
  GeometricObject* bestShape = nullptr;
  size_t bestVertexIndex = 0;
  
  auto checkShapeVertices = [&](auto& container) {
    for (auto& shapePtr : container) {
      if (!shapePtr || !shapePtr->isValid() || !shapePtr->isVisible()) continue;
      
      auto vertices = shapePtr->getInteractableVertices();
      // Debug: Log vertex search
      std::cout << "[PointUtils] Checking shape type=" << static_cast<int>(shapePtr->getType()) 
                << " with " << vertices.size() << " vertices" << std::endl;
      
      for (size_t i = 0; i < vertices.size(); ++i) {
        float vx = static_cast<float>(CGAL::to_double(vertices[i].x()));
        float vy = static_cast<float>(CGAL::to_double(vertices[i].y()));
        float dx = vx - worldPos_sfml.x;
        float dy = vy - worldPos_sfml.y;
        float d2 = dx * dx + dy * dy;
        
        // Debug: Log every vertex check
        std::cout << "  Vertex " << i << " at (" << vx << "," << vy 
                  << ") dist^2=" << d2 << " threshold^2=" << tolerance2 << std::endl;
        
        if (d2 <= tolerance2 && d2 < bestDist2) {
          std::cout << "  -> HIT! Vertex is within tolerance" << std::endl;
          bestShape = shapePtr.get();
          bestVertexIndex = i;
          bestDist2 = d2;
          best = nullptr; // Clear any previously found point, we'll create one if this remains best
        }
      }
    }
  };
  
  checkShapeVertices(editor.rectangles);
  checkShapeVertices(editor.polygons);
  checkShapeVertices(editor.regularPolygons);
  checkShapeVertices(editor.triangles);
  
  // Check circle centers
  for (auto& circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid() || !circlePtr->isVisible()) continue;
    
    // Check center point
    Point_2 center = circlePtr->getCenterPoint();
    float cx = static_cast<float>(CGAL::to_double(center.x()));
    float cy = static_cast<float>(CGAL::to_double(center.y()));
    float dx = cx - worldPos_sfml.x;
    float dy = cy - worldPos_sfml.y;
    float d2 = dx * dx + dy * dy;
    
    if (d2 <= tolerance2 && d2 < bestDist2) {
      // Circle center is an actual Point object
      Point* centerPt = circlePtr->getCenterPointObject();
      if (centerPt) {
        // Find the shared_ptr in editor.points
        for (auto& pt : editor.points) {
          if (pt.get() == centerPt) {
            best = pt;
            bestDist2 = d2;
            bestShape = nullptr; // Prioritize existing point
            break;
          }
        }
      }
    }
  }
  
  // If we found a shape vertex and it's better than any existing point
  if (!best && bestShape) {
       // Create a new ObjectPoint attached to this vertex
       // We map Vertex Index i to Edge Index i at position 0.0
       // This works for Rectangle, Triangle, Polygon, RegularPolygon where |Vertices| == |Edges|
       // and Vertex i is the start of Edge i.
       
       // Need to find the shared_ptr for the shape
       std::shared_ptr<GeometricObject> shapeSharedPtr;
       
       // Helper to find shared_ptr in container (a bit slow but robust)
       auto findShared = [&](auto& container) -> std::shared_ptr<GeometricObject> {
           for(auto& ptr : container) {
               if(ptr.get() == bestShape) return ptr;
           }
           return nullptr;
       };
       
       if (bestShape->getType() == ObjectType::Rectangle || bestShape->getType() == ObjectType::RectangleRotatable) 
           shapeSharedPtr = findShared(editor.rectangles);
       else if (bestShape->getType() == ObjectType::Triangle)
           shapeSharedPtr = findShared(editor.triangles);
       else if (bestShape->getType() == ObjectType::Polygon)
           shapeSharedPtr = findShared(editor.polygons);
       else if (bestShape->getType() == ObjectType::RegularPolygon)
           shapeSharedPtr = findShared(editor.regularPolygons);
           
       if (shapeSharedPtr) {
           // Verify edge count matches vertex count implies standard closed polygon topology
           // Vertex i -> Edge i (start)
           // Safety check:
           if (bestVertexIndex < shapeSharedPtr->getEdges().size()) {
               auto newPoint = ObjectPoint::createOnShapeEdge(shapeSharedPtr, bestVertexIndex, 0.0);
               if (newPoint) {
                   newPoint->setIsVertexAnchor(true);  // Mark as vertex anchor - resizes shape when dragged
               }
               // We return the new point. The caller is responsible for adding it 
               // to the editor.ObjectPoints if they intend to keep it (e.g. on click).
               // editor.ObjectPoints.push_back(newPoint); // REMOVED to prevent clutter on hover
               return newPoint;
           }
       }
  }
  
  return best;
}

std::optional<EdgeHitResult> PointUtils::findNearestEdge(
    GeometryEditor& editor,
    const sf::Vector2f& worldPos_sfml,
    float tolerance) {
  
  Point_2 mousePos(FT(worldPos_sfml.x), FT(worldPos_sfml.y));
  
  std::optional<EdgeHitResult> best = std::nullopt;
  double bestDist = static_cast<double>(tolerance);
  
  auto checkShapeEdges = [&](auto& container) {
    for (auto& shapePtr : container) {
      if (!shapePtr || !shapePtr->isValid()) continue;
      
      auto edges = shapePtr->getEdges();
      for (size_t i = 0; i < edges.size(); ++i) {
        const Segment_2& edge = edges[i];
        Point_2 proj;
        double relPos;
        double dist = projectPointOntoSegment(mousePos, edge, proj, relPos);
        
        if (dist < bestDist) {
          EdgeHitResult result;
          result.host = shapePtr.get();
          result.edgeIndex = i;
          result.edge = edge;
          result.projectedPoint = proj;
          result.relativePosition = relPos;
          result.distance = dist;
          
          best = result;
          bestDist = dist;
        }
      }
    }
  };
  
  checkShapeEdges(editor.rectangles);
  checkShapeEdges(editor.polygons);
  checkShapeEdges(editor.regularPolygons);
  checkShapeEdges(editor.triangles);
  
  // Check circle circumference
  for (auto& circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid()) continue;
    
    if (circlePtr->isCircumferenceHovered(worldPos_sfml, tolerance)) {
      // Project onto circumference
      Point_2 proj = circlePtr->projectOntoCircumference(mousePos);
      
      // Calculate distance
      double dx = CGAL::to_double(proj.x()) - worldPos_sfml.x;
      double dy = CGAL::to_double(proj.y()) - worldPos_sfml.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      
      if (dist < bestDist) {
        EdgeHitResult result;
        result.host = circlePtr.get();
        result.edgeIndex = 0;  // Circle has only one "edge" (circumference)
        // For circles we don't use Segment_2, but we store the projected point
        result.projectedPoint = proj;
        
        // Calculate relative position as angle (0 to 1 representing 0 to 2π)
        Point_2 center = circlePtr->getCenterPoint();
        double angle = std::atan2(
          CGAL::to_double(proj.y()) - CGAL::to_double(center.y()),
          CGAL::to_double(proj.x()) - CGAL::to_double(center.x())
        );
        if (angle < 0) angle += 2.0 * M_PI;
        result.relativePosition = angle / (2.0 * M_PI);
        result.distance = dist;
        
        best = result;
        bestDist = dist;
      }
    }
  }
  
  return best;
}

bool PointUtils::findShapeVertex(
    GeometryEditor& editor,
    const sf::Vector2f& worldPos_sfml,
    float tolerance,
    GeometricObject*& outShape,
    size_t& outVertexIndex) {
  
  const float tolerance2 = tolerance * tolerance;
  float bestDist2 = std::numeric_limits<float>::max();
  bool found = false;
  
  auto checkVertices = [&](auto& container) {
    for (auto& shapePtr : container) {
      if (!shapePtr || !shapePtr->isValid()) continue;
      
      auto vertices = shapePtr->getInteractableVertices();
      for (size_t i = 0; i < vertices.size(); ++i) {
        float vx = static_cast<float>(CGAL::to_double(vertices[i].x()));
        float vy = static_cast<float>(CGAL::to_double(vertices[i].y()));
        float dx = vx - worldPos_sfml.x;
        float dy = vy - worldPos_sfml.y;
        float d2 = dx * dx + dy * dy;
        
        if (d2 <= tolerance2 && d2 < bestDist2) {
          outShape = shapePtr.get();
          outVertexIndex = i;
          bestDist2 = d2;
          found = true;
        }
      }
    }
  };
  
  checkVertices(editor.rectangles);
  checkVertices(editor.polygons);
  checkVertices(editor.regularPolygons);
  checkVertices(editor.triangles);
  
  // Check circle centers
  for (auto& circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid()) continue;
    
    auto vertices = circlePtr->getInteractableVertices();
    for (size_t i = 0; i < vertices.size(); ++i) {
      float vx = static_cast<float>(CGAL::to_double(vertices[i].x()));
      float vy = static_cast<float>(CGAL::to_double(vertices[i].y()));
      float dx = vx - worldPos_sfml.x;
      float dy = vy - worldPos_sfml.y;
      float d2 = dx * dx + dy * dy;
      
      if (d2 <= tolerance2 && d2 < bestDist2) {
        outShape = circlePtr.get();
        outVertexIndex = i;
        bestDist2 = d2;
        found = true;
      }
    }
  }
  
  return found;
}

double PointUtils::projectPointOntoSegment(
    const Point_2& point,
    const Segment_2& segment,
    Point_2& outProjection,
    double& outRelativePos) {
  
  Point_2 a = segment.source();
  Point_2 b = segment.target();
  
  Vector_2 ab = b - a;
  Vector_2 ap = point - a;
  
  double ab2 = CGAL::to_double(ab.squared_length());
  
  if (ab2 < Constants::EPSILON * Constants::EPSILON) {
    // Degenerate segment (zero length)
    outProjection = a;
    outRelativePos = 0.0;
    double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(point, a)));
    return dist;
  }
  
  double t = CGAL::to_double(ab * ap) / ab2;
  
  // Clamp t to [0, 1] to stay on segment
  t = std::max(0.0, std::min(1.0, t));
  
  outRelativePos = t;
  outProjection = a + t * ab;
  
  double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(point, outProjection)));
  return dist;
}


#include <CGAL/intersections.h>

std::optional<PointUtils::IntersectionHit> PointUtils::getHoveredIntersection(
    GeometryEditor& editor, 
    const sf::Vector2f& worldPos_sfml, 
    float tolerance) {
    std::optional<IntersectionHit> bestHit;
    double minDistSq = tolerance * tolerance;
    Point_2 cursorCgal = editor.toCGALPoint(worldPos_sfml);

    std::vector<std::shared_ptr<GeometricObject>> objects;
    objects.reserve(editor.lines.size() + editor.circles.size() + editor.rectangles.size() +
                    editor.polygons.size() + editor.regularPolygons.size() +
                    editor.triangles.size());

    for (auto &line : editor.lines) {
      if (line && line->isValid()) {
        objects.push_back(line);
      }
    }
    for (auto &circle : editor.circles) {
      if (circle && circle->isValid()) {
        objects.push_back(circle);
      }
    }
    for (auto &rect : editor.rectangles) {
      if (rect && rect->isValid()) {
        objects.push_back(rect);
      }
    }
    for (auto &poly : editor.polygons) {
      if (poly && poly->isValid()) {
        objects.push_back(poly);
      }
    }
    for (auto &rpoly : editor.regularPolygons) {
      if (rpoly && rpoly->isValid()) {
        objects.push_back(rpoly);
      }
    }
    for (auto &tri : editor.triangles) {
      if (tri && tri->isValid()) {
        objects.push_back(tri);
      }
    }

    for (size_t i = 0; i < objects.size(); ++i) {
      for (size_t j = i + 1; j < objects.size(); ++j) {
        const auto &a = objects[i];
        const auto &b = objects[j];
        if (!a || !b) {
          continue;
        }

        std::vector<Point_2> intersections;
        try {
          intersections = IntersectionSystem::computeIntersections(*a, *b);
        } catch (...) {
          continue;
        }

        for (const auto &p : intersections) {
          double distSq = CGAL::to_double(CGAL::squared_distance(p, cursorCgal));
          if (distSq < minDistSq) {
            minDistSq = distSq;
            IntersectionHit hit;
            hit.position = p;
            hit.obj1 = a;
            hit.obj2 = b;
            hit.distanceSquared = distSq;
            hit.line1 = std::dynamic_pointer_cast<Line>(a);
            hit.line2 = std::dynamic_pointer_cast<Line>(b);
            bestHit = hit;
          }
        }
      }
    }

    return bestHit;
}

PointUtils::SnapState PointUtils::checkSnapping(
    GeometryEditor &editor,
    const sf::Vector2f &worldPos_sfml,
    float tolerance) {
  // --- DEBUG DIAGNOSTIC START ---
  {
    float debugTol = tolerance * 2.0f;
    Point_2 cgalMouse = editor.toCGALPoint(worldPos_sfml);
    std::vector<GeometricObject*> nearby;

    for (const auto &l : editor.lines) {
      if (l && l->isValid() && l->contains(worldPos_sfml, debugTol)) {
        nearby.push_back(l.get());
      }
    }
    for (const auto &c : editor.circles) {
      if (c && c->isValid() && c->isCircumferenceHovered(worldPos_sfml, debugTol)) {
        nearby.push_back(c.get());
      }
    }

    for (size_t i = 0; i < nearby.size(); ++i) {
      for (size_t j = i + 1; j < nearby.size(); ++j) {
        auto pts = IntersectionSystem::computeIntersections(*nearby[i], *nearby[j]);
        for (const auto &p : pts) {
          double dSq = CGAL::to_double(CGAL::squared_distance(p, cgalMouse));
          if (dSq < debugTol * debugTol) {
            double d = std::sqrt(dSq);
            std::cout << "[DEBUG] Intersection Candidate FOUND: " << p
                      << " between " << static_cast<int>(nearby[i]->getType())
                      << " and " << static_cast<int>(nearby[j]->getType())
                      << " (dist=" << d << ")" << std::endl;
          }
        }
      }
    }
  }
  // --- DEBUG DIAGNOSTIC END ---

  // 1. PRIORITY CHECK: Intersections (Line-Line, Line-Circle, Circle-Circle)
  // We check for intersections between any two objects near the mouse.
  Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
  std::vector<GeometricObject *> nearbyObjects;
  nearbyObjects.reserve(editor.lines.size() + editor.circles.size());

  for (const auto &line : editor.lines) {
    if (line && line->isValid() && line->contains(worldPos_sfml, tolerance)) {
      nearbyObjects.push_back(line.get());
    }
  }
  for (const auto &circle : editor.circles) {
    if (circle && circle->isValid() && circle->isCircumferenceHovered(worldPos_sfml, tolerance)) {
      nearbyObjects.push_back(circle.get());
    }
  }

  if (nearbyObjects.size() >= 2) {
    const double snapTolSq = static_cast<double>(tolerance * tolerance);

    for (size_t i = 0; i < nearbyObjects.size(); ++i) {
      for (size_t j = i + 1; j < nearbyObjects.size(); ++j) {
        auto intersections = IntersectionSystem::computeIntersections(*nearbyObjects[i], *nearbyObjects[j]);

        for (const auto &pt : intersections) {
          double distSq = CGAL::to_double(CGAL::squared_distance(pt, cgalWorldPos));

          if (distSq < snapTolSq) {
            SnapState state;
            state.kind = SnapState::Kind::Intersection;
            state.position = pt;

            auto sharedA = editor.findSharedPtr(nearbyObjects[i]);
            auto sharedB = editor.findSharedPtr(nearbyObjects[j]);
            state.line1 = std::dynamic_pointer_cast<Line>(sharedA);
            state.line2 = std::dynamic_pointer_cast<Line>(sharedB);

            return state;  // Prioritize intersection over other snaps
          }
        }
      }
    }
  }

  SnapState state;
  const float tolerance2 = tolerance * tolerance;

  // 1) Existing points (free + object points)
  for (auto &pt : editor.points) {
    if (pt && pt->isValid() && pt->contains(worldPos_sfml, tolerance)) {
      state.kind = SnapState::Kind::ExistingPoint;
      state.point = pt;
      state.position = pt->getCGALPosition();
      return state;
    }
  }
  for (auto &op : editor.ObjectPoints) {
    if (op && op->isValid() && op->contains(worldPos_sfml, tolerance)) {
      state.kind = SnapState::Kind::ExistingPoint;
      state.point = std::static_pointer_cast<Point>(op);
      state.position = op->getCGALPosition();
      return state;
    }
  }

  // 2) Intersection
  if (auto hit = getHoveredIntersection(editor, worldPos_sfml, tolerance)) {
    state.kind = SnapState::Kind::Intersection;
    state.position = hit->position;
    state.line1 = hit->line1;
    state.line2 = hit->line2;
    return state;
  }

  // 3) Shape vertex
  GeometricObject *shape = nullptr;
  size_t vertexIndex = 0;
  if (findShapeVertex(editor, worldPos_sfml, tolerance, shape, vertexIndex)) {
    state.kind = SnapState::Kind::ShapeVertex;
    state.shape = editor.findSharedPtr(shape);
    state.edgeIndex = vertexIndex;
    if (state.shape) {
      auto verts = state.shape->getInteractableVertices();
      if (vertexIndex < verts.size()) {
        state.position = verts[vertexIndex];
      }
    }
    return state;
  }

  // 4) Shape edges
  if (auto edgeHit = findNearestEdge(editor, worldPos_sfml, tolerance)) {
    state.kind = SnapState::Kind::ShapeEdge;
    state.shape = editor.findSharedPtr(edgeHit->host);
    state.edgeIndex = edgeHit->edgeIndex;
    state.edgeRelative = edgeHit->relativePosition;
    state.position = edgeHit->projectedPoint;
    return state;
  }

  // 5) Line objects
  Point_2 cursor = editor.toCGALPoint(worldPos_sfml);
  double bestDist = static_cast<double>(tolerance);
  std::shared_ptr<Line> bestLine = nullptr;
  Point_2 bestProj(FT(0), FT(0));
  double bestRel = 0.0;

  for (auto &linePtr : editor.lines) {
    if (!linePtr || !linePtr->isValid()) continue;
    if (!linePtr->contains(worldPos_sfml, tolerance)) continue;

    try {
      Point_2 proj = linePtr->getCGALLine().projection(cursor);
      double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(proj, cursor)));
      if (dist < bestDist) {
        bestDist = dist;
        bestLine = linePtr;
        bestProj = proj;

        Point_2 startP = linePtr->getStartPoint();
        Point_2 endP = linePtr->getEndPoint();
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
    state.kind = SnapState::Kind::Line;
    state.line = bestLine;
    state.position = bestProj;
    state.lineRelative = bestRel;
    return state;
  }

  return state;
}

void PointUtils::drawSnappingVisuals(sf::RenderWindow &window, const SnapState &state, float scale) {
  if (state.kind == SnapState::Kind::None) return;

  // Scale marker in world units so it appears at a consistent pixel size.
  const float baseRadius = 6.0f;
  const float radius = baseRadius * scale;

  sf::CircleShape marker(radius);
  marker.setOrigin(radius, radius);
  marker.setOutlineThickness(1.0f * scale);
  marker.setOutlineColor(sf::Color::Black);

  if (state.kind == SnapState::Kind::Intersection) {
    marker.setFillColor(sf::Color::Magenta);
  } else {
    marker.setFillColor(sf::Color::Cyan);
  }

  marker.setPosition(static_cast<float>(CGAL::to_double(state.position.x())),
                     static_cast<float>(CGAL::to_double(state.position.y())));
  window.draw(marker);
}

std::shared_ptr<Point> PointUtils::createSmartPoint(
    GeometryEditor &editor,
    const sf::Vector2f &worldPos_sfml,
    float tolerance) {
  const bool isAltPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) ||
                            sf::Keyboard::isKeyPressed(sf::Keyboard::RAlt);
  const bool allowExistingPointSnap = isAltPressed;
  const bool allowObjectProjection = (editor.m_currentToolType == ObjectType::Point) || isAltPressed;
  // Limit object snapping in world units to avoid aggressive snapping when zoomed out
  const float objectSnapTolerance = std::min(tolerance, 0.35f);
  // 1. INSTANCE CHECK (Topological Glue)
  // Check if the click is on an EXISTING physical Point entity
  if (allowExistingPointSnap) {
    for (const auto& pt : editor.points) {
      if (pt && pt->isValid() && pt->isVisible() && pt->contains(worldPos_sfml, tolerance)) {
          std::cout << "[TOPOLOGY] Found existing instance. Merging." << std::endl;
          return pt; // Return the ACTUAL shared_ptr
      }
    }
  }

  // 2) Intersection
  if (auto hit = getHoveredIntersection(editor, worldPos_sfml, tolerance)) {
      auto existingIntersection = [&](const Point_2 &p) -> std::shared_ptr<Point> {
        const double dupTol2 = 1e-6;
        for (const auto &pt : editor.points) {
          if (pt && pt->isValid()) {
            if (CGAL::to_double(CGAL::squared_distance(pt->getCGALPosition(), p)) < dupTol2) {
              return pt;
            }
          }
        }
        return nullptr;
      };

      if (hit->obj1 && hit->obj2) {
        auto points = DynamicIntersection::createGenericIntersection(
            hit->obj1, hit->obj2, editor);
        if (!points.empty()) {
          std::shared_ptr<Point> bestPoint;
          double bestDistSq = std::numeric_limits<double>::max();
          for (const auto &pt : points) {
            if (!pt) continue;
            double distSq = CGAL::to_double(CGAL::squared_distance(pt->getCGALPosition(),
                                                                  hit->position));
            if (distSq < bestDistSq) {
              bestDistSq = distSq;
              bestPoint = pt;
            }
          }
          if (bestPoint) {
            return bestPoint;
          }
        }
      }

      if (hit->line1 && hit->line2) {
        auto ip = DynamicIntersection::createLineLineIntersection(hit->line1, hit->line2, editor);
        if (ip) {
          return ip;
        }
      }

      if (auto existing = existingIntersection(hit->position)) {
        return existing;
      }

      // Use centralized factory
      auto newPoint = editor.createPoint(hit->position);
      newPoint->setIntersectionPoint(true);
      newPoint->setFillColor(Constants::INTERSECTION_POINT_COLOR);
      newPoint->setSelected(false);
      newPoint->lock();
      return newPoint;
  }

  // 2b) Existing ObjectPoints (after intersections)
  if (allowExistingPointSnap) {
    for (const auto &op : editor.ObjectPoints) {
      if (op && op->isValid() && op->isVisible() && op->contains(worldPos_sfml, objectSnapTolerance)) {
        return std::static_pointer_cast<Point>(op);
      }
    }

    // 2b) Line endpoints (prioritize over line edge)
    {
      std::shared_ptr<Point> bestEndpoint = nullptr;
      float bestDist2 = tolerance * tolerance;
      for (auto &linePtr : editor.lines) {
        if (!linePtr || !linePtr->isValid()) continue;

        auto startPt = linePtr->getStartPointObjectShared();
        if (startPt && startPt->isValid()) {
          sf::Vector2f ptPos = startPt->getSFMLPosition();
          float dx = ptPos.x - worldPos_sfml.x;
          float dy = ptPos.y - worldPos_sfml.y;
          float d2 = dx * dx + dy * dy;
          if (d2 <= bestDist2) {
            bestDist2 = d2;
            bestEndpoint = startPt;
          }
        }

        auto endPt = linePtr->getEndPointObjectShared();
        if (endPt && endPt->isValid()) {
          sf::Vector2f ptPos = endPt->getSFMLPosition();
          float dx = ptPos.x - worldPos_sfml.x;
          float dy = ptPos.y - worldPos_sfml.y;
          float d2 = dx * dx + dy * dy;
          if (d2 <= bestDist2) {
            bestDist2 = d2;
            bestEndpoint = endPt;
          }
        }
      }

      if (bestEndpoint) {
        return bestEndpoint;
      }
    }
  }

  // 3) Lines/edges (ObjectPoints) - allow for Point tool and Alt mode
  if (allowObjectProjection) {
    // 3a) Shape vertex
    GeometricObject *shape = nullptr;
    size_t vertexIndex = 0;
    if (findShapeVertex(editor, worldPos_sfml, objectSnapTolerance, shape, vertexIndex)) {
      auto hostPtr = editor.findSharedPtr(shape);
      if (hostPtr) {
        auto objPoint = ObjectPoint::createOnShapeEdge(hostPtr, vertexIndex, 0.0);
        if (objPoint && objPoint->isValid()) {
          objPoint->setIsVertexAnchor(true);
          objPoint->setRadius(editor.currentPointSize);
          editor.ObjectPoints.push_back(objPoint);
          return objPoint;
        }
      }
    }

    // 3b) Shape edge
    if (auto edgeHit = findNearestEdge(editor, worldPos_sfml, objectSnapTolerance)) {
      if (edgeHit->host) {
        if (auto circle = dynamic_cast<Circle *>(edgeHit->host)) {
          auto circlePtr = std::dynamic_pointer_cast<Circle>(editor.findSharedPtr(circle));
          if (circlePtr) {
            auto objPoint = ObjectPoint::create(
                circlePtr, edgeHit->relativePosition * 2.0 * M_PI,
                Constants::OBJECT_POINT_DEFAULT_COLOR);
            if (objPoint && objPoint->isValid()) {
              objPoint->setRadius(editor.getCurrentPointSize());
              editor.ObjectPoints.push_back(objPoint);
              return objPoint;
            }
          }
        } else {
          auto hostPtr = editor.findSharedPtr(edgeHit->host);
          if (hostPtr) {
            auto objPoint = ObjectPoint::createOnShapeEdge(
                hostPtr, edgeHit->edgeIndex, edgeHit->relativePosition);
            if (objPoint && objPoint->isValid()) {
              objPoint->setIsVertexAnchor(false);
              objPoint->setRadius(editor.getCurrentPointSize());
              editor.ObjectPoints.push_back(objPoint);
              return objPoint;
            }
          }
        }
      }
    }

    // 3c) Line object
    Point_2 cursor = editor.toCGALPoint(worldPos_sfml);
    double bestDist = static_cast<double>(objectSnapTolerance);
    std::shared_ptr<Line> bestLine = nullptr;
    double bestRel = 0.0;
    for (auto &linePtr : editor.lines) {
      if (!linePtr || !linePtr->isValid()) continue;
      if (!linePtr->contains(worldPos_sfml, objectSnapTolerance)) continue;
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
      auto objPoint = ObjectPoint::create(bestLine, bestRel,
                                          Constants::OBJECT_POINT_DEFAULT_COLOR);
      if (objPoint && objPoint->isValid()) {
        objPoint->setRadius(editor.getCurrentPointSize());
        editor.ObjectPoints.push_back(objPoint);
        return objPoint;
      }
    }
  }

  // 4) Free point
  // 4) Free point
  Point_2 cgalPos = editor.toCGALPoint(worldPos_sfml);
  // Centralized factory handles creation, labeling, and registration
  return editor.createPoint(cgalPos);
}

// --- LabelManager Implementation ---

std::string LabelManager::getNextLabel(const std::vector<std::shared_ptr<Point>>& existingPoints) {
    // Collect specific used names for fast lookup
    std::unordered_set<std::string> usedNames;
    for (const auto& pt : existingPoints) {
        if (pt) {
            std::string label = pt->getLabel();
            if (!label.empty()) {
                usedNames.insert(label);
            }
        }
    }

    // Generator sequences
    // 1. A..Z
    for (char c = 'A'; c <= 'Z'; ++c) {
        std::string candidate(1, c);
        if (usedNames.find(candidate) == usedNames.end()) {
            return candidate;
        }
    }

    // 2. A'..Z'
    for (char c = 'A'; c <= 'Z'; ++c) {
        std::string candidate(1, c);
        candidate += "'";
        if (usedNames.find(candidate) == usedNames.end()) {
            return candidate;
        }
    }

    // 3. A_1..Z_1, A_2..Z_2, etc. (Fallback)
    int subscript = 1;
    while (true) {
        for (char c = 'A'; c <= 'Z'; ++c) {
            std::string candidate(1, c);
            candidate += "_" + std::to_string(subscript);
            if (usedNames.find(candidate) == usedNames.end()) {
                return candidate;
            }
        }
        subscript++;
        if (subscript > 1000) break; // Safety break
    }

    return "P?"; // Fallback (should rarely reach here)
}
