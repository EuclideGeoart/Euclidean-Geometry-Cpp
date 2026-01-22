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
#include "Constants.h"
#include <cmath>
#include <limits>
#include <iostream>

std::shared_ptr<Point> PointUtils::findAnchorPoint(
    GeometryEditor& editor,
    const sf::Vector2f& worldPos_sfml,
    float tolerance) {
  
  std::shared_ptr<Point> best = nullptr;
  float bestDist2 = std::numeric_limits<float>::max();
  const float tolerance2 = tolerance * tolerance;
  
  // Priority 1: Search free points
  for (auto& pt : editor.points) {
    if (pt && pt->isValid()) {
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
    if (op && op->isValid()) {
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
    return best;
  }
  
  // Priority 3: Search shape vertices via generic getInteractableVertices()
  GeometricObject* bestShape = nullptr;
  size_t bestVertexIndex = 0;
  
  auto checkShapeVertices = [&](auto& container) {
    for (auto& shapePtr : container) {
      if (!shapePtr || !shapePtr->isValid()) continue;
      
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
    if (!circlePtr || !circlePtr->isValid()) continue;
    
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
