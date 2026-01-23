#pragma once

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#endif

#include "Types.h"
#include <SFML/Graphics.hpp>
#include <memory>
#include <optional>
#include <vector>

// Forward declarations
class GeometryEditor;
class Point;
class ObjectPoint;
class Line;
class GeometricObject;

/**
 * @brief Result of an edge hit detection
 */
struct EdgeHitResult {
  GeometricObject* host = nullptr;  // Shape that owns the edge
  size_t edgeIndex = 0;             // Index of the edge in the shape
  Segment_2 edge;                   // The edge itself
  Point_2 projectedPoint;           // Projection of mouse onto edge
  double relativePosition = 0.0;    // Position along edge (0.0 to 1.0)
  double distance = 0.0;            // Distance from mouse to projected point
};

/**
 * @brief Utility class for unified anchor point detection and edge detection
 * 
 * This class provides generic methods for finding points and edges across
 * all geometric objects, enabling shape vertices to be used as anchors
 * for any tool.
 */
class PointUtils {
public:
  struct SnapState {
    enum class Kind { None, ExistingPoint, Intersection, ShapeVertex, ShapeEdge, Line };
    Kind kind = Kind::None;
    Point_2 position = Point_2(FT(0), FT(0));
    std::shared_ptr<Point> point;
    std::shared_ptr<Line> line1;
    std::shared_ptr<Line> line2;
    std::shared_ptr<Line> line;
    std::shared_ptr<GeometricObject> shape;
    size_t edgeIndex = 0;
    double edgeRelative = 0.0;
    double lineRelative = 0.0;
  };

  static SnapState checkSnapping(
      GeometryEditor &editor,
      const sf::Vector2f &worldPos_sfml,
      float tolerance);

  static void drawSnappingVisuals(
      sf::RenderWindow &window,
      const SnapState &state);

  static std::shared_ptr<Point> createSmartPoint(
      GeometryEditor &editor,
      const sf::Vector2f &worldPos_sfml,
      float tolerance);

  /**
   * @brief Find any point-like entity at the given position
   * 
   * Search order (highest priority first):
   * 1. Free points (editor.points)
   * 2. ObjectPoints (editor.ObjectPoints)
   * 3. Shape vertices (via getInteractableVertices())
   * 4. Line endpoints
   * 
   * @param editor The geometry editor
   * @param worldPos_sfml Position in world coordinates (SFML)
   * @param tolerance Hit detection tolerance
   * @return shared_ptr to Point if found, nullptr otherwise
   */
  static std::shared_ptr<Point> findAnchorPoint(
    GeometryEditor& editor,
    const sf::Vector2f& worldPos_sfml,
    float tolerance);
    
  /**
   * @brief Find the nearest edge to the given position
   * 
   * Searches all shapes for edges (via getEdges()) and finds the closest one.
   * Also searches circles for circumference proximity.
   * 
   * @param editor The geometry editor
   * @param worldPos_sfml Position in world coordinates (SFML)
   * @param tolerance Maximum distance from edge to be considered a hit
   * @return EdgeHitResult if an edge is within tolerance, std::nullopt otherwise
   */
  static std::optional<EdgeHitResult> findNearestEdge(
    GeometryEditor& editor,
    const sf::Vector2f& worldPos_sfml,
    float tolerance);
    
  /**
   * @brief Check if a position is near a shape vertex and return the vertex
   * 
   * Generic method that checks all shapes' vertices via getInteractableVertices().
   * Returns the shape and vertex index if found.
   * 
   * @param editor The geometry editor  
   * @param worldPos_sfml Position in world coordinates (SFML)
   * @param tolerance Hit detection tolerance
   * @param outShape Output: the shape containing the vertex (if found)
   * @param outVertexIndex Output: index of the vertex (if found)
   * @return true if a vertex was found within tolerance
   */
  static bool findShapeVertex(
    GeometryEditor& editor,
    const sf::Vector2f& worldPos_sfml,
    float tolerance,
    GeometricObject*& outShape,
    size_t& outVertexIndex);

struct IntersectionHit {
  Point_2 position;
  std::shared_ptr<Line> line1;
  std::shared_ptr<Line> line2;
  double distanceSquared;
};

// NEW: Intersection detection
static std::optional<IntersectionHit> getHoveredIntersection(
    GeometryEditor& editor, 
    const sf::Vector2f& worldPos_sfml, 
    float tolerance);
    
  /**
   * @brief Project a point onto a segment
   * 
   * @param point The point to project
   * @param segment The segment to project onto
   * @param outProjection Output: the projected point
   * @param outRelativePos Output: relative position along segment (0.0 to 1.0)
   * @return Distance from point to projected point
   */
  static double projectPointOntoSegment(
    const Point_2& point,
    const Segment_2& segment,
    Point_2& outProjection,
    double& outRelativePos);
};
