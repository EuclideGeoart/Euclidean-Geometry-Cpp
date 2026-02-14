#ifndef INTERSECTION_H
#define INTERSECTION_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "CharTraitsFix.h"
#include "Types.h"  // Includes Point_2, Line_2, Circle_2
#include <memory>
#include <optional> // For std::optional
#include <vector>

// Forward declarations
class Line;
class Circle;
class Point;
class GeometricObject;
class GeometryEditor;

// Find intersection point between two Line_2 objects
std::optional<Point_2> findIntersection(const Line_2 &line1,
                                        const Line_2 &line2);

// Find intersection points between a Line_2 and a Circle_2
std::vector<Point_2>
findIntersection(const Line_2 &line,
                 const Circle_2 &circle); // Changed from CGAL_Circle_2

// Find intersection points between two Circle_2 objects
std::vector<Point_2>
findIntersection(const Circle_2 &circle1,  // Changed from CGAL_Circle_2
                 const Circle_2 &circle2); // Changed from CGAL_Circle_2

// Overloads for convenience (calling the above)
inline std::vector<Point_2>
findIntersection(const Circle_2 &circle, // Changed from CGAL_Circle_2
                 const Line_2 &line) {
  return findIntersection(line, circle);
}

namespace DynamicIntersection {
struct IntersectionConstraint {
  std::weak_ptr<GeometricObject> A;
  std::weak_ptr<GeometricObject> B;
  std::vector<std::weak_ptr<Point>> resultingPoints;
};

// Existing functions
std::shared_ptr<Point> createLineLineIntersection(const std::shared_ptr<Line> &line1,
                          const std::shared_ptr<Line> &line2,
                          GeometryEditor &editor);
std::vector<std::shared_ptr<Point>> createLineCircleIntersection(
  const std::shared_ptr<Line> &line,
  const std::shared_ptr<Circle> &circle,
  GeometryEditor &editor);
std::vector<std::shared_ptr<Point>> createCircleCircleIntersection(
  const std::shared_ptr<Circle> &circle1,
  const std::shared_ptr<Circle> &circle2,
  GeometryEditor &editor);
void updateAllIntersections(GeometryEditor &editor);
void removeConstraintsInvolving(const GeometricObject *obj, GeometryEditor &editor);
void removeIntersectionsInvolving(const std::shared_ptr<Line> &line, GeometryEditor &editor);
void removeIntersectionPoint(const std::shared_ptr<Point> &point, GeometryEditor &editor);

// Save/load helpers
std::vector<IntersectionConstraint> getActiveIntersectionConstraints();
void clearAllIntersectionConstraints(GeometryEditor &editor);
void registerIntersectionConstraint(const std::shared_ptr<GeometricObject> &A,
                                    const std::shared_ptr<GeometricObject> &B,
                                    const std::vector<std::shared_ptr<Point>> &points);

// Generic intersection creation for arbitrary shapes
std::vector<std::shared_ptr<Point>> createGenericIntersection(
  const std::shared_ptr<GeometricObject> &A,
  const std::shared_ptr<GeometricObject> &B,
  GeometryEditor &editor);

// Add new selective intersection functions
std::shared_ptr<Point> createLineLineIntersectionSelective(
  const std::shared_ptr<Line> &line1,
  const std::shared_ptr<Line> &line2,
  GeometryEditor &editor);
std::vector<std::shared_ptr<Point>> createLineCircleIntersectionSelective(
  const std::shared_ptr<Line> &line,
  const std::shared_ptr<Circle> &circle,
  GeometryEditor &editor);
std::vector<std::shared_ptr<Point>> createCircleCircleIntersectionSelective(
  const std::shared_ptr<Circle> &circle1,
  const std::shared_ptr<Circle> &circle2,
  GeometryEditor &editor);

// Add these new functions to control automatic updates
void enableAutoIntersections();
void disableAutoIntersections();
bool isAutoIntersectionsEnabled();

// Also add this method to Point
// Helper method for validation
} // namespace DynamicIntersection

#endif
