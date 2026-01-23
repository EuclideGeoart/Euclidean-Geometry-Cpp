#ifndef INTERSECTION_H
#define INTERSECTION_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "CharTraitsFix.h"
#include "Types.h"  // Includes Point_2, Line_2, Circle_2
#include <optional> // For std::optional
#include <vector>

// Forward declarations
class Line;
class Circle;
class Point;

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
// Existing functions
Point *createLineLineIntersection(Line *line1, Line *line2);
std::vector<Point *> createLineCircleIntersection(Line *line, Circle *circle);
std::vector<Point *> createCircleCircleIntersection(Circle *circle1,
                                                    Circle *circle2);
void updateAllIntersections();
void removeIntersectionsInvolving(Line *line);

// Add new selective intersection functions
Point *createLineLineIntersectionSelective(Line *line1, Line *line2);
std::vector<Point *> createLineCircleIntersectionSelective(Line *line,
                                                           Circle *circle);
std::vector<Point *> createCircleCircleIntersectionSelective(Circle *circle1,
                                                             Circle *circle2);

// Add these new functions to control automatic updates
void enableAutoIntersections();
void disableAutoIntersections();
bool isAutoIntersectionsEnabled();

// Also add this method to Point
// Helper method for validation
} // namespace DynamicIntersection

#endif
