#pragma once

#include <vector>

// Forward declarations
class Line;
class Circle;
class Point;

namespace DynamicIntersection {

// Functions to enable/disable automatic intersection updates
void enableAutoIntersections();
void disableAutoIntersections();
bool isAutoIntersectionsEnabled();

// Selective intersection creation functions - user-driven approach
Point *createLineLineIntersectionSelective(Line *line1, Line *line2);
std::vector<Point *> createLineCircleIntersectionSelective(Line *line,
                                                           Circle *circle);
std::vector<Point *> createCircleCircleIntersectionSelective(Circle *circle1,
                                                             Circle *circle2);

} // namespace DynamicIntersection
