#pragma once

#include <memory>
#include <vector>

// Forward declarations
class Line;
class Circle;
class Point;
class GeometricObject;
class GeometryEditor;

namespace DynamicIntersection {

// Functions to enable/disable automatic intersection updates
void enableAutoIntersections();
void disableAutoIntersections();
bool isAutoIntersectionsEnabled();

// Selective intersection creation functions - user-driven approach
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

// Generic intersection creation for arbitrary shapes
std::vector<std::shared_ptr<Point>> createGenericIntersection(
    const std::shared_ptr<GeometricObject> &A,
    const std::shared_ptr<GeometricObject> &B,
    GeometryEditor &editor);

// Update all active intersections
void updateAllIntersections(GeometryEditor &editor);

// Remove constraints involving a specific object
void removeConstraintsInvolving(const GeometricObject *obj, GeometryEditor &editor);

// Cleanup helpers
void removeIntersectionsInvolving(const std::shared_ptr<Line> &line, GeometryEditor &editor);

// Standard intersection creation for known types
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

} // namespace DynamicIntersection
