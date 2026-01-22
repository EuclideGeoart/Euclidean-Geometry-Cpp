#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message(                                                               \
    "CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif

// Create this new implementation file for the selective intersection functions

#include "DynamicIntersection.h"
#include "Circle.h"
#include "Constants.h"
#include "Intersection.h"
#include "Line.h"
#include "Point.h"
#include <iostream>

float zoomFctor = 1.0f; // Global zoom factor for the application
namespace DynamicIntersection {

// Implementation of selective line-line intersection
Point *createLineLineIntersectionSelective(Line *line1, Line *line2) {
  if (!line1 || !line2) {
    std::cerr << "Invalid line pointers" << std::endl;
    return nullptr;
  }

  try {
    // Find the intersection point
    auto optIntersect =
        findIntersection(line1->getCGALLine(), line2->getCGALLine());
    if (!optIntersect) {
      std::cout << "No intersection found between lines" << std::endl;
      return nullptr;
    }

    // Create a new point at the intersection
    auto newPoint =
        new Point(*optIntersect,zoomFctor, Constants::INTERSECTION_POINT_COLOR);
    newPoint->setIntersectionPoint(true);

    std::cout << "Line-line intersection created" << std::endl;
    return newPoint;
  } catch (const std::exception &e) {
    std::cerr << "Error in createLineLineIntersectionSelective: " << e.what()
              << std::endl;
    return nullptr;
  }
}

// Implementation of selective line-circle intersection
std::vector<Point *> createLineCircleIntersectionSelective(Line *line,
                                                           Circle *circle) {
  std::vector<Point *> result;
  if (!line || !circle) {
    std::cerr << "Invalid line or circle pointers" << std::endl;
    return result;
  }

  try {
    // Find the intersection points
    auto intersections =
        findIntersection(line->getCGALLine(), circle->getCGALCircle());

    // Create a point for each intersection
    for (const auto &cgalPoint : intersections) {
      auto newPoint = new Point(cgalPoint,zoomFctor, Constants::INTERSECTION_POINT_COLOR);
      newPoint->setIntersectionPoint(true);
      result.push_back(newPoint);
    }

    std::cout << "Found " << result.size() << " line-circle intersections"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error in createLineCircleIntersectionSelective: " << e.what()
              << std::endl;
  }

  return result;
}

// Implementation of selective circle-circle intersection
std::vector<Point *> createCircleCircleIntersectionSelective(Circle *circle1,
                                                             Circle *circle2) {
  std::vector<Point *> result;
  if (!circle1 || !circle2) {
    std::cerr << "Invalid circle pointers" << std::endl;
    return result;
  }

  try {
    // Find the intersection points
    auto intersections =
        findIntersection(circle1->getCGALCircle(), circle2->getCGALCircle());

    // Create a point for each intersection
    for (const auto &cgalPoint : intersections) {
      auto newPoint = new Point(cgalPoint,zoomFctor, Constants::INTERSECTION_POINT_COLOR);
      newPoint->setIntersectionPoint(true);
      result.push_back(newPoint);
    }

    std::cout << "Found " << result.size() << " circle-circle intersections"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error in createCircleCircleIntersectionSelective: "
              << e.what() << std::endl;
  }

  return result;
}

} // namespace DynamicIntersection
