#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message(                                                               \
    "CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif

#include "Intersection.h"
#include "CGALSafeUtils.h"
#include "Circle.h"
#include "Constants.h"
#include "Line.h"
#include "Point.h"
#include <CGAL/Uncertain.h>
#include <CGAL/intersections.h>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <algorithm>

double GEOMETRY_DBL_EPSILON = 1e-10;

// Data structure to hold all dynamic intersections
namespace {
struct LineLineIntersection {
  Line *line1;
  Line *line2;
  Point *point;
};

struct LineCircleIntersection {
  Line *line;
  Circle *circle;
  std::vector<Point *> points;
};

struct CircleCircleIntersection {
  Circle *circle1;
  Circle *circle2;
  std::vector<Point *> points;
};

// Storage for all registered intersections
std::vector<LineLineIntersection> lineLineIntersections;
std::vector<LineCircleIntersection> lineCircleIntersections;
std::vector<CircleCircleIntersection> circleCircleIntersections;

// Configuration flag for automatic intersection updates
bool g_autoUpdateIntersections = false; // Default to false
} // namespace

// Core intersection computation functions

std::optional<Point_2> findIntersection(const Line_2 &line1,
                                        const Line_2 &line2) {
  try {
    if (line1.is_degenerate() || line2.is_degenerate()) {
      return std::nullopt;
    }

    // Handle uncertain equality safely
    CGAL::Uncertain<bool> areEqual = (line1 == line2);

    // Use CGAL's uncertainty handling functions
    if (CGAL::certainly(areEqual)) {
      // Lines are definitely identical
      return std::nullopt; // No unique intersection point
    }

    // Continue only if lines are definitely not equal, or if equality is
    // uncertain
    FT a1 = line1.a(), b1 = line1.b(), c1 = line1.c();
    FT a2 = line2.a(), b2 = line2.b(), c2 = line2.c();

    FT det = a1 * b2 - a2 * b1;

    if (CGAL::is_zero(det)) {
      // Lines are parallel and distinct (since identical case was handled)
      return std::nullopt;
    }

    // Compute intersection point using Cramer's rule
    FT x_coord = (b1 * c2 - b2 * c1) / det;
    FT y_coord = (a2 * c1 - a1 * c2) / det;

    return Point_2(x_coord, y_coord);
  } catch (const std::exception &e) {
    std::cerr << "Exception in findIntersection(Line_2, Line_2): " << e.what()
              << std::endl;
    return std::nullopt;
  }
}

std::vector<Point_2>
findIntersection(const Line_2 &line,
                 const CGAL::Circle_2<CGAL::Epeck> &circle) {
  std::vector<Point_2> intersections;

  try {
    if (line.is_degenerate() || circle.is_degenerate()) {
      return intersections;
    }

    // Standard line-circle intersection algorithm
    Point_2 center = circle.center();
    FT r_squared = circle.squared_radius();

    // Find the distance from center to the line
    Point_2 projection = line.projection(center);
    FT dist_sq = CGAL::squared_distance(center, projection);

    // Compare with square of radius
    int comparison = CGAL::compare(dist_sq, r_squared);
    if (comparison > 0) {
      // No intersection, distance > radius
      return intersections;
    } else if (comparison == 0) {
      // Tangent point - only one intersection
      intersections.push_back(projection);
    } else {
      // Two intersection points
      FT h_sq = r_squared - dist_sq;
      double h = std::sqrt(CGAL::to_double(h_sq));

      Vector_2 v = line.to_vector();
      double v_len = std::sqrt(CGAL::to_double(v.squared_length()));

      if (v_len < 0.0000001) {
        return intersections; // Invalid vector length
      }

      // Normalize the vector
      double dx = CGAL::to_double(v.x()) / v_len;
      double dy = CGAL::to_double(v.y()) / v_len;

      // Calculate points on both sides of the projection
      double px = CGAL::to_double(projection.x());
      double py = CGAL::to_double(projection.y());

      intersections.push_back(Point_2(px + h * dy, py - h * dx));
      intersections.push_back(Point_2(px - h * dy, py + h * dx));
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception in findIntersection(Line_2, Circle_2): " << e.what()
              << std::endl;
  }

  return intersections;
}

std::vector<Point_2>
findIntersection(const CGAL::Circle_2<CGAL::Epeck> &circle1,
                 const CGAL::Circle_2<CGAL::Epeck> &circle2) {
  std::vector<Point_2> intersections;

  try {
    if (circle1.is_degenerate() || circle2.is_degenerate()) {
      return intersections;
    }

    Point_2 c1 = circle1.center();
    Point_2 c2 = circle2.center();
    FT r1_sq = circle1.squared_radius();
    FT r2_sq = circle2.squared_radius();

    // Calculate the distance between circle centers
    FT dist_sq = CGAL::squared_distance(c1, c2);
    double r1 = std::sqrt(CGAL::to_double(r1_sq));
    double r2 = std::sqrt(CGAL::to_double(r2_sq));
    double dist = std::sqrt(CGAL::to_double(dist_sq));

    // Check for cases where circles don't intersect
    if (dist > r1 + r2 || dist < std::abs(r1 - r2)) {
      return intersections;
    }

    // Check for tangent case
    if (std::abs(dist - (r1 + r2)) < GEOMETRY_DBL_EPSILON ||
        std::abs(dist - std::abs(r1 - r2)) < GEOMETRY_DBL_EPSILON) {
      // Single tangent point
      double factor = (r1 * r1 - r2 * r2 + dist * dist) / (2.0 * dist);
      double x =
          CGAL::to_double(c1.x()) +
          factor * (CGAL::to_double(c2.x()) - CGAL::to_double(c1.x())) / dist;
      double y =
          CGAL::to_double(c1.y()) +
          factor * (CGAL::to_double(c2.y()) - CGAL::to_double(c1.y())) / dist;

      intersections.push_back(Point_2(x, y));
      return intersections;
    }

    // Calculate two intersection points
    double a = (r1 * r1 - r2 * r2 + dist * dist) / (2.0 * dist);
    double h = std::sqrt(r1 * r1 - a * a);

    double x2 = CGAL::to_double(c1.x()) +
                (CGAL::to_double(c2.x()) - CGAL::to_double(c1.x())) * a / dist;
    double y2 = CGAL::to_double(c1.y()) +
                (CGAL::to_double(c2.y()) - CGAL::to_double(c1.y())) * a / dist;

    // Calculate the two intersection points
    double x3 =
        x2 + h * (CGAL::to_double(c2.y()) - CGAL::to_double(c1.y())) / dist;
    double y3 =
        y2 - h * (CGAL::to_double(c2.x()) - CGAL::to_double(c1.x())) / dist;
    double x4 =
        x2 - h * (CGAL::to_double(c2.y()) - CGAL::to_double(c1.y())) / dist;
    double y4 =
        y2 + h * (CGAL::to_double(c2.x()) - CGAL::to_double(c1.x())) / dist;

    intersections.push_back(Point_2(x3, y3));
    intersections.push_back(Point_2(x4, y4));
  } catch (const std::exception &e) {
    std::cerr << "Exception in findIntersection(Circle_2, Circle_2): "
              << e.what() << std::endl;
  }

  return intersections;
}

// DynamicIntersection namespace implementation - all in one file

namespace DynamicIntersection {

// Control functions for auto intersection updates
void enableAutoIntersections() {
  g_autoUpdateIntersections = true;
  std::cout << "Automatic intersection updates enabled" << std::endl;
}

void disableAutoIntersections() {
  g_autoUpdateIntersections = false;
  std::cout << "Automatic intersection updates disabled" << std::endl;
}

bool isAutoIntersectionsEnabled() { return g_autoUpdateIntersections; }

void removeIntersectionsInvolving(Line *line) {
  if (!line) return;

  lineLineIntersections.erase(
      std::remove_if(lineLineIntersections.begin(), lineLineIntersections.end(),
                     [line](const LineLineIntersection &item) {
                       return item.line1 == line || item.line2 == line;
                     }),
      lineLineIntersections.end());

  lineCircleIntersections.erase(
      std::remove_if(lineCircleIntersections.begin(), lineCircleIntersections.end(),
                     [line](const LineCircleIntersection &item) {
                       return item.line == line;
                     }),
      lineCircleIntersections.end());
}

// Implementation for standard intersection creation (persistent)
Point *createLineLineIntersection(Line *line1, Line *line2) {
  if (!line1 || !line2)
    return nullptr;

  // Check if we already have this intersection
  for (const auto &intersection : lineLineIntersections) {
    if ((intersection.line1 == line1 && intersection.line2 == line2) ||
        (intersection.line1 == line2 && intersection.line2 == line1)) {
      return intersection.point;
    }
  }

  // Find the intersection point
  auto optIntersect =
      findIntersection(line1->getCGALLine(), line2->getCGALLine());
  if (!optIntersect)
    return nullptr;

  // Create a new point at the intersection
  auto newPoint =
      new Point(*optIntersect, 1.0f, Constants::INTERSECTION_POINT_COLOR);
  newPoint->setIntersectionPoint(true);
  newPoint->lock(); // Lock intersection points so they can't be moved directly

  // Register the intersection for automatic updates
  lineLineIntersections.push_back({line1, line2, newPoint});

  return newPoint;
}

std::vector<Point *> createLineCircleIntersection(Line *line, Circle *circle) {
  std::vector<Point *> result;
  if (!line || !circle)
    return result;

  // Check if we already have this intersection
  for (const auto &intersection : lineCircleIntersections) {
    if (intersection.line == line && intersection.circle == circle) {
      return intersection.points;
    }
  }

  // Find the intersection points
  auto intersections =
      findIntersection(line->getCGALLine(), circle->getCGALCircle());

  // Create points for each intersection
  LineCircleIntersection newIntersection{line, circle, {}};
  for (const auto &cgalPoint : intersections) {
    auto newPoint =
        new Point(cgalPoint, 1.0f, Constants::INTERSECTION_POINT_COLOR);
    newPoint->setIntersectionPoint(true);
    newPoint->lock();

    newIntersection.points.push_back(newPoint);
    result.push_back(newPoint);
  }

  if (!newIntersection.points.empty()) {
    lineCircleIntersections.push_back(std::move(newIntersection));
  }

  return result;
}

std::vector<Point *> createCircleCircleIntersection(Circle *circle1,
                                                    Circle *circle2) {
  std::vector<Point *> result;
  if (!circle1 || !circle2)
    return result;

  // Check if we already have this intersection
  for (const auto &intersection : circleCircleIntersections) {
    if ((intersection.circle1 == circle1 && intersection.circle2 == circle2) ||
        (intersection.circle1 == circle2 && intersection.circle2 == circle1)) {
      return intersection.points;
    }
  }

  // Find the intersection points
  auto intersections =
      findIntersection(circle1->getCGALCircle(), circle2->getCGALCircle());

  // Create points for each intersection
  CircleCircleIntersection newIntersection{circle1, circle2, {}};
  for (const auto &cgalPoint : intersections) {
    auto newPoint =
        new Point(cgalPoint, 1.0f, Constants::INTERSECTION_POINT_COLOR);
    newPoint->setIntersectionPoint(true);
    newPoint->lock();

    newIntersection.points.push_back(newPoint);
    result.push_back(newPoint);
  }

  if (!newIntersection.points.empty()) {
    circleCircleIntersections.push_back(std::move(newIntersection));
  }

  return result;
}

// Selective intersection creation - for user-driven individual selections
Point *createLineLineIntersectionSelective(Line *line1, Line *line2) {
  if (!line1 || !line2) {
    std::cerr << "Invalid line pointers" << std::endl;
    return nullptr;
  }

  try {
    // First check if intersection already exists in our registry
    for (const auto &intersection : lineLineIntersections) {
      if ((intersection.line1 == line1 && intersection.line2 == line2) ||
          (intersection.line1 == line2 && intersection.line2 == line1)) {
        if (intersection.point) {
          std::cout << "Found existing line-line intersection" << std::endl;
          return intersection.point;
        }
      }
    }

    // If not found, create a new one and register it
    auto optIntersect =
        findIntersection(line1->getCGALLine(), line2->getCGALLine());
    if (!optIntersect) {
      std::cout << "No intersection found between lines" << std::endl;
      return nullptr;
    }

    // Create a new point at the intersection
    auto newPoint =
        new Point(*optIntersect, 1.0f, Constants::INTERSECTION_POINT_COLOR);
    newPoint->setIntersectionPoint(true);
    newPoint->lock(); // Lock it so it can't be moved directly

    // Register the intersection for updates
    lineLineIntersections.push_back({line1, line2, newPoint});

    std::cout << "Line-line intersection created" << std::endl;
    return newPoint;
  } catch (const std::exception &e) {
    std::cerr << "Error in createLineLineIntersectionSelective: " << e.what()
              << std::endl;
    return nullptr;
  }
}

std::vector<Point *> createLineCircleIntersectionSelective(Line *line,
                                                           Circle *circle) {
  std::vector<Point *> result;
  if (!line || !circle) {
    std::cerr << "Invalid line or circle pointers" << std::endl;
    return result;
  }

  try {
    // Check if we already have this intersection
    for (const auto &intersection : lineCircleIntersections) {
      if (intersection.line == line && intersection.circle == circle) {
        return intersection.points;
      }
    }

    // Find the intersection points
    auto intersections =
        findIntersection(line->getCGALLine(), circle->getCGALCircle());

    // Create points for each intersection
    LineCircleIntersection newIntersection{line, circle, {}};
    for (const auto &cgalPoint : intersections) {
      auto newPoint =
          new Point(cgalPoint, 1.0f, Constants::INTERSECTION_POINT_COLOR);
      newPoint->setIntersectionPoint(true);
      newPoint->lock();

      newIntersection.points.push_back(newPoint);
      result.push_back(newPoint);
    }

    if (!newIntersection.points.empty()) {
      lineCircleIntersections.push_back(std::move(newIntersection));
    }

    std::cout << "Found " << result.size() << " line-circle intersections"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error in createLineCircleIntersectionSelective: " << e.what()
              << std::endl;
  }

  return result;
}

std::vector<Point *> createCircleCircleIntersectionSelective(Circle *circle1,
                                                             Circle *circle2) {
  std::vector<Point *> result;
  if (!circle1 || !circle2) {
    std::cerr << "Invalid circle pointers" << std::endl;
    return result;
  }

  try {
    // Check if we already have this intersection
    for (const auto &intersection : circleCircleIntersections) {
      if ((intersection.circle1 == circle1 &&
           intersection.circle2 == circle2) ||
          (intersection.circle1 == circle2 &&
           intersection.circle2 == circle1)) {
        return intersection.points;
      }
    }

    // Find the intersection points
    auto intersections =
        findIntersection(circle1->getCGALCircle(), circle2->getCGALCircle());

    // Create points for each intersection
    CircleCircleIntersection newIntersection{circle1, circle2, {}};
    for (const auto &cgalPoint : intersections) {
      auto newPoint =
          new Point(cgalPoint, 1.0f, Constants::INTERSECTION_POINT_COLOR);
      newPoint->setIntersectionPoint(true);
      newPoint->lock();

      newIntersection.points.push_back(newPoint);
      result.push_back(newPoint);
    }

    if (!newIntersection.points.empty()) {
      circleCircleIntersections.push_back(std::move(newIntersection));
    }

    std::cout << "Found " << result.size() << " circle-circle intersections"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error in createCircleCircleIntersectionSelective: "
              << e.what() << std::endl;
  }

  return result;
}

void updateAllIntersections() {
  // Always update intersections when objects move, regardless of auto-update
  // setting This ensures intersection points always follow their source objects
  try {
    // Update line-line intersections
    for (auto &intersection : lineLineIntersections) {
      try {
        // Safety checks
        if (!intersection.line1 || !intersection.line2 || !intersection.point) {
          continue; // Skip invalid entries
        }

        // Update the intersection point position
        auto optIntersect = findIntersection(intersection.line1->getCGALLine(),
                                             intersection.line2->getCGALLine());
        if (optIntersect) {
          intersection.point->setCGALPosition(*optIntersect);
        }
      } catch (const std::exception &e) {
        std::cerr << "Error updating line-line intersection: " << e.what()
                  << std::endl;
      }
    }

    // Update line-circle intersections
    for (auto &intersection : lineCircleIntersections) {
      try {
        if (!intersection.line || !intersection.circle ||
            intersection.points.empty()) {
          continue;
        }

        auto newIntersections =
            findIntersection(intersection.line->getCGALLine(),
                             intersection.circle->getCGALCircle());

        if (newIntersections.size() == intersection.points.size()) {
          // Same number of points, just update positions
          for (size_t i = 0; i < newIntersections.size(); i++) {
            if (intersection.points[i]) {
              intersection.points[i]->setCGALPosition(newIntersections[i]);
            }
          }
        }
        // If different count, we'd need more complex logic...
      } catch (const std::exception &e) {
        std::cerr << "Error updating line-circle intersection: " << e.what()
                  << std::endl;
      }
    }

    // Update circle-circle intersections
    for (auto &intersection : circleCircleIntersections) {
      try {
        if (!intersection.circle1 || !intersection.circle2 ||
            intersection.points.empty()) {
          continue;
        }

        auto newIntersections =
            findIntersection(intersection.circle1->getCGALCircle(),
                             intersection.circle2->getCGALCircle());

        if (newIntersections.size() == intersection.points.size()) {
          for (size_t i = 0; i < newIntersections.size(); i++) {
            if (intersection.points[i]) {
              intersection.points[i]->setCGALPosition(newIntersections[i]);
            }
          }
        }
      } catch (const std::exception &e) {
        std::cerr << "Error updating circle-circle intersection: " << e.what()
                  << std::endl;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Error in updateAllIntersections: " << e.what() << std::endl;
  }
}

} // namespace DynamicIntersection
