#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message(                                                               \
    "CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif

#include "Intersection.h"
#include "CGALSafeUtils.h"
#include "Circle.h"
#include "Constants.h"
#include "GeometryEditor.h"
#include "GeometricObject.h"
#include "IntersectionSystem.h"
//#include "LabelManager.h"
#include "Line.h"
#include "Point.h"
#include <CGAL/Uncertain.h>
#include <CGAL/intersections.h>
#include <iostream>
#include <limits>
#include <memory>
#include <algorithm>

double GEOMETRY_DBL_EPSILON = 1e-10;

// Data structure to hold all dynamic intersections
namespace {
// Configuration flag for automatic intersection updates
bool g_autoUpdateIntersections = false; // Default to false
} // namespace

namespace DynamicIntersection {
static std::vector<IntersectionConstraint> activeConstraints;
}

namespace {
void removePointFromEditor(GeometryEditor &editor, const std::shared_ptr<Point> &pt) {
  if (!pt) {
    return;
  }
  if (editor.selectedObject == pt.get()) {
    editor.selectedObject = nullptr;
  }
  if (editor.hoveredObject == pt.get()) {
    editor.hoveredObject = nullptr;
  }
  auto &vec = editor.points;
  vec.erase(std::remove_if(vec.begin(), vec.end(),
                           [&](const std::shared_ptr<Point> &sp) { return sp == pt; }),
            vec.end());
}
}

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

    // Ambiguous call in some CGAL configurations - relying on manual calculation below
    /*
    CGAL::Object obj = CGAL::intersection(line, circle);
    if (const auto *p = CGAL::object_cast<Point_2>(&obj)) {
      intersections.push_back(*p);
      return intersections;
    }
    if (const auto *pp = CGAL::object_cast<std::pair<Point_2, Point_2>>(&obj)) {
      intersections.push_back(pp->first);
      intersections.push_back(pp->second);
      return intersections;
    }
    */

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

      // Move along the line vector (dx, dy) by distance h
      intersections.push_back(Point_2(px + h * dx, py + h * dy));
      intersections.push_back(Point_2(px - h * dx, py - h * dy));
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

void removeIntersectionsInvolving(const std::shared_ptr<Line> &line, GeometryEditor &editor) {
  if (!line) {
    return;
  }
  removeConstraintsInvolving(line.get(), editor);
}

void removeConstraintsInvolving(const GeometricObject *obj, GeometryEditor &editor) {
  auto &constraints = activeConstraints;
  constraints.erase(
      std::remove_if(constraints.begin(), constraints.end(),
                     [&](const IntersectionConstraint &item) {
                       auto lockA = item.A.lock();
                       auto lockB = item.B.lock();
                       if (!lockA || !lockB) {
                         for (const auto &wp : item.resultingPoints) {
                           if (auto sp = wp.lock()) {
                             removePointFromEditor(editor, sp);
                           }
                         }
                         return true;
                       }

                       if (obj && (lockA.get() == obj || lockB.get() == obj)) {
                         for (const auto &wp : item.resultingPoints) {
                           if (auto sp = wp.lock()) {
                             removePointFromEditor(editor, sp);
                           }
                         }
                         return true;
                       }
                       return false;
                     }),
      constraints.end());
}

void removeIntersectionPoint(const std::shared_ptr<Point> &point, GeometryEditor &editor) {
  if (!point) {
    return;
  }

  auto &constraints = activeConstraints;
  constraints.erase(
      std::remove_if(constraints.begin(), constraints.end(),
                     [&](const IntersectionConstraint &item) {
                       bool ownsPoint = false;
                       for (const auto &wp : item.resultingPoints) {
                         if (auto sp = wp.lock()) {
                           if (sp == point) {
                             ownsPoint = true;
                             break;
                           }
                         }
                       }

                       if (!ownsPoint) {
                         return false;
                       }

                       for (const auto &wp : item.resultingPoints) {
                         if (auto sp = wp.lock()) {
                           removePointFromEditor(editor, sp);
                         }
                       }
                       return true;
                     }),
      constraints.end());
}

// Implementation for standard intersection creation (persistent)
std::vector<std::shared_ptr<Point>> createGenericIntersection(
    const std::shared_ptr<GeometricObject> &A,
    const std::shared_ptr<GeometricObject> &B,
    GeometryEditor &editor) {
  std::vector<std::shared_ptr<Point>> result;
  if (!A || !B) {
    return result;
  }

  for (auto &constraint : activeConstraints) {
    auto lockA = constraint.A.lock();
    auto lockB = constraint.B.lock();
    if (!lockA || !lockB) {
      continue;
    }
    if ((lockA == A && lockB == B) || (lockA == B && lockB == A)) {
      for (const auto &wp : constraint.resultingPoints) {
        if (auto sp = wp.lock()) {
          result.push_back(sp);
        }
      }
      return result;
    }
  }

  std::vector<Point_2> intersections;
  try {
    intersections = IntersectionSystem::computeIntersections(*A, *B);
  } catch (...) {
    return result;
  }

  IntersectionConstraint constraint{A, B, {}};
  constraint.resultingPoints.reserve(intersections.size());

  for (const auto &p : intersections) {
    auto newPoint = std::make_shared<Point>(p, Constants::CURRENT_ZOOM,
                                            Constants::INTERSECTION_POINT_COLOR);
    newPoint->setID(editor.objectIdCounter++);
    newPoint->setIntersectionPoint(true);
    newPoint->setDependent(true);
    newPoint->setSelected(false);
    newPoint->lock();
    newPoint->setVisible(true);
    
    // Assign label to intersection point
    std::string label = LabelManager::instance().getNextLabel(editor.getAllPoints());
    newPoint->setLabel(label);
    newPoint->setShowLabel(true);  // Show label by default
    
    newPoint->setCGALPosition(newPoint->getCGALPosition());
    newPoint->update();
    editor.points.push_back(newPoint);
    constraint.resultingPoints.push_back(newPoint);
    result.push_back(newPoint);
  }

  activeConstraints.push_back(std::move(constraint));
  return result;
}

std::vector<IntersectionConstraint> getActiveIntersectionConstraints() {
  return activeConstraints;
}

void clearAllIntersectionConstraints(GeometryEditor &editor) {
  (void)editor;
  activeConstraints.clear();
}

void registerIntersectionConstraint(const std::shared_ptr<GeometricObject> &A,
                                    const std::shared_ptr<GeometricObject> &B,
                                    const std::vector<std::shared_ptr<Point>> &points) {
  IntersectionConstraint constraint{A, B, {}};
  constraint.resultingPoints.reserve(points.size());
  for (const auto &pt : points) {
    if (pt) constraint.resultingPoints.push_back(pt);
  }
  activeConstraints.push_back(std::move(constraint));
}

std::shared_ptr<Point> createLineLineIntersection(const std::shared_ptr<Line> &line1,
                                                  const std::shared_ptr<Line> &line2,
                                                  GeometryEditor &editor) {
  auto points = createGenericIntersection(line1, line2, editor);
  return points.empty() ? nullptr : points.front();
}

std::vector<std::shared_ptr<Point>> createLineCircleIntersection(
    const std::shared_ptr<Line> &line,
    const std::shared_ptr<Circle> &circle,
    GeometryEditor &editor) {
  return createGenericIntersection(line, circle, editor);
}

std::vector<std::shared_ptr<Point>> createCircleCircleIntersection(
    const std::shared_ptr<Circle> &circle1,
    const std::shared_ptr<Circle> &circle2,
    GeometryEditor &editor) {
  return createGenericIntersection(circle1, circle2, editor);
}

// Selective intersection creation - for user-driven individual selections
std::shared_ptr<Point> createLineLineIntersectionSelective(
    const std::shared_ptr<Line> &line1,
    const std::shared_ptr<Line> &line2,
    GeometryEditor &editor) {
  auto points = createGenericIntersection(line1, line2, editor);
  return points.empty() ? nullptr : points.front();
}

std::vector<std::shared_ptr<Point>> createLineCircleIntersectionSelective(
    const std::shared_ptr<Line> &line,
    const std::shared_ptr<Circle> &circle,
    GeometryEditor &editor) {
  return createGenericIntersection(line, circle, editor);
}

std::vector<std::shared_ptr<Point>> createCircleCircleIntersectionSelective(
    const std::shared_ptr<Circle> &circle1,
    const std::shared_ptr<Circle> &circle2,
    GeometryEditor &editor) {
  return createGenericIntersection(circle1, circle2, editor);
}

void updateAllIntersections(GeometryEditor &editor) {
  // Always update intersections when objects move, regardless of auto-update
  // setting This ensures intersection points always follow their source objects
  try {
    for (auto it = activeConstraints.begin(); it != activeConstraints.end();) {
      try {
        auto lockA = it->A.lock();
        auto lockB = it->B.lock();
        if (!lockA || !lockB) {
          for (const auto &wp : it->resultingPoints) {
            if (auto sp = wp.lock()) {
              removePointFromEditor(editor, sp);
            }
          }
          it = activeConstraints.erase(it);
          continue;
        }

        auto newIntersections =
            IntersectionSystem::computeIntersections(*lockA, *lockB);

        std::vector<std::shared_ptr<Point>> existing;
        existing.reserve(it->resultingPoints.size());
        for (const auto &wp : it->resultingPoints) {
          if (auto sp = wp.lock()) {
            existing.push_back(sp);
          }
        }

        const size_t existingCount = existing.size();
        const size_t newCount = newIntersections.size();

        std::vector<bool> used(existingCount, false);
        std::vector<std::shared_ptr<Point>> updated;
        updated.reserve(std::max(existingCount, newCount));

        // Match new intersections to existing points by proximity
        for (const auto &p : newIntersections) {
          double bestDist = std::numeric_limits<double>::max();
          size_t bestIdx = existingCount;

          for (size_t i = 0; i < existingCount; ++i) {
            if (used[i] || !existing[i]) continue;
            double dist = CGAL::to_double(CGAL::squared_distance(existing[i]->getCGALPosition(), p));
            if (dist < bestDist) {
              bestDist = dist;
              bestIdx = i;
            }
          }

          if (bestIdx < existingCount) {
            auto &pt = existing[bestIdx];
            used[bestIdx] = true;
            pt->setVisible(true);
            pt->setIsValid(true);
            pt->setIntersectionPoint(true);
            pt->lock();
            pt->setCGALPosition(p);
            pt->update();
            pt->updateConnectedLines();
            updated.push_back(pt);
          } else {
            auto newPoint = std::make_shared<Point>(p, Constants::CURRENT_ZOOM,
                                                    Constants::INTERSECTION_POINT_COLOR);
            newPoint->setID(editor.objectIdCounter++);
            newPoint->setIntersectionPoint(true);
            newPoint->setDependent(true);
            newPoint->setSelected(false);
            newPoint->lock();
            newPoint->setVisible(true);
            
            // Assign label to intersection point
            std::string label = LabelManager::instance().getNextLabel(editor.getAllPoints());
            newPoint->setLabel(label);
            newPoint->setShowLabel(true);  // Show label by default
            
            newPoint->setCGALPosition(newPoint->getCGALPosition());
            newPoint->update();
            editor.points.push_back(newPoint);
            updated.push_back(newPoint);
          }
        }

        // Hide unused existing points
        for (size_t i = 0; i < existingCount; ++i) {
          if (!used[i] && existing[i]) {
            existing[i]->setVisible(false);
            existing[i]->setIsValid(false);
            existing[i]->updateConnectedLines();
          }
        }

        it->resultingPoints.clear();
        for (const auto &pt : updated) {
          it->resultingPoints.push_back(pt);
        }

        ++it;
      } catch (const std::exception &e) {
        std::cerr << "Error updating generic intersection: " << e.what() << std::endl;
        ++it;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Error in updateAllIntersections: " << e.what() << std::endl;
  }
}

} // namespace DynamicIntersection
