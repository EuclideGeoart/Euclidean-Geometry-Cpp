// ProjectionUtils.h
#pragma once

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "Circle.h"
#include "GeometryEditor.h"
#include "Line.h"
#include "Types.h" // Ensure this includes Point_2 and other necessary types
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <algorithm>
#include <cmath>


namespace ProjectionUtils {
/**
 * @brief Calculates the relative parameter of a point projected onto a line
 * segment or infinite line. The parameter 't' is such that the projection P' =
 * A + t * (B - A).
 * @param p The point to project.
 * @param line_start The starting point of the line/segment.
 * @param line_end The ending point of the line/segment.
 * @param is_segment If true, the parameter 't' is clamped to [0, 1].
 * @return The relative parameter 't'. Returns 0.5 for zero-length
 * lines/segments as a fallback.
 */
// In ProjectionUtils.h, around line 35
inline double getRelativePositionOnLine(const Point_2 &p,
                                        const Point_2 &line_start,
                                        const Point_2 &line_end,
                                        bool is_segment) {
  Vector_2 line_vec = line_end - line_start;
  Vector_2 point_vec = p - line_start;

  // The crash likely happens here:
  Kernel::FT line_sq_len = line_vec.squared_length();

  // Add safety checks BEFORE calling is_zero
  try {
    // Check if the vector components are finite first
    if (!CGAL::is_finite(line_vec.x()) || !CGAL::is_finite(line_vec.y())) {
      std::cerr << "ProjectionUtils: Line vector has non-finite components"
                << std::endl;
      return 0.0;
    }

    // Use a more robust zero check
    double line_sq_len_double = CGAL::to_double(line_sq_len);
    if (!std::isfinite(line_sq_len_double) ||
        line_sq_len_double < Constants::CGAL_EPSILON_SQUARED) {
      std::cerr << "ProjectionUtils: Line has zero or invalid length"
                << std::endl;
      return 0.0;
    }

    // Now safe to proceed with the calculation
    Kernel::FT dot_product = point_vec * line_vec;
    double dot_double = CGAL::to_double(dot_product);
    double relative_pos = dot_double / line_sq_len_double;

    if (is_segment) {
      relative_pos = std::max(0.0, std::min(1.0, relative_pos));
    }

    return relative_pos;

  } catch (const std::exception &e) {
    std::cerr << "ProjectionUtils: Exception in getRelativePositionOnLine: "
              << e.what() << std::endl;
    return 0.0;
  }
}

inline void createObjectPointOnLine(GeometryEditor &editor, Line *lineHost,
                                    const Point_2 &cgalWorldPos) {
  try {
    // Validate input points first
    Point_2 line_start = lineHost->getStartPoint();
    Point_2 line_end = lineHost->getEndPoint();

    // Check if points are finite
    if (!CGAL::is_finite(line_start.x()) || !CGAL::is_finite(line_start.y()) ||
        !CGAL::is_finite(line_end.x()) || !CGAL::is_finite(line_end.y()) ||
        !CGAL::is_finite(cgalWorldPos.x()) ||
        !CGAL::is_finite(cgalWorldPos.y())) {
      std::cerr << "createObjectPointOnLine: Non-finite coordinates detected"
                << std::endl;
      return;
    }

    // Check if line has reasonable length
    double start_x = CGAL::to_double(line_start.x());
    double start_y = CGAL::to_double(line_start.y());
    double end_x = CGAL::to_double(line_end.x());
    double end_y = CGAL::to_double(line_end.y());

    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double line_length_sq = dx * dx + dy * dy;

    if (!std::isfinite(line_length_sq) || line_length_sq < 1e-20) {
      std::cerr << "createObjectPointOnLine: Line too short or invalid"
                << std::endl;
      return;
    }

    // Now safe to call ProjectionUtils
    double relativePos = ProjectionUtils::getRelativePositionOnLine(
        cgalWorldPos, line_start, line_end, lineHost->isSegment());

    // Continue with ObjectPoint creation...

  } catch (const std::exception &e) {
    std::cerr << "createObjectPointOnLine: Exception: " << e.what()
              << std::endl;
  }
}
} // namespace ProjectionUtils

inline Point_2 projectPointOntoLine(const Point_2 &p, const Line *lineObj,
                                    bool isSegment) {
  if (isSegment && lineObj) {
    // Get segment endpoints from the provided line object
    Point_2 A = lineObj->getStartPoint();
    Point_2 B = lineObj->getEndPoint();

    // Convert points to doubles.
    double Ax = CGAL::to_double(A.x());
    double Ay = CGAL::to_double(A.y());
    double Bx = CGAL::to_double(B.x());
    double By = CGAL::to_double(B.y());
    double Px = CGAL::to_double(p.x());
    double Py = CGAL::to_double(p.y());

    // Compute vector and projection parameter
    double ABx = Bx - Ax;
    double ABy = By - Ay;
    double APx = Px - Ax;
    double APy = Py - Ay;
    double dotAP_AB = ABx * APx + ABy * APy;
    double lenAB2 = ABx * ABx + ABy * ABy;
    double t = (lenAB2 == 0.0) ? 0.0 : dotAP_AB / lenAB2;

    // Clamp t to [0,1]
    t = std::clamp(t, 0.0, 1.0);

    // Compute and return the projection point
    double Qx = Ax + t * ABx;
    double Qy = Ay + t * ABy;
    return Point_2(Qx, Qy);
  } else {
    if (lineObj != nullptr) {
      return lineObj->getCGALLine().projection(p);
    } else {
      // Handle the error appropriately, e.g., throw an exception or return a
      // default value
      throw std::runtime_error("lineObj is null");
    }
  }
}

inline Point_2 projectPointOntoLine(const Point_2 &p, const Line_2 &line,
                                    bool isSegment) {
  if (isSegment) {
    // Get segment endpoints from the provided line object
    Point_2 A = line.point(0);
    Point_2 B = line.point(1);

    // Convert points to doubles.
    double Ax = CGAL::to_double(A.x());
    double Ay = CGAL::to_double(A.y());
    double Bx = CGAL::to_double(B.x());
    double By = CGAL::to_double(B.y());
    double Px = CGAL::to_double(p.x());
    double Py = CGAL::to_double(p.y());

    // Compute vector and projection parameter
    double ABx = Bx - Ax;
    double ABy = By - Ay;
    double APx = Px - Ax;
    double APy = Py - Ay;
    double dotAP_AB = ABx * APx + ABy * APy;
    double lenAB2 = ABx * ABx + ABy * ABy;
    double t = (lenAB2 == 0.0) ? 0.0 : dotAP_AB / lenAB2;

    // Clamp t to [0,1]
    t = std::clamp(t, 0.0, 1.0);

    // Compute and return the projection point
    double Qx = Ax + t * ABx;
    double Qy = Ay + t * ABy;
    return Point_2(Qx, Qy);
  } else {
    return line.projection(p);
  }
}

inline Point_2 projectPointOntoCircle(const Point_2 &p, const Point_2 &center,
                                      double radius, double tolerance = 1e-6) {
  double dx = CGAL::to_double(p.x() - center.x());
  double dy = CGAL::to_double(p.y() - center.y());
  double len = std::sqrt(dx * dx + dy * dy);

  if (std::abs(len - radius) <= tolerance)
    return p;

  if (len < tolerance)
    return Point_2(CGAL::to_double(center.x()) + radius,
                   CGAL::to_double(center.y()));

  double new_x = CGAL::to_double(center.x()) + (dx / len) * radius;
  double new_y = CGAL::to_double(center.y()) + (dy / len) * radius;
  return Point_2(new_x, new_y);
}

inline Point_2 projectOntoSegment(const Point_2 &p, const Point_2 &a,
                                  const Point_2 &b) {
  // Compute the vector ab
  Vector_2 ab = b - a;
  // Compute the vector ap
  Vector_2 ap = p - a;
  // Compute the squared length of ab
  double ab_squared =
      CGAL::to_double(ab.squared_length()); // Use CGAL::to_double
  if (ab_squared == 0) {
    // a and b are the same point, return a (or b)
    return a;
  }
  // Compute the projection scalar of ap onto ab
  // Use (ap * ab) for dot product and CGAL::to_double
  double t =
      std::max(0.0, std::min(1.0, CGAL::to_double(ap * ab) / ab_squared));
  // Compute the projected point
  return a + t * ab;
}

// namespace ProjectionUtils
inline sf::Vector2f projectPointOntoLine(const sf::Vector2f &point,
                                         const sf::Vector2f &start,
                                         const sf::Vector2f &end) {
  sf::Vector2f p1 = start;
  sf::Vector2f p2 = end;
  sf::Vector2f ap = point - p1;
  sf::Vector2f ab = p2 - p1;

  float ab2 = ab.x * ab.x + ab.y * ab.y;
  if (ab2 == 0)
    return p1; // Handle degenerate case (point line)

  float t = (ap.x * ab.x + ap.y * ab.y) / ab2;
  return p1 + ab * t;
}
// namespace ProjectionUtils