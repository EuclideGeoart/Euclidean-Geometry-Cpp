// ProjectionUtils.h
#pragma once
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <algorithm>
#include <cmath>
#include "Types.h"  // Ensure this includes Point_2 and other necessary types

namespace ProjectionUtils {

    inline Point_2 projectPointOntoLine(const Point_2& p, const Line* lineObj, bool isSegment) {
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
        }
        else {
            if (lineObj != nullptr) {
                return lineObj->getCgalLine().projection(p);
            } else {
                // Handle the error appropriately, e.g., throw an exception or return a default value
                throw std::runtime_error("lineObj is null");
            }
        }
    }

    inline Point_2 projectPointOntoLine(const Point_2& p, const Line_2& line, bool isSegment) {
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
        }
        else {
            return line.projection(p);
        }
    }

    inline Point_2 projectPointOntoCircle(const Point_2& p, const Point_2& center, double radius, double tolerance = 1e-6) {
        double dx = CGAL::to_double(p.x() - center.x());
        double dy = CGAL::to_double(p.y() - center.y());
        double len = std::sqrt(dx * dx + dy * dy);

        if (std::abs(len - radius) <= tolerance)
            return p;

        if (len < tolerance)
            return Point_2(CGAL::to_double(center.x()) + radius, CGAL::to_double(center.y()));

        double new_x = CGAL::to_double(center.x()) + (dx / len) * radius;
        double new_y = CGAL::to_double(center.y()) + (dy / len) * radius;
        return Point_2(new_x, new_y);
    }

}  // namespace ProjectionUtils
inline sf::Vector2f projectPointOntoLine(const sf::Vector2f& point, const sf::Vector2f& start, const sf::Vector2f& end) {
    sf::Vector2f p1 = start;
    sf::Vector2f p2 = end;
    sf::Vector2f ap = point - p1;
    sf::Vector2f ab = p2 - p1;

    float ab2 = ab.x * ab.x + ab.y * ab.y;
    if (ab2 == 0) return p1; // Handle degenerate case (point line)

    float t = (ap.x * ab.x + ap.y * ab.y) / ab2;
    return p1 + ab * t;
}
