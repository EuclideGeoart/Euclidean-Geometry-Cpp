// GeometryFactory.h
#pragma once
#include "Line.h"
#include "Circle.h"
#include "ObjectPoint.h"
#include <CGAL/Point_2.h>
#include <cmath>

namespace GeometryFactory {

    inline Line* createLineFromPoints(const ObjectPoint* p1, const ObjectPoint* p2) {
        // Use their current positions for constructing a new Line.
        return new Line(p1->getPositionCGAL(), p2->getPositionCGAL(), /*isSegment=*/false);
    }

    inline Line* createSegmentFromPoints(const ObjectPoint* p1, const ObjectPoint* p2) {
        return new Line(p1->getPositionCGAL(), p2->getPositionCGAL(), /*isSegment=*/true);
    }

    inline Circle* createCircleFromPoints(const ObjectPoint* center, const ObjectPoint* perimeter) {
        // Compute radius from center and perimeter point, then construct a Circle.
        Point_2 c = center->getPositionCGAL();
        Point_2 p = perimeter->getPositionCGAL();
        double radius = std::sqrt(CGAL::to_double((p - c).squared_length()));
        return new Circle(c, radius);
    }
}
