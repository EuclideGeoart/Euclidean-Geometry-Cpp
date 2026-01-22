#pragma once
#include "Types.h"
#include <boost/optional.hpp>
#include <boost/variant.hpp>

namespace CGALWrapper {
    boost::optional<boost::variant<Point_2, Line_2>> 
    findIntersection(const Line_2& line1, const Line_2& line2);
    
    double distance(const Point_2& p1, const Point_2& p2);
    
    // Declare other CGAL operations you need
}