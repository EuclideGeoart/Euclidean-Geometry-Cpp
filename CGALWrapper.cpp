#include "Types.h"
#include <CGAL/Cartesian_converter.h>
#include <CGAL/Interval_nt.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Lazy.h>
#include <CGAL/intersections.h>

// Define any functions that use CGAL static variables here
// This ensures they're all in the same compilation unit

namespace CGALWrapper {
    // Wrapper for intersection
    boost::optional<boost::variant<Point_2, Line_2>> 
    findIntersection(const Line_2& line1, const Line_2& line2) {
        return CGAL::intersection(line1, line2);
    }
    
    // Wrapper for distance
    double distance(const Point_2& p1, const Point_2& p2) {
        return CGAL::to_double(CGAL::squared_distance(p1, p2));
    }
    
    // Add other CGAL operations you need
}