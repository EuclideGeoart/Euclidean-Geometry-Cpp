#include "Intersection.h"
#include <CGAL/intersections.h>
#include <boost/variant.hpp>


// Use a visitor to safely handle the variant alternatives.
// This avoids the ambiguous duplicate type problem when using get_if.

bool findIntersection(const Line_2& line1, const Line_2& line2, Point_2& intersection) {
    auto result = CGAL::intersection(line1, line2);
    if (!result) {
        return false;
    }

    // Use a visitor to safely handle the variant alternatives.
    bool found = false;
    std::visit([&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, Point_2>) {
            intersection = arg;
            found = true;
        }
        else if constexpr (std::is_same_v<T, K::Segment_2>) {
            // If the intersection is a segment, choose its midpoint.
            intersection = CGAL::midpoint(arg.source(), arg.target());
            found = true;
        }
        }, *result);

    return found;
}





