#pragma once

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#endif

#include "Types.h"
#include <vector>

class GeometricObject;

namespace IntersectionSystem {
std::vector<Point_2> computeIntersections(const GeometricObject &a,
                                          const GeometricObject &b);
}
