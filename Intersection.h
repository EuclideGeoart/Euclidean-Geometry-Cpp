#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "Types.h"
#include <SFML/Graphics.hpp>
#include <CGAL/Simple_cartesian.h>

bool findIntersection(const Line_2& line1, const Line_2& line2, Point_2& intersection);

#endif


