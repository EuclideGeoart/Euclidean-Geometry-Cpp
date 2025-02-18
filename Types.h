#ifndef TYPES_H
#define TYPES_H

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <boost/variant/get.hpp>
#include <SFML/Graphics.hpp>
#include <CGAL/squared_distance_2.h> 



typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Line_2 Line_2;
typedef K::Segment_2 Segment_2;
typedef K::Intersect_2 Intersect_2;
typedef K::Vector_2 Vector_2;
typedef K::FT FT;
typedef K::Circle_2 CGAL_Circle_2;
typedef CGAL::cpp11::result_of<K::Intersect_2(Line_2, Line_2)>::type Intersection_result;

 //Add squared distance computation
#endif
