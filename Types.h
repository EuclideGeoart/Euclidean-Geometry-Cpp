#pragma once
#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif
// CGAL Kernel
#include <CGAL/Aff_transformation_2.h> // For Aff_transformation_2
#include <CGAL/Circle_2.h>
#include <CGAL/Direction_2.h> // For Direction_2
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Line_2.h>
#include <CGAL/Point_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/number_utils.h> // For CGAL::is_finite

// Define the kernel
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;

// Define Number Type (FieldType)
using FT = Kernel::FT;

// Define basic geometric types using the chosen kernel
using Point_2 = Kernel::Point_2;
using Vector_2 = Kernel::Vector_2;
using Line_2 = Kernel::Line_2;
using Segment_2 = Kernel::Segment_2;
using Circle_2 = Kernel::Circle_2;
using Direction_2 = Kernel::Direction_2;
using Aff_transformation_2 = Kernel::Aff_transformation_2;

// CoordinateTransform can be an alias for Aff_transformation_2 or a custom
// struct/class For now, let's assume it's related to Aff_transformation_2 or
// defined elsewhere if custom. If it's just an affine transformation:
using CoordinateTransform = Aff_transformation_2;

// Helper function to create a "safe" zero FT
inline FT safe_zero_ft() { return FT(0); }

// Helper function to create a "safe" one FT
inline FT safe_one_ft() { return FT(1); }

// Line decoration symbols
enum class DecorationType {
    None,
    Tick1, Tick2, Tick3,    // For Equality (e.g., AB = CD)
    Arrow1, Arrow2, Arrow3  // For Parallelism (e.g., AB || CD)
};
