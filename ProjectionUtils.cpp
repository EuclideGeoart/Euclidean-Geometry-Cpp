#include "ProjectionUtils.h"

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message(                                                               \
    "CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif

// If projectOntoSegment is defined inline in ProjectionUtils.h,
// its definition should be removed from this .cpp file.

// Other non-inline utility function definitions can go here.
// For now, this file can be empty if all utilities are inline in the header.
