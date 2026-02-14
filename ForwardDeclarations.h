#pragma once
#ifndef FORWARD_DECLARATIONS_H
#define FORWARD_DECLARATIONS_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif
// Forward declarations for common classes to avoid circular includes
class GeometricObject;
class Point;
class Line;
class Circle;
class ObjectPoint;
class Angle;
class RayObject;
class VectorObject;

// Add missing enum class declarations
enum class DragMode;
enum class ObjectType;
enum class EndpointSelection;

#endif // FORWARD_DECLARATIONS_H