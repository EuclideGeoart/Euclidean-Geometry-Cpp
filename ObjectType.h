// ObjectType.h
#pragma once
#ifndef OBJECT_TYPE_H
#define OBJECT_TYPE_H

// Enum to track what kind of geometric object this point is attached to.
enum class ObjectType {
  None,                    // Default state, no specific object type
  Point,                   // Basic point
  Line,                    // Infinite line
  LineSegment,             // Line segment (finite line)
  ObjectPoint,             // Point dependent on another object
  Circle,                  // Circle
  Rectangle,               // Axis-aligned rectangle
  RectangleRotatable,      // Rotatable rectangle
  Polygon,                 // General polygon (user-drawn vertices)
  RegularPolygon,          // Regular polygon with N sides
  Triangle,                // General triangle (3 arbitrary vertices)

  Angle,                   // Angle measurement
  Hide,                    // Hide tool mode
  Detach,                  // Detach tool mode

  Intersection,            // Intersection tool
  IntersectionPoint,       // Resulting intersection point
  ParallelLine,            // Tool for creating parallel lines
  PerpendicularLine        // Tool for creating perpendicular lines
                           // Add more object types as needed
};

// Define drag modes here since it's used by multiple classes
enum class DragMode {
  None,
  MoveFreePoint,
  MoveLineEndpointStart,
  MoveLineEndpointEnd,
  TranslateLine,
  TranslateShape,
  MoveShapeVertex,
  DragObjectPoint,
  InteractWithCircle,
  CreateCircleCenter,
  CreateLineP1
};

// Defined here for convenience since it's related to drag operations
enum class EndpointSelection { None, Start, End };

class Point;
class Line;
class Circle;
class ObjectPoint;
class GeometricObject;
class GeometryEditor;

#endif // OBJECT_TYPE_H
