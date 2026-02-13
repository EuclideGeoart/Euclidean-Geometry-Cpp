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
  Ray,                     // Ray (half-infinite line)
  Vector,                  // Vector (directed segment)
  ObjectPoint,             // Point dependent on another object
  Circle,                  // Circle
  Semicircle,              // Semicircle
  Rectangle,               // Axis-aligned rectangle
  RectangleRotatable,      // Rotatable rectangle
  Polygon,                 // General polygon (user-drawn vertices)
  RegularPolygon,          // Regular polygon with N sides
  RegularPolygonEdge,      // Regular polygon defined by one edge (two adjacent vertices)
  Triangle,                // General triangle (3 arbitrary vertices)

  Angle,                   // Angle measurement
  AngleGiven,              // Angle with given size (popup input)
  TextLabel,               // Text/LaTeX label
  Hide,                    // Hide tool mode
  Detach,                  // Detach tool mode

  Intersection,            // Intersection tool
  IntersectionPoint,       // Resulting intersection point
  Circle3P,                // Circle defined by 3 points
  ParallelLine,            // Tool for creating parallel lines
  PerpendicularLine,       // Tool for creating perpendicular lines
  PerpendicularBisector,   // Tool for constructing perpendicular bisectors
  AngleBisector,           // Tool for constructing angle bisectors
  TangentLine,             // Tool for constructing tangents to a circle
  Midpoint,                // Tool for creating midpoints
  Compass,                 // Tool for creating circles with specific radius
  ReflectAboutLine,        // Transform: reflect point about line
  ReflectAboutPoint,       // Transform: reflect point about point
  ReflectAboutCircle,      // Transform: inversion about circle
  RotateAroundPoint,       // Transform: rotate around point
  TranslateByVector,       // Transform: translate by vector
  DilateFromPoint          // Transform: dilate from point
                           // Add more object types as needed
};

// Enum to track what kind of labeling to show
enum class LabelMode {
    Hidden,
    Name,
    NameAndValue,
    Value,
    Caption
};

// Enum to track what kind of transformation is applied to an object
enum class TransformationType {
    None,
    Translate,
    Rotate,
    Reflect,
    ReflectPoint,
    ReflectCircle,
    Dilate
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
  CreateLineP1,
  RotatedRectP2,
  RotatedRectHeight,
  CreatingRectangle
};

// Defined here for convenience since it's related to drag operations
enum class EndpointSelection { None, Start, End };

class GeometricObject;
class GeometryEditor;
class Point;
class Line;
class Circle;
class ObjectPoint;

#endif // OBJECT_TYPE_H
