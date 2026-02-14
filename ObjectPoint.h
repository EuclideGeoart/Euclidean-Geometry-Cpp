#pragma once
#ifndef OBJECT_POINT_H
#define OBJECT_POINT_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "CharTraitsFix.h" // Ensure this is very early
#include <string>          // Ensure standard string is included very early

#include "Constants.h" // For default colors, etc.
#include "GeometricObject.h"
#include "ObjectType.h"
#include "Point.h" // Base class
#include "Types.h"
#include <SFML/Graphics.hpp>
#include <memory>
// Forward declarations - they should already be in ForwardDeclarations.h
// but including them here for safety
class Line;
class Circle;

class ObjectPoint : public Point,
                    public std::enable_shared_from_this<ObjectPoint> {
  // Use shared_ptr for ObjectPoint to manage ownership and lifetime
  using ObjectPointPtr = std::shared_ptr<ObjectPoint>;

  // Constructor for attaching to a Line
  ObjectPoint(Line *hostLine, const Point_2 &initialPos,
              const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);
  ObjectPoint(Line *hostLine, double relativePos,
              const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);

public:
  // Constructors
  ObjectPoint(std::shared_ptr<Line> hostLine, const Point_2 &initialPos,
              const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);
  ObjectPoint(std::shared_ptr<Line> hostLine, double relativePos,
              const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);

  // Constructor for attaching to a Circle
  ObjectPoint(std::shared_ptr<Circle> hostCircle, const Point_2 &initialPos,
              const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);
  ObjectPoint(std::shared_ptr<Circle> hostCircle, double angleRad,
              const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);

  // Stub constructor for deserialization Pass 1
  ObjectPoint(const Point_2 &pos, float zoom, const sf::Color &color, unsigned int id)
      : Point(pos, zoom, color, id) {}

  virtual ~ObjectPoint(); // Virtual destructor
  // --- Factory Methods ---
  static std::shared_ptr<ObjectPoint> create(std::shared_ptr<Line> hostLine,
                                             double relativePosition,
                                             const sf::Color &color);

  static std::shared_ptr<ObjectPoint> create(std::shared_ptr<Circle> hostCircle,
                                             double angleRad,
                                             const sf::Color &color);
  // Factory for attaching directly to a circle's center
  static std::shared_ptr<ObjectPoint> createCenter(std::shared_ptr<Circle> hostCircle,
                                                   const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);
                                                   
  // Factory for attaching to any shape's edge
  // @param hostShape The shape that owns the edge
  // @param edgeIndex Index of the edge (from getEdges())
  // @param relativePosition Position along the edge (0.0 to 1.0)
  // @param color Point color
  static std::shared_ptr<ObjectPoint> createOnShapeEdge(
      std::shared_ptr<GeometricObject> hostShape,
      size_t edgeIndex,
      double relativePosition,
      const sf::Color &color = Constants::OBJECT_POINT_DEFAULT_COLOR);
                                                   
  void projectOntoHost(const Point_2 &clickPos);

  // --- GeometricObject Overrides ---
  virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
  bool contains(const sf::Vector2f &worldPos, float tolerance) const override;
  void setSelected(bool sel) override;
  ObjectType getType() const override { return ObjectType::ObjectPoint; }

  void update() override;
  void updatePositionFromParameters();
  // --- Host Interaction ---
  GeometricObject *getHostObject() const { return m_hostObject.lock().get(); }
  ObjectType getHostType() const { return m_hostType; }

  // Called by the host object when the host has changed
  void updatePositionFromHost();

  // Called when the ObjectPoint itself is dragged by the mouse
  void dragTo(const sf::Vector2f &targetSfmlPos) override;

  // --- Position Management ---
  void setPosition(const sf::Vector2f &newSfmlPos) override;

  // --- Specific Getters for Attachment Info ---
  double getRelativePositionOnLine() const;
  double getAngleOnCircle() const;
  void setCGALPosition(const Point_2 &newPos) override;
  void translate(const Vector_2 &offset) override;
  void setHost(std::shared_ptr<GeometricObject> host, ObjectType hostType);
  void setHost(std::shared_ptr<Line> lineHost);
  void setHost(std::shared_ptr<Circle> circleHost);
  void clearHost(); // Make sure this is public and correctly implemented
  void relinkHost(std::shared_ptr<GeometricObject> host, double t, ObjectType hostType);
  void validate() const;

  // Add the getPositionCGAL method declaration to the public section of your
  // class
  Point_2 getPositionCGAL() const;

  // Method to get the position in SFML coordinates
  sf::Vector2f getSfmlPosition() const;

  // Method to update position based on mouse position
  void updateFromMousePos(const sf::Vector2f &mousePos);

  // --- New Members Based on Suggested Changes ---
  bool isValid() const override;
  
  // Vertex anchor flag - determines if dragging resizes shape or slides on edge
  void setIsVertexAnchor(bool isAnchor) { m_isVertexAnchor = isAnchor; }
  bool isVertexAnchor() const { return m_isVertexAnchor; }

  // Shape edge attachment accessors
  bool isShapeEdgeAttachment() const { return m_isShapeEdgeAttachment; }
  size_t getEdgeIndex() const { return m_edgeIndex; }
  double getEdgeRelativePosition() const { return m_edgeRelativePos; }

protected:
  void updateSFMLShape() override;
  void updateSFMLShape(const sf::Vector2f &position) override;
  void initializeShape();

private:
  // Private default constructor for factory methods (like createOnShapeEdge)
  ObjectPoint() : Point(Point_2(0, 0), 1.0f, sf::Color::Yellow),
                  m_hostType(ObjectType::None),
                  m_relativePositionOnLine(safe_zero_ft()),
                  m_angleOnCircleRad(0.0),
                  m_isCircleCenterAttachment(false),
                  m_edgeIndex(0),
                  m_edgeRelativePos(0.0),
                  m_isShapeEdgeAttachment(false),
                  m_isBeingDragged(false) {}
                  
  std::weak_ptr<GeometricObject> m_hostObject;
  ObjectType m_hostType = ObjectType::None;

  // Host-specific pointers
  std::weak_ptr<Line> m_hostLine;
  std::weak_ptr<Circle> m_hostCircle;
  double m_angle = 0.0;
  double m_relativePos = 0.0; // For lines (0.0 to 1.0 for segments)
  // Attachment-specific data
  double m_relativePosition = 0.0;
  Kernel::FT m_relativePositionOnLine = safe_zero_ft(); // For lines (0.0 to 1.0 for segments)
  double m_angleOnCircleRad = 0.0;       // For circles (radians)
  bool m_isCircleCenterAttachment = false; // If true, attach to circle center
  std::vector<Point_2> *m_childPoints = nullptr; // For hosted points
  void calculateAttachmentParameters();
  Point_2 calculatePositionOnHost() const;

  // Shape edge attachment data
  std::weak_ptr<GeometricObject> m_hostShape;  // For edge-attached points
  size_t m_edgeIndex = 0;                      // Which edge of the shape
  double m_edgeRelativePos = 0.0;              // Position along edge (0.0 to 1.0)
  bool m_isShapeEdgeAttachment = false;        // True if attached to a shape edge
  bool m_isVertexAnchor = false;               // True if created as vertex anchor (resizes shape)
                                               // False if created on edge (slides only)

  // Drag state
  bool m_isBeingDragged = false;
};

#endif // OBJECT_POINT_H