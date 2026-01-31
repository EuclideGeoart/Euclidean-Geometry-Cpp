#ifndef GEOMETRIC_OBJECT_H
#define GEOMETRIC_OBJECT_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "CharTraitsFix.h"
#include <string>

#include "ForwardDeclarations.h"  // For ObjectType enum
#include "ObjectType.h"
#include "Constants.h"
#include "Types.h"  // For Point_2 and other CGAL types
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

// Abstract base class for all geometric objects
class GeometricObject {
 public:
  // Add constructor that takes ObjectType and color
  GeometricObject(ObjectType type, const sf::Color &color)
      : m_type(type), m_color(color), m_selected(false), m_hovered(false), m_isValid(true) {}

  // NEW constructor to handle Point_2 and ID
  GeometricObject(ObjectType type, const sf::Color &color, const Point_2 &cgal_pos,
                  unsigned int id);
  GeometricObject(ObjectType type, const sf::Color &color, unsigned int id);
  virtual ~GeometricObject() = default;

  // Object properties
  virtual unsigned int getID() const { return m_id; }
  void setID(unsigned int id) { m_id = id; }
  virtual ObjectType getType() const { return m_type; }

  // Geometry operations
  virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const = 0;
  // NEW: Screen-space label rendering
  virtual void drawLabel(sf::RenderWindow &window, const sf::View &worldView) const {}
  virtual bool contains(const sf::Vector2f &worldPos, float tolerance) const = 0;
  virtual sf::FloatRect getGlobalBounds() const = 0;
  virtual void update() {}

  // Position access
  virtual Point_2 getCGALPosition() const = 0;
  virtual void setCGALPosition(const Point_2 &newPos) = 0;
  virtual void setPosition(const sf::Vector2f &newSfmlPos) = 0;
  virtual void setColor(const sf::Color &color);
  virtual void translate(const Vector_2 &offset) { (void)offset; }  // No default implementation
  void move(const Vector_2 &delta);
  
  // Point/Edge provider interface for generic anchor point detection
  // Shapes override these to expose their vertices and edges to all tools
  virtual std::vector<Point_2> getInteractableVertices() const { return {}; }
  virtual std::vector<Segment_2> getEdges() const { return {}; }
  virtual std::vector<Segment_2> getBoundarySegments() const { return getEdges(); }
  virtual bool getClosestPointOnPerimeter(const Point_2 &query, Point_2 &outPoint) const;
  
  // virtual int getIDIfAvailable() const { return m_id; }
  //  Selection/hover state
  virtual void setSelected(bool selected);
  virtual bool isSelected() const;
  virtual void setHovered(bool hovered);
  bool isHovered() const;

  // Visibility
  virtual void setVisible(bool visible) { m_visible = visible; }
  virtual bool isVisible() const { return m_visible; }

  // Label controls
  virtual void setShowLabel(bool show) { m_showLabel = show; }
  virtual bool getShowLabel() const { return m_showLabel; }
  virtual sf::FloatRect getLabelBounds(const sf::View &view) const {
    (void)view;
    return sf::FloatRect();
  }

  // Thickness (used by UI slider for line rendering)
  virtual void setThickness(float thickness) { m_thickness = thickness; }
  virtual float getThickness() const { return m_thickness; }

  // Vertex handle size (used by shapes with draggable vertices)
  virtual void setVertexHandleSize(float size) { m_vertexHandleSize = size; }
  virtual float getVertexHandleSize() const { return m_vertexHandleSize; }

  // Locking
  void setLocked(bool locked);
  virtual bool isLocked() const;
  virtual bool isDependent() const { return false; }
  void setDecoration(DecorationType t) { m_decoration = t; }
  DecorationType getDecoration() const { return m_decoration; }

  // Added validation method that can be overridden by derived classes
  virtual bool isValid() const {
    return m_isValid;
  }
  
  // Added getColor for serialization support
  virtual sf::Color getColor() const { return m_color; }
  
  // Hosted ObjectPoint management (Generic for all shapes)
  virtual void addChildPoint(std::shared_ptr<ObjectPoint> point);
  virtual void removeChildPoint(ObjectPoint* point);
  virtual void updateHostedPoints();
  const std::vector<std::weak_ptr<ObjectPoint>>& getHostedObjectPoints() const { return m_hostedObjectPoints; }

 protected:
  std::vector<std::weak_ptr<ObjectPoint>> m_hostedObjectPoints;
  ObjectType m_type;
  sf::Color m_color;
  unsigned int m_id;
  // Point_2 m_cgalPositionIfBase; // If Point_2 is stored in the base for all
  // objects

  bool m_selected = false;
  bool m_hovered = false;
  bool m_isValid = true;  // Assume valid on construction unless proven otherwise
  bool m_visible = true;
  bool m_locked = false;
  bool m_showLabel = true;
  sf::Vector2f m_labelOffset = {0.f, 0.f};
  float m_thickness = Constants::LINE_THICKNESS_DEFAULT;
  float m_vertexHandleSize = 4.0f;  // Default vertex handle size
  DecorationType m_decoration = DecorationType::None;
};

#endif  // GEOMETRIC_OBJECT_H