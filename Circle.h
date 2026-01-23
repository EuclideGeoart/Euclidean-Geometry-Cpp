#ifndef CIRCLE_H
#define CIRCLE_H

#include "GeometricObject.h"
#include "ObjectType.h"
#include "Types.h"
#include "Point.h"
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>

// Forward declaration
class ObjectPoint;

class Circle : public GeometricObject, public std::enable_shared_from_this<Circle> {
 public:
  // Constructor - now takes a Point pointer for the center
  Circle(Point *centerPoint, double radius, const sf::Color &color = sf::Color::Blue);
  ~Circle();

  // Factory method
  static std::shared_ptr<Circle> create(Point *centerPoint, double radius,
                                        const sf::Color &color = sf::Color::Blue);

  // Geometry
  Point_2 getCenterPoint() const;
  Point *getCenterPointObject() const { return m_centerPoint; }  // Get the Point object
  double getRadius() const { return m_radius; }
  Circle_2 getCGALCircle() const;

  void setRadius(double newRadius);
  void setCenter(const Point_2 &newCenter);

  // GeometricObject overrides
  ObjectType getType() const override { return ObjectType::Circle; }
  virtual void draw(sf::RenderWindow &window, float scale) const override;
  bool contains(const sf::Vector2f &worldPos, float tolerance) const override;
  bool isValid() const override;
  void update() override;
  sf::FloatRect getGlobalBounds() const override;
  Point_2 getCGALPosition() const override { return getCenterPoint(); }
  void setCGALPosition(const Point_2 &newPos) override { setCenter(newPos); }
  void translate(const Vector_2 &offset) override;
  void setPosition(const sf::Vector2f &newSfmlPos) override;
  void setColor(const sf::Color &color) override { m_outlineColor = color; updateSFMLShape(); }
  void setSelected(bool sel) override;
  void setHovered(bool hov) override;
  
  // Point/Edge provider interface overrides
  // Circle returns center as interactable vertex; circumference handled via ObjectPoint creation
  std::vector<Point_2> getInteractableVertices() const override;

  // Interaction helpers (for HandleEvents.cpp)
  bool isCenterPointHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const;
  bool isCircumferenceHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const;
  Point_2 projectOntoCircumference(const Point_2 &p) const;

  // Child ObjectPoint management
  // Child ObjectPoint management
  // addChildPoint, removeChildPoint, updateHostedPoints, getHostedObjectPoints inherited from GeometricObject

  // Lock/state
  void setLocked(bool locked) { m_isLocked = locked; }
  bool isLocked() const { return m_isLocked; }

 private:
  // Geometry
  Point *m_centerPoint;  // Reference to the actual center Point object
  double m_radius;

  // Visual
  sf::CircleShape m_sfmlShape;
  sf::CircleShape m_centerVisual;
  sf::Color m_outlineColor;
  sf::Color m_fillColor;

  // State
  bool m_isLocked = false;

  // Child ObjectPoints
  // std::vector<std::weak_ptr<ObjectPoint>> m_hostedObjectPoints; // Inherited from GeometricObject

  // Helper
  void updateSFMLShape();
};

#endif  // CIRCLE_H
