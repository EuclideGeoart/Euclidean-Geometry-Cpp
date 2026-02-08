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
    Circle(Point *centerPoint, std::shared_ptr<Point> radiusPoint, double radius,
      const sf::Color &color = sf::Color::Blue);
  ~Circle();

  // Factory method
  static std::shared_ptr<Circle> create(Point *centerPoint, std::shared_ptr<Point> radiusPoint,
                                        double radius, const sf::Color &color = sf::Color::Blue);

  // Geometry
  Point_2 getCenterPoint() const;
  Point *getCenterPointObject() const { return m_centerPoint; }  // Get the Point object
  Point *getRadiusPointObject() const { return m_radiusPoint.get(); }
  double getRadius() const { return m_radius; }
  Circle_2 getCGALCircle() const;

  virtual void setRadius(double newRadius);
  void setCenter(const Point_2 &newCenter);
  void setCenterPointObject(Point *newCenterPoint);
  void clearCenterPoint();
  void setRadiusPoint(std::shared_ptr<Point> pt);

  // GeometricObject overrides
  ObjectType getType() const override { return ObjectType::Circle; }
  virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
  virtual void drawLabel(sf::RenderWindow &window, const sf::View &worldView) const override;
  bool contains(const sf::Vector2f &worldPos, float tolerance) const override;
  bool isValid() const override;
  void update() override;
  void updateDependentShape() override;
  sf::FloatRect getGlobalBounds() const override;
  Point_2 getCGALPosition() const override { return getCenterPoint(); }
  void setCGALPosition(const Point_2 &newPos) override { setCenter(newPos); }
  void translate(const Vector_2 &offset) override;
  void setPosition(const sf::Vector2f &newSfmlPos) override;
  void setColor(const sf::Color &color) override {
    m_color = color;
    m_fillColor = sf::Color(color.r, color.g, color.b, color.a);
    updateSFMLShape();
  }
  sf::Color getFillColor() const { return m_sfmlShape.getFillColor(); }
  void setSelected(bool sel) override;
  void setHovered(bool hov) override;
   // Helper
  void updateSFMLShape();
  // Point/Edge provider interface overrides
  // Circle returns center as interactable vertex; circumference handled via ObjectPoint creation
  std::vector<Point_2> getInteractableVertices() const override;
  bool getClosestPointOnPerimeter(const Point_2 &query, Point_2 &outPoint) const override;

  // Interaction helpers (for HandleEvents.cpp)
  bool isCenterPointHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const;
  bool isCircumferenceHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const;
  Point_2 projectOntoCircumference(const Point_2 &p) const;

  // Child ObjectPoint management
  // Child ObjectPoint management
  // addChildPoint, removeChildPoint, updateHostedPoints, getHostedObjectPoints inherited from GeometricObject

  // CHILD OBJECTPOINT MANAGEMENT handled by GeometricObject

  // Semicircle support
  void setSemicircle(bool isSemi) { m_isSemicircle = isSemi; }
  bool isSemicircle() const { return m_isSemicircle; }
    void setSemicircleDiameterPoints(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2) {
      m_diameterP1 = p1;
      m_diameterP2 = p2;
    }
    std::shared_ptr<Point> getDiameterP1() const { return m_diameterP1; }
    std::shared_ptr<Point> getDiameterP2() const { return m_diameterP2; }
  void setSemicircleBasis(const Point_2& p1, const Point_2& p2) {
      // Optional: Store basis points if needed for exact arc recalculation
      // For now, relies on center + radius + angular logic in draw()
      m_semicircleStart = p1;
      m_semicircleEnd = p2;
  }

  // 3-Point Circle support
  void set3PointDefinition(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2, std::shared_ptr<Point> p3);
  bool is3PointCircle() const { return m_is3PointCircle; }
  void get3PointDefinition(std::shared_ptr<Point>& p1, std::shared_ptr<Point>& p2, std::shared_ptr<Point>& p3) const {
    p1 = m_p1; p2 = m_p2; p3 = m_p3;
  }

 private:
  // Geometry
  Point *m_centerPoint;  // Reference to the actual center Point object
  std::shared_ptr<Point> m_radiusPoint;
  double m_radius;

  // Visual
  sf::CircleShape m_sfmlShape;
  sf::CircleShape m_centerVisual;
  sf::Color m_outlineColor;
  sf::Color m_fillColor;

  // State
  
  // Semicircle State
  bool m_isSemicircle = false;
  std::shared_ptr<Point> m_diameterP1;
  std::shared_ptr<Point> m_diameterP2;
  Point_2 m_semicircleStart; // For defining the arc range
  Point_2 m_semicircleEnd;
  
  // 3-Point Circle State
  bool m_is3PointCircle = false;
  std::shared_ptr<Point> m_p1;
  std::shared_ptr<Point> m_p2;
  std::shared_ptr<Point> m_p3;
  
  // Child ObjectPoints
  // std::vector<std::weak_ptr<ObjectPoint>> m_hostedObjectPoints; // Inherited from GeometricObject

 
};
#endif  // CIRCLE_H
