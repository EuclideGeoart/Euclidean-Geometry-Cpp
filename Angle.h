#pragma once
#ifndef ANGLE_H
#define ANGLE_H

#include "GeometricObject.h"
#include "Point.h"
#include "Constants.h"
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>

class Angle : public GeometricObject {
 public:
  Angle(const std::shared_ptr<Point> &a, const std::shared_ptr<Point> &vertex,
        const std::shared_ptr<Point> &b, bool reflex = false,
        const sf::Color &color = sf::Color(255, 200, 0));
  Angle(const std::shared_ptr<Point> &a, const Point_2 &virtualVertex,
    const std::shared_ptr<Point> &b, bool reflex = false,
    const sf::Color &color = sf::Color(255, 200, 0));

  ObjectType getType() const override { return ObjectType::Angle; }
  void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
  bool contains(const sf::Vector2f &worldPos, float tolerance) const override;
  bool isMouseOverArc(const sf::Vector2f &worldPos, float tolerance) const;
      bool isValid() const override { return m_isValid; }
  sf::FloatRect getGlobalBounds() const override { return m_bounds; }
  void update() override { updateSFMLShape(); }

  Point_2 getCGALPosition() const override { return m_vertexPoint; }
  void setCGALPosition(const Point_2 &newPos) override { (void)newPos; }
  void setPosition(const sf::Vector2f &newPos) override { (void)newPos; }
  void setColor(const sf::Color &color) override;
  sf::Color getFillColor() const { return m_fillColor; }
  void translate(const Vector_2 &offset) override { (void)offset; }

  void setReflex(bool reflex);
  bool isReflex() const { return m_isReflex; }
  float getCurrentDegrees() const { return m_currentDegrees; }
  void setRadius(double radius);
  double getRadius() const { return m_arcRadius; }
  void setRightAngleMarkerSize(double size);
  double getRightAngleMarkerSize() const { return m_markerSize; }
  void setVirtualVertex(const Point_2& vertex);
  bool hasVirtualVertex() const { return m_hasVirtualVertex; }
  void clearVirtualVertex();
  void setFallbackDirections(const Vector_2& dirA, const Vector_2& dirB);
  void drawVertexHandles(sf::RenderWindow &window, float scale) const;
  
  // Label overrides
  void drawLabel(sf::RenderWindow &window, const sf::View &view) const override;
  sf::Vector2f getLabelAnchor(const sf::View &view) const override;
  
  void relinkTransformation(std::shared_ptr<GeometricObject> p1, std::shared_ptr<GeometricObject> p2, std::shared_ptr<GeometricObject> p3 = nullptr) override {
    m_vertex = std::dynamic_pointer_cast<Point>(p1);
    m_pointA = std::dynamic_pointer_cast<Point>(p2);
    m_pointB = std::dynamic_pointer_cast<Point>(p3);
    clearVirtualVertex();
    if (auto v = m_vertex.lock()) v->addDependent(m_selfHandle);
    if (auto a = m_pointA.lock()) a->addDependent(m_selfHandle);
    if (auto b = m_pointB.lock()) b->addDependent(m_selfHandle);
    updateSFMLShape();
  }
  
  // Label offset support (already in base)
  
  // Access to weak_ptr points (for serialization)
  std::weak_ptr<Point> getPointA() const { return m_pointA; }
  std::weak_ptr<Point> getVertex() const { return m_vertex; }
  std::weak_ptr<Point> getPointB() const { return m_pointB; }

 private:
  void updateSFMLShape();
  bool resolvePoints(Point_2 &a, Point_2 &v, Point_2 &b) const;
  static double normalizeSignedPi(double angleRad);
  static double normalizePositive(double angleRad);

  std::weak_ptr<Point> m_pointA;
  std::weak_ptr<Point> m_vertex;
  std::weak_ptr<Point> m_pointB;
  bool m_hasVirtualVertex = false;
  Point_2 m_virtualVertex = Point_2(0, 0);

  bool m_isReflex = false;
  float m_currentDegrees = 0.0f;

  sf::VertexArray m_arc{sf::LineStrip};
  sf::Text m_text;
  sf::FloatRect m_bounds;

  Point_2 m_vertexPoint = Point_2(0, 0);
  double m_arcRadius = 20.0;
  double m_visualRadius = 40.0;
  double m_markerSize = 12.0;
  double m_startAngle = 0.0;
  double m_sweepAngle = 0.0;
  bool m_hasFallbackDirA = false;
  bool m_hasFallbackDirB = false;
  double m_fallbackDirAx = 0.0;
  double m_fallbackDirAy = 0.0;
  double m_fallbackDirBx = 0.0;
  double m_fallbackDirBy = 0.0;
  
  sf::Color m_fillColor;
  sf::Color m_outlineColor;
  
  // Resize support
  double m_customRadius = -1.0; // -1 means auto-calculate
  sf::CircleShape m_resizeHandle;
  
  // Fill support
  sf::VertexArray m_fillFan{sf::TriangleFan};
  
  // Resize support

  static sf::Font s_font;
  static bool s_fontLoaded;
};

#endif // ANGLE_H
