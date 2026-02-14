#pragma once

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "GeometricObject.h"
#include "Types.h"
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>

class Point;

/**
 * @brief RegularPolygon class for creating regular polygons with N sides
 */
class RegularPolygon : public GeometricObject {
 public:
  enum class CreationMode { CenterAndVertex, Edge };

  /**
   * @brief Constructor for regular polygon
   * @param center Center point of the polygon (CGAL)
   * @param firstVertex Point on the polygon (defines radius)
   * @param numSides Number of sides for the polygon
   * @param color Fill color
   * @param id Unique identifier
   */
  RegularPolygon(const Point_2 &center, const Point_2 &firstVertex, int numSides,
                 const sf::Color &color = sf::Color::White, unsigned int id = 0);

  RegularPolygon(const std::shared_ptr<Point> &center, const std::shared_ptr<Point> &firstVertex,
                 int numSides, const sf::Color &color = sf::Color::White, unsigned int id = 0);

  RegularPolygon(const std::shared_ptr<Point> &edgeStart, const std::shared_ptr<Point> &edgeEnd,
                 int numSides, const sf::Color &color, unsigned int id, CreationMode mode);

  virtual ~RegularPolygon() = default;

  // GeometricObject interface
  virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
  virtual void drawLabel(sf::RenderWindow &window, const sf::View &worldView) const override;
  virtual void update() override;
  virtual void updateDependentShape() override;
  virtual void setColor(const sf::Color &color) override;
  sf::Color getFillColor() const { return m_sfmlShape.getFillColor(); }
  virtual bool contains(const sf::Vector2f &screenPos, float tolerance) const override;
  virtual std::string getTypeString() const { return "RegularPolygon"; }
  virtual void translate(const Vector_2 &translation) override;
  void rotateCCW(const Point_2 &center, double angleRadians);
  virtual Point_2 getCGALPosition() const override;
  virtual void setCGALPosition(const Point_2 &newPos) override;
  virtual void setPosition(const sf::Vector2f &newSfmlPos) override;
  virtual sf::FloatRect getGlobalBounds() const override;
  
  // Point/Edge provider interface overrides
  std::vector<Point_2> getInteractableVertices() const override;
  std::vector<Segment_2> getEdges() const override;

  Point_2 getCenter() const;
  bool isWithinDistance(const sf::Vector2f &screenPos, float tolerance) const;

  // RegularPolygon-specific methods
  std::shared_ptr<Point> getCenterPoint() const { return m_centerPoint; }
  std::shared_ptr<Point> getFirstVertexPoint() const { return m_firstVertexPoint; }
  std::shared_ptr<Point> getEdgeStartPoint() const { return m_edgeStartPoint; }
  std::shared_ptr<Point> getEdgeEndPoint() const { return m_edgeEndPoint; }
  const std::vector<std::shared_ptr<Point>>& getDerivedVertices() const { return m_derivedVertices; }
  CreationMode getCreationMode() const { return m_creationMode; }
  int getNumSides() const { return m_numSides; }
  double getRadius() const { return m_radius; }
  double getRotationAngle() const { return m_rotationAngle; }
  const std::vector<Point_2> &getVertices() const { return m_vertices; }
  std::vector<sf::Vector2f> getVerticesSFML() const;
  void setVertexPosition(size_t index, const Point_2 &value);
  void setHoveredVertex(int index) { m_hoveredVertex = index; }
  void setActiveVertex(int index) { m_activeVertex = index; }
  int getHoveredVertex() const { return m_hoveredVertex; }
  int getActiveVertex() const { return m_activeVertex; }

  // Setters
  void setNumSides(int numSides);
  void setRadius(double radius);

  // Creation point methods (for dragging: center + first vertex only)
  std::vector<sf::Vector2f> getCreationPointsSFML() const;
  void setCreationPointPosition(size_t index, const Point_2& value);  // 0=center, 1=first vertex

 private:
  CreationMode m_creationMode = CreationMode::CenterAndVertex;
  std::shared_ptr<Point> m_centerPoint;      // Center point
  std::shared_ptr<Point> m_firstVertexPoint; // First vertex point
  std::shared_ptr<Point> m_edgeStartPoint;   // Edge mode start point
  std::shared_ptr<Point> m_edgeEndPoint;     // Edge mode end point
  std::vector<std::shared_ptr<Point>> m_derivedVertices; // Persistent derived polygon vertices
  std::vector<Point_2> m_vertices;  // Polygon vertices
  int m_numSides;                   // Number of sides (must be >= 3)
  double m_radius;                  // Distance from center to vertex
  double m_rotationAngle;           // Rotation angle in radians
  sf::ConvexShape m_sfmlShape;      // SFML shape for rendering
  int m_hoveredVertex = -1;
  int m_activeVertex = -1;
  bool m_isUpdating = false;        // Recursion guard

  // Helper methods
  void generateVertices();
  void ensureDerivedVertices();
  void syncDerivedVertices();
  void updateSFMLShape();
  void updateSFMLShapeInternal();
  void drawVertexHandles(sf::RenderWindow &window, float scale) const;
};
