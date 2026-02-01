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
 * @brief Polygon class for user-drawn polygons with arbitrary vertices
 */
class Polygon : public GeometricObject {
 public:
  /**
   * @brief Constructor with initial vertices
   * @param vertices Vector of CGAL points representing polygon vertices
   * @param color Fill color
   * @param id Unique identifier
   */
  Polygon(const std::vector<Point_2> &vertices, const sf::Color &color = sf::Color::White,
          unsigned int id = 0);

  Polygon(const std::vector<std::shared_ptr<Point>> &vertices,
        const sf::Color &color = sf::Color::White, unsigned int id = 0);

  virtual ~Polygon() = default;

  // GeometricObject interface
  virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
      virtual void update() override;
  virtual void setColor(const sf::Color &color) override;
  virtual bool contains(const sf::Vector2f &screenPos, float tolerance) const override;
  virtual std::string getTypeString() const { return "Polygon"; }
  virtual void translate(const Vector_2 &translation) override;
    virtual void updateDependentShape() override;
  virtual Point_2 getCGALPosition() const override;
  virtual void setCGALPosition(const Point_2 &newPos) override;
  virtual void setPosition(const sf::Vector2f &newSfmlPos) override;
  virtual sf::FloatRect getGlobalBounds() const override;
  
  // Point/Edge provider interface overrides
  std::vector<Point_2> getInteractableVertices() const override;
  std::vector<Segment_2> getEdges() const override;

  Point_2 getCenter() const;
  bool isWithinDistance(const sf::Vector2f &screenPos, float tolerance) const;
  void rotateCCW(const Point_2 &center, double angleRadians);

  // Polygon-specific methods
  std::shared_ptr<Point> getVertexPoint(size_t index) const {
       if (index < m_vertices.size()) return m_vertices[index];
       return nullptr;
  }
      void addVertex(const Point_2 &vertex);
      void addVertex(const std::shared_ptr<Point> &vertex);
  void removeLastVertex();
      std::vector<Point_2> getVertices() const;
  size_t getVertexCount() const { return m_vertices.size(); }
        void setVertexPosition(size_t index, const Point_2 &value);
        void setHoveredVertex(int index) { m_hoveredVertex = index; }
        void setActiveVertex(int index) { m_activeVertex = index; }
        int getHoveredVertex() const { return m_hoveredVertex; }
        int getActiveVertex() const { return m_activeVertex; }
        std::vector<sf::Vector2f> getVerticesSFML() const;
  bool isValid() const override { return m_vertices.size() >= 3; }

 private:
      std::vector<std::shared_ptr<Point>> m_vertices;  // Polygon vertices
  sf::ConvexShape m_sfmlShape;      // SFML shape for rendering (works for convex polygons)
        int m_hoveredVertex = -1;         // For handle coloring
        int m_activeVertex = -1;          // For handle coloring

  // Helper methods
  void updateSFMLShape();
  void updateSFMLShapeInternal();
        void drawVertexHandles(sf::RenderWindow &window, float scale) const;
};
