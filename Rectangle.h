#pragma once
#include "CharTraitsFix.h"

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>

#include "GeometricObject.h"
#include "Types.h"

class Point;

/**
 * @brief Rectangle class supporting both axis-aligned and rotatable variants
 */
class Rectangle : public GeometricObject {
 public:
  /**
   * @brief Constructor for axis-aligned rectangle
   * @param corner1 First corner (CGAL point)
   * @param corner2 Opposite corner (CGAL point)
   * @param isRotatable Whether the rectangle is rotatable
   * @param color Fill color
   * @param id Unique identifier
   */
  Rectangle(const Point_2& corner1, const Point_2& corner2, bool isRotatable = false, const sf::Color& color = sf::Color::White, unsigned int id = 0);

  Rectangle(const std::shared_ptr<Point>& corner1,
            const std::shared_ptr<Point>& corner2,
            bool isRotatable = false,
            const sf::Color& color = sf::Color::White,
            unsigned int id = 0);

  /**
   * @brief Constructor for rotatable rectangle
   * @param corner First corner (CGAL point)
   * @param adjacentPoint Point defining one side and rotation
   * @param width Width of the rectangle
   * @param color Fill color
   * @param id Unique identifier
   */
  Rectangle(const Point_2& corner, const Point_2& adjacentPoint, double width, const sf::Color& color = sf::Color::White, unsigned int id = 0);

  Rectangle(const std::shared_ptr<Point>& corner,
            const std::shared_ptr<Point>& adjacentPoint,
            double width,
            const sf::Color& color = sf::Color::White,
            unsigned int id = 0);

  // Takes pointers to 4 existing points so the shape stays connected to them.
  Rectangle(std::shared_ptr<Point> p1,
            std::shared_ptr<Point> p2,
            std::shared_ptr<Point> pb,
            std::shared_ptr<Point> pd,
            bool isRotatable,
            const sf::Color& color,
            unsigned int id);

  // new  Polygon style constructor
  Rectangle(const std::vector<std::shared_ptr<Point>>& vertices, bool isRotatable, 
          const sf::Color& color, 
          unsigned int id);

  virtual ~Rectangle() = default;

  // GeometricObject interface
  virtual void draw(sf::RenderWindow& window, float scale, bool forceVisible = false) const override;
  virtual void drawLabel(sf::RenderWindow& window, const sf::View& worldView) const override;
  virtual void update() override;
  virtual void updateDependentShape() override;
  virtual void setColor(const sf::Color& color) override;
  virtual bool contains(const sf::Vector2f& screenPos, float tolerance) const override;
  virtual std::string getTypeString() const { return "Rectangle"; }
  virtual void translate(const Vector_2& translation) override;
  virtual Point_2 getCGALPosition() const override;
  virtual void setCGALPosition(const Point_2& newPos) override;
  virtual void setPosition(const sf::Vector2f& newSfmlPos) override;
  virtual sf::FloatRect getGlobalBounds() const override;

  // Point/Edge provider interface overrides
  std::vector<Point_2> getInteractableVertices() const override;
  std::vector<Segment_2> getEdges() const override;

  // Additional methods (not overrides)
  sf::Color getColor() const override { return m_color; }
  Point_2 getCenter() const;
  sf::Color getFillColor() const { return m_sfmlShape.getFillColor(); }
  bool isWithinDistance(const sf::Vector2f& screenPos, float tolerance) const;
  void rotateCCW(const Point_2& center, double angleRadians);

  // Rectangle-specific getters
  std::shared_ptr<Point> getCorner1Point() const { return m_corner1; }
  std::shared_ptr<Point> getCorner2Point() const { return m_corner2; }
  std::shared_ptr<Point> getCornerBPoint() const { return m_cornerB; }
  std::shared_ptr<Point> getCornerDPoint() const { return m_cornerD; }
  void setDependentCornerPoints(const std::shared_ptr<Point>& b, const std::shared_ptr<Point>& d);
  Point_2 getCorner1Position() const;
  Point_2 getCorner2Position() const;
  void setCorner1Position(const Point_2& pos, bool triggerUpdate = true);
  void setCorner2Position(const Point_2& pos, bool triggerUpdate = true);
  Point_2 getCorner1() const;
  Point_2 getCorner2() const;
  bool isRotatable() const { return m_isRotatable; }
      void setUseExplicitVertices(bool use) { m_useExplicitVertices = use; }
  double getWidth() const { return m_width; }
  double getHeight() const { return m_height; }
  double getRotationAngle() const { return m_rotationAngle; }
  std::vector<Point_2> getVertices() const;
  std::vector<sf::Vector2f> getVerticesSFML() const;
  void setVertexPosition(size_t index, const Point_2& value);
  void setHoveredVertex(int index) { m_hoveredVertex = index; }
  void setActiveVertex(int index) { m_activeVertex = index; }
  int getHoveredVertex() const { return m_hoveredVertex; }
  int getActiveVertex() const { return m_activeVertex; }

  // Label offset support for vertex labels
  void setVertexLabelOffset(size_t vertexIndex, const sf::Vector2f& offset);
  sf::Vector2f getVertexLabelOffset(size_t vertexIndex) const;

  // Rectangle-specific setters
  void setCorners(const Point_2& corner1, const Point_2& corner2);
  void setRotation(double angleRadians);
  void setHeight(double height);

 private:
  std::shared_ptr<Point> m_corner1;  // First corner point
  std::shared_ptr<Point> m_corner2;  // Second corner point
  std::shared_ptr<Point> m_cornerB;  // Dependent corner (B)
  std::shared_ptr<Point> m_cornerD;  // Dependent corner (D)
  bool m_isRotatable;                // True if rectangle can be rotated
        bool m_useExplicitVertices = false; // True when vertices are authoritative
  bool m_isUpdating = false;         // Recursion guard to prevent infinite update loops
  double m_width;                    // Width of the rectangle
  double m_height;                   // Height of the rectangle
  double m_rotationAngle;            // Rotation angle in radians (for rotatable rectangles)
  Point_2 m_center;                  // Geometric center (authoritative for rotatable rectangles)
  sf::ConvexShape m_sfmlShape;       // SFML shape for rendering
  int m_hoveredVertex = -1;
  int m_activeVertex = -1;
  std::vector<sf::Vector2f> m_vertexLabelOffsets;  // Per-vertex label offsets (screen pixels)
  std::vector<std::string> m_generatedLabels;      // Explicit labels for the 4 vertices (A, B, C, D replacement)

  // Helper methods
  void updateSFMLShape();
  void updateDimensionsFromCorners();
  void updateCornerPositions();
  void drawVertexHandles(sf::RenderWindow& window, float scale) const;
  void syncDependentCorners();
  void syncRotatableFromAnchors();
};
