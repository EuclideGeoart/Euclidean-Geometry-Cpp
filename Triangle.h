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
 * @brief Triangle class for general (non-regular) triangles with arbitrary vertices
 * 
 * Supports any three non-collinear points. No geometric constraints are applied.
 * Vertices can be manipulated independently while maintaining triangle validity.
 */
class Triangle : public GeometricObject {
public:
    /**
     * @brief Constructor for general triangle
     * @param v1 First vertex (CGAL point)
     * @param v2 Second vertex (CGAL point)
     * @param v3 Third vertex (CGAL point)
     * @param color Fill color
     * @param id Unique identifier
     */
    Triangle(const Point_2& v1, const Point_2& v2, const Point_2& v3,
             const sf::Color& color = sf::Color::White, unsigned int id = 0);

    Triangle(const std::shared_ptr<Point>& v1, const std::shared_ptr<Point>& v2,
             const std::shared_ptr<Point>& v3, const sf::Color& color = sf::Color::White,
             unsigned int id = 0);

    virtual ~Triangle() = default;

    // GeometricObject interface
    virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
    virtual void drawLabel(sf::RenderWindow &window, const sf::View &worldView) const override;
    virtual void update() override;
    virtual void setColor(const sf::Color& color) override;
    virtual bool contains(const sf::Vector2f& screenPos, float tolerance) const override;
    virtual std::string getTypeString() const { return "Triangle"; }
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
    bool isWithinDistance(const sf::Vector2f& screenPos, float tolerance) const;
    void rotateCCW(const Point_2& center, double angleRadians);

    // Triangle-specific methods
    std::shared_ptr<Point> getVertexPoint(size_t index) const {
        if (index < m_vertices.size()) return m_vertices[index];
        return nullptr;
    }
    std::vector<Point_2> getVertices() const;
    std::vector<sf::Vector2f> getVerticesSFML() const;
    void setVertexPosition(size_t index, const Point_2& value);
    void setHoveredVertex(int index) { m_hoveredVertex = index; }
    void setActiveVertex(int index) { m_activeVertex = index; }
    int getHoveredVertex() const { return m_hoveredVertex; }
    int getActiveVertex() const { return m_activeVertex; }
    bool isValid() const override { return m_vertices.size() == 3; }

private:
    std::vector<std::shared_ptr<Point>> m_vertices;  // Exactly 3 vertices
    sf::Color m_color;                // Fill color
    sf::ConvexShape m_sfmlShape;      // SFML shape for rendering
    int m_hoveredVertex = -1;         // For handle coloring
    int m_activeVertex = -1;          // For handle coloring

    // Helper methods
    void updateSFMLShape();
    void updateSFMLShapeInternal();
    void drawVertexHandles(sf::RenderWindow &window, float scale) const;
};
