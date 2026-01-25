#include "Triangle.h"
#include "Point.h"
#include "VertexLabelManager.h"
#include <CGAL/Polygon_2.h>
#include <CGAL/enum.h>
#include <cmath>
#include <iostream>

Triangle::Triangle(const Point_2& v1, const Point_2& v2, const Point_2& v3,
                   const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::Triangle, color, id) {
    
    // Validate that points are non-collinear using CGAL
    if (CGAL::collinear(v1, v2, v3)) {
        std::cerr << "Warning: Triangle vertices are collinear!" << std::endl;
        // Still create the triangle but it will be degenerate
    }
    
    m_vertices.reserve(3);
    m_vertices.push_back(std::make_shared<Point>(v1, 1.0f));
    m_vertices.push_back(std::make_shared<Point>(v2, 1.0f));
    m_vertices.push_back(std::make_shared<Point>(v3, 1.0f));
    
    sf::Color base = color;
    base.a = 128;  // Semi-transparent fill
    m_color = base;
    
    updateSFMLShape();
}

Triangle::Triangle(const std::shared_ptr<Point>& v1, const std::shared_ptr<Point>& v2,
                   const std::shared_ptr<Point>& v3, const sf::Color& color,
                   unsigned int id)
    : GeometricObject(ObjectType::Triangle, color, id) {
    if (v1 && v2 && v3 &&
        CGAL::collinear(v1->getCGALPosition(), v2->getCGALPosition(), v3->getCGALPosition())) {
        std::cerr << "Warning: Triangle vertices are collinear!" << std::endl;
    }

    m_vertices.reserve(3);
    m_vertices.push_back(v1);
    m_vertices.push_back(v2);
    m_vertices.push_back(v3);

    sf::Color base = color;
    base.a = 128;
    m_color = base;

    updateSFMLShape();
}

void Triangle::updateSFMLShape() {
    if (m_vertices.size() != 3) return;
    updateSFMLShapeInternal();
}

void Triangle::updateSFMLShapeInternal() {
    m_sfmlShape.setPointCount(3);
    
    for (size_t i = 0; i < 3; ++i) {
        if (!m_vertices[i]) return;
        Point_2 pos = m_vertices[i]->getCGALPosition();
        double x = CGAL::to_double(pos.x());
        double y = CGAL::to_double(pos.y());
        m_sfmlShape.setPoint(i, sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
    }
    
    m_sfmlShape.setFillColor(m_color);
    m_sfmlShape.setOutlineThickness(1.5f);
    m_sfmlShape.setOutlineColor(sf::Color::Black);
}

void Triangle::draw(sf::RenderWindow& window, float scale, bool forceVisible) const {
    if ((!m_visible && !forceVisible) || !isValid()) return;

    if (m_vertices.size() == 3) {
        sf::ConvexShape shape = m_sfmlShape;
        shape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);

        // GHOST MODE: Apply transparency if hidden but forced visible
        if (!m_visible && forceVisible) {
            sf::Color ghostFill = shape.getFillColor();
            ghostFill.a = 50; // Faint alpha
            shape.setFillColor(ghostFill);
            
            sf::Color ghostOutline = shape.getOutlineColor();
            ghostOutline.a = 50;
            shape.setOutlineColor(ghostOutline);
        }

        window.draw(shape);
        
        // Draw selection highlight if selected
        if (isSelected()) {
            sf::ConvexShape highlight = m_sfmlShape;
            highlight.setFillColor(sf::Color::Transparent);
            highlight.setOutlineThickness(3.0f * scale);
            highlight.setOutlineColor(sf::Color::Yellow);
            window.draw(highlight);
        } else if (isHovered()) {
            sf::ConvexShape highlight = m_sfmlShape;
            highlight.setFillColor(sf::Color::Transparent);
            highlight.setOutlineThickness(2.0f * scale);
            highlight.setOutlineColor(sf::Color::Cyan); // Cyan for hover
            window.draw(highlight);
        }
        
        drawVertexHandles(window, scale);
    }
}

void Triangle::update() {
    updateSFMLShape();
    updateHostedPoints();
}

void Triangle::setColor(const sf::Color& color) {
    m_color = color;
    m_color.a = 128;  // Maintain semi-transparency
    if (m_vertices.size() == 3) {
        m_sfmlShape.setFillColor(m_color);
    }
}

Point_2 Triangle::getCenter() const {
    if (m_vertices.size() != 3) return Point_2(FT(0), FT(0));
    
    // Calculate centroid
    double sumX = 0, sumY = 0;
    for (const auto& v : m_vertices) {
        if (!v) continue;
        Point_2 pos = v->getCGALPosition();
        sumX += CGAL::to_double(pos.x());
        sumY += CGAL::to_double(pos.y());
    }
    
    return Point_2(FT(sumX / 3.0), FT(sumY / 3.0));
}

bool Triangle::contains(const sf::Vector2f& screenPos, float tolerance) const {
    if (m_vertices.size() != 3) return false;
    
    // Use CGAL's point-in-triangle test
    Point_2 queryPoint(FT(screenPos.x), FT(screenPos.y));
    
    // Create CGAL polygon from vertices
    CGAL::Polygon_2<Kernel> poly;
    for (const auto& v : m_vertices) {
        if (!v) return false;
        poly.push_back(v->getCGALPosition());
    }
    
    // Check if point is inside or on boundary
    CGAL::Bounded_side side = poly.bounded_side(queryPoint);
    if (side == CGAL::ON_BOUNDED_SIDE || side == CGAL::ON_BOUNDARY) {
        return true;
    }
    
    // Also check with tolerance for edges
    sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
    bounds.left -= tolerance;
    bounds.top -= tolerance;
    bounds.width += 2 * tolerance;
    bounds.height += 2 * tolerance;
    return bounds.contains(screenPos);
}

bool Triangle::isWithinDistance(const sf::Vector2f& screenPos, float tolerance) const {
    if (m_vertices.size() != 3) return false;
    
    sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
    bounds.left -= tolerance;
    bounds.top -= tolerance;
    bounds.width += tolerance * 2;
    bounds.height += tolerance * 2;
    return bounds.contains(screenPos);
}

void Triangle::translate(const Vector_2& translation) {
    for (auto& v : m_vertices) {
        if (!v) continue;
        Point_2 pos = v->getCGALPosition();
        v->setCGALPosition(Point_2(pos.x() + translation.x(), pos.y() + translation.y()));
    }
    updateSFMLShape();
    updateHostedPoints();
}

void Triangle::setVertexPosition(size_t index, const Point_2& value) {
    if (index >= 3) return;
    if (!m_vertices[index]) return;
    m_vertices[index]->setCGALPosition(value);
    
    // Check if vertices are still non-collinear
    if (m_vertices[0] && m_vertices[1] && m_vertices[2] &&
        CGAL::collinear(m_vertices[0]->getCGALPosition(), m_vertices[1]->getCGALPosition(),
                        m_vertices[2]->getCGALPosition())) {
        std::cerr << "Warning: Triangle vertices became collinear after vertex move!" << std::endl;
    }
    
    updateSFMLShape();
    updateHostedPoints();
}

std::vector<sf::Vector2f> Triangle::getVerticesSFML() const {
    std::vector<sf::Vector2f> verts;
    verts.reserve(3);
    for (const auto& v : m_vertices) {
        if (!v) continue;
        Point_2 pos = v->getCGALPosition();
        verts.emplace_back(static_cast<float>(CGAL::to_double(pos.x())),
                          static_cast<float>(CGAL::to_double(pos.y())));
    }
    return verts;
}

void Triangle::drawVertexHandles(sf::RenderWindow& window, float scale) const {
    const float handleRadius = 4.0f * scale;
    const char* labels[] = {"A", "B", "C"};
    
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        sf::CircleShape handle(handleRadius);
        handle.setOrigin(handleRadius, handleRadius);
        if (!m_vertices[i]) continue;
        Point_2 pos = m_vertices[i]->getCGALPosition();
        float x = static_cast<float>(CGAL::to_double(pos.x()));
        float y = static_cast<float>(CGAL::to_double(pos.y()));
        handle.setPosition(x, y);
        
        sf::Color base;
        if (static_cast<int>(i) == m_activeVertex) {
            base = sf::Color(255, 140, 0);  // Orange for active drag
        } else if (static_cast<int>(i) == m_hoveredVertex) {
            base = sf::Color::Yellow;
        } else {
            base = sf::Color(180, 180, 180);
        }
        
        handle.setFillColor(base);
        handle.setOutlineThickness(1.0f * scale);
        handle.setOutlineColor(sf::Color::Black);
        window.draw(handle);
        
        // Draw vertex label
        VertexLabelManager::instance().drawLabel(window, sf::Vector2f(x, y), labels[i]);
    }
}

void Triangle::rotateCCW(const Point_2& center, double angleRadians) {
    double centerX = CGAL::to_double(center.x());
    double centerY = CGAL::to_double(center.y());
    double cos_a = std::cos(angleRadians);
    double sin_a = std::sin(angleRadians);
    
    for (auto& v : m_vertices) {
        if (!v) continue;
        Point_2 pos = v->getCGALPosition();
        double x = CGAL::to_double(pos.x()) - centerX;
        double y = CGAL::to_double(pos.y()) - centerY;
        double newX = x * cos_a - y * sin_a + centerX;
        double newY = x * sin_a + y * cos_a + centerY;
        v->setCGALPosition(Point_2(FT(newX), FT(newY)));
    }
    
    updateSFMLShape();
    updateHostedPoints();
}

sf::FloatRect Triangle::getGlobalBounds() const {
    return m_sfmlShape.getGlobalBounds();
}

Point_2 Triangle::getCGALPosition() const {
    return getCenter();
}

void Triangle::setCGALPosition(const Point_2& newPos) {
    if (m_vertices.size() != 3) return;
    
    Point_2 oldCentroid = getCGALPosition();
    Vector_2 translation = newPos - oldCentroid;
    
    for (auto& v : m_vertices) {
        if (!v) continue;
        Point_2 pos = v->getCGALPosition();
        v->setCGALPosition(Point_2(pos.x() + translation.x(), pos.y() + translation.y()));
    }
    
    updateSFMLShape();
    updateHostedPoints();
}

void Triangle::setPosition(const sf::Vector2f& newSfmlPos) {
    Point_2 newCGALPos(FT(newSfmlPos.x), FT(newSfmlPos.y));
    setCGALPosition(newCGALPos);
}

std::vector<Point_2> Triangle::getInteractableVertices() const {
    return getVertices();
}

std::vector<Segment_2> Triangle::getEdges() const {
    std::vector<Segment_2> edges;
    edges.reserve(3);
    
    auto verts = getVertices();
    if (verts.size() == 3) {
        edges.emplace_back(verts[0], verts[1]);
        edges.emplace_back(verts[1], verts[2]);
        edges.emplace_back(verts[2], verts[0]);
    }
    
    return edges;
}

std::vector<Point_2> Triangle::getVertices() const {
    std::vector<Point_2> verts;
    verts.reserve(m_vertices.size());
    for (const auto& v : m_vertices) {
        if (!v) continue;
        verts.push_back(v->getCGALPosition());
    }
    return verts;
}
