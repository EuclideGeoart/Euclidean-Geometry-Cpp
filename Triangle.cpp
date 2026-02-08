#include "Triangle.h"
// Force recompile 

#include "Point.h"
#include "Line.h"
#include "PointUtils.h"
#include "Circle.h"
#include "Types.h"
#include <CGAL/Polygon_2.h>
#include <CGAL/enum.h>
#include <cmath>
#include <iostream>

Triangle::Triangle(const Point_2& p1, const Point_2& p2, const Point_2& p3, const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::Triangle, color, id) {
    m_color.a = 0; // Default transparent fill
    
    // Validate that points are non-collinear using CGAL
    if (CGAL::collinear(p1, p2, p3)) {
        std::cerr << "Warning: Triangle vertices are collinear!" << std::endl;
        // Still create the triangle but it will be degenerate
    }
    
    m_vertices.reserve(3);
    m_vertices.push_back(std::make_shared<Point>(p1, 1.0f));
    m_vertices.push_back(std::make_shared<Point>(p2, 1.0f));
    m_vertices.push_back(std::make_shared<Point>(p3, 1.0f));
    
    for (size_t i = 0; i < 3; ++i) {
        m_vertices[i]->setCreatedWithShape(true);
    }
    
    updateSFMLShape();
    setShowLabel(true);
}

Triangle::Triangle(const std::shared_ptr<Point>& p1, const std::shared_ptr<Point>& p2,
                   const std::shared_ptr<Point>& p3, const sf::Color& color,
                   unsigned int id)
    : GeometricObject(ObjectType::Triangle, color, id) {
    m_color.a = 0; // Default transparent fill
    if (p1 && p2 && p3 &&
        CGAL::collinear(p1->getCGALPosition(), p2->getCGALPosition(), p3->getCGALPosition())) {
        std::cerr << "Warning: Triangle vertices are collinear!" << std::endl;
    }

    m_vertices.reserve(3);
    m_vertices.push_back(p1);
    m_vertices.push_back(p2);
    m_vertices.push_back(p3);

    for (size_t i = 0; i < 3; ++i) {
        if (m_vertices[i]) {
            m_vertices[i]->addDependent(m_selfHandle);
        }
    }

    updateSFMLShape();
    setShowLabel(true);
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
    m_sfmlShape.setOutlineThickness(m_thickness);
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
            highlight.setOutlineColor(Constants::SELECTION_COLOR);
            window.draw(highlight);
        } else if (isHovered()) {
            sf::ConvexShape highlight = m_sfmlShape;
            highlight.setFillColor(sf::Color::Transparent);
            highlight.setOutlineThickness(1.0f * scale);
            highlight.setOutlineColor(sf::Color(0, 255, 255, 100)); // Dim Cyan
            
            if (getHoveredEdge() == -1) {
                highlight.setOutlineThickness(2.0f * scale);
                highlight.setOutlineColor(sf::Color::Cyan); // Full Cyan
            }
            window.draw(highlight);
            
            if (getHoveredEdge() >= 0) {
                 int idx = getHoveredEdge();
                 if (idx >= 0 && idx < 3) {
                     sf::Vector2f p1 = m_sfmlShape.getPoint(idx);
                     sf::Vector2f p2 = m_sfmlShape.getPoint((idx + 1) % 3);
                     
                     // Helper math for thick line
                     sf::Vector2f dir = p2 - p1;
                     float len = std::sqrt(dir.x*dir.x + dir.y*dir.y);
                     if (len > 0.1f) {
                         dir /= len;
                         sf::Vector2f perp(-dir.y, dir.x);
                         float thickness = 4.0f * scale; 
                         
                         sf::ConvexShape thickLine;
                         thickLine.setPointCount(4);
                         thickLine.setPoint(0, p1 + perp * thickness * 0.5f);
                         thickLine.setPoint(1, p2 + perp * thickness * 0.5f);
                         thickLine.setPoint(2, p2 - perp * thickness * 0.5f);
                         thickLine.setPoint(3, p1 - perp * thickness * 0.5f);
                         thickLine.setFillColor(sf::Color(255, 100, 50, 200)); 
                         window.draw(thickLine);
                     }
                 }
            }
        }
        
        // Delegate drawing to the constituent points
        for (auto& pt : m_vertices) {
            if (pt) pt->draw(window, scale, forceVisible);
        }

        drawVertexHandles(window, scale);
    }
}

void Triangle::drawLabel(sf::RenderWindow& window, const sf::View& worldView) const {
  if (!isVisible() || getLabelMode() == LabelMode::Hidden || !Point::commonFont) return;


  // 2. Draw triangle's own label (e.g., name or area) at center
  std::string labelStr = "";
  switch (getLabelMode()) {
    case LabelMode::Name: labelStr = getLabel(); break;
    case LabelMode::Value: {
       std::vector<Point_2> verts = getVertices();
       if (verts.size() == 3) {
         double area = std::abs(CGAL::to_double(CGAL::area(verts[0], verts[1], verts[2])));
         labelStr = std::to_string(static_cast<int>(std::round(area)));
       }
       break;
    }
    case LabelMode::NameAndValue: {
       labelStr = getLabel();
       std::vector<Point_2> verts = getVertices();
       if (verts.size() == 3) {
         double area = std::abs(CGAL::to_double(CGAL::area(verts[0], verts[1], verts[2])));
         labelStr += (labelStr.empty() ? "" : " = ") + std::to_string(static_cast<int>(std::round(area)));
       }
       break;
    }
    case LabelMode::Caption: labelStr = getCaption(); break;
    default: break;
  }

  if (labelStr.empty()) return;

  Point_2 center = getCenter();
  sf::Vector2i screenPos = window.mapCoordsToPixel(Point::cgalToSFML(center), worldView);
  
  sf::Text text;
  text.setFont(*Point::commonFont);
  text.setString(sf::String::fromUtf8(labelStr.begin(), labelStr.end()));
  text.setCharacterSize(LabelManager::instance().getFontSize());
  
  sf::Color textColor = m_color;
  if (textColor.r > 200 && textColor.g > 200 && textColor.b > 200) textColor = sf::Color::Black;
  text.setFillColor(textColor);

  text.setPosition(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
  sf::FloatRect bounds = text.getLocalBounds();
  text.setOrigin(bounds.width / 2.0f, bounds.height / 2.0f);

  window.draw(text);
}

void Triangle::update() {
    if (isDependent()) {
        updateDependentShape();
    } else {
        updateSFMLShape();
        updateHostedPoints();
    }
}

void Triangle::updateDependentShape() {
    auto parent = m_parentSource.lock();
    if (!parent || !parent->isValid()) {
        updateSFMLShape();
        updateHostedPoints();
        return;
    }

    auto sourceTri = std::dynamic_pointer_cast<Triangle>(parent);
    if (!sourceTri || !sourceTri->isValid()) {
        updateSFMLShape();
        updateHostedPoints();
        return;
    }

    auto aux = m_auxObject.lock();

    auto reflectAcrossLine = [](const Point_2& p, const std::shared_ptr<Line>& line) -> Point_2 {
        Point_2 a = line->getStartPoint();
        Point_2 b = line->getEndPoint();

        Vector_2 ab = b - a;
        double abLenSq = CGAL::to_double(ab.squared_length());
        if (abLenSq < 1e-12) {
            return p;
        }

        Vector_2 ap = p - a;
        FT t = (ap * ab) / ab.squared_length();
        Point_2 h = a + ab * t;
        return p + (h - p) * FT(2.0);
    };

    auto transformPoint = [&](const Point_2& p) -> std::optional<Point_2> {
        switch (m_transformType) {
            case TransformationType::Reflect: {
                auto line = std::dynamic_pointer_cast<Line>(aux);
                if (!line || !line->isValid()) return std::nullopt;
                return reflectAcrossLine(p, line);
            }
            case TransformationType::ReflectPoint: {
                auto center = std::dynamic_pointer_cast<Point>(aux);
                if (!center || !center->isValid()) return std::nullopt;
                Point_2 c = center->getCGALPosition();
                return c + (c - p);
            }
            case TransformationType::ReflectCircle: {
                auto circle = std::dynamic_pointer_cast<Circle>(aux);
                if (!circle || !circle->isValid()) return std::nullopt;
                Point_2 o = circle->getCenterPoint();
                double r = circle->getRadius();
                Vector_2 op = p - o;
                double opLenSq = CGAL::to_double(op.squared_length());
                if (opLenSq < 1e-12) return std::nullopt;
                double scale = (r * r) / opLenSq;
                return o + op * FT(scale);
            }
            case TransformationType::Translate: {
                Vector_2 delta = m_translationVector;
                auto line = std::dynamic_pointer_cast<Line>(aux);
                if (line && line->isValid()) {
                    delta = line->getEndPoint() - line->getStartPoint();
                }
                return p + delta;
            }
            case TransformationType::Rotate: {
                auto center = std::dynamic_pointer_cast<Point>(aux);
                if (!center || !center->isValid()) return std::nullopt;
                Point_2 c = center->getCGALPosition();
                double rad = m_transformValue * 3.14159265358979323846 / 180.0;
                double s = std::sin(rad);
                double c_val = std::cos(rad);
                double dx = CGAL::to_double(p.x() - c.x());
                double dy = CGAL::to_double(p.y() - c.y());
                return Point_2(c.x() + FT(dx * c_val - dy * s),
                               c.y() + FT(dx * s + dy * c_val));
            }
            case TransformationType::Dilate: {
                auto center = std::dynamic_pointer_cast<Point>(aux);
                if (!center || !center->isValid()) return std::nullopt;
                Point_2 c = center->getCGALPosition();
                double scale = m_transformValue;
                return c + (p - c) * FT(scale);
            }
            default:
                return p;
        }
    };

    auto sourceVerts = sourceTri->getVertices();
    if (sourceVerts.size() != 3) {
        setVisible(false);
        return;
    }

    for (auto& v : m_vertices) {
        if (v) v->setDeferConstraintUpdates(true);
    }

    for (size_t i = 0; i < 3; ++i) {
        auto tp = transformPoint(sourceVerts[i]);
        if (!tp.has_value()) {
            setVisible(false);
            return;
        }
        if (i < m_vertices.size() && m_vertices[i]) {
            m_vertices[i]->setCGALPosition(flattenPoint(*tp));
        }
    }

    for (auto& v : m_vertices) {
        if (v) v->forceConstraintUpdate();
    }

    if (!hasVisibilityUserOverride()) {
        setVisible(true);
    }
    updateSFMLShape();
    updateHostedPoints();
}

void Triangle::setColor(const sf::Color& color) {
    m_color = color;
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
        v->setCGALPosition(flattenPoint(Point_2(pos.x() + translation.x(), pos.y() + translation.y())));
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
    const float handleRadius = m_vertexHandleSize * scale;
    
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
        v->setCGALPosition(flattenPoint(Point_2(pos.x() + translation.x(), pos.y() + translation.y())));
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
