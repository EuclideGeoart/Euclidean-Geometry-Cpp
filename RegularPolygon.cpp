#include "RegularPolygon.h"
#include "Point.h"
#include "Line.h"
#include "Circle.h"
#include "VertexLabelManager.h"
#include <cmath>
RegularPolygon::RegularPolygon(const Point_2 &center, const Point_2 &firstVertex, int numSides,
                               const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RegularPolygon, color, id),
  m_centerPoint(std::make_shared<Point>(center, 1.0f)),
  m_firstVertexPoint(std::make_shared<Point>(firstVertex, 1.0f)),
      m_numSides(numSides),
      m_rotationAngle(0) {
  m_color.a = 0;  // Default to transparent fill
  if (m_numSides < 3) m_numSides = 3;

  // Calculate radius from center to first vertex
  double dx = CGAL::to_double(firstVertex.x()) - CGAL::to_double(center.x());
  double dy = CGAL::to_double(firstVertex.y()) - CGAL::to_double(center.y());
  m_radius = std::sqrt(dx * dx + dy * dy);

  // Calculate initial rotation angle
  m_rotationAngle = std::atan2(dy, dx);

  generateVertices();
  updateSFMLShape();
}

RegularPolygon::RegularPolygon(const std::shared_ptr<Point> &center,
                               const std::shared_ptr<Point> &firstVertex, int numSides,
                               const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RegularPolygon, color, id),
      m_centerPoint(center ? center : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_firstVertexPoint(firstVertex ? firstVertex
                                     : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_numSides(numSides),
      m_rotationAngle(0) {
  m_color.a = 0;
  if (m_numSides < 3) m_numSides = 3;

  Point_2 centerPos = m_centerPoint->getCGALPosition();
  Point_2 firstPos = m_firstVertexPoint->getCGALPosition();
  double dx = CGAL::to_double(firstPos.x()) - CGAL::to_double(centerPos.x());
  double dy = CGAL::to_double(firstPos.y()) - CGAL::to_double(centerPos.y());
  m_radius = std::sqrt(dx * dx + dy * dy);
  m_rotationAngle = std::atan2(dy, dx);

  generateVertices();
  updateSFMLShape();
}

void RegularPolygon::generateVertices() {
  m_vertices.clear();

  Point_2 centerPos = getCenter();
  if (m_centerPoint && m_firstVertexPoint) {
    Point_2 firstPos = m_firstVertexPoint->getCGALPosition();
    double dx = CGAL::to_double(firstPos.x()) - CGAL::to_double(centerPos.x());
    double dy = CGAL::to_double(firstPos.y()) - CGAL::to_double(centerPos.y());
    m_radius = std::sqrt(dx * dx + dy * dy);
    m_rotationAngle = std::atan2(dy, dx);
  }

  double centerX = CGAL::to_double(centerPos.x());
  double centerY = CGAL::to_double(centerPos.y());
  double angleStep = 2.0 * 3.14159265359 / m_numSides;

  for (int i = 0; i < m_numSides; ++i) {
    double angle = m_rotationAngle + i * angleStep;
    double x = centerX + m_radius * std::cos(angle);
    double y = centerY + m_radius * std::sin(angle);
    m_vertices.push_back(Point_2(FT(x), FT(y)));
  }
}

void RegularPolygon::updateSFMLShape() {
  generateVertices();
  updateSFMLShapeInternal();
}

void RegularPolygon::updateSFMLShapeInternal() {
  m_sfmlShape.setPointCount(m_vertices.size());

  for (size_t i = 0; i < m_vertices.size(); ++i) {
    double x = CGAL::to_double(m_vertices[i].x());
    double y = CGAL::to_double(m_vertices[i].y());
    m_sfmlShape.setPoint(i, sf::Vector2f(x, y));
  }

  m_sfmlShape.setFillColor(m_color);
  m_sfmlShape.setOutlineThickness(m_thickness);
  m_sfmlShape.setOutlineColor(sf::Color::Black);
}

void RegularPolygon::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!isVisible() && !forceVisible) return;

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
         size_t count = m_sfmlShape.getPointCount();
         int idx = getHoveredEdge();
         if (idx >= 0 && idx < static_cast<int>(count)) {
             sf::Vector2f p1 = m_sfmlShape.getPoint(idx);
             sf::Vector2f p2 = m_sfmlShape.getPoint((idx + 1) % count);
             
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

  drawVertexHandles(window, scale);
}

void RegularPolygon::update() {
  if (isDependent()) {
    updateDependentShape();
  } else {
    updateSFMLShape();
    updateHostedPoints();
  }
}

void RegularPolygon::updateDependentShape() {
  auto parent = m_parentSource.lock();
  if (!parent || !parent->isValid()) {
    updateSFMLShape();
    updateHostedPoints();
    return;
  }

  auto sourcePoly = std::dynamic_pointer_cast<RegularPolygon>(parent);
  if (!sourcePoly || !sourcePoly->isValid()) {
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

  auto srcCenter = sourcePoly->getCenter();
  auto srcFirst = sourcePoly->getFirstVertexPoint();
  if (!srcFirst) {
    setVisible(false);
    return;
  }

  auto newCenter = transformPoint(srcCenter);
  auto newFirst = transformPoint(srcFirst->getCGALPosition());
  if (!newCenter.has_value() || !newFirst.has_value()) {
    setVisible(false);
    return;
  }

  if (m_centerPoint) m_centerPoint->setCGALPosition(*newCenter);
  if (m_firstVertexPoint) m_firstVertexPoint->setCGALPosition(*newFirst);

  setVisible(true);
  updateSFMLShape();
  updateHostedPoints();
}

void RegularPolygon::setColor(const sf::Color &color) {
  m_color = color;
  m_sfmlShape.setFillColor(color);
}

bool RegularPolygon::contains(const sf::Vector2f &screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += 2 * tolerance;
  bounds.height += 2 * tolerance;
  return bounds.contains(screenPos);
}

bool RegularPolygon::isWithinDistance(const sf::Vector2f &screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += tolerance * 2;
  bounds.height += tolerance * 2;
  return bounds.contains(screenPos);
}

void RegularPolygon::translate(const Vector_2 &translation) {
  if (m_centerPoint) {
    Point_2 centerPos = m_centerPoint->getCGALPosition();
    m_centerPoint->setCGALPosition(
        Point_2(centerPos.x() + translation.x(), centerPos.y() + translation.y()));
  }
  if (m_firstVertexPoint) {
    Point_2 firstPos = m_firstVertexPoint->getCGALPosition();
    m_firstVertexPoint->setCGALPosition(
        Point_2(firstPos.x() + translation.x(), firstPos.y() + translation.y()));
  }

  updateSFMLShape();
  updateHostedPoints();
}

void RegularPolygon::setVertexPosition(size_t index, const Point_2 &value) {
  if (index >= m_vertices.size()) return;

  // Constrain scaling to the reference (first) vertex direction and keep orientation fixed
  if (index != 0) {
    return;  // Only the first vertex drives radius adjustments
  }

  double dirX = std::cos(m_rotationAngle);
  double dirY = std::sin(m_rotationAngle);

  Vector_2 delta = value - getCenter();
  double projection = CGAL::to_double(delta.x()) * dirX + CGAL::to_double(delta.y()) * dirY;

  const double minRadius = 1e-6;
  double newRadius = std::max(minRadius, projection);

  m_radius = newRadius;

  if (m_firstVertexPoint) {
    Point_2 centerPos = getCenter();
    Point_2 newFirst(FT(CGAL::to_double(centerPos.x()) + dirX * m_radius),
                     FT(CGAL::to_double(centerPos.y()) + dirY * m_radius));
    m_firstVertexPoint->setCGALPosition(newFirst);
  }

  generateVertices();
  updateSFMLShape();
  updateHostedPoints();
}

std::vector<sf::Vector2f> RegularPolygon::getVerticesSFML() const {
  std::vector<sf::Vector2f> verts;
  verts.reserve(m_vertices.size());
  for (const auto &v : m_vertices) {
    verts.emplace_back(static_cast<float>(CGAL::to_double(v.x())),
                       static_cast<float>(CGAL::to_double(v.y())));
  }
  return verts;
}

void RegularPolygon::drawVertexHandles(sf::RenderWindow &window, float scale) const {
  const float handleRadius = m_vertexHandleSize * scale;
  
  // Draw center point (creation point 0)
  {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    Point_2 centerPos = getCenter();
    float x = static_cast<float>(CGAL::to_double(centerPos.x()));
    float y = static_cast<float>(CGAL::to_double(centerPos.y()));
    handle.setPosition(x, y);
    
    sf::Color base = sf::Color(100, 100, 255);  // Blue for center
    if (m_activeVertex == 0) {
      base = sf::Color(255, 140, 0);  // Orange for active
    } else if (m_hoveredVertex == 0) {
      base = sf::Color::Yellow;
    }
    handle.setFillColor(base);
    handle.setOutlineThickness(1.0f * scale);
    handle.setOutlineColor(sf::Color::Black);
    window.draw(handle);
  }
  
  // Draw first vertex
  if (!m_vertices.empty()) {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    float x = static_cast<float>(CGAL::to_double(m_vertices[0].x()));
    float y = static_cast<float>(CGAL::to_double(m_vertices[0].y()));
    handle.setPosition(x, y);
    
    sf::Color base = sf::Color(180, 180, 180);
    if (m_activeVertex == 1) {
      base = sf::Color(255, 140, 0);
    } else if (m_hoveredVertex == 1) {
      base = sf::Color::Yellow;
    }
    handle.setFillColor(base);
    handle.setOutlineThickness(1.0f * scale);
    handle.setOutlineColor(sf::Color::Black);
    window.draw(handle);
  }
  
  // Draw other vertices
  for (size_t i = 1; i < m_vertices.size(); ++i) {
    sf::CircleShape handle(handleRadius * 0.6f);  // Smaller
    handle.setOrigin(handleRadius * 0.6f, handleRadius * 0.6f);
    handle.setPosition(static_cast<float>(CGAL::to_double(m_vertices[i].x())),
                       static_cast<float>(CGAL::to_double(m_vertices[i].y())));
    handle.setFillColor(sf::Color(150, 150, 150, 128));  // Semi-transparent
    handle.setOutlineThickness(0.5f * scale);
    handle.setOutlineColor(sf::Color(100, 100, 100));
    window.draw(handle);
  }
}

void RegularPolygon::rotateCCW(const Point_2 &center, double angleRadians) {
  double centerX = CGAL::to_double(center.x());
  double centerY = CGAL::to_double(center.y());
  double cos_a = std::cos(angleRadians);
  double sin_a = std::sin(angleRadians);

  // Rotate center
  if (m_centerPoint) {
    Point_2 centerPos = m_centerPoint->getCGALPosition();
    double x = CGAL::to_double(centerPos.x()) - centerX;
    double y = CGAL::to_double(centerPos.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    m_centerPoint->setCGALPosition(Point_2(FT(newX), FT(newY)));
  }

  if (m_firstVertexPoint) {
    Point_2 firstPos = m_firstVertexPoint->getCGALPosition();
    double x = CGAL::to_double(firstPos.x()) - centerX;
    double y = CGAL::to_double(firstPos.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    m_firstVertexPoint->setCGALPosition(Point_2(FT(newX), FT(newY)));
  }

  updateSFMLShape();
  updateHostedPoints();
}

void RegularPolygon::setNumSides(int numSides) {
  if (numSides >= 3) {
    m_numSides = numSides;
    generateVertices();
    updateSFMLShape();
    updateHostedPoints();
  }
}

void RegularPolygon::setRadius(double radius) {
  if (radius > 0) {
    m_radius = radius;
    generateVertices();
    updateSFMLShape();
    updateHostedPoints();
  }
}
sf::FloatRect RegularPolygon::getGlobalBounds() const {
  return m_sfmlShape.getGlobalBounds();
}

void RegularPolygon::setCGALPosition(const Point_2 &newPos) {
  Vector_2 translation = newPos - getCenter();
  translate(translation);
}

void RegularPolygon::setPosition(const sf::Vector2f &newSfmlPos) {
  // Get current position
  sf::Vector2f currentPos = m_sfmlShape.getPosition();
  
  // Calculate translation
  sf::Vector2f translation = newSfmlPos - currentPos;
  
  // Translate the shape
  // FIX: Properly update logic instead of just moving SFML shape
  Vector_2 delta(translation.x, translation.y);
  translate(delta);
}

std::vector<sf::Vector2f> RegularPolygon::getCreationPointsSFML() const {
  std::vector<sf::Vector2f> pts;
  pts.reserve(2);
  
  // Creation point 0: Center
  Point_2 centerPos = getCenter();
  pts.emplace_back(static_cast<float>(CGAL::to_double(centerPos.x())),
                   static_cast<float>(CGAL::to_double(centerPos.y())));
  
  // Creation point 1: First vertex (defines radius)
  if (m_firstVertexPoint) {
    Point_2 firstPos = m_firstVertexPoint->getCGALPosition();
    pts.emplace_back(static_cast<float>(CGAL::to_double(firstPos.x())),
                     static_cast<float>(CGAL::to_double(firstPos.y())));
  }
  
  return pts;
}

void RegularPolygon::setCreationPointPosition(size_t index, const Point_2& value) {
  if (index == 0) {
    // Translate entire shape by moving center
    Vector_2 delta = value - getCenter();
    translate(delta);
  } else if (index >= 1) {
    Point_2 centerPos = getCenter();
    double dx = CGAL::to_double(value.x() - centerPos.x());
    double dy = CGAL::to_double(value.y() - centerPos.y());
    double newRadius = std::sqrt(dx * dx + dy * dy);
    const double minRadius = 1e-6;
    if (newRadius < minRadius) newRadius = minRadius;

    m_rotationAngle = std::atan2(dy, dx);
    m_radius = newRadius;

    if (m_firstVertexPoint) {
      m_firstVertexPoint->setCGALPosition(value);
    }

    generateVertices();
    updateSFMLShape();
    updateHostedPoints();
  }
}

Point_2 RegularPolygon::getCenter() const {
  if (m_centerPoint) {
    return m_centerPoint->getCGALPosition();
  }
  return Point_2(FT(0), FT(0));
}

Point_2 RegularPolygon::getCGALPosition() const {
  return getCenter();
}

std::vector<Point_2> RegularPolygon::getInteractableVertices() const {
  // Return center + all vertices for regular polygon
  std::vector<Point_2> result;
  result.reserve(m_vertices.size() + 1);
  result.push_back(getCenter());
  result.insert(result.end(), m_vertices.begin(), m_vertices.end());
  return result;
}

std::vector<Segment_2> RegularPolygon::getEdges() const {
  std::vector<Segment_2> edges;
  size_t n = m_vertices.size();
  
  if (n >= 3) {
    edges.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      size_t next = (i + 1) % n;
      edges.emplace_back(m_vertices[i], m_vertices[next]);
    }
  }
  
  return edges;
}