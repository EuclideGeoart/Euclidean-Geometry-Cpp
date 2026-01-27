#include "RegularPolygon.h"
#include "Point.h"
#include "VertexLabelManager.h"
#include <cmath>
RegularPolygon::RegularPolygon(const Point_2 &center, const Point_2 &firstVertex, int numSides,
                               const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RegularPolygon, color, id),
  m_centerPoint(std::make_shared<Point>(center, 1.0f)),
  m_firstVertexPoint(std::make_shared<Point>(firstVertex, 1.0f)),
      m_numSides(numSides),
      m_rotationAngle(0),
      m_color(color) {
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
      m_rotationAngle(0),
      m_color(color) {
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
  m_sfmlShape.setOutlineThickness(1.5f);
  m_sfmlShape.setOutlineColor(sf::Color::Black);
}

void RegularPolygon::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!m_visible && !forceVisible) return;

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

void RegularPolygon::update() {
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
  const float handleRadius = 4.0f * scale;
  
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
  } else if (index == 1) {
    // Scale by moving first vertex (uses existing setVertexPosition which preserves regularity)
    setVertexPosition(0, value);
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