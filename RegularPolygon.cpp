#include "RegularPolygon.h"
#include "VertexLabelManager.h"
#include <cmath>
RegularPolygon::RegularPolygon(const Point_2 &center, const Point_2 &firstVertex, int numSides,
                               const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RegularPolygon, color, id),
      m_center(center),
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

void RegularPolygon::generateVertices() {
  m_vertices.clear();

  double centerX = CGAL::to_double(m_center.x());
  double centerY = CGAL::to_double(m_center.y());
  double angleStep = 2.0 * 3.14159265359 / m_numSides;

  for (int i = 0; i < m_numSides; ++i) {
    double angle = m_rotationAngle + i * angleStep;
    double x = centerX + m_radius * std::cos(angle);
    double y = centerY + m_radius * std::sin(angle);
    m_vertices.push_back(Point_2(FT(x), FT(y)));
  }
}

void RegularPolygon::updateSFMLShape() {
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

void RegularPolygon::draw(sf::RenderWindow &window) const {
  window.draw(m_sfmlShape);

  // Draw selection highlight if selected
  if (isSelected()) {
    sf::ConvexShape highlight = m_sfmlShape;
    highlight.setFillColor(sf::Color::Transparent);
    highlight.setOutlineThickness(3.0f);
    highlight.setOutlineColor(sf::Color::Yellow);
    window.draw(highlight);
  } else if (isHovered()) {
    sf::ConvexShape highlight = m_sfmlShape;
    highlight.setFillColor(sf::Color::Transparent);
    highlight.setOutlineThickness(2.0f);
    highlight.setOutlineColor(sf::Color::Cyan); // Cyan for hover
    window.draw(highlight);
  }

  drawVertexHandles(window);
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
  m_center = Point_2(m_center.x() + translation.x(), m_center.y() + translation.y());

  for (auto &v : m_vertices) {
    v = Point_2(v.x() + translation.x(), v.y() + translation.y());
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

  Vector_2 delta = value - m_center;
  double projection = CGAL::to_double(delta.x()) * dirX + CGAL::to_double(delta.y()) * dirY;

  const double minRadius = 1e-6;
  double newRadius = std::max(minRadius, projection);

  m_radius = newRadius;
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

void RegularPolygon::drawVertexHandles(sf::RenderWindow &window) const {
  const float handleRadius = 4.0f;
  
  // Draw center point (creation point 0)
  {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    float x = static_cast<float>(CGAL::to_double(m_center.x()));
    float y = static_cast<float>(CGAL::to_double(m_center.y()));
    handle.setPosition(x, y);
    
    sf::Color base = sf::Color(100, 100, 255);  // Blue for center
    if (m_activeVertex == 0) {
      base = sf::Color(255, 140, 0);  // Orange for active
    } else if (m_hoveredVertex == 0) {
      base = sf::Color::Yellow;
    }
    handle.setFillColor(base);
    handle.setOutlineThickness(1.0f);
    handle.setOutlineColor(sf::Color::Black);
    window.draw(handle);
    VertexLabelManager::instance().drawLabel(window, sf::Vector2f(x, y), "C");
  }
  
  // Draw first vertex (creation point 1, handles scaling)
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
    handle.setOutlineThickness(1.0f);
    handle.setOutlineColor(sf::Color::Black);
    window.draw(handle);
    VertexLabelManager::instance().drawLabel(window, sf::Vector2f(x, y), "V₁");
  }
  
  // Draw other vertices as non-draggable (smaller, no hover)
  for (size_t i = 1; i < m_vertices.size(); ++i) {
    sf::CircleShape handle(handleRadius * 0.6f);  // Smaller
    handle.setOrigin(handleRadius * 0.6f, handleRadius * 0.6f);
    handle.setPosition(static_cast<float>(CGAL::to_double(m_vertices[i].x())),
                       static_cast<float>(CGAL::to_double(m_vertices[i].y())));
    handle.setFillColor(sf::Color(150, 150, 150, 128));  // Semi-transparent
    handle.setOutlineThickness(0.5f);
    handle.setOutlineColor(sf::Color(100, 100, 100));
    window.draw(handle);
  }
}

void RegularPolygon::rotateCCW(const Point_2 &center, double angleRadians) {
  m_rotationAngle += angleRadians;

  double centerX = CGAL::to_double(center.x());
  double centerY = CGAL::to_double(center.y());
  double cos_a = std::cos(angleRadians);
  double sin_a = std::sin(angleRadians);

  // Rotate center
  {
    double x = CGAL::to_double(m_center.x()) - centerX;
    double y = CGAL::to_double(m_center.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    m_center = Point_2(FT(newX), FT(newY));
  }

  // Rotate vertices
  for (auto &v : m_vertices) {
    double x = CGAL::to_double(v.x()) - centerX;
    double y = CGAL::to_double(v.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    v = Point_2(FT(newX), FT(newY));
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
  Vector_2 translation = newPos - m_center;
  m_center = newPos;
  generateVertices();
  updateSFMLShape();
  updateHostedPoints();
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
  pts.emplace_back(static_cast<float>(CGAL::to_double(m_center.x())),
                   static_cast<float>(CGAL::to_double(m_center.y())));
  
  // Creation point 1: First vertex (defines radius)
  if (!m_vertices.empty()) {
    pts.emplace_back(static_cast<float>(CGAL::to_double(m_vertices[0].x())),
                     static_cast<float>(CGAL::to_double(m_vertices[0].y())));
  }
  
  return pts;
}

void RegularPolygon::setCreationPointPosition(size_t index, const Point_2& value) {
  if (index == 0) {
    // Translate entire shape by moving center
    Vector_2 delta = value - m_center;
    translate(delta);
  } else if (index == 1) {
    // Scale by moving first vertex (uses existing setVertexPosition which preserves regularity)
    setVertexPosition(0, value);
  }
}

std::vector<Point_2> RegularPolygon::getInteractableVertices() const {
  // Return center + all vertices for regular polygon
  std::vector<Point_2> result;
  result.reserve(m_vertices.size() + 1);
  result.push_back(m_center);
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