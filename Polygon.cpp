#include "Polygon.h"
#include "VertexLabelManager.h"
#include <cmath>


Polygon::Polygon(const std::vector<Point_2> &vertices, const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::Polygon, color, id), m_vertices(vertices) {
  sf::Color base = color;
  base.a = 0;
  m_color = base;
  updateSFMLShape();
}

void Polygon::addVertex(const Point_2 &vertex) {
  m_vertices.push_back(vertex);
  updateSFMLShape();
  updateHostedPoints();
}

void Polygon::removeLastVertex() {
  if (!m_vertices.empty()) {
    m_vertices.pop_back();
    updateSFMLShape();
    updateHostedPoints();
  }
}

void Polygon::updateSFMLShape() {
  if (m_vertices.size() < 3) return;

  updateSFMLShapeInternal();
}

void Polygon::updateSFMLShapeInternal() {
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

void Polygon::draw(sf::RenderWindow &window, float scale) const {
  if (m_vertices.size() >= 3) {
    sf::ConvexShape shape = m_sfmlShape;
    shape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);
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

void Polygon::setColor(const sf::Color &color) {
  m_color = color;
  if (m_vertices.size() >= 3) {
    m_sfmlShape.setFillColor(color);
  }
}

Point_2 Polygon::getCenter() const {
  if (m_vertices.empty()) return Point_2(FT(0), FT(0));

  double sumX = 0, sumY = 0;
  for (const auto &v : m_vertices) {
    sumX += CGAL::to_double(v.x());
    sumY += CGAL::to_double(v.y());
  }

  return Point_2(FT(sumX / m_vertices.size()), FT(sumY / m_vertices.size()));
}

bool Polygon::contains(const sf::Vector2f &screenPos, float tolerance) const {
  if (m_vertices.size() < 3) return false;
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += 2 * tolerance;
  bounds.height += 2 * tolerance;
  return bounds.contains(screenPos);
}

bool Polygon::isWithinDistance(const sf::Vector2f &screenPos, float tolerance) const {
  if (m_vertices.size() < 3) return false;

  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += tolerance * 2;
  bounds.height += tolerance * 2;
  return bounds.contains(screenPos);
}

void Polygon::translate(const Vector_2 &translation) {
  for (auto &v : m_vertices) {
    v = Point_2(v.x() + translation.x(), v.y() + translation.y());
  }
  updateSFMLShape();
  updateHostedPoints();
}

void Polygon::setVertexPosition(size_t index, const Point_2 &value) {
  if (index >= m_vertices.size()) return;
  m_vertices[index] = value;
  updateSFMLShape();
  updateHostedPoints();
}

std::vector<sf::Vector2f> Polygon::getVerticesSFML() const {
  std::vector<sf::Vector2f> verts;
  verts.reserve(m_vertices.size());
  for (const auto &v : m_vertices) {
    verts.emplace_back(static_cast<float>(CGAL::to_double(v.x())),
                       static_cast<float>(CGAL::to_double(v.y())));
  }
  return verts;
}

void Polygon::drawVertexHandles(sf::RenderWindow &window, float scale) const {
  const float handleRadius = 4.0f * scale;
  
  for (size_t i = 0; i < m_vertices.size(); ++i) {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    float x = static_cast<float>(CGAL::to_double(m_vertices[i].x()));
    float y = static_cast<float>(CGAL::to_double(m_vertices[i].y()));
    handle.setPosition(x, y);
    
    sf::Color base = sf::Color(180, 180, 180);
    if (static_cast<int>(i) == m_activeVertex) {
      base = sf::Color(255, 140, 0);  // Orange for active drag
    } else if (static_cast<int>(i) == m_hoveredVertex) {
      base = sf::Color::Yellow;
    }
    handle.setFillColor(base);
    handle.setOutlineThickness(1.0f * scale);
    handle.setOutlineColor(sf::Color::Black);
    window.draw(handle);
    
    // Draw vertex label (P1, P2, P3, ...)
    std::string label = "P" + std::to_string(i + 1);
    VertexLabelManager::instance().drawLabel(window, sf::Vector2f(x, y), label);
  }
}

void Polygon::rotateCCW(const Point_2 &center, double angleRadians) {
  double centerX = CGAL::to_double(center.x());
  double centerY = CGAL::to_double(center.y());
  double cos_a = std::cos(angleRadians);
  double sin_a = std::sin(angleRadians);

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
sf::FloatRect Polygon::getGlobalBounds() const {
  return m_sfmlShape.getGlobalBounds();
}

Point_2 Polygon::getCGALPosition() const {
  if (m_vertices.empty()) {
    return Point_2(FT(0), FT(0));
  }
  // Return centroid of polygon
  double sumX = 0, sumY = 0;
  for (const auto &v : m_vertices) {
    sumX += CGAL::to_double(v.x());
    sumY += CGAL::to_double(v.y());
  }
  return Point_2(FT(sumX / m_vertices.size()), FT(sumY / m_vertices.size()));
}

void Polygon::setCGALPosition(const Point_2 &newPos) {
  if (m_vertices.empty()) return;
  
  Point_2 oldCentroid = getCGALPosition();
  Vector_2 translation = newPos - oldCentroid;
  
  for (auto &v : m_vertices) {
    v = v + translation;
  }
  
  updateSFMLShape();
  updateHostedPoints();
}

void Polygon::setPosition(const sf::Vector2f &newSfmlPos) {
  // Get current position
  sf::Vector2f currentPos = m_sfmlShape.getPosition();
  
  // Calculate translation  
  sf::Vector2f translation = newSfmlPos - currentPos;
  
  // Translate the shape
  // FIX: Properly update vertices instead of just moving the SFML shape
  Vector_2 delta(translation.x, translation.y);
  translate(delta);
}

std::vector<Point_2> Polygon::getInteractableVertices() const {
  return m_vertices;
}

std::vector<Segment_2> Polygon::getEdges() const {
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