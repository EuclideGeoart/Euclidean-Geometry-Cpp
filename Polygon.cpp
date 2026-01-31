#include "Polygon.h"
#include "Point.h"
#include "VertexLabelManager.h"
#include <cmath>


Polygon::Polygon(const std::vector<Point_2> &vertices, const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::Polygon, color, id) {
  m_vertices.reserve(vertices.size());
  for (const auto &v : vertices) {
    m_vertices.push_back(std::make_shared<Point>(v, 1.0f));
  }
  sf::Color base = color;
  base.a = 0;
  m_color = base;
  updateSFMLShape();
}

Polygon::Polygon(const std::vector<std::shared_ptr<Point>> &vertices, const sf::Color &color,
                 unsigned int id)
    : GeometricObject(ObjectType::Polygon, color, id), m_vertices(vertices) {
  sf::Color base = color;
  base.a = 0;
  m_color = base;
  updateSFMLShape();
}

void Polygon::addVertex(const Point_2 &vertex) {
  m_vertices.push_back(std::make_shared<Point>(vertex, 1.0f));
  updateSFMLShape();
  updateHostedPoints();
}

void Polygon::addVertex(const std::shared_ptr<Point> &vertex) {
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
    if (!m_vertices[i]) return;
    Point_2 pos = m_vertices[i]->getCGALPosition();
    double x = CGAL::to_double(pos.x());
    double y = CGAL::to_double(pos.y());
    m_sfmlShape.setPoint(i, sf::Vector2f(x, y));
  }

  m_sfmlShape.setFillColor(m_color);
  m_sfmlShape.setOutlineThickness(m_thickness);
  m_sfmlShape.setOutlineColor(sf::Color::Black);
}

void Polygon::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if ((!m_visible && !forceVisible) || m_vertices.size() < 3) return;

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

void Polygon::update() {
  updateSFMLShape();
  updateHostedPoints();
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
    if (!v) continue;
    Point_2 pos = v->getCGALPosition();
    sumX += CGAL::to_double(pos.x());
    sumY += CGAL::to_double(pos.y());
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
    if (!v) continue;
    Point_2 pos = v->getCGALPosition();
    v->setCGALPosition(Point_2(pos.x() + translation.x(), pos.y() + translation.y()));
  }
  updateSFMLShape();
  updateHostedPoints();
}

void Polygon::setVertexPosition(size_t index, const Point_2 &value) {
  if (index >= m_vertices.size()) return;
  if (!m_vertices[index]) return;
  m_vertices[index]->setCGALPosition(value);
  updateSFMLShape();
  updateHostedPoints();
}

std::vector<sf::Vector2f> Polygon::getVerticesSFML() const {
  std::vector<sf::Vector2f> verts;
  verts.reserve(m_vertices.size());
  for (const auto &v : m_vertices) {
    if (!v) continue;
    Point_2 pos = v->getCGALPosition();
    verts.emplace_back(static_cast<float>(CGAL::to_double(pos.x())),
                       static_cast<float>(CGAL::to_double(pos.y())));
  }
  return verts;
}

void Polygon::drawVertexHandles(sf::RenderWindow &window, float scale) const {
  const float handleRadius = m_vertexHandleSize * scale;
  
  for (size_t i = 0; i < m_vertices.size(); ++i) {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    if (!m_vertices[i]) continue;
    Point_2 pos = m_vertices[i]->getCGALPosition();
    float x = static_cast<float>(CGAL::to_double(pos.x()));
    float y = static_cast<float>(CGAL::to_double(pos.y()));
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
  }
}

void Polygon::rotateCCW(const Point_2 &center, double angleRadians) {
  double centerX = CGAL::to_double(center.x());
  double centerY = CGAL::to_double(center.y());
  double cos_a = std::cos(angleRadians);
  double sin_a = std::sin(angleRadians);

  for (auto &v : m_vertices) {
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
    if (!v) continue;
    Point_2 pos = v->getCGALPosition();
    sumX += CGAL::to_double(pos.x());
    sumY += CGAL::to_double(pos.y());
  }
  return Point_2(FT(sumX / m_vertices.size()), FT(sumY / m_vertices.size()));
}

void Polygon::setCGALPosition(const Point_2 &newPos) {
  if (m_vertices.empty()) return;
  
  Point_2 oldCentroid = getCGALPosition();
  Vector_2 translation = newPos - oldCentroid;
  
  for (auto &v : m_vertices) {
    if (!v) continue;
    Point_2 pos = v->getCGALPosition();
    v->setCGALPosition(Point_2(pos.x() + translation.x(), pos.y() + translation.y()));
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
  return getVertices();
}

std::vector<Segment_2> Polygon::getEdges() const {
  std::vector<Segment_2> edges;
  auto verts = getVertices();
  size_t n = verts.size();
  
  if (n >= 3) {
    edges.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      size_t next = (i + 1) % n;
      edges.emplace_back(verts[i], verts[next]);
    }
  }
  
  return edges;
}

std::vector<Point_2> Polygon::getVertices() const {
  std::vector<Point_2> verts;
  verts.reserve(m_vertices.size());
  for (const auto &v : m_vertices) {
    if (!v) continue;
    verts.push_back(v->getCGALPosition());
  }
  return verts;
}