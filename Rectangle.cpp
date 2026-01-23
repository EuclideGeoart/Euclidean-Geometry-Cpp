#include "Rectangle.h"
#include "VertexLabelManager.h"
#include <cmath>
#include <iostream>

Rectangle::Rectangle(const Point_2 &corner1, const Point_2 &corner2, bool isRotatable,
                     const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
      m_corner1(corner1),
      m_corner2(corner2),
      m_isRotatable(isRotatable),
      m_width(0),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)),
      m_color(color) {
  m_color.a = 0;
  updateDimensionsFromCorners();
  updateSFMLShape();
}

Rectangle::Rectangle(const Point_2 &corner, const Point_2 &adjacentPoint, double width,
                     const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RectangleRotatable, color, id),
      m_corner1(corner),
      m_corner2(corner),
      m_isRotatable(true),
      m_width(width),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)),
      m_color(color) {
  m_color.a = 0;
  // Calculate rotation angle from adjacentPoint
  double dx = CGAL::to_double(adjacentPoint.x()) - CGAL::to_double(corner.x());
  double dy = CGAL::to_double(adjacentPoint.y()) - CGAL::to_double(corner.y());
  m_rotationAngle = std::atan2(dy, dx);

  // Calculate height based on adjacent point
  m_height = std::sqrt(dx * dx + dy * dy);

  // Center is halfway along both local axes from the original corner
  double cos_a = std::cos(m_rotationAngle);
  double sin_a = std::sin(m_rotationAngle);
  double localX = m_width * 0.5;
  double localY = m_height * 0.5;
  double cx = CGAL::to_double(corner.x()) + localX * cos_a - localY * sin_a;
  double cy = CGAL::to_double(corner.y()) + localX * sin_a + localY * cos_a;
  m_center = Point_2(FT(cx), FT(cy));

  updateSFMLShape();
}

void Rectangle::updateDimensionsFromCorners() {
  double x1 = CGAL::to_double(m_corner1.x());
  double y1 = CGAL::to_double(m_corner1.y());
  double x2 = CGAL::to_double(m_corner2.x());
  double y2 = CGAL::to_double(m_corner2.y());

  m_width = std::abs(x2 - x1);
  m_height = std::abs(y2 - y1);
  m_rotationAngle = 0;  // Axis-aligned rectangles have no rotation
  m_center = Point_2(FT((x1 + x2) * 0.5), FT((y1 + y2) * 0.5));
}

void Rectangle::updateSFMLShape() {
  m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(m_width), static_cast<float>(m_height)));
  m_sfmlShape.setFillColor(m_color);
  m_sfmlShape.setOutlineThickness(1.5f);
  m_sfmlShape.setOutlineColor(sf::Color::Black);

  if (m_isRotatable) {
    double cx = CGAL::to_double(m_center.x());
    double cy = CGAL::to_double(m_center.y());
    m_sfmlShape.setOrigin(static_cast<float>(m_width * 0.5), static_cast<float>(m_height * 0.5));
    m_sfmlShape.setPosition(static_cast<float>(cx), static_cast<float>(cy));
    m_sfmlShape.setRotation(static_cast<float>(m_rotationAngle * 180.0 / 3.14159265359));
  } else {
    double x1 = CGAL::to_double(m_corner1.x());
    double y1 = CGAL::to_double(m_corner1.y());
    double x2 = CGAL::to_double(m_corner2.x());
    double y2 = CGAL::to_double(m_corner2.y());

    double minX = std::min(x1, x2);
    double minY = std::min(y1, y2);

    m_sfmlShape.setOrigin(0.0f, 0.0f);
    m_sfmlShape.setPosition(sf::Vector2f(static_cast<float>(minX), static_cast<float>(minY)));
    m_sfmlShape.setRotation(0.0f);
  }
}

void Rectangle::draw(sf::RenderWindow &window, float scale) const {
  // Scale the main shape's outline
  sf::RectangleShape scaledShape = m_sfmlShape;
  scaledShape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);
  window.draw(scaledShape);

  // Draw selection highlight if selected
  if (isSelected()) {
    sf::RectangleShape highlight = m_sfmlShape;
    highlight.setFillColor(sf::Color::Transparent);
    highlight.setOutlineThickness(3.0f * scale);
    highlight.setOutlineColor(sf::Color::Yellow);
    window.draw(highlight);
  } else if (isHovered()) {
    sf::RectangleShape highlight = m_sfmlShape;
    highlight.setFillColor(sf::Color::Transparent);
    highlight.setOutlineThickness(2.0f * scale); // Thinner than selection
    highlight.setOutlineColor(sf::Color::Cyan); // Cyan for hover
    window.draw(highlight);
  }

  drawVertexHandles(window, scale);
}

void Rectangle::setColor(const sf::Color &color) {
  m_color = color;
  m_sfmlShape.setFillColor(color);
}

Point_2 Rectangle::getCenter() const {
  return m_center;
}

bool Rectangle::contains(const sf::Vector2f &screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += 2 * tolerance;
  bounds.height += 2 * tolerance;
  return bounds.contains(screenPos);
}

bool Rectangle::isWithinDistance(const sf::Vector2f &screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += tolerance * 2;
  bounds.height += tolerance * 2;
  return bounds.contains(screenPos);
}

void Rectangle::translate(const Vector_2 &translation) {
  m_corner1 = Point_2(m_corner1.x() + translation.x(), m_corner1.y() + translation.y());
  m_corner2 = Point_2(m_corner2.x() + translation.x(), m_corner2.y() + translation.y());
  m_center = Point_2(m_center.x() + translation.x(), m_center.y() + translation.y());
  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::rotateCCW(const Point_2 &center, double angleRadians) {
  if (!m_isRotatable) return;

  m_rotationAngle += angleRadians;

  // Rotate corners around the center point
  double centerX = CGAL::to_double(center.x());
  double centerY = CGAL::to_double(center.y());
  double cos_a = std::cos(angleRadians);
  double sin_a = std::sin(angleRadians);

  auto rotatePoint = [centerX, centerY, cos_a, sin_a](const Point_2 &p) {
    double x = CGAL::to_double(p.x()) - centerX;
    double y = CGAL::to_double(p.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    return Point_2(FT(newX), FT(newY));
  };

  m_corner1 = rotatePoint(m_corner1);
  m_corner2 = rotatePoint(m_corner2);
  m_center = rotatePoint(m_center);
  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setCorners(const Point_2 &corner1, const Point_2 &corner2) {
  m_corner1 = corner1;
  m_corner2 = corner2;
  updateDimensionsFromCorners();
  updateSFMLShape();
}

std::vector<Point_2> Rectangle::getVertices() const {
  std::vector<Point_2> verts;
  verts.reserve(4);

  // Use SFML transform to capture rotation as well
  sf::Transform tf = m_sfmlShape.getTransform();
  sf::Vector2f size = m_sfmlShape.getSize();
  sf::Vector2f points[4] = {{0.f, 0.f}, {size.x, 0.f}, {size.x, size.y}, {0.f, size.y}};
  for (int i = 0; i < 4; ++i) {
    sf::Vector2f p = tf.transformPoint(points[i]);
    verts.emplace_back(FT(p.x), FT(p.y));
  }
  return verts;
}

std::vector<sf::Vector2f> Rectangle::getVerticesSFML() const {
  std::vector<sf::Vector2f> verts;
  verts.reserve(4);
  auto cgalVerts = getVertices();
  for (const auto &v : cgalVerts) {
    verts.emplace_back(static_cast<float>(CGAL::to_double(v.x())),
                       static_cast<float>(CGAL::to_double(v.y())));
  }
  return verts;
}

void Rectangle::setVertexPosition(size_t index, const Point_2 &value) {
  if (index >= 4) return;

  if (!m_isRotatable) {
    auto verts = getVertices();
    if (verts.size() != 4) return;

    Point_2 opposite = verts[(index + 2) % 4];
    setCorners(value, opposite);
    m_center = Point_2(FT((CGAL::to_double(value.x()) + CGAL::to_double(opposite.x())) * 0.5),
                       FT((CGAL::to_double(value.y()) + CGAL::to_double(opposite.y())) * 0.5));
    return;
  }

  auto verts = getVertices();
  if (verts.size() != 4) return;

  Point_2 center = getCenter();
  double cos_a = std::cos(m_rotationAngle);
  double sin_a = std::sin(m_rotationAngle);

  auto projectU = [&](const Vector_2 &v) {
    return CGAL::to_double(v.x()) * cos_a + CGAL::to_double(v.y()) * sin_a;
  };
  auto projectV = [&](const Vector_2 &v) {
    return CGAL::to_double(v.x()) * -sin_a + CGAL::to_double(v.y()) * cos_a;
  };

  Vector_2 toNew = value - center;
  double projU = projectU(toNew);
  double projV = projectV(toNew);

  Vector_2 currentVec = verts[index] - center;
  double signU = projectU(currentVec) >= 0.0 ? 1.0 : -1.0;
  double signV = projectV(currentVec) >= 0.0 ? 1.0 : -1.0;

  const double minExtent = 1e-3;
  double halfWidth = std::max(minExtent, std::abs(projU));
  double halfHeight = std::max(minExtent, std::abs(projV));

  m_width = halfWidth * 2.0;
  m_height = halfHeight * 2.0;

  double cx = CGAL::to_double(center.x());
  double cy = CGAL::to_double(center.y());

  double localX = -m_width * 0.5;
  double localY = -m_height * 0.5;
  double rotatedX = localX * cos_a - localY * sin_a;
  double rotatedY = localX * sin_a + localY * cos_a;
  m_corner1 = Point_2(FT(cx + rotatedX), FT(cy + rotatedY));
  m_corner2 = Point_2(FT(cx - rotatedX), FT(cy - rotatedY));
  m_center = center;

  // Align active vertex to the dragged quadrant
  double offsetX = signU * halfWidth;
  double offsetY = signV * halfHeight;
  double targetX = cx + offsetX * cos_a - offsetY * sin_a;
  double targetY = cy + offsetX * sin_a + offsetY * cos_a;
  (void)targetX;
  (void)targetY;

  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setRotation(double angleRadians) {
  m_rotationAngle = angleRadians;
  updateSFMLShape();
  updateHostedPoints();
}
sf::FloatRect Rectangle::getGlobalBounds() const {
  return m_sfmlShape.getGlobalBounds();
}

Point_2 Rectangle::getCGALPosition() const {
  return m_center;
}

void Rectangle::setCGALPosition(const Point_2 &newPos) {
  Vector_2 translation = newPos - getCGALPosition();
  m_corner1 = Point_2(m_corner1.x() + translation.x(), m_corner1.y() + translation.y());
  m_corner2 = Point_2(m_corner2.x() + translation.x(), m_corner2.y() + translation.y());
  m_center = Point_2(m_center.x() + translation.x(), m_center.y() + translation.y());
  if (!m_isRotatable) {
    updateDimensionsFromCorners();
  }
  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setPosition(const sf::Vector2f &newSfmlPos) {
  // Get current position
  sf::Vector2f currentPos = m_sfmlShape.getPosition();
  
  // Calculate translation
  sf::Vector2f translation = newSfmlPos - currentPos;
  
  // Translate the shape
  Vector_2 delta(translation.x, translation.y);
  translate(delta);
}

void Rectangle::drawVertexHandles(sf::RenderWindow &window, float scale) const {
  const float handleRadius = 4.0f * scale;
  auto verts = getVertices();
  const char* labels[] = {"A", "B", "C", "D"};
  
  for (size_t i = 0; i < verts.size(); ++i) {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    float x = static_cast<float>(CGAL::to_double(verts[i].x()));
    float y = static_cast<float>(CGAL::to_double(verts[i].y()));
    handle.setPosition(x, y);
    
    sf::Color base = sf::Color(180, 180, 180);
    if (static_cast<int>(i) == m_activeVertex) {
      base = sf::Color(255, 140, 0);
    } else if (static_cast<int>(i) == m_hoveredVertex) {
      base = sf::Color::Yellow;
    }
    handle.setFillColor(base);
    handle.setOutlineThickness(1.0f * scale);
    handle.setOutlineColor(sf::Color::Black);
    window.draw(handle);
    
    // Draw vertex label
    VertexLabelManager::instance().drawLabel(window, sf::Vector2f(x, y), labels[i]);
  }
}

std::vector<Point_2> Rectangle::getInteractableVertices() const {
  return getVertices();  // Delegate to existing method
}

std::vector<Segment_2> Rectangle::getEdges() const {
  auto verts = getVertices();
  std::vector<Segment_2> edges;
  edges.reserve(4);
  
  if (verts.size() == 4) {
    // Create 4 edges connecting consecutive vertices
    edges.emplace_back(verts[0], verts[1]);
    edges.emplace_back(verts[1], verts[2]);
    edges.emplace_back(verts[2], verts[3]);
    edges.emplace_back(verts[3], verts[0]);
  }
  
  return edges;
}