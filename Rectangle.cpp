#include "Rectangle.h"
// Force recompile to resolve vtable linker error.

#include "Point.h"
#include "VertexLabelManager.h"
#include <cmath>

Rectangle::Rectangle(const Point_2 &corner1, const Point_2 &corner2, bool isRotatable,
                     const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
  m_corner1(std::make_shared<Point>(corner1, 1.0f)),
  m_corner2(std::make_shared<Point>(corner2, 1.0f)),
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

Rectangle::Rectangle(const std::shared_ptr<Point> &corner1,
                     const std::shared_ptr<Point> &corner2, bool isRotatable,
                     const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
      m_corner1(corner1 ? corner1 : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_corner2(corner2 ? corner2 : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
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
  m_corner1(std::make_shared<Point>(corner, 1.0f)),
  m_corner2(std::make_shared<Point>(adjacentPoint, 1.0f)),
      m_isRotatable(true),
      m_width(width),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)),
      m_color(color) {
  m_color.a = 0;
  syncRotatableFromAnchors();

  updateSFMLShape();
}

Rectangle::Rectangle(const std::shared_ptr<Point> &corner,
                     const std::shared_ptr<Point> &adjacentPoint, double width,
                     const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RectangleRotatable, color, id),
      m_corner1(corner ? corner : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_corner2(adjacentPoint ? adjacentPoint
                              : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_isRotatable(true),
      m_width(width),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)),
      m_color(color) {
  m_color.a = 0;
  syncRotatableFromAnchors();
  updateSFMLShape();
}

Point_2 Rectangle::getCorner1() const {
  return getCorner1Position();
}

Point_2 Rectangle::getCorner2() const {
  return getCorner2Position();
}

Point_2 Rectangle::getCorner1Position() const {
  if (m_corner1) {
    return m_corner1->getCGALPosition();
  }
  return Point_2(FT(0), FT(0));
}

Point_2 Rectangle::getCorner2Position() const {
  if (m_corner2) {
    return m_corner2->getCGALPosition();
  }
  return Point_2(FT(0), FT(0));
}

void Rectangle::setCorner1Position(const Point_2 &pos) {
  if (m_corner1) {
    m_corner1->setCGALPosition(pos);
  } else {
    m_corner1 = std::make_shared<Point>(pos, 1.0f);
  }
}

void Rectangle::setCorner2Position(const Point_2 &pos) {
  if (m_corner2) {
    m_corner2->setCGALPosition(pos);
  } else {
    m_corner2 = std::make_shared<Point>(pos, 1.0f);
  }
}

void Rectangle::syncRotatableFromAnchors() {
  if (!m_isRotatable) return;

  Point_2 corner = getCorner1Position();
  Point_2 adjacent = getCorner2Position();

  double dx = CGAL::to_double(adjacent.x()) - CGAL::to_double(corner.x());
  double dy = CGAL::to_double(adjacent.y()) - CGAL::to_double(corner.y());
  m_rotationAngle = std::atan2(dy, dx);
  double baseLen = std::sqrt(dx * dx + dy * dy);
  m_width = baseLen;

  const double minExtent = 1e-3;
  double heightSign = (m_height >= 0.0) ? 1.0 : -1.0;
  double heightAbs = std::abs(m_height);
  if (heightAbs < minExtent) {
    heightAbs = minExtent;
  }
  m_height = heightAbs * heightSign;
  if (baseLen < minExtent) {
    return;
  }

  double nx = -dy / baseLen;
  double ny = dx / baseLen;

  double midX = CGAL::to_double(corner.x()) + dx * 0.5;
  double midY = CGAL::to_double(corner.y()) + dy * 0.5;

  double toCenterX = CGAL::to_double(m_center.x()) - midX;
  double toCenterY = CGAL::to_double(m_center.y()) - midY;
  double sideSign = (toCenterX * nx + toCenterY * ny);
  double centerDistSq = toCenterX * toCenterX + toCenterY * toCenterY;
  if (centerDistSq < 1e-12) {
    sideSign = heightSign;
  } else if (std::abs(sideSign) < 1e-12) {
    sideSign = heightSign;
  } else {
    sideSign = (sideSign >= 0.0) ? 1.0 : -1.0;
  }

  double cx = midX + nx * (heightAbs * 0.5) * sideSign;
  double cy = midY + ny * (heightAbs * 0.5) * sideSign;
  m_center = Point_2(FT(cx), FT(cy));
}

void Rectangle::updateDimensionsFromCorners() {
  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  double x1 = CGAL::to_double(c1.x());
  double y1 = CGAL::to_double(c1.y());
  double x2 = CGAL::to_double(c2.x());
  double y2 = CGAL::to_double(c2.y());

  if (m_isRotatable) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double baseLen = std::sqrt(dx * dx + dy * dy);
    m_width = baseLen;
    m_rotationAngle = std::atan2(dy, dx);

    const double minExtent = 1e-3;
    double heightSign = (m_height >= 0.0) ? 1.0 : -1.0;
    double heightAbs = std::abs(m_height);
    if (heightAbs < minExtent) {
      heightAbs = minExtent;
    }
    m_height = heightAbs * heightSign;

    if (baseLen < minExtent) {
      return;
    }

    double nx = -dy / baseLen;
    double ny = dx / baseLen;
    double midX = x1 + dx * 0.5;
    double midY = y1 + dy * 0.5;

    double toCenterX = CGAL::to_double(m_center.x()) - midX;
    double toCenterY = CGAL::to_double(m_center.y()) - midY;
    double sideSign = (toCenterX * nx + toCenterY * ny);
    double centerDistSq = toCenterX * toCenterX + toCenterY * toCenterY;
    if (centerDistSq < 1e-12 || std::abs(sideSign) < 1e-12) {
      sideSign = heightSign;
    } else {
      sideSign = (sideSign >= 0.0) ? 1.0 : -1.0;
    }

    double cx = midX + nx * (heightAbs * 0.5) * sideSign;
    double cy = midY + ny * (heightAbs * 0.5) * sideSign;
    m_center = Point_2(FT(cx), FT(cy));
    return;
  }

  m_width = std::abs(x2 - x1);
  m_height = std::abs(y2 - y1);
  m_rotationAngle = 0;  // Axis-aligned rectangles have no rotation
  m_center = Point_2(FT((x1 + x2) * 0.5), FT((y1 + y2) * 0.5));
}

void Rectangle::updateSFMLShape() {
  if (m_isRotatable) {
    syncRotatableFromAnchors();
  } else {
    updateDimensionsFromCorners();
  }
  if (m_isRotatable) {
    double heightAbs = std::abs(m_height);
    m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(m_width), static_cast<float>(heightAbs)));
  } else {
    m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(m_width), static_cast<float>(m_height)));
  }
  m_sfmlShape.setFillColor(m_color);
  m_sfmlShape.setOutlineThickness(1.5f);
  m_sfmlShape.setOutlineColor(sf::Color::Black);

  if (m_isRotatable) {
    double cx = CGAL::to_double(m_center.x());
    double cy = CGAL::to_double(m_center.y());
    double heightAbs = std::abs(m_height);
    m_sfmlShape.setOrigin(static_cast<float>(m_width * 0.5), static_cast<float>(heightAbs * 0.5));
    m_sfmlShape.setPosition(static_cast<float>(cx), static_cast<float>(cy));
    m_sfmlShape.setRotation(static_cast<float>(m_rotationAngle * 180.0 / 3.14159265359));
  } else {
    Point_2 c1 = getCorner1Position();
    Point_2 c2 = getCorner2Position();
    double x1 = CGAL::to_double(c1.x());
    double y1 = CGAL::to_double(c1.y());
    double x2 = CGAL::to_double(c2.x());
    double y2 = CGAL::to_double(c2.y());

    double minX = std::min(x1, x2);
    double minY = std::min(y1, y2);
    double width = std::abs(x2 - x1);
    double height = std::abs(y2 - y1);

    m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(width), static_cast<float>(height)));
    m_sfmlShape.setOrigin(0.0f, 0.0f);
    m_sfmlShape.setPosition(sf::Vector2f(static_cast<float>(minX), static_cast<float>(minY)));
    m_sfmlShape.setRotation(0.0f);
  }

  syncDependentCorners();
}

void Rectangle::setDependentCornerPoints(const std::shared_ptr<Point> &b,
                                         const std::shared_ptr<Point> &d) {
  m_cornerB = b;
  m_cornerD = d;
  syncDependentCorners();
}

void Rectangle::syncDependentCorners() {
  if (!m_cornerB && !m_cornerD) return;

  if (!m_isRotatable) {
    Point_2 c1 = getCorner1Position();
    Point_2 c2 = getCorner2Position();
    double x1 = CGAL::to_double(c1.x());
    double y1 = CGAL::to_double(c1.y());
    double x2 = CGAL::to_double(c2.x());
    double y2 = CGAL::to_double(c2.y());

    if (m_cornerB) {
      m_cornerB->setCGALPosition(Point_2(FT(x2), FT(y1)));
    }
    if (m_cornerD) {
      m_cornerD->setCGALPosition(Point_2(FT(x1), FT(y2)));
    }
    return;
  }

  Point_2 a = getCorner1Position();
  Point_2 b = getCorner2Position();
  double dx = CGAL::to_double(b.x()) - CGAL::to_double(a.x());
  double dy = CGAL::to_double(b.y()) - CGAL::to_double(a.y());
  double baseLen = std::sqrt(dx * dx + dy * dy);
  if (baseLen < 1e-9) return;

  double ux = -dy / baseLen;
  double uy = dx / baseLen;

  double heightAbs = std::abs(m_height);
  double heightSign = (m_height >= 0.0) ? 1.0 : -1.0;

  Point_2 c(FT(CGAL::to_double(b.x()) + ux * heightAbs * heightSign),
            FT(CGAL::to_double(b.y()) + uy * heightAbs * heightSign));
  Point_2 d(FT(CGAL::to_double(a.x()) + ux * heightAbs * heightSign),
            FT(CGAL::to_double(a.y()) + uy * heightAbs * heightSign));

  if (m_cornerB) m_cornerB->setCGALPosition(c);
  if (m_cornerD) m_cornerD->setCGALPosition(d);
}

void Rectangle::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!m_visible && !forceVisible) return;

  // Scale the main shape's outline
  sf::RectangleShape scaledShape = m_sfmlShape;
  scaledShape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);

  // GHOST MODE: Apply transparency if hidden but forced visible
  if (!m_visible && forceVisible) {
      sf::Color ghostFill = scaledShape.getFillColor();
      ghostFill.a = 50; // Faint alpha
      scaledShape.setFillColor(ghostFill);
      
      sf::Color ghostOutline = scaledShape.getOutlineColor();
      ghostOutline.a = 50;
      scaledShape.setOutlineColor(ghostOutline);
  }

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

void Rectangle::update() {
  updateSFMLShape();
  updateHostedPoints();
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
  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  setCorner1Position(Point_2(c1.x() + translation.x(), c1.y() + translation.y()));
  setCorner2Position(Point_2(c2.x() + translation.x(), c2.y() + translation.y()));
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

  setCorner1Position(rotatePoint(getCorner1Position()));
  setCorner2Position(rotatePoint(getCorner2Position()));
  m_center = rotatePoint(m_center);
  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setCorners(const Point_2 &corner1, const Point_2 &corner2) {
  setCorner1Position(corner1);
  setCorner2Position(corner2);
  updateDimensionsFromCorners();
  updateSFMLShape();
}

std::vector<Point_2> Rectangle::getVertices() const {
  std::vector<Point_2> verts;
  verts.reserve(4);

  if (!m_isRotatable) {
    Point_2 c1 = getCorner1Position();
    Point_2 c2 = getCorner2Position();
    double x1 = CGAL::to_double(c1.x());
    double y1 = CGAL::to_double(c1.y());
    double x2 = CGAL::to_double(c2.x());
    double y2 = CGAL::to_double(c2.y());

    verts.emplace_back(FT(x1), FT(y1));
    verts.emplace_back(FT(x2), FT(y1));
    verts.emplace_back(FT(x2), FT(y2));
    verts.emplace_back(FT(x1), FT(y2));
    return verts;
  }

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

  if (m_isRotatable) {
    Point_2 a = getCorner1Position();
    Point_2 b = getCorner2Position();

    if (index == 0) {
      setCorner1Position(value);
      updateSFMLShape();
      updateHostedPoints();
      return;
    }
    if (index == 1) {
      setCorner2Position(value);
      updateSFMLShape();
      updateHostedPoints();
      return;
    }

    double dx = CGAL::to_double(b.x()) - CGAL::to_double(a.x());
    double dy = CGAL::to_double(b.y()) - CGAL::to_double(a.y());
    double length = std::sqrt(dx * dx + dy * dy);
    const double minExtent = 1e-3;
    if (length < minExtent) return;

    double nx = -dy / length;
    double ny = dx / length;

    double vx = CGAL::to_double(value.x()) - CGAL::to_double(b.x());
    double vy = CGAL::to_double(value.y()) - CGAL::to_double(b.y());
    double signedDist = vx * nx + vy * ny;
    if (std::abs(signedDist) < minExtent) {
      signedDist = (signedDist >= 0.0 ? 1.0 : -1.0) * minExtent;
    }

    m_height = signedDist;

    double midX = CGAL::to_double(a.x()) + dx * 0.5;
    double midY = CGAL::to_double(a.y()) + dy * 0.5;
    m_center = Point_2(FT(midX + nx * signedDist * 0.5), FT(midY + ny * signedDist * 0.5));

    updateSFMLShape();
    updateHostedPoints();
    return;
  }

  double newX = CGAL::to_double(value.x());
  double newY = CGAL::to_double(value.y());

  switch (index) {
    case 0: // Dragging Corner 1
      setCorner1Position(value);
      break;
    case 1: // Dragging Corner (C2.x, C1.y)
      setCorner2Position(Point_2(FT(newX), getCorner2Position().y()));
      setCorner1Position(Point_2(getCorner1Position().x(), FT(newY)));
      break;
    case 2: // Dragging Corner 2
      setCorner2Position(value);
      break;
    case 3: // Dragging Corner (C1.x, C2.y)
      setCorner1Position(Point_2(FT(newX), getCorner1Position().y()));
      setCorner2Position(Point_2(getCorner2Position().x(), FT(newY)));
      break;
  }

  Point_2 finalC1 = getCorner1Position();
  Point_2 finalC2 = getCorner2Position();
  double finalX1 = CGAL::to_double(finalC1.x());
  double finalY1 = CGAL::to_double(finalC1.y());
  double finalX2 = CGAL::to_double(finalC2.x());
  double finalY2 = CGAL::to_double(finalC2.y());

  m_center = Point_2(FT((finalX1 + finalX2) * 0.5), FT((finalY1 + finalY2) * 0.5));

  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setRotation(double angleRadians) {
  m_rotationAngle = angleRadians;
  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setHeight(double height) {
  if (m_isRotatable) {
    if (!std::isfinite(height)) return;
    const double minExtent = 1e-3;
    if (std::abs(height) < minExtent) {
      height = (height >= 0.0 ? 1.0 : -1.0) * minExtent;
    }
    m_height = height;
  } else {
    if (!std::isfinite(height)) return;
    m_height = std::abs(height);
  }
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
  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  setCorner1Position(Point_2(c1.x() + translation.x(), c1.y() + translation.y()));
  setCorner2Position(Point_2(c2.x() + translation.x(), c2.y() + translation.y()));
  m_center = Point_2(m_center.x() + translation.x(), m_center.y() + translation.y());
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
  }
}

// void Rectangle::drawLabel(sf::RenderWindow &window, const sf::View &worldView) const {
//   if (!m_isRotatable || !m_visible) return;

//   auto verts = getVerticesSFML();
//   const char* labels[] = {"A", "B", "C", "D"};

//   // Switch to Screen Space is handled by caller (GeometryEditor::render) or strictly here?
//   // GeometryEditor::render sets DefaultView BEFORE calling this. 
//   // But we need to map World Coords -> Screen Coords.
  
//   // Note: window should be in DefaultView when this is called, 
//   // BUT we need worldView to project the points.
  
//   for (size_t i = 0; i < verts.size() && i < 4; ++i) {
//       sf::Vector2f worldPos = verts[i];
//       sf::Vector2i screenPos = window.mapCoordsToPixel(worldPos, worldView);
//       sf::Vector2f drawPos = window.mapPixelToCoords(screenPos, window.getView());
      
//       VertexLabelManager::instance().drawLabel(window, drawPos, labels[i]);
//   }
// }

std::vector<Point_2> Rectangle::getInteractableVertices() const {
  if (!m_isRotatable) {
    std::vector<Point_2> verts;
    verts.reserve(4);
    Point_2 c1 = getCorner1Position();
    Point_2 c2 = getCorner2Position();
    verts.emplace_back(c1);
    verts.emplace_back(Point_2(c2.x(), c1.y()));
    verts.emplace_back(c2);
    verts.emplace_back(Point_2(c1.x(), c2.y()));
    return verts;
  }
  return getVertices();
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