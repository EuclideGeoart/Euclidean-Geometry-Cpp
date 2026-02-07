#include "Rectangle.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>

#include "Circle.h"
#include "Constants.h"
#include "Line.h"
#include "Point.h"
#include "PointUtils.h"
#include "Types.h"

Rectangle::Rectangle(const Point_2& corner1, const Point_2& corner2, bool isRotatable, const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
      m_corner1(std::make_shared<Point>(corner1, 1.0f)),
      m_corner2(std::make_shared<Point>(corner2, 1.0f)),
      m_isRotatable(isRotatable),
      m_width(0),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  // Initialize B and D
  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  updateDimensionsFromCorners();
  updateSFMLShape();
  setShowLabel(true);
}

Rectangle::Rectangle(const std::shared_ptr<Point>& corner1,
                     const std::shared_ptr<Point>& corner2,
                     bool isRotatable,
                     const sf::Color& color,
                     unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
      m_corner1(corner1 ? corner1 : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_corner2(corner2 ? corner2 : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_isRotatable(isRotatable),
      m_width(0),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  updateDimensionsFromCorners();
  updateSFMLShape();
  setShowLabel(true);
}

Rectangle::Rectangle(const Point_2& corner, const Point_2& adjacentPoint, double width, const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::RectangleRotatable, color, id),
      m_corner1(std::make_shared<Point>(corner, 1.0f)),
      m_corner2(std::make_shared<Point>(adjacentPoint, 1.0f)),
      m_isRotatable(true),
      m_width(0),
      m_height(width),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  syncRotatableFromAnchors();
  updateSFMLShape();
  setShowLabel(true);
}

Rectangle::Rectangle(const std::shared_ptr<Point>& corner,
                     const std::shared_ptr<Point>& adjacentPoint,
                     double width,
                     const sf::Color& color,
                     unsigned int id)
    : GeometricObject(ObjectType::RectangleRotatable, color, id),
      m_corner1(corner ? corner : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_corner2(adjacentPoint ? adjacentPoint : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_isRotatable(true),
      m_width(0),
      m_height(width),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  syncRotatableFromAnchors();
  updateSFMLShape();
  setShowLabel(true);
}

Point_2 Rectangle::getCorner1() const { return getCorner1Position(); }

Point_2 Rectangle::getCorner2() const { return getCorner2Position(); }

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

void Rectangle::setCorner1Position(const Point_2& pos, bool triggerUpdate) {
  if (m_corner1) {
    m_corner1->setCGALPosition(pos);
  } else {
    m_corner1 = std::make_shared<Point>(pos, 1.0f);
  }
  if (triggerUpdate) {
    updateSFMLShape();
    updateHostedPoints();  // This calls notifyDependents()
  }
}

void Rectangle::setCorner2Position(const Point_2& pos, bool triggerUpdate) {
  if (m_corner2) {
    m_corner2->setCGALPosition(pos);
  } else {
    m_corner2 = std::make_shared<Point>(pos, 1.0f);
  }
  if (triggerUpdate) {
    updateSFMLShape();
    updateHostedPoints();  // This calls notifyDependents()
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

  if (baseLen < 1e-9) return;

  double nx = -dy / baseLen;
  double ny = dx / baseLen;

  double midX = CGAL::to_double(corner.x()) + dx * 0.5;
  double midY = CGAL::to_double(corner.y()) + dy * 0.5;

  double cx = midX + nx * (m_height * 0.5);
  double cy = midY + ny * (m_height * 0.5);
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

    double cx = midX + nx * (m_height * 0.5);
    double cy = midY + ny * (m_height * 0.5);
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

    // --- FIX FOR ROTATED RECTANGLE DRAG FLIP ---
    // If height is negative, we flip the shape along the Y axis.
    // This ensures that Vertex 2 (Bottom Right in local coords) physically moves to the
    // "negative" side relative to the base line, matching the logical coordinates.
    if (m_height < 0) {
      m_sfmlShape.setScale(1.0f, -1.0f);
    } else {
      m_sfmlShape.setScale(1.0f, 1.0f);
    }
    // -------------------------------------------
  } else {
    m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(m_width), static_cast<float>(m_height)));
    m_sfmlShape.setScale(1.0f, 1.0f);  // Reset scale for non-rotatable
  }

  m_sfmlShape.setFillColor(m_color);
  m_sfmlShape.setOutlineThickness(m_thickness);
  m_sfmlShape.setOutlineColor(sf::Color::Black);

  if (m_isRotatable) {
    double cx = CGAL::to_double(m_center.x());
    double cy = CGAL::to_double(m_center.y());
    double heightAbs = std::abs(m_height);

    // Origin is at the center of the rectangle
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

void Rectangle::setDependentCornerPoints(const std::shared_ptr<Point>& b, const std::shared_ptr<Point>& d) {
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

  Point_2 c(FT(CGAL::to_double(b.x()) + ux * heightAbs * heightSign), FT(CGAL::to_double(b.y()) + uy * heightAbs * heightSign));
  Point_2 d(FT(CGAL::to_double(a.x()) + ux * heightAbs * heightSign), FT(CGAL::to_double(a.y()) + uy * heightAbs * heightSign));

  if (m_cornerB) m_cornerB->setCGALPosition(c);
  if (m_cornerD) m_cornerD->setCGALPosition(d);
}

void Rectangle::draw(sf::RenderWindow& window, float scale, bool forceVisible) const {
  if (!m_visible && !forceVisible) return;

  // Scale the main shape's outline
  sf::RectangleShape scaledShape = m_sfmlShape;
  scaledShape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);

  // GHOST MODE: Apply transparency if hidden but forced visible
  if (!m_visible && forceVisible) {
    sf::Color ghostFill = scaledShape.getFillColor();
    ghostFill.a = 50;  // Faint alpha
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
    highlight.setOutlineColor(Constants::SELECTION_COLOR);
    window.draw(highlight);
  } else if (isHovered()) {
    // If specific edge hovered, highlight ONLY that edge, OR highlight shape + edge
    // User requested "highlighted separately from whole shape".
    // We'll draw the whole shape with thinner/alpha cyan, and the edge with thick distinct color.

    sf::RectangleShape highlight = m_sfmlShape;
    highlight.setFillColor(sf::Color::Transparent);
    highlight.setOutlineThickness(1.0f * scale);
    highlight.setOutlineColor(sf::Color(0, 255, 255, 100));  // Dim Cyan
    if (getHoveredEdge() == -1) {
      highlight.setOutlineThickness(2.0f * scale);
      highlight.setOutlineColor(sf::Color::Cyan);  // Full Cyan if generic hover
    }
    window.draw(highlight);

    if (getHoveredEdge() >= 0) {
      auto verts = getVerticesSFML();
      int idx = getHoveredEdge();
      if (idx >= 0 && idx < static_cast<int>(verts.size())) {
        sf::Vector2f p1 = verts[idx];
        sf::Vector2f p2 = verts[(idx + 1) % verts.size()];

        sf::Vector2f dir = p2 - p1;
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
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
          thickLine.setFillColor(sf::Color(255, 100, 50, 200));  // Red-Orange with alpha
          window.draw(thickLine);
        }
      }
    }
  }

  // Delegate drawing to the constituent points
  if (m_corner1) m_corner1->draw(window, scale, forceVisible);
  if (m_cornerB) m_cornerB->draw(window, scale, forceVisible);
  if (m_corner2) m_corner2->draw(window, scale, forceVisible);
  if (m_cornerD) m_cornerD->draw(window, scale, forceVisible);

  drawVertexHandles(window, scale);
}

void Rectangle::updateDependentShape() {
  if (m_corner1) m_corner1->setDeferConstraintUpdates(true);
  if (m_corner2) m_corner2->setDeferConstraintUpdates(true);
  if (m_cornerB) m_cornerB->setDeferConstraintUpdates(true);
  if (m_cornerD) m_cornerD->setDeferConstraintUpdates(true);

  auto parent = m_parentSource.lock();
  if (!parent || !parent->isValid()) {
    updateSFMLShape();
    updateHostedPoints();
    if (m_corner1) m_corner1->forceConstraintUpdate();
    if (m_corner2) m_corner2->forceConstraintUpdate();
    if (m_cornerB) m_cornerB->forceConstraintUpdate();
    if (m_cornerD) m_cornerD->forceConstraintUpdate();
    return;
  }

  auto sourceRect = std::dynamic_pointer_cast<Rectangle>(parent);
  if (!sourceRect || !sourceRect->isValid()) {
    updateSFMLShape();
    updateHostedPoints();
    if (m_corner1) m_corner1->forceConstraintUpdate();
    if (m_corner2) m_corner2->forceConstraintUpdate();
    if (m_cornerB) m_cornerB->forceConstraintUpdate();
    if (m_cornerD) m_cornerD->forceConstraintUpdate();
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
        // Convert degrees to radians
        double rad = m_transformValue * 3.14159265358979323846 / 180.0;
        double s = std::sin(rad);
        double c_val = std::cos(rad);
        double dx = CGAL::to_double(p.x() - c.x());
        double dy = CGAL::to_double(p.y() - c.y());
        return Point_2(c.x() + FT(dx * c_val - dy * s), c.y() + FT(dx * s + dy * c_val));
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

  auto verts = sourceRect->getVertices();
  if (verts.size() < 4) {
    setVisible(false);
    return;
  }

  if (!isRotatable()) {
    auto p0 = transformPoint(verts[0]);
    auto p2 = transformPoint(verts[2]);
    if (!p0 || !p2) {
      setVisible(false);
      return;
    }
    setCorner1Position(flattenPoint(*p0), false);  // Don't auto-update
    setCorner2Position(flattenPoint(*p2), false);  // Don't auto-update
    updateSFMLShape();
    updateHostedPoints();  // Notifies our dependents
    setVisible(sourceRect->isVisible());
    if (m_corner1) m_corner1->forceConstraintUpdate();
    if (m_corner2) m_corner2->forceConstraintUpdate();
    if (m_cornerB) m_cornerB->forceConstraintUpdate();
    if (m_cornerD) m_cornerD->forceConstraintUpdate();
    return;
  }

  auto p0 = transformPoint(verts[0]);
  auto p1 = transformPoint(verts[1]);
  auto p3 = transformPoint(verts[3]);
  if (!p0 || !p1 || !p3) {
    setVisible(false);
    return;
  }

  setCorner1Position(flattenPoint(*p0), false);  // Don't auto-update
  setCorner2Position(flattenPoint(*p1), false);  // Don't auto-update

  double dx = CGAL::to_double(p1->x() - p0->x());
  double dy = CGAL::to_double(p1->y() - p0->y());
  double hx = CGAL::to_double(p3->x() - p0->x());
  double hy = CGAL::to_double(p3->y() - p0->y());
  double det = dx * hy - dy * hx;
  double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(*p0, *p3)));
  double h = (det >= 0) ? dist : -dist;

  setHeight(h);
  setVisible(sourceRect->isVisible());
  if (m_corner1) m_corner1->forceConstraintUpdate();
  if (m_corner2) m_corner2->forceConstraintUpdate();
  if (m_cornerB) m_cornerB->forceConstraintUpdate();
  if (m_cornerD) m_cornerD->forceConstraintUpdate();
}

void Rectangle::update() {
  if (isDependent()) {
    updateDependentShape();
  } else {
    updateSFMLShape();
    updateHostedPoints();
  }

  // Suppress point labels favoring the Rectangle's own screen-space labels
  // This prevents duplication where both GeometryEditor and Rectangle draw labels
  // CHANGED: We now want points to manage their own labels for Global Consistency and SVG export.
  // NO OP.
  // if (m_corner1) m_corner1->setShowLabel(false);
  // if (m_corner2) m_corner2->setShowLabel(false);
  // if (m_cornerB) m_cornerB->setShowLabel(false);
  // if (m_cornerD) m_cornerD->setShowLabel(false);
}

void Rectangle::setColor(const sf::Color& color) {
  m_color = color;
  m_sfmlShape.setFillColor(color);
}

Point_2 Rectangle::getCenter() const { return m_center; }

bool Rectangle::contains(const sf::Vector2f& screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += 2 * tolerance;
  bounds.height += 2 * tolerance;
  return bounds.contains(screenPos);
}

bool Rectangle::isWithinDistance(const sf::Vector2f& screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += tolerance * 2;
  bounds.height += tolerance * 2;
  return bounds.contains(screenPos);
}

void Rectangle::translate(const Vector_2& translation) {
  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  setCorner1Position(flattenPoint(Point_2(c1.x() + translation.x(), c1.y() + translation.y())), false);
  setCorner2Position(flattenPoint(Point_2(c2.x() + translation.x(), c2.y() + translation.y())), false);
  m_center = flattenPoint(Point_2(m_center.x() + translation.x(), m_center.y() + translation.y()));
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
}

void Rectangle::rotateCCW(const Point_2& center, double angleRadians) {
  if (!m_isRotatable) return;

  m_rotationAngle += angleRadians;

  // Rotate corners around the center point
  double centerX = CGAL::to_double(center.x());
  double centerY = CGAL::to_double(center.y());
  double cos_a = std::cos(angleRadians);
  double sin_a = std::sin(angleRadians);

  auto rotatePoint = [centerX, centerY, cos_a, sin_a](const Point_2& p) {
    double x = CGAL::to_double(p.x()) - centerX;
    double y = CGAL::to_double(p.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    return Point_2(FT(newX), FT(newY));
  };

  setCorner1Position(rotatePoint(getCorner1Position()), false);
  setCorner2Position(rotatePoint(getCorner2Position()), false);
  m_center = rotatePoint(m_center);
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
}

void Rectangle::setCorners(const Point_2& corner1, const Point_2& corner2) {
  setCorner1Position(corner1, false);
  setCorner2Position(corner2, false);
  updateDimensionsFromCorners();
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
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
  for (const auto& v : cgalVerts) {
    verts.emplace_back(static_cast<float>(CGAL::to_double(v.x())), static_cast<float>(CGAL::to_double(v.y())));
  }
  return verts;
}

void Rectangle::setVertexPosition(size_t index, const Point_2& value) {
  if (index >= 4) return;

  if (m_isRotatable) {
    Point_2 a = getCorner1Position();
    Point_2 b = getCorner2Position();

    if (index == 0) {
      setCorner1Position(value, false);  // Don't auto-update, we'll do it below
      updateSFMLShape();
      updateHostedPoints();  // Notifies dependents
      return;
    }
    if (index == 1) {
      setCorner2Position(value, false);  // Don't auto-update, we'll do it below
      updateSFMLShape();
      updateHostedPoints();  // Notifies dependents
      return;
    }

    // For vertices 2 (C) and 3 (D), we need to adjust the height
    // The base edge is A->B (corner1->corner2)
    double dx = CGAL::to_double(b.x()) - CGAL::to_double(a.x());
    double dy = CGAL::to_double(b.y()) - CGAL::to_double(a.y());
    double length = std::sqrt(dx * dx + dy * dy);
    const double minExtent = 1e-3;
    if (length < minExtent) return;

    // Normal vector perpendicular to the base (A->B), pointing "up"
    double nx = -dy / length;
    double ny = dx / length;

    // For index 2 (vertex C): measure from B
    // For index 3 (vertex D): measure from A
    double vx, vy;
    if (index == 2) {
      // Vertex C: calculate vector from B to the dragged position
      vx = CGAL::to_double(value.x()) - CGAL::to_double(b.x());
      vy = CGAL::to_double(value.y()) - CGAL::to_double(b.y());
    } else {
      // Vertex D: calculate vector from A to the dragged position
      vx = CGAL::to_double(value.x()) - CGAL::to_double(a.x());
      vy = CGAL::to_double(value.y()) - CGAL::to_double(a.y());
    }

    // Project the vector onto the normal to get signed distance
    double signedDist = vx * nx + vy * ny;

    // Allow crossing the center line (changing sign), but prevent exact zero/degenerate height
    if (std::abs(signedDist) < minExtent) {
      signedDist = (signedDist >= 0.0 ? 1.0 : -1.0) * minExtent;
    }

    // Delegate height/center logic to setHeight (which calls updateSFMLShape and updateHostedPoints)
    setHeight(signedDist);
    // Note: setHeight() already calls updateSFMLShape() and updateHostedPoints()
    // No need to call updateSFMLShape() again here
    return;
  }

  double newX = CGAL::to_double(value.x());
  double newY = CGAL::to_double(value.y());

  switch (index) {
    case 0:  // Dragging Corner 1
      setCorner1Position(value, false);
      break;
    case 1:  // Dragging Corner (C2.x, C1.y)
      setCorner2Position(Point_2(FT(newX), getCorner2Position().y()), false);
      setCorner1Position(Point_2(getCorner1Position().x(), FT(newY)), false);
      break;
    case 2:  // Dragging Corner 2
      setCorner2Position(value, false);
      break;
    case 3:  // Dragging Corner (C1.x, C2.y)
      setCorner1Position(Point_2(FT(newX), getCorner1Position().y()), false);
      setCorner2Position(Point_2(getCorner2Position().x(), FT(newY)), false);
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
sf::FloatRect Rectangle::getGlobalBounds() const { return m_sfmlShape.getGlobalBounds(); }

Point_2 Rectangle::getCGALPosition() const { return m_center; }

void Rectangle::setCGALPosition(const Point_2& newPos) {
  Vector_2 translation = newPos - getCGALPosition();
  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  setCorner1Position(flattenPoint(Point_2(c1.x() + translation.x(), c1.y() + translation.y())), false);
  setCorner2Position(flattenPoint(Point_2(c2.x() + translation.x(), c2.y() + translation.y())), false);
  m_center = flattenPoint(Point_2(m_center.x() + translation.x(), m_center.y() + translation.y()));
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
}

void Rectangle::setPosition(const sf::Vector2f& newSfmlPos) {
  // Get current position
  sf::Vector2f currentPos = m_sfmlShape.getPosition();

  // Calculate translation
  sf::Vector2f translation = newSfmlPos - currentPos;

  // Translate the shape
  Vector_2 delta(translation.x, translation.y);
  translate(delta);
}

void Rectangle::drawVertexHandles(sf::RenderWindow& window, float scale) const {
  const float handleRadius = m_vertexHandleSize * scale;
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

void Rectangle::drawLabel(sf::RenderWindow& window, const sf::View& worldView) const {
  if (!isVisible() || getLabelMode() == LabelMode::Hidden || !Point::commonFont) return;

  // 1. Delegate label drawing to the constituent corner points
  if (m_corner1) m_corner1->drawLabelExplicit(window, worldView);
  if (m_cornerB) m_cornerB->drawLabelExplicit(window, worldView);
  if (m_corner2) m_corner2->drawLabelExplicit(window, worldView);
  if (m_cornerD) m_cornerD->drawLabelExplicit(window, worldView);

  // 2. Draw the rectangle's own label (e.g., name or area) at center
  std::string labelStr = "";
  switch (getLabelMode()) {
    case LabelMode::Name: labelStr = getLabel(); break;
    case LabelMode::Value: {
       double area = m_width * std::abs(m_height);
       labelStr = std::to_string(static_cast<int>(std::round(area)));
       break;
    }
    case LabelMode::NameAndValue: {
       labelStr = getLabel();
       double area = m_width * std::abs(m_height);
       labelStr += (labelStr.empty() ? "" : " = ") + std::to_string(static_cast<int>(std::round(area)));
       break;
    }
    case LabelMode::Caption: labelStr = getCaption(); break;
    default: break;
  }

  if (labelStr.empty()) return;

  sf::Vector2i screenPos = window.mapCoordsToPixel(Point::cgalToSFML(m_center), worldView);
  
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

void Rectangle::setVertexLabelOffset(size_t vertexIndex, const sf::Vector2f& offset) {
  // Ensure vector is sized appropriately
  if (m_vertexLabelOffsets.size() < 4) {
    m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));
  }
  if (vertexIndex < 4) {
    m_vertexLabelOffsets[vertexIndex] = offset;
    // Sync to Point if exists
    if (vertexIndex == 0 && m_corner1) m_corner1->setLabelOffset(offset);
    if (vertexIndex == 1 && m_cornerB) m_cornerB->setLabelOffset(offset);
    if (vertexIndex == 2 && m_corner2) m_corner2->setLabelOffset(offset);
    if (vertexIndex == 3 && m_cornerD) m_cornerD->setLabelOffset(offset);
  }
}

sf::Vector2f Rectangle::getVertexLabelOffset(size_t vertexIndex) const {
  if (vertexIndex < m_vertexLabelOffsets.size()) {
    return m_vertexLabelOffsets[vertexIndex];
  }
  return sf::Vector2f(0.f, 0.f);
}
