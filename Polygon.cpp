#include "Polygon.h"
#include "Constants.h"
#include "Point.h"
#include <iostream>
#include "Line.h"
#include "Circle.h"
#include <cmath>
#include "Types.h" // Ensure this is included for flattenPoint
#include "PointUtils.h"

Polygon::Polygon(const std::vector<Point_2>& vertices, const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::Polygon, color, id) {
  m_color.a = 0; // Default transparent fill
  m_vertices.reserve(vertices.size());
  for (const auto& v : vertices) {
    // Flatten initial vertices just to be safe (though usually they are fresh)
    auto pt = std::make_shared<Point>(flattenPoint(v), 1.0f);
    pt->setCreatedWithShape(true);
    m_vertices.push_back(pt);
  }
  updateSFMLShape();
  setShowLabel(true);
}

Polygon::Polygon(const std::vector<std::shared_ptr<Point>> &vertices, const sf::Color &color,
                 unsigned int id)
    : GeometricObject(ObjectType::Polygon, color, id), m_vertices(vertices) {
  m_color.a = 0; // Default transparent fill
  for (auto& v : m_vertices) {
    if (v) v->addDependent(m_selfHandle);
  }
  updateSFMLShape();
  setShowLabel(true);
}

void Polygon::addVertex(const Point_2 &vertex) {
  m_vertices.push_back(std::make_shared<Point>(flattenPoint(vertex), 1.0f));
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
  if ((!isVisible() && !forceVisible) || m_vertices.size() < 3) return;

  sf::ConvexShape shape = m_sfmlShape;
  shape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);

  // GHOST MODE: Apply transparency if hidden but forced visible
  if (!isVisible() && forceVisible) {
      sf::Color ghostFill = shape.getFillColor();
      ghostFill.a = 50; // Faint alpha
      shape.setFillColor(ghostFill);
      
      sf::Color ghostOutline = shape.getOutlineColor();
      ghostOutline.a = 50;
      shape.setOutlineColor(ghostOutline);
  }

  window.draw(shape);

  // Delegate drawing to the constituent points
  for (auto& pt : m_vertices) {
      if (pt) pt->draw(window, scale, forceVisible);
  }

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
        auto sfmlVerts = getVerticesSFML();
        int idx = getHoveredEdge();
        if (idx >= 0 && idx < static_cast<int>(sfmlVerts.size())) {
            sf::Vector2f p1 = sfmlVerts[idx];
            sf::Vector2f p2 = sfmlVerts[(idx + 1) % sfmlVerts.size()];
            
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

void Polygon::drawLabel(sf::RenderWindow& window, const sf::View& worldView) const {
  if (!isVisible() || getLabelMode() == LabelMode::Hidden || !Point::commonFont) return;

  // 1. Draw labels of constituent points
  for (auto& pt : m_vertices) {
    if (pt) pt->drawLabelExplicit(window, worldView);
  }

  // 2. Draw polygon's own label at centroid
  std::string labelStr = "";
  switch (getLabelMode()) {
    case LabelMode::Name: labelStr = getLabel(); break;
    case LabelMode::Value: {
      std::vector<Point_2> verts = getVertices();
      if (verts.size() >= 3) {
        double area = 0.0;
        for (size_t i = 0; i < verts.size(); ++i) {
          size_t next = (i + 1) % verts.size();
          area += CGAL::to_double(verts[i].x() * verts[next].y() - verts[next].x() * verts[i].y());
        }
        area = 0.5 * std::abs(area);
        labelStr = std::to_string(static_cast<int>(std::round(area)));
      }
      break;
    }
    case LabelMode::NameAndValue: {
      labelStr = getLabel();
      std::vector<Point_2> verts = getVertices();
      if (verts.size() >= 3) {
        double area = 0.0;
        for (size_t i = 0; i < verts.size(); ++i) {
          size_t next = (i + 1) % verts.size();
          area += CGAL::to_double(verts[i].x() * verts[next].y() - verts[next].x() * verts[i].y());
        }
        area = 0.5 * std::abs(area);
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

void Polygon::update() {
  if (isDependent()) {
    updateDependentShape();
  } else {
    updateSFMLShape();
    updateHostedPoints();
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
    // FIX 2: Flatten points during translation to prevent lazy evaluation stack overflow
    Point_2 newPos = pos + translation;
    v->setCGALPosition(flattenPoint(newPos));
  }
  updateSFMLShape();
  updateHostedPoints();
}

void Polygon::updateDependentShape() {
  auto parent = m_parentSource.lock();
  if (!parent || !parent->isValid()) {
    updateSFMLShape();
    updateHostedPoints();
    setVisible(false);
    return;
  }

  auto sourcePoly = std::dynamic_pointer_cast<Polygon>(parent);
  if (!sourcePoly) {
    updateSFMLShape();
    updateHostedPoints();
    return;
  }

  auto sourceVerts = sourcePoly->getVertices();
  if (sourceVerts.empty()) return;

  auto aux = m_auxObject.lock();

  // Helper lambda for math
  auto transformPoint = [&](const Point_2& p) -> std::optional<Point_2> {
    switch (m_transformType) {
      case TransformationType::Reflect: {
        // Reflect across Line
        if (aux && (aux->getType() == ObjectType::Line || aux->getType() == ObjectType::LineSegment || aux->getType() == ObjectType::Ray)) {
             auto line = std::dynamic_pointer_cast<Line>(aux);
             if (!line || !line->isValid()) return std::nullopt;
             
             Point_2 a = line->getStartPoint();
             Point_2 b = line->getEndPoint();
             Vector_2 ab = b - a;
             double abLenSq = CGAL::to_double(ab.squared_length());
             if (abLenSq < 1e-12) return p;

             Vector_2 ap = p - a;
             FT t = (ap * ab) / ab.squared_length();
             Point_2 h = a + ab * t;
             return p + (h - p) * FT(2.0);
        }
        return std::nullopt;
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
        if (aux && (aux->getType() == ObjectType::Line || aux->getType() == ObjectType::LineSegment ||
                    aux->getType() == ObjectType::Ray || aux->getType() == ObjectType::Vector)) {
             auto line = std::dynamic_pointer_cast<Line>(aux);
             if (line && line->isValid()) {
                delta = line->getEndPoint() - line->getStartPoint();
             }
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

    for (auto& v : m_vertices) {
      if (v) v->setDeferConstraintUpdates(true);
    }

    for (size_t i = 0; i < sourceVerts.size(); ++i) {
      if (i >= m_vertices.size()) break;
      if (!m_vertices[i]) continue;
      
      auto newPos = transformPoint(sourceVerts[i]);
      if (newPos.has_value()) {
          // CRITICAL FIX: Flatten vertex position to prevent Stack Overflow
          // This breaks the lazy evaluation history chain.
          m_vertices[i]->setCGALPosition(flattenPoint(*newPos));
      }
  }

  for (auto& v : m_vertices) {
      if (v) v->forceConstraintUpdate();
  }

  updateSFMLShape();
  updateHostedPoints();
  if (!hasVisibilityUserOverride()) {
    setVisible(parent->isVisible());
  }
}

void Polygon::setVertexPosition(size_t index, const Point_2 &value) {
  if (index >= m_vertices.size()) return;
  if (!m_vertices[index]) return;
  m_vertices[index]->setCGALPosition(flattenPoint(value)); // Fix here too
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
    // Already double-based (flat), safe.
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
    // FIX 3: Flatten here
    v->setCGALPosition(flattenPoint(pos + translation));
  }
  
  updateSFMLShape();
  updateHostedPoints();
}

void Polygon::setPosition(const sf::Vector2f &newSfmlPos) {
  sf::Vector2f currentPos = m_sfmlShape.getPosition();
  sf::Vector2f translation = newSfmlPos - currentPos;
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