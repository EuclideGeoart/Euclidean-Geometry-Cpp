#include "RegularPolygon.h"
#include "Point.h"
#include "Line.h"
#include "Circle.h"
#include "PointUtils.h"
#include "Types.h"
#include <cmath>
RegularPolygon::RegularPolygon(const Point_2 &center, const Point_2 &firstVertex, int numSides,
                               const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RegularPolygon, color, id),
  m_creationMode(CreationMode::CenterAndVertex),
      m_centerPoint(std::make_shared<Point>(center, 1.0f)),
      m_firstVertexPoint(std::make_shared<Point>(firstVertex, 1.0f)),
      m_numSides(numSides),
      m_rotationAngle(0) {
  m_color.a = 0; // Default transparent fill
  if (m_numSides < 3) m_numSides = 3;

  double dx = CGAL::to_double(firstVertex.x()) - CGAL::to_double(center.x());
  double dy = CGAL::to_double(firstVertex.y()) - CGAL::to_double(center.y());
  m_radius = std::sqrt(dx * dx + dy * dy);
  m_rotationAngle = std::atan2(dy, dx);

  generateVertices();
  updateSFMLShape();
  setShowLabel(true);
}

RegularPolygon::RegularPolygon(const std::shared_ptr<Point> &center,
                               const std::shared_ptr<Point> &firstVertex, int numSides,
                               const sf::Color &color, unsigned int id)
    : GeometricObject(ObjectType::RegularPolygon, color, id),
  m_creationMode(CreationMode::CenterAndVertex),
      m_centerPoint(center ? center : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_firstVertexPoint(firstVertex ? firstVertex
                                     : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_numSides(numSides),
      m_rotationAngle(0) {
  m_color.a = 0; // Default transparent fill
  if (m_numSides < 3) m_numSides = 3;

  if (m_centerPoint) m_centerPoint->addDependent(m_selfHandle);
  if (m_firstVertexPoint) m_firstVertexPoint->addDependent(m_selfHandle);

  Point_2 centerPos = m_centerPoint->getCGALPosition();
  Point_2 firstPos = m_firstVertexPoint->getCGALPosition();
  double dx = CGAL::to_double(firstPos.x()) - CGAL::to_double(centerPos.x());
  double dy = CGAL::to_double(firstPos.y()) - CGAL::to_double(centerPos.y());
  m_radius = std::sqrt(dx * dx + dy * dy);
  m_rotationAngle = std::atan2(dy, dx);

  generateVertices();
  updateSFMLShape();
  setShowLabel(true);
}

RegularPolygon::RegularPolygon(const std::shared_ptr<Point> &edgeStart,
                               const std::shared_ptr<Point> &edgeEnd, int numSides,
                               const sf::Color &color, unsigned int id,
                               CreationMode mode)
    : GeometricObject(ObjectType::RegularPolygon, color, id),
      m_creationMode(mode),
      m_centerPoint(std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_firstVertexPoint(std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_edgeStartPoint(edgeStart ? edgeStart : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_edgeEndPoint(edgeEnd ? edgeEnd : std::make_shared<Point>(Point_2(FT(1), FT(0)), 1.0f)),
      m_numSides(numSides),
      m_rotationAngle(0) {
  if (m_creationMode != CreationMode::Edge) {
    m_creationMode = CreationMode::CenterAndVertex;
  }

  m_color.a = 0;
  if (m_numSides < 3) m_numSides = 3;

  if (m_edgeStartPoint) m_edgeStartPoint->addDependent(m_selfHandle);
  if (m_edgeEndPoint) m_edgeEndPoint->addDependent(m_selfHandle);

  generateVertices();
  ensureDerivedVertices();
  syncDerivedVertices();
  updateSFMLShape();
  setShowLabel(true);
}

void RegularPolygon::ensureDerivedVertices() {
  size_t required = 0;
  if (m_creationMode == CreationMode::Edge) {
    if (m_vertices.size() > 2) required = m_vertices.size() - 2;
  } else {
    if (m_vertices.size() > 1) required = m_vertices.size() - 1;
  }

  while (m_derivedVertices.size() < required) {
    auto pt = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
    pt->setLocked(true);
    pt->setDependent(true);
    pt->setShowLabel(true);
    pt->setVisible(true);
    m_derivedVertices.push_back(pt);
  }

  if (m_derivedVertices.size() > required) {
    m_derivedVertices.resize(required);
  }
}

void RegularPolygon::syncDerivedVertices() {
  ensureDerivedVertices();

  size_t offset = (m_creationMode == CreationMode::Edge) ? 2u : 1u;
  for (size_t i = 0; i < m_derivedVertices.size(); ++i) {
    auto& pt = m_derivedVertices[i];
    if (!pt) continue;
    size_t vertexIndex = offset + i;
    if (vertexIndex >= m_vertices.size()) continue;
    pt->setLocked(true);
    pt->setDependent(true);
    pt->setShowLabel(true);
    pt->setVisible(true);
    pt->setCGALPosition(m_vertices[vertexIndex]);
  }
}

void RegularPolygon::setDerivedVertices(const std::vector<std::shared_ptr<Point>>& derivedVertices) {
  m_derivedVertices.clear();
  m_derivedVertices.reserve(derivedVertices.size());

  for (const auto& pt : derivedVertices) {
    if (!pt) continue;
    pt->setLocked(true);
    pt->setDependent(true);
    pt->setShowLabel(true);
    pt->setVisible(true);
    pt->addDependent(m_selfHandle);
    m_derivedVertices.push_back(pt);
  }

  ensureDerivedVertices();
  syncDerivedVertices();
}

void RegularPolygon::generateVertices() {
    m_vertices.clear();

  if (m_creationMode == CreationMode::Edge) {
    if (!m_edgeStartPoint || !m_edgeEndPoint || m_numSides < 3) return;

    Point_2 start = m_edgeStartPoint->getCGALPosition();
    Point_2 end = m_edgeEndPoint->getCGALPosition();

    double sx = CGAL::to_double(start.x());
    double sy = CGAL::to_double(start.y());
    double ex = CGAL::to_double(end.x());
    double ey = CGAL::to_double(end.y());

    double edgeX = ex - sx;
    double edgeY = ey - sy;
    double edgeLen = std::sqrt(edgeX * edgeX + edgeY * edgeY);
    if (edgeLen < 1e-12) return;

    double exterior = 2.0 * Constants::PI / static_cast<double>(m_numSides);
    double cosA = std::cos(exterior);
    double sinA = std::sin(exterior);

    m_vertices.reserve(static_cast<size_t>(m_numSides));
    m_vertices.push_back(start);
    m_vertices.push_back(end);

    Point_2 current = end;
    double curEdgeX = edgeX;
    double curEdgeY = edgeY;

    for (int i = 2; i < m_numSides; ++i) {
      double nextEdgeX = curEdgeX * cosA - curEdgeY * sinA;
      double nextEdgeY = curEdgeX * sinA + curEdgeY * cosA;

      current = Point_2(FT(CGAL::to_double(current.x()) + nextEdgeX),
                FT(CGAL::to_double(current.y()) + nextEdgeY));
      m_vertices.push_back(current);

      curEdgeX = nextEdgeX;
      curEdgeY = nextEdgeY;
    }

    double cx = 0.0;
    double cy = 0.0;
    for (const auto& v : m_vertices) {
      cx += CGAL::to_double(v.x());
      cy += CGAL::to_double(v.y());
    }
    cx /= static_cast<double>(m_vertices.size());
    cy /= static_cast<double>(m_vertices.size());

    if (m_centerPoint) {
      m_centerPoint->setCGALPosition(Point_2(FT(cx), FT(cy)));
    }
    if (m_firstVertexPoint && !m_vertices.empty()) {
      m_firstVertexPoint->setCGALPosition(m_vertices[0]);
    }

    m_radius = std::sqrt(CGAL::to_double(CGAL::squared_distance(m_vertices[0], Point_2(FT(cx), FT(cy)))));
    m_rotationAngle = std::atan2(CGAL::to_double(m_vertices[0].y()) - cy,
                   CGAL::to_double(m_vertices[0].x()) - cx);
    ensureDerivedVertices();
    syncDerivedVertices();
    return;
  }
    
    // 1. If we are Dependent/Transformed, we rely on the specific construction logic
    // However, RegularPolygon usually just stores Center+Vertex.
    // If you want it to behave like Polygon, you should have stored ALL vertices 
    // in the serializer step (Pass 2d).
    
    // CHECK: Did you implement the RegularPolygon loader to load "vertexIds" list?
    // If not, it relies on Center+Vertex.
    
    if (m_centerPoint && m_firstVertexPoint) {
        Point_2 center = m_centerPoint->getCGALPosition();
        Point_2 start = m_firstVertexPoint->getCGALPosition();
        
        // Recalculate based on current live positions of Center and Vertex
        double dx = CGAL::to_double(start.x() - center.x());
        double dy = CGAL::to_double(start.y() - center.y());
        double radius = std::sqrt(dx*dx + dy*dy);
        double startAngle = std::atan2(dy, dx);
        
        for (int i = 0; i < m_numSides; ++i) {
            double theta = startAngle + 2.0 * Constants::PI * i / m_numSides;
            double px = CGAL::to_double(center.x()) + radius * std::cos(theta);
            double py = CGAL::to_double(center.y()) + radius * std::sin(theta);
            m_vertices.push_back(Point_2(px, py));
        }
    }

      ensureDerivedVertices();
      syncDerivedVertices();
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
    m_sfmlShape.setPoint(i, sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
  }

  m_sfmlShape.setFillColor(m_color);
  m_sfmlShape.setOutlineThickness(m_thickness);
  m_sfmlShape.setOutlineColor(sf::Color::Black);
}

void RegularPolygon::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!isVisible() && !forceVisible) return;

  sf::ConvexShape shape = m_sfmlShape;
  
  // If styled (dashed/dotted), we disable the SFML outline and draw manually
  bool isStyled = (m_lineStyle != LineStyle::Solid);
  if (isStyled) {
      shape.setOutlineThickness(0);
  } else {
      shape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);
  }

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

  // --- Styled Outline Rendering ---
  if (isStyled) {
      auto sfmlVerts = getVerticesSFML();
      if (!sfmlVerts.empty()) {
          sf::Color drawOutlineColor = m_sfmlShape.getOutlineColor();
          float baseThickness = m_thickness;
          if (isSelected()) baseThickness += 2.0f;
          else if (isHovered()) baseThickness += 1.0f;
          float pixelThickness = std::round(baseThickness);
          if (pixelThickness < 1.0f) pixelThickness = 1.0f;

          size_t n = sfmlVerts.size();
          for (size_t i = 0; i < n; ++i) {
              sf::Vector2f p1 = sfmlVerts[i];
              sf::Vector2f p2 = sfmlVerts[(i + 1) % n];
              GeometricObject::drawStyledLine(window, p1, p2, m_lineStyle, pixelThickness, drawOutlineColor);
          }
      }
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

  // Delegate drawing to key points
  if (m_creationMode == CreationMode::Edge) {
    if (m_edgeStartPoint) m_edgeStartPoint->draw(window, scale, forceVisible);
    if (m_edgeEndPoint) m_edgeEndPoint->draw(window, scale, forceVisible);
  } else {
    if (m_centerPoint) m_centerPoint->draw(window, scale, forceVisible);
    if (m_firstVertexPoint) m_firstVertexPoint->draw(window, scale, forceVisible);
  }

  drawVertexHandles(window, scale);
}

// RegularPolygon-specific drawing passers for labels
void RegularPolygon::drawLabel(sf::RenderWindow &window, const sf::View &worldView) const {
  if (!isVisible()) return;

  if (m_creationMode == CreationMode::Edge) {
    if (m_edgeStartPoint) m_edgeStartPoint->drawLabelExplicit(window, worldView);
    if (m_edgeEndPoint) m_edgeEndPoint->drawLabelExplicit(window, worldView);
  } else {
    if (m_centerPoint) m_centerPoint->drawLabelExplicit(window, worldView);
    if (m_firstVertexPoint) m_firstVertexPoint->drawLabelExplicit(window, worldView);
  }

  for (const auto& p : m_derivedVertices) {
    if (p && p->isValid()) p->drawLabelExplicit(window, worldView);
  }

  if (getLabelMode() == LabelMode::Hidden) return;
  

  // 2. Draw polygon's own label at center
  std::string labelStr = "";
  switch (getLabelMode()) {
    case LabelMode::Name: labelStr = getLabel(); break;
    case LabelMode::Value: {
       // Area of regular polygon: 0.5 * n * R^2 * sin(2*pi/n)
       double n = static_cast<double>(m_numSides);
       double R = m_radius;
       double area = 0.5 * n * R * R * std::sin(2.0 * 3.141592653589793 / n);
       labelStr = std::to_string(static_cast<int>(std::round(area)));
       break;
    }
    case LabelMode::NameAndValue: {
       labelStr = getLabel();
       double n = static_cast<double>(m_numSides);
       double R = m_radius;
       double area = 0.5 * n * R * R * std::sin(2.0 * 3.141592653589793 / n);
       labelStr += (labelStr.empty() ? "" : " = ") + std::to_string(static_cast<int>(std::round(area)));
       break;
    }
    case LabelMode::Caption: labelStr = getCaption(); break;
    default: break;
  }

  if (labelStr.empty()) return;

  Point_2 center = getCenter();
  sf::Vector2i screenPos = window.mapCoordsToPixel(Point::cgalToSFML(center), worldView);
  
  sf::Text text;
  text.setFont(LabelManager::instance().getSelectedFont());
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

void RegularPolygon::update() {
  // Recursion guard: prevent infinite loops when points notify us
  if (m_isUpdating) return;
  m_isUpdating = true;

  if (isDependent()) {
    updateDependentShape();
  } else {
    if (m_creationMode == CreationMode::Edge) {
      updateSFMLShape();
    } else {
      updateSFMLShape();
    }
    updateHostedPoints();
  }

  m_isUpdating = false;
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

  if (m_creationMode == CreationMode::Edge && sourcePoly->getCreationMode() == CreationMode::Edge) {
    auto srcEdgeStart = sourcePoly->getEdgeStartPoint();
    auto srcEdgeEnd = sourcePoly->getEdgeEndPoint();
    if (!srcEdgeStart || !srcEdgeEnd) {
      setVisible(false);
      return;
    }

    auto newEdgeStart = transformPoint(srcEdgeStart->getCGALPosition());
    auto newEdgeEnd = transformPoint(srcEdgeEnd->getCGALPosition());
    if (!newEdgeStart.has_value() || !newEdgeEnd.has_value()) {
      setVisible(false);
      return;
    }

    if (m_edgeStartPoint) m_edgeStartPoint->setDeferConstraintUpdates(true);
    if (m_edgeEndPoint) m_edgeEndPoint->setDeferConstraintUpdates(true);

    if (m_edgeStartPoint) m_edgeStartPoint->setCGALPosition(flattenPoint(*newEdgeStart));
    if (m_edgeEndPoint) m_edgeEndPoint->setCGALPosition(flattenPoint(*newEdgeEnd));

    if (m_edgeStartPoint) m_edgeStartPoint->forceConstraintUpdate();
    if (m_edgeEndPoint) m_edgeEndPoint->forceConstraintUpdate();
  } else {
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

    if (m_centerPoint) m_centerPoint->setDeferConstraintUpdates(true);
    if (m_firstVertexPoint) m_firstVertexPoint->setDeferConstraintUpdates(true);

    if (m_centerPoint) m_centerPoint->setCGALPosition(flattenPoint(*newCenter));
    if (m_firstVertexPoint) m_firstVertexPoint->setCGALPosition(flattenPoint(*newFirst));

    if (m_centerPoint) m_centerPoint->forceConstraintUpdate();
    if (m_firstVertexPoint) m_firstVertexPoint->forceConstraintUpdate();
  }

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
  auto translatePoint = [&](const std::shared_ptr<Point>& p) {
    if (!p) return;
    Point_2 pos = p->getCGALPosition();
    p->setCGALPosition(flattenPoint(Point_2(pos.x() + translation.x(), pos.y() + translation.y())));
  };

  if (m_creationMode == CreationMode::Edge) {
    translatePoint(m_edgeStartPoint);
    translatePoint(m_edgeEndPoint);
  } else {
    translatePoint(m_centerPoint);
    translatePoint(m_firstVertexPoint);
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
  sf::Color defaultHandleColor(m_color.r, m_color.g, m_color.b, 255);

  if (m_creationMode == CreationMode::Edge) {
    if (m_edgeStartPoint) {
      sf::CircleShape handle(handleRadius);
      handle.setOrigin(handleRadius, handleRadius);
      Point_2 p = m_edgeStartPoint->getCGALPosition();
      handle.setPosition(static_cast<float>(CGAL::to_double(p.x())), static_cast<float>(CGAL::to_double(p.y())));
      sf::Color base = defaultHandleColor;
      if (m_activeVertex == 0) {
        base = sf::Color(255, 140, 0);
      } else if (m_hoveredVertex == 0) {
        base = sf::Color::Yellow;
      }
      handle.setFillColor(base);
      handle.setOutlineThickness(1.0f * scale);
      handle.setOutlineColor(sf::Color::Black);
      window.draw(handle);
    }

    if (m_edgeEndPoint) {
      sf::CircleShape handle(handleRadius);
      handle.setOrigin(handleRadius, handleRadius);
      Point_2 p = m_edgeEndPoint->getCGALPosition();
      handle.setPosition(static_cast<float>(CGAL::to_double(p.x())), static_cast<float>(CGAL::to_double(p.y())));
      sf::Color base = defaultHandleColor;
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

    for (size_t i = 0; i < m_vertices.size(); ++i) {
      sf::CircleShape handle(handleRadius * 0.6f);
      handle.setOrigin(handleRadius * 0.6f, handleRadius * 0.6f);
      handle.setPosition(static_cast<float>(CGAL::to_double(m_vertices[i].x())),
                         static_cast<float>(CGAL::to_double(m_vertices[i].y())));
      handle.setFillColor(sf::Color(defaultHandleColor.r, defaultHandleColor.g, defaultHandleColor.b, 128));
      handle.setOutlineThickness(0.5f * scale);
      handle.setOutlineColor(sf::Color(100, 100, 100));
      window.draw(handle);
    }
    return;
  }
  
  // Draw center point (creation point 0)
  {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    Point_2 centerPos = getCenter();
    float x = static_cast<float>(CGAL::to_double(centerPos.x()));
    float y = static_cast<float>(CGAL::to_double(centerPos.y()));
    handle.setPosition(x, y);
    
    sf::Color base = defaultHandleColor;
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
    
    sf::Color base = defaultHandleColor;
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
    handle.setFillColor(sf::Color(defaultHandleColor.r, defaultHandleColor.g, defaultHandleColor.b, 128));
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

  auto rotatePointInPlace = [&](const std::shared_ptr<Point>& p) {
    if (!p) return;
    Point_2 pos = p->getCGALPosition();
    double x = CGAL::to_double(pos.x()) - centerX;
    double y = CGAL::to_double(pos.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    p->setCGALPosition(Point_2(FT(newX), FT(newY)));
  };

  if (m_creationMode == CreationMode::Edge) {
    rotatePointInPlace(m_edgeStartPoint);
    rotatePointInPlace(m_edgeEndPoint);
  } else {
    rotatePointInPlace(m_centerPoint);
    rotatePointInPlace(m_firstVertexPoint);
  }

  updateSFMLShape();
  updateHostedPoints();
}

void RegularPolygon::setNumSides(int numSides) {
  if (numSides >= 3) {
    m_numSides = numSides;
    generateVertices();
    ensureDerivedVertices();
    syncDerivedVertices();
    updateSFMLShape();
    updateHostedPoints();
  }
}

void RegularPolygon::setRadius(double radius) {
  if (m_creationMode == CreationMode::Edge) return;

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

  if (m_creationMode == CreationMode::Edge) {
    if (m_edgeStartPoint) {
      Point_2 p = m_edgeStartPoint->getCGALPosition();
      pts.emplace_back(static_cast<float>(CGAL::to_double(p.x())), static_cast<float>(CGAL::to_double(p.y())));
    }
    if (m_edgeEndPoint) {
      Point_2 p = m_edgeEndPoint->getCGALPosition();
      pts.emplace_back(static_cast<float>(CGAL::to_double(p.x())), static_cast<float>(CGAL::to_double(p.y())));
    }
    return pts;
  }
  
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
  if (m_creationMode == CreationMode::Edge) {
    if (index == 0 && m_edgeStartPoint && m_edgeEndPoint) {
      Point_2 oldStart = m_edgeStartPoint->getCGALPosition();
      Vector_2 delta = value - oldStart;
      m_edgeStartPoint->setCGALPosition(value);
      Point_2 endPos = m_edgeEndPoint->getCGALPosition();
      m_edgeEndPoint->setCGALPosition(Point_2(endPos.x() + delta.x(), endPos.y() + delta.y()));
    } else if (index >= 1 && m_edgeEndPoint) {
      m_edgeEndPoint->setCGALPosition(value);
    }

    generateVertices();
    updateSFMLShape();
    updateHostedPoints();
    return;
  }

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
  if (m_creationMode == CreationMode::Edge && !m_vertices.empty()) {
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& v : m_vertices) {
      cx += CGAL::to_double(v.x());
      cy += CGAL::to_double(v.y());
    }
    cx /= static_cast<double>(m_vertices.size());
    cy /= static_cast<double>(m_vertices.size());
    return Point_2(FT(cx), FT(cy));
  }

  if (m_centerPoint) {
    return m_centerPoint->getCGALPosition();
  }
  return Point_2(FT(0), FT(0));
}

Point_2 RegularPolygon::getCGALPosition() const {
  return getCenter();
}

std::vector<Point_2> RegularPolygon::getInteractableVertices() const {
  // Return defining points + derived points for smart snapping/reference
  std::vector<Point_2> result;
  if (m_creationMode == CreationMode::Edge) {
    result.reserve(2 + m_derivedVertices.size());
    if (m_edgeStartPoint) result.push_back(m_edgeStartPoint->getCGALPosition());
    if (m_edgeEndPoint) result.push_back(m_edgeEndPoint->getCGALPosition());
  } else {
    result.reserve(2 + m_derivedVertices.size());
    if (m_centerPoint) result.push_back(m_centerPoint->getCGALPosition());
    if (m_firstVertexPoint) result.push_back(m_firstVertexPoint->getCGALPosition());
  }

  for (const auto& p : m_derivedVertices) {
    if (p && p->isValid()) {
      result.push_back(p->getCGALPosition());
    }
  }
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