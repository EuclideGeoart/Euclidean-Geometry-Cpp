#include "GeometricObject.h"
#include "ObjectPoint.h"
#include <CGAL/squared_distance_2.h>
#include <algorithm>
#include <iostream>
#include <cmath>

// Constructor for generic shapes
GeometricObject::GeometricObject(ObjectType type, const sf::Color &color, unsigned int id)
    : m_type(type), m_color(color), m_id(id), m_selected(false), m_hovered(false), m_isValid(true) {
  m_selfHandle = std::shared_ptr<GeometricObject>(this, [](GeometricObject*) {});
}

// Constructor for shapes with initial position (point-based)
GeometricObject::GeometricObject(ObjectType type, const sf::Color &color, const Point_2 &cgal_pos,
                                 unsigned int id)
    : m_type(type), m_color(color), m_id(id), m_selected(false), m_hovered(false), m_isValid(true) {
  // m_cgalPosition would be set by derived class or virtual calls
  m_selfHandle = std::shared_ptr<GeometricObject>(this, [](GeometricObject*) {});
}

void GeometricObject::setAuxObject(std::shared_ptr<GeometricObject> aux) {
  if (auto oldAux = m_auxObject.lock()) {
    oldAux->removeDependent(this);
  }

  m_auxObject = aux;

  if (aux && m_selfHandle) {
    aux->addDependent(m_selfHandle);
  }
}

void GeometricObject::setSelected(bool selected) { m_selected = selected; }

bool GeometricObject::isSelected() const { return m_selected; }

void GeometricObject::setHovered(bool hovered) { 
  m_hovered = hovered; 
  if (!hovered) m_hoveredEdgeIndex = -1;
}

bool GeometricObject::isHovered() const { return m_hovered; }

void GeometricObject::setColor(const sf::Color &color) { m_color = color; }

void GeometricObject::setLocked(bool locked) { m_locked = locked; }

bool GeometricObject::isLocked() const { return m_locked; }

void GeometricObject::move(const Vector_2 &delta) {
  // Prevent manual movement of dependent or locked objects
  if (isDependent() || isLocked()) {
    return;
  }
  translate(delta);
}

// --- Hosted ObjectPoint Management ---

void GeometricObject::addChildPoint(std::shared_ptr<ObjectPoint> point) {
  if (!point) return;
  
  // Check if already exists to avoid duplicates
  for (const auto& wp : m_hostedObjectPoints) {
    if (auto sp = wp.lock()) {
      if (sp == point) return;
    }
  }
  
  m_hostedObjectPoints.push_back(point);
  // std::cout << "GeometricObject::addChildPoint: Added point to shape " << m_id << std::endl;
}

void GeometricObject::removeChildPoint(ObjectPoint* point) {
  if (!point) return;
  
  auto newEnd = std::remove_if(m_hostedObjectPoints.begin(), m_hostedObjectPoints.end(),
                               [point](const std::weak_ptr<ObjectPoint>& wp) {
                                 if (auto sp = wp.lock()) {
                                   return sp.get() == point;
                                 }
                                 return true; // Remove expired pointers too
                               });
                               
  if (newEnd != m_hostedObjectPoints.end()) {
    m_hostedObjectPoints.erase(newEnd, m_hostedObjectPoints.end());
    // std::cout << "GeometricObject::removeChildPoint: Removed point from shape " << m_id << std::endl;
  }
}

void GeometricObject::updateHostedPoints() {
  // Notify all hosted ObjectPoints to update their positions
  std::vector<std::weak_ptr<ObjectPoint>> validPoints;
  validPoints.reserve(m_hostedObjectPoints.size());
  
  for (auto& wp : m_hostedObjectPoints) {
    if (auto sp = wp.lock()) {
      sp->updatePositionFromHost();
      validPoints.push_back(wp);
    }
  }
  
  if (validPoints.size() != m_hostedObjectPoints.size()) {
    m_hostedObjectPoints = std::move(validPoints);
  }

  // Also notify generic dependents
  notifyDependents();
}

void GeometricObject::addDependent(std::shared_ptr<GeometricObject> obj) {
    if (!obj) return;
    // Avoid self-dependency
    if (obj.get() == this) return;

    // Check if already exists
    for (const auto& wp : m_dependents) {
        if (auto sp = wp.lock()) {
            if (sp == obj) return;
        }
    }
    m_dependents.push_back(obj);
}

void GeometricObject::removeDependent(GeometricObject* obj) {
    if (!obj) return;
    m_dependents.erase(std::remove_if(m_dependents.begin(), m_dependents.end(),
        [obj](const std::weak_ptr<GeometricObject>& wp) {
            auto sp = wp.lock();
            return !sp || sp.get() == obj;
        }), m_dependents.end());
}

void GeometricObject::notifyDependents() {
  auto dependentsSnapshot = m_dependents;
  for (auto& wp : dependentsSnapshot) {
    if (auto sp = wp.lock()) {
      sp->update();
    }
  }

  m_dependents.erase(std::remove_if(m_dependents.begin(), m_dependents.end(),
    [](const std::weak_ptr<GeometricObject>& wp) {
      return wp.expired();
    }), m_dependents.end());
}

// --- NEW Zoom-Independent Halo Logic ---
void GeometricObject::drawHalo(sf::RenderTarget& target, float screenRadius) const {
    // 1. Get the current view
    const sf::View& view = target.getView();

    // 2. Calculate Scale Factor (1 pixel in screen = ? units in world)
    // We use the width ratio of the view size to the target pixel size.
    float zoomFactor = 1.0f;
    if (target.getSize().x > 0) {
        zoomFactor = view.getSize().x / static_cast<float>(target.getSize().x);
    }

    // 3. Scale the radius so it remains constant in Screen Pixels
    float worldRadius = screenRadius * zoomFactor;

    sf::CircleShape halo(worldRadius);
    halo.setOrigin(worldRadius, worldRadius);
    
    // Get position from virtual interface (Point, ObjectPoint, etc should implement this)
    Point_2 cgalPos = getCGALPosition();
    float x = static_cast<float>(CGAL::to_double(cgalPos.x()));
    float y = static_cast<float>(CGAL::to_double(cgalPos.y()));
    
    halo.setPosition(x, y);
    
    // Transparent fill
    halo.setFillColor(sf::Color::Transparent);
    
    // Use universal selection color for the halo
    halo.setOutlineColor(Constants::SELECTION_UNIVERSAL_COLOR);
    // Constant screen thickness (e.g., 2 pixels)
    halo.setOutlineThickness(2.0f * zoomFactor);
    
    target.draw(halo);
}

namespace {
Point_2 projectPointOntoSegmentCgal(const Point_2 &point, const Segment_2 &segment) {
  const Point_2 a = segment.source();
  const Point_2 b = segment.target();
  const Vector_2 ab = b - a;
  const FT ab2 = ab.squared_length();
  if (ab2 == FT(0)) {
    return a;
  }
  const Vector_2 ap = point - a;
  FT t = (ab * ap) / ab2;
  if (t < FT(0)) {
    t = FT(0);
  } else if (t > FT(1)) {
    t = FT(1);
  }
  return a + ab * t;
}
}

bool GeometricObject::getClosestPointOnPerimeter(const Point_2 &query, Point_2 &outPoint) const {
  auto segments = getBoundarySegments();
  if (segments.empty()) {
    return false;
  }

  bool hasBest = false;
  FT bestDist = FT(0);
  for (const auto &seg : segments) {
    Point_2 candidate = projectPointOntoSegmentCgal(query, seg);
    FT dist = CGAL::squared_distance(query, candidate);
    if (!hasBest || dist < bestDist) {
      bestDist = dist;
      outPoint = candidate;
      hasBest = true;
    }
  }

  return hasBest;
}

// --- Line Style Rendering Helper ---
void GeometricObject::drawStyledLine(sf::RenderWindow& window, 
                                      const sf::Vector2f& start, 
                                      const sf::Vector2f& end,
                                      LineStyle style, 
                                      float thickness, 
                                      const sf::Color& color) {
  // Solid lines use default rendering
  if (style == LineStyle::Solid) {
    sf::Vertex line[] = {sf::Vertex(start, color), sf::Vertex(end, color)};
    window.draw(line, 2, sf::Lines);
    return;
  }
  
  // 1. Map world start/end to screen pixels for dash calculation
  sf::Vector2i sPx_i = window.mapCoordsToPixel(start);
  sf::Vector2i ePx_i = window.mapCoordsToPixel(end);
  sf::Vector2f sPx(static_cast<float>(sPx_i.x), static_cast<float>(sPx_i.y));
  sf::Vector2f ePx(static_cast<float>(ePx_i.x), static_cast<float>(ePx_i.y));
  
  sf::Vector2f deltaPx = ePx - sPx;
  float lenPx = std::sqrt(deltaPx.x * deltaPx.x + deltaPx.y * deltaPx.y);
  
  if (lenPx < 1.0f) return; // Too small on screen
  
  sf::Vector2f dirPx = deltaPx / lenPx;
  sf::Vector2f normPx(-dirPx.y, dirPx.x);

  // 1.1 Calculate World-to-Pixel scale for normalization
  // We need to know how large one screen pixel is in world units to set the correct radii
  sf::Vector2f worldP0 = window.mapPixelToCoords(sf::Vector2i(0, 0));
  sf::Vector2f worldP1_x = window.mapPixelToCoords(sf::Vector2i(1, 0));
  sf::Vector2f worldP1_y = window.mapPixelToCoords(sf::Vector2i(0, 1));
  float worldUnitPerPixelX = std::abs(worldP1_x.x - worldP0.x);
  float worldUnitPerPixelY = std::abs(worldP1_y.y - worldP0.y);
  float worldUnitPerPixel = std::max(worldUnitPerPixelX, worldUnitPerPixelY);
  
  // 2. High-Quality Rendering Patterns
  if (style == LineStyle::Dotted) {
    // Premium Dotted: Circular dots
    float radiusPx = thickness * 0.6f; 
    float radiusWorld = radiusPx * worldUnitPerPixel;
    float spacingPx = std::max(radiusPx * 4.0f, 6.0f);
    
    sf::CircleShape dot(radiusWorld);
    dot.setOrigin(radiusWorld, radiusWorld);
    dot.setFillColor(color);
    
    for (float d = 0; d <= lenPx; d += spacingPx) {
      sf::Vector2f posPx = sPx + dirPx * d;
      dot.setPosition(window.mapPixelToCoords(sf::Vector2i(std::round(posPx.x), std::round(posPx.y))));
      window.draw(dot);
    }
  } 
  else if (style == LineStyle::Dashed) {
    // Premium Dashed: Pill-shaped dashes (quad + 2 rounded caps)
    float dashLenPx = 16.0f;
    float gapLenPx = 10.0f;
    float cycleLenPx = dashLenPx + gapLenPx;
    
    float radiusPx = thickness * 0.5f;
    float radiusWorld = radiusPx * worldUnitPerPixel;
    sf::CircleShape cap(radiusWorld);
    cap.setOrigin(radiusWorld, radiusWorld);
    cap.setFillColor(color);
    
    std::vector<sf::Vertex> vertices;
    vertices.reserve(static_cast<size_t>((lenPx / cycleLenPx) * 4 + 4));
    
    for (float d = 0; d < lenPx; d += cycleLenPx) {
      float endD = std::min(d + dashLenPx, lenPx);
      if (endD <= d) break;
      
      sf::Vector2f p1Px = sPx + dirPx * d;
      sf::Vector2f p2Px = sPx + dirPx * endD;
      
      // Map pixel positions to world for the quad
      sf::Vector2f p1World = window.mapPixelToCoords(sf::Vector2i(std::round(p1Px.x), std::round(p1Px.y)));
      sf::Vector2f p2World = window.mapPixelToCoords(sf::Vector2i(std::round(p2Px.x), std::round(p2Px.y)));
      
      // Get world-space direction for the quad offsets
      sf::Vector2f dirWorld = p2World - p1World;
      float lenWorld = std::sqrt(dirWorld.x * dirWorld.x + dirWorld.y * dirWorld.y);
      if (lenWorld < 1e-6f) continue;
      dirWorld /= lenWorld;
      sf::Vector2f normWorld(-dirWorld.y, dirWorld.x);
      
      sf::Vector2f halfNormWorld = normWorld * radiusWorld;
      
      vertices.push_back(sf::Vertex(p1World + halfNormWorld, color));
      vertices.push_back(sf::Vertex(p2World + halfNormWorld, color));
      vertices.push_back(sf::Vertex(p2World - halfNormWorld, color));
      vertices.push_back(sf::Vertex(p1World - halfNormWorld, color));
      
      // Draw Round Caps
      cap.setPosition(p1World);
      window.draw(cap);
      cap.setPosition(p2World);
      window.draw(cap);
    }
    
    if (!vertices.empty()) {
      window.draw(vertices.data(), vertices.size(), sf::Quads);
    }
  }
}

