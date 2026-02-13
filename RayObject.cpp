#include "RayObject.h"
#include "Constants.h"
#include <cmath>

void RayObject::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!forceVisible && !isVisible()) return;

  sf::Vector2f p1 = getStartPointObjectShared() ? getStartPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f p2 = getEndPointObjectShared() ? getEndPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f dir = p2 - p1;
  float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  if (len < 1e-6f) return;
  dir /= len;

  float extent = std::max(10000.0f, 1000.0f * scale);
  sf::Vector2f farPoint = p1 + dir * extent;

  float pixelThickness = std::max(1.0f, std::round(m_thickness));
  if (isSelected()) pixelThickness += 2.0f;
  else if (isHovered()) pixelThickness += 1.0f;

  GeometricObject::drawStyledLine(window, p1, farPoint, m_lineStyle, pixelThickness, getColor());

  if (isSelected()) {
    sf::CircleShape marker(Constants::POINT_RADIUS_SELECTED);
    marker.setFillColor(Constants::SELECTION_COLOR_POINT_OUTLINE);
    marker.setOrigin(Constants::POINT_RADIUS_SELECTED, Constants::POINT_RADIUS_SELECTED);
    marker.setPosition(p1);
    window.draw(marker);
  }
}

bool RayObject::contains(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  sf::Vector2f p1 = getStartPointObjectShared() ? getStartPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f p2 = getEndPointObjectShared() ? getEndPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f dir = p2 - p1;
  float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  if (len < 1e-6f) return false;
  dir /= len;

  sf::Vector2f v = worldPos_sfml - p1;
  float t = v.x * dir.x + v.y * dir.y;
  if (t < 0.0f) return false;

  sf::Vector2f proj = p1 + dir * t;
  float dx = worldPos_sfml.x - proj.x;
  float dy = worldPos_sfml.y - proj.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist <= tolerance;
}

std::vector<Segment_2> RayObject::getEdges() const {
  std::vector<Segment_2> edges;
  sf::Vector2f p1 = getStartPointObjectShared() ? getStartPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f p2 = getEndPointObjectShared() ? getEndPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f dir = p2 - p1;
  float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  if (len < 1e-6f) return edges;
  dir /= len;

  float extent = 10000.0f;
  sf::Vector2f farPoint = p1 + dir * extent;
  edges.emplace_back(Point_2(p1.x, p1.y), Point_2(farPoint.x, farPoint.y));
  return edges;
}
