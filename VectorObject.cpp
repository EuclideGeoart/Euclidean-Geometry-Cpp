#include "VectorObject.h"
#include <algorithm>
#include <cmath>

void VectorObject::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!forceVisible && !isVisible()) return;

  sf::Vector2f p1 = getStartPointObjectShared() ? getStartPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f p2 = getEndPointObjectShared() ? getEndPointObjectShared()->getSFMLPosition() : sf::Vector2f();

  sf::VertexArray line(sf::Lines, 2);
  line[0] = sf::Vertex(p1, getColor());
  line[1] = sf::Vertex(p2, getColor());
  window.draw(line);

  sf::Vector2f dir = p2 - p1;
  float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  if (len > 1e-6f) {
    dir /= len;
    sf::Vector2f perp(-dir.y, dir.x);
    float arrowLen = std::max(12.0f, 8.0f * scale);
    float arrowWidth = std::max(6.0f, 4.0f * scale);

    sf::Vector2f tip = p2;
    sf::Vector2f left = tip - dir * arrowLen + perp * arrowWidth;
    sf::Vector2f right = tip - dir * arrowLen - perp * arrowWidth;

    sf::VertexArray arrow(sf::Triangles, 3);
    arrow[0] = sf::Vertex(tip, getColor());
    arrow[1] = sf::Vertex(left, getColor());
    arrow[2] = sf::Vertex(right, getColor());
    window.draw(arrow);
  }

  if (isSelected()) {
    sf::CircleShape marker(Constants::POINT_RADIUS_SELECTED);
    marker.setFillColor(Constants::SELECTION_COLOR_POINT_OUTLINE);
    marker.setOrigin(Constants::POINT_RADIUS_SELECTED, Constants::POINT_RADIUS_SELECTED);
    marker.setPosition(p1);
    window.draw(marker);
  }
}


bool VectorObject::contains(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  sf::Vector2f p1 = getStartPointObjectShared() ? getStartPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f p2 = getEndPointObjectShared() ? getEndPointObjectShared()->getSFMLPosition() : sf::Vector2f();

  sf::Vector2f v = p2 - p1;
  float len2 = v.x * v.x + v.y * v.y;
  if (len2 < 1e-6f) {
    float dx = worldPos_sfml.x - p1.x;
    float dy = worldPos_sfml.y - p1.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    return dist <= tolerance;
  }

  sf::Vector2f w = worldPos_sfml - p1;
  float t = (w.x * v.x + w.y * v.y) / len2;
  t = std::max(0.0f, std::min(1.0f, t));
  sf::Vector2f proj = p1 + v * t;
  float dx = worldPos_sfml.x - proj.x;
  float dy = worldPos_sfml.y - proj.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist <= tolerance;
}

std::vector<Segment_2> VectorObject::getEdges() const {
  std::vector<Segment_2> edges;
  sf::Vector2f p1 = getStartPointObjectShared() ? getStartPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  sf::Vector2f p2 = getEndPointObjectShared() ? getEndPointObjectShared()->getSFMLPosition() : sf::Vector2f();
  edges.emplace_back(Point_2(p1.x, p1.y), Point_2(p2.x, p2.y));
  return edges;
}
