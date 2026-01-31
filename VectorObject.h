#pragma once

#include "Line.h"

class VectorObject : public Line {
 public:
  VectorObject(std::shared_ptr<Point> start, std::shared_ptr<Point> end,
               const sf::Color &color = Constants::LINE_DEFAULT_COLOR)
      : Line(start, end, false, color) {}

  VectorObject(std::shared_ptr<Point> start, std::shared_ptr<Point> end,
               const sf::Color &color, unsigned int id)
      : Line(start, end, false, color, id) {}

  ObjectType getType() const override { return ObjectType::Vector; }

  void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
  bool contains(const sf::Vector2f &worldPos_sfml, float tolerance = Constants::LINE_INTERACTION_RADIUS) const override;
  std::vector<Segment_2> getEdges() const override;
};
