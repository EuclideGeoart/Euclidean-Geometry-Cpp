#pragma once
#include "CharTraitsFix.h"
#include <SFML/Graphics.hpp>

class Config {
public:
  static const sf::Vector2f BUTTON_SIZE;
  static const float POINT_SIZE;
  static const float GRID_SIZE;
  static const float ZOOM_FACTOR;
  static const float SELECTION_THRESHOLD;

  struct Colors {
    static const sf::Color POINT;
    static const sf::Color DEPENDENT_POINT;
    static const sf::Color LINE;
    static const sf::Color GRID;
    static const sf::Color PREVIEW;
    static const sf::Color BUTTON_ACTIVE;
    static const sf::Color BUTTON_INACTIVE;
  };
};
