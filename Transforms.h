#pragma once

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "CharTraitsFix.h"
#include <SFML/Graphics.hpp>

class TransformUtils { // Renamed from CoordinateTransform
public:
  static sf::Vector2f pixelToWorld(const sf::RenderWindow &window,
                                   const sf::Vector2i &pixelPos);
  static sf::Vector2i worldToPixel(const sf::RenderWindow &window,
                                   const sf::Vector2f &worldPos);
  static float calculateViewScale(const sf::RenderWindow &window);

private:
  static sf::Vector2f roundCoordinates(sf::Vector2f pos);
};
