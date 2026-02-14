#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message(                                                               \
    "CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif
#include <cmath>
#include "Transforms.h"
#include "CharTraitsFix.h"
#include "Constants.h"

sf::Vector2f TransformUtils::pixelToWorld(const sf::RenderWindow &window,
                                          const sf::Vector2i &pixelPos) {
  // Direct mapping with no rounding or snapping
  return window.mapPixelToCoords(pixelPos);
}

sf::Vector2i TransformUtils::worldToPixel(const sf::RenderWindow &window,
                                          const sf::Vector2f &worldPos) {
  return window.mapCoordsToPixel(worldPos);
}

float TransformUtils::calculateViewScale(const sf::RenderWindow &window) {
  return window.getView().getSize().x / window.getDefaultView().getSize().x;
}

sf::Vector2f TransformUtils::roundCoordinates(sf::Vector2f pos) {
  float x = std::round(pos.x / Constants::COORDINATE_PRECISION) *
            Constants::COORDINATE_PRECISION;
  float y = std::round(pos.y / Constants::COORDINATE_PRECISION) *
            Constants::COORDINATE_PRECISION;
  return sf::Vector2f(x, y);
}
