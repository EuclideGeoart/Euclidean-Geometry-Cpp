#include "Transforms.h"
#include "Constants.h"

sf::Vector2f CoordinateTransform::pixelToWorld(const sf::RenderWindow& window, const sf::Vector2i& pixelPos) {
    // Direct mapping with no rounding or snapping
    return window.mapPixelToCoords(pixelPos);
}

sf::Vector2i CoordinateTransform::worldToPixel(const sf::RenderWindow& window, const sf::Vector2f& worldPos) {
    return window.mapCoordsToPixel(worldPos);
}


float CoordinateTransform::calculateViewScale(const sf::RenderWindow& window) {
    return window.getView().getSize().x / window.getDefaultView().getSize().x;
}

sf::Vector2f CoordinateTransform::roundCoordinates(sf::Vector2f pos) {
    float x = std::round(pos.x / Constants::COORDINATE_PRECISION) * Constants::COORDINATE_PRECISION;
    float y = std::round(pos.y / Constants::COORDINATE_PRECISION) * Constants::COORDINATE_PRECISION;
    return sf::Vector2f(x, y);
}
