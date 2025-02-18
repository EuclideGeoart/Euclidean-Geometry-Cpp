#pragma once
#include <SFML/Graphics.hpp>

class CoordinateTransform {
public:
    static sf::Vector2f pixelToWorld(const sf::RenderWindow& window, const sf::Vector2i& pixelPos);
    static sf::Vector2i worldToPixel(const sf::RenderWindow& window, const sf::Vector2f& worldPos);
    static float calculateViewScale(const sf::RenderWindow& window);
private:
    static sf::Vector2f roundCoordinates(sf::Vector2f pos);
};
