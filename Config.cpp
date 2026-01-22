#include "Config.h"
#include "CharTraitsFix.h"


const sf::Vector2f Config::BUTTON_SIZE(100.0f, 30.0f);
const float Config::POINT_SIZE = 5.0f;
const float Config::GRID_SIZE = 100.0f;
const float Config::ZOOM_FACTOR = 1.1f;
const float Config::SELECTION_THRESHOLD = 5.0f;

const sf::Color Config::Colors::POINT = sf::Color::Blue;
const sf::Color Config::Colors::DEPENDENT_POINT = sf::Color::Red;
const sf::Color Config::Colors::LINE = sf::Color::Black;
const sf::Color Config::Colors::GRID = sf::Color(200, 200, 200);
const sf::Color Config::Colors::PREVIEW = sf::Color(100, 100, 100);
const sf::Color Config::Colors::BUTTON_ACTIVE = sf::Color(100, 100, 255);
const sf::Color Config::Colors::BUTTON_INACTIVE = sf::Color(50, 50, 150);
