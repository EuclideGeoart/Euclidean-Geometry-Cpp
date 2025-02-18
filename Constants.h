#pragma once
#include <SFML/Graphics.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <limits>
#include<cmath>

namespace Constants {
    // Colors
    const sf::Color INTERSECTION_POINT_COLOR = sf::Color::Red; // Orange color for intersection points
    const sf::Color GRID_COLOR = sf::Color(200, 200, 200,150);
    inline const sf::Color POINT_COLOR = sf::Color::Blue;
    const sf::Color DEPENDENT_POINT_COLOR = sf::Color::Yellow;
    const sf::Color LINE_COLOR = sf::Color::Black;
    const sf::Color LINE_SEGMENT_COLOR = sf::Color::Blue;
    const sf::Color PREVIEW_COLOR = sf::Color(100, 100, 100, 128);
    const sf::Color LINE_COLOR_HOVER = sf::Color(100, 100, 255);
    const sf::Color LINE_SEGMENT_COLOR_HOVER = sf::Color(255, 100, 100);
    const sf::Color SELECTION_COLOR = sf::Color(255, 255, 0, 128);

    
    constexpr float POINT_SIZE = 5.0f;
	constexpr float GRID_SIZE = 50.0f;
    static const float MIN_GRID_SPACING = GRID_SIZE / 2.0f;
    static const float MAX_GRID_SPACING = GRID_SIZE * 2.0f;
    inline constexpr float ZOOM_FACTOR = 1.1f;

    // Sizes and Dimensions
      // 100 pixels = 1 unit
    const float LINE_THICKNESS = 2.0f;
    const float HOVER_DISTANCE = 5.0f;
    const sf::Vector2f BUTTON_SIZE(80.f, 30.f);
    const float SELECTION_THICKNESS = 3.0f;
	const float MOUSE_OVER_TOLERANCE = 5.0f;

    // Interaction Constants
    const float HOVER_ALPHA = 150;
    const float HOVER_SCALE = 1.5f;
    constexpr float HOVER_OUTLINE_THICKNESS = 3.0f;

    // View Control Constants
    const float PAN_SPEED = 100.0f;
    const float MIN_PAN_BOUND = -1000.0f;
    const float MAX_PAN_BOUND = 1000.0f;

    // GUI Constants
    const float GUI_HEIGHT = 40.0f;
    const sf::Color BUTTON_ACTIVE_COLOR(100, 100, 255);
    const sf::Color BUTTON_INACTIVE_COLOR(180, 180, 180);
    const sf::Color BUTTON_HOVER_COLOR(130, 130, 255);

    // Line Drawing Constants
    const float INFINITE_LINE_EXTENSION = 1e+5f;
    const float LINE_SELECTION_THRESHOLD = 3.0f;
    const float COORDINATE_PRECISION = 0.001f;  // For rounding coordinates
    const float SNAP_THRESHOLD_BASE = 0.1f;      // Base snapping threshold
    const float MIN_VISIBLE_SIZE = 0.5f;         // Minimum visible size in world units// Minimum grid spacing // Maximum grid spacing

}
