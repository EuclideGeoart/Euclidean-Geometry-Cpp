#include "Grid.h"
#include "Constants.h"
#include <iostream>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

void Grid::draw(sf::RenderWindow& window, const sf::View& view) {
    float spacing;
    calculateGridSpacing(view, spacing, window);
    drawGridLines(window, view, spacing);
    drawAxes(window, view);
    drawAxisLabels(window, view, spacing); // Draw axis labels
}
void Grid::calculateGridSpacing(const sf::View& view, float& spacing, const sf::RenderWindow& window) {
    float viewWidth = view.getSize().x;
    float baseSpacing = Constants::GRID_SIZE;

    // Calculate grid spacing based on zoom level
    spacing = baseSpacing * (viewWidth / window.getSize().x);

    // Ensure the grid spacing doesn't become too small or too large
    if (spacing < Constants::MIN_GRID_SPACING) {
        spacing = Constants::MIN_GRID_SPACING;
    }
    else if (spacing > Constants::MAX_GRID_SPACING) {
        spacing = Constants::MAX_GRID_SPACING;
    }
}

void Grid::drawGridLines(sf::RenderWindow& window, const sf::View& view, float spacing) {
    sf::Vector2f viewSize = view.getSize();
    sf::Vector2f viewCenter = view.getCenter();
    float left = viewCenter.x - viewSize.x / 2;
    float right = viewCenter.x + viewSize.x / 2;
    float top = viewCenter.y - viewSize.y / 2;
    float bottom = viewCenter.y + viewSize.y / 2;

    // Draw vertical grid lines
    for (float x = left; x <= right; x += spacing) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(x, top), Constants::GRID_COLOR),
            sf::Vertex(sf::Vector2f(x, bottom), Constants::GRID_COLOR)
        };
        window.draw(line, 2, sf::Lines);
    }

    // Draw horizontal grid lines
    for (float y = top; y <= bottom; y += spacing) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(left, y), Constants::GRID_COLOR),
            sf::Vertex(sf::Vector2f(right, y), Constants::GRID_COLOR)
        };
        window.draw(line, 2, sf::Lines);
    }
}



void Grid::drawAxes(sf::RenderWindow& window, const sf::View& view) {
    sf::Vector2f viewSize = view.getSize();
    sf::Vector2f viewCenter = view.getCenter();

    // Create CGAL points for axes
    Point_2 xAxisStart(viewCenter.x - viewSize.x, 0);
    Point_2 xAxisEnd(viewCenter.x + viewSize.x, 0);
    Point_2 yAxisStart(0, viewCenter.y - viewSize.y);
    Point_2 yAxisEnd(0, viewCenter.y + viewSize.y);

    // Create CGAL lines
    Line_2 xAxis(xAxisStart, xAxisEnd);
    Line_2 yAxis(yAxisStart, yAxisEnd);

    // Convert back to SFML for rendering
    sf::Vertex xAxisLine[] = {
        sf::Vertex(sf::Vector2f(CGAL::to_double(xAxisStart.x()), CGAL::to_double(xAxisStart.y())), sf::Color::Black),
        sf::Vertex(sf::Vector2f(CGAL::to_double(xAxisEnd.x()), CGAL::to_double(xAxisEnd.y())), sf::Color::Black)
    };

    sf::Vertex yAxisLine[] = {
        sf::Vertex(sf::Vector2f(CGAL::to_double(yAxisStart.x()), CGAL::to_double(yAxisStart.y())), sf::Color::Black),
        sf::Vertex(sf::Vector2f(CGAL::to_double(yAxisEnd.x()), CGAL::to_double(yAxisEnd.y())), sf::Color::Black)
    };

    window.draw(xAxisLine, 2, sf::Lines);
    window.draw(yAxisLine, 2, sf::Lines);
}
void Grid::drawAxisLabels(sf::RenderWindow& window, const sf::View& view, float spacing) {
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) { // Update path to match your project structure
        std::cerr << "Error loading font!" << std::endl;
        return;
    }

        sf::Vector2f viewSize = view.getSize();
        sf::Vector2f viewCenter = view.getCenter();
        float left = viewCenter.x - viewSize.x / 2;
        float right = viewCenter.x + viewSize.x / 2;
        float top = viewCenter.y - viewSize.y / 2;
        float bottom = viewCenter.y + viewSize.y / 2;

        // Draw x-axis labels
        for (float x = left; x <= right; x += spacing) {
            sf::Text label;
            label.setFont(font);
            label.setCharacterSize(12); // Adjust as needed
            label.setFillColor(sf::Color::Black);
            label.setString(std::to_string(static_cast<int>(x))); // Cast to int for cleaner labels

            // Position the label below the x-axis
            label.setPosition(x, bottom + 5.f); // Small offset to avoid overlapping the grid line

            window.draw(label);
        }

        // Draw y-axis labels
        for (float y = top; y <= bottom; y += spacing) {
            sf::Text label;
            label.setFont(font);
            label.setCharacterSize(12);
            label.setFillColor(sf::Color::Black);
            label.setString(std::to_string(static_cast<int>(y)));

            // Position the label to the left of the y-axis
            label.setPosition(left - 40.f, y); // Offset to avoid overlapping the grid line

            window.draw(label);
        }
    }