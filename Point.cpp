#include "Point.h"
#include "Line.h"
#include "Constants.h"
#include <algorithm>    // For std::remove
#include <cmath>
#include "Types.h"
#include <CGAL/number_utils.h> // For CGAL::to_double
#include <iostream>

Point::Point(const Point_2& position, const sf::Color& color, bool isLocked)
    : position(sfToSFML(position)), color(color), isLocked(isLocked)
{
    shape.setRadius(Constants::POINT_SIZE);
    shape.setFillColor(color);
    shape.setOrigin(Constants::POINT_SIZE, Constants::POINT_SIZE);
    updatePosition();
}
sf::Vector2f Point::sfToSFML(const Point_2& point) const {
    return sf::Vector2f(static_cast<float>(CGAL::to_double(point.x())), static_cast<float>(CGAL::to_double(point.y())));
}
void Point::setPosition(const Point_2& pos) {
    // Update our own stored position...
    position = sf::Vector2f(CGAL::to_double(pos.x()), CGAL::to_double(pos.y()));
    shape.setPosition(position);

    // Notify all connected geometry that this point’s position has changed.
    updateConnectedGeometry(position);
    // Ensure that sfToSFML is a function and not a variable or an object
    // If sfToSFML is a function, it should be defined as follows:
}
void Point::updatePosition() {
    shape.setPosition(position);
}

bool Point::contains(const sf::Vector2f& point, float tolerance) const {
    double dx = position.x - point.x;
    double dy = position.y - point.y;
    return (dx * dx + dy * dy) <= tolerance * tolerance;
}

void Point::setSelected(bool selected) {
    isSelected = selected;
    shape.setFillColor(isSelected ? sf::Color::Red : color);
}

void Point::addConnectedLine(Line* line) {
    // Avoid duplicate insertion if necessary.
    for (auto l : connectedLines) {
        if (l == line) return;
    }
    connectedLines.push_back(line);
}

void Point::removeConnectedLine(Line* line) {
    connectedLines.erase(std::remove(connectedLines.begin(), connectedLines.end(), line), connectedLines.end());
}

void Point::updateConnectedGeometry(const sf::Vector2f& newPos) {
    for (auto* connectedLine : connectedLines) {
        connectedLine->updatePosition(this, newPos);
    }
}

void Point::translate(const sf::Vector2f& offset) {
    // Update the stored position.
    position += offset;
    // Update the rendered shape's position.
    shape.setPosition(position);
    // Notify connected geometry that this point moved.
    updateConnectedGeometry(position);
}
void Point::finalizeTranslation() {
    // Update the underlying points by adding the offset.
    // This requires that your Point class supports a method to translate its position.
    // For example, assume Point has a translate(const sf::Vector2f&) method.
    for (auto* connectedLine : connectedLines) {
        Point* startPointPtr = connectedLine->getStartPointPtr();
        Point* endPointPtr = connectedLine->getEndPointPtr();
        if (startPointPtr)
            startPointPtr->translate(translationOffset);
        if (endPointPtr)
            endPointPtr->translate(translationOffset);
    }

    // Reset the translation offset.
    translationOffset = sf::Vector2f(0.f, 0.f);
    // Finally, re-read geometry from the free points.
    updateGeometry();
}
void Point::updateGeometry() {
    // Simply update the rendered shape's position.
    shape.setPosition(position);
    // Optionally, update connected geometry if needed:
    //updateConnectedGeometry(position);
}

//void Point::draw(sf::RenderWindow& window, const sf::View& view) const {
//    sf::CircleShape shape(Constants::POINT_SIZE);
//    shape.setOrigin(Constants::POINT_SIZE, Constants::POINT_SIZE);
//    shape.setFillColor(color);
//
//    shape.setPosition(position);
//
//    window.draw(shape);
//}
void Point::draw(sf::RenderWindow& window) const {
        double sx = CGAL::to_double(position.x);
        double sy = CGAL::to_double(position.y);
        sf::Vector2f center(static_cast<float>(sx), static_cast<float>(sy));

        // Get current view so we can compute a fixed screen scale for the point.
        const sf::View& view = window.getView();
        // Compute how many pixels per world unit (note: you can use any side; here we use width):
        float scale = static_cast<float>(window.getSize().x) / view.getSize().x;
        // For a fixed size in screen pixels, divide constant by scale:
        float radius = Constants::POINT_SIZE / scale;
        sf::CircleShape circle(radius);
        circle.setFillColor(color);
        // Center the circle at the point's position:
        circle.setOrigin(radius, radius);
        circle.setPosition(position);
        window.draw(circle);

        if (isSelected) {
            // Outer glow circle (larger, semi-transparent)
            sf::CircleShape glow(shape.getRadius() * 1.5f);
            glow.setFillColor(sf::Color(255, 165, 0, 128)); // Semi-transparent orange
            glow.setPosition(position);
            glow.setOrigin(glow.getRadius(), glow.getRadius());
            window.draw(glow);
            circle.setOutlineThickness(2.f);

            // Inner highlight circle
            sf::CircleShape highlight(shape.getRadius() * 1.2f);
            highlight.setFillColor(sf::Color(255, 215, 0, 180)); // Semi-transparent gold
            highlight.setPosition(position);
            highlight.setOrigin(highlight.getRadius(), highlight.getRadius());
            window.draw(highlight);
        }
        else circle.setOutlineThickness(0.f);
        circle.setOutlineColor(sf::Color::Black);
        window.draw(circle);
    }
    // Get current view so we can compute a fixed screen scale for the point.

void Point::setHovered(bool hovered) {
    isHovered = hovered;
    if (isHovered) {
        shape.setOutlineThickness(Constants::HOVER_OUTLINE_THICKNESS);
        shape.setOutlineColor(sf::Color(220, 50, 20));
    }
    else {
        shape.setOutlineThickness(0);
    }
}
