#include "Circle.h"  
#define _USE_MATH_DEFINES // For M_PI  
#include <cmath>  
#include <math.h> // Add this line to include math.h
#include <CGAL/Point_2.h> // Include CGAL Point_2
#include <CGAL/squared_distance_2.h> // Include CGAL squared_distance_2
#include "GeometricObject.h"
#include <sstream>
#include "ObjectPoint.h"
#include "ProjectionUtils.h"

const double pi = M_PI;
// Constructor for Circle
Circle::Circle(const sf::Vector2f& centerPos, const sf::Color& outlineColor)
    : center(centerPos),
    radius(0.f),
    selected(false),
    creationFinished(false),
    outlineColor(outlineColor),
    radiusPoint(centerPos)
{
    shape.setRadius(0.f);
    shape.setOrigin(0.f, 0.f);
    shape.setFillColor(sf::Color::Transparent);
    shape.setOutlineColor(outlineColor);
    shape.setOutlineThickness(2.f);
    shape.setPosition(center);
}
// Constructor for Circle
Circle::Circle(const Point_2& center, double radius)
    : center(CGALToSFML(center)), radius(static_cast<float>(radius)), selected(false), creationFinished(true), outlineColor(sf::Color::Blue) {
    shape.setRadius(static_cast<float>(radius));
    shape.setOrigin(static_cast<float>(radius), static_cast<float>(radius));
    shape.setFillColor(sf::Color::Transparent);
    shape.setOutlineColor(outlineColor);
    shape.setOutlineThickness(2.f);
    shape.setPosition(this->center);
}

void Circle::updateRadius(const sf::Vector2f& currentPos) {
   float dx = currentPos.x - center.x;
   float dy = currentPos.y - center.y;
   radius = std::sqrt(dx * dx + dy * dy);
   radiusPoint = constrainToCircumference(currentPos); // Ensure the radius point is on the circumference
}
sf::Vector2f Circle::constrainToCircumference(const sf::Vector2f& point) const {
   sf::Vector2f direction = point - center;
   float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
   if (length == 0) return center;
   return center + (direction / length) * radius;
}
bool Circle::containsCenter(const sf::Vector2f& pos, float tolerance) const {
    float dx = pos.x - center.x;
    float dy = pos.y - center.y;
    return (std::sqrt(dx * dx + dy * dy) <= tolerance);
}

float Circle::distance(const sf::Vector2f& a, const sf::Vector2f& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
bool Circle::containsCircumference(const sf::Vector2f& pos, float tolerance) const {
    float dx = pos.x - center.x;
    float dy = pos.y - center.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    return (std::fabs(dist - radius) <= tolerance);
}
bool Circle::isNearCircumference(const sf::Vector2f& point, float tolerance) const {
    float dist = distance(center, point);
    return std::abs(dist - radius) <= tolerance;
}
bool Circle::isMouseOver(const sf::Vector2f& pos) const {
    return containsCenter(pos, 5.0f) || containsCircumference(pos, 5.0f);
}
void Circle::addObjectPoint(ObjectPoint* objPoint) {
    attachedObjectPoints.push_back(objPoint);
}

// Remove ObjectPoint from the Circle
void Circle::removeObjectPoint(ObjectPoint* objPoint) {
    attachedObjectPoints.erase(
        std::remove(attachedObjectPoints.begin(), attachedObjectPoints.end(), objPoint),
        attachedObjectPoints.end()
    );
}

// Notify all attached ObjectPoints to update their positions
void Circle::notifyObjectPoints() {
    for (auto& objPoint : attachedObjectPoints) {
        if (objPoint) {
            objPoint->moveOnObject(this->getCenter()); // Fix for E0415
        }
    }
}
void Circle::finishCreation() {
   creationFinished = true;
}

void Circle::draw(sf::RenderWindow& window) const {
   sf::Color drawColor = selected ? sf::Color::Green : outlineColor;
   // Draw the circle as a series of line segments.
   const int pointCount = 100;
   sf::VertexArray circleVertices(sf::LineStrip, pointCount + 1);

   for (int i = 0; i <= pointCount; ++i) {
       float theta = 2.f * pi * static_cast<float>(i) / static_cast<float>(pointCount);
       float x = center.x + radius * std::cos(theta);
       float y = center.y + radius * std::sin(theta);
       circleVertices[i].position = sf::Vector2f(x, y);
       circleVertices[i].color = drawColor;
   }
   window.draw(circleVertices);

   // Draw the center indicator as a small filled circle.
   sf::CircleShape centerPoint(3.f);
   centerPoint.setPosition(center - sf::Vector2f(3.f, 3.f));
   centerPoint.setFillColor(sf::Color::Red);
   window.draw(centerPoint);

   // Draw the radius line if the radiusPoint is not the center
   if (radiusPoint != center) {
       sf::Vertex line[] =
       {
           sf::Vertex(center, sf::Color::Blue),
           sf::Vertex(radiusPoint, sf::Color::Blue)
       };
       window.draw(line, 2, sf::Lines);
   }

   // Draw child ObjectPoints if any
   // Assuming childPoints is accessible or provide a method to iterate
}

void Circle::update() {
    if (!childPoints.empty()) {
        for (auto& child : childPoints) {
            // Update each ObjectPoint's position based on the circle's current state
            sf::Vector2f currentPos = CGALToSFML(child->getPosition());
            child->moveOnObject(currentPos);
            updateObjectPoints();
        }
    }

    // Update the SFML shape's position and appearance
    shape.setPosition(center);
    if (selected) {
        shape.setOutlineColor(sf::Color::Red);
    }
    else {
        shape.setOutlineColor(outlineColor);
    }
}
void Circle::move(const sf::Vector2f& offset) {
   center += offset;
   /*radiusPoint += offset;*/ // Move radiusPoint along with the center
   notifyObjectPoints();
}

void Circle::setRadius(float newRadius) {
   radius = newRadius;
   // Update radiusPoint to maintain consistency
   radiusPoint = sf::Vector2f(center.x + newRadius, center.y);
   notifyObjectPoints();
}

sf::Vector2f Circle::getCenter() const {
   return center;
}

float Circle::getRadius() const {
   return radius;
}
bool Circle::hasRadius() const {
    return radius > 0.f;
}
bool Circle::isCreated() const {
   return creationFinished;
}

void Circle::setSelected(bool s) {
   selected = s;
}
void Circle::moveCenter(const sf::Vector2f& newCenter) {
    // Update the center position
    center = CGALToSFML(Point_2(newCenter.x, newCenter.y));

    // Update any related geometry...

    // Notify ObjectPoints of the update
    notifyObjectPoints();
}
void Circle::setRadiusPoint(const sf::Vector2f& point) {
   radiusPoint = constrainToCircumference(point);
   // Update radius to match the new radiusPoint
   radius = distance(center, radiusPoint);
}

sf::Vector2f Circle::CGALToSFML(const Point_2& point) const {
    return sf::Vector2f(
        static_cast<float>(CGAL::to_double(point.x())),
        static_cast<float>(CGAL::to_double(point.y()))
    );
}

float Circle::length(const sf::Vector2f& point) const {
   return std::sqrt(point.x * point.x + point.y * point.y);
}
void Circle::addChildPoint(ObjectPoint* child) {
    childPoints.push_back(child);
}
ObjectType Circle::getAttachedType() const {
    return ObjectType::Circle;
}
//bool Circle::isMouseOver(const sf::RenderWindow& window) const {
//   // Implementation to check if mouse is over the circle
//   // Example implementation:
//   sf::Vector2i mousePos = sf::Mouse::getPosition(window);
//   sf::Vector2f worldPos = window.mapPixelToCoords(mousePos);
//   float distance = std::sqrt(std::pow(worldPos.x - center.x, 2) + std::pow(worldPos.y - center.y, 2));
//   return distance <= 50.f; // Example radius
//}
//bool Circle::isMouseOver(const sf::Vector2f& pos) const {
//    // Implementation to check if mouse is over the circle
//    // Example implementation:
//    sf::Vector2f diff = pos - center;
//    float dist2 = diff.x * diff.x + diff.y * diff.y;
//    return (std::abs(std::sqrt(dist2) - radius) < 5.f);
//}
void Circle::updateObjectPoints() {
    for (auto& child : childPoints) {
        if (child) {
            child->setPosition(center);
        }
    }
}
void Circle::resize(float newRadius) {
    radius = newRadius;
    shape.setRadius(radius);
    shape.setOrigin(radius, radius); // Ensure the origin is at the center of the circle
}
void Circle::updateConnectedPoints() {
    for (auto* point : attachedObjectPoints) {
        if (auto* objPoint = dynamic_cast<ObjectPoint*>(point)) {
            Point_2 currentPos = (objPoint->getPosition());
            Point_2 circleCenter = toCGALPoint(center);
            Point_2 projectedPoint = ProjectionUtils::projectPointOntoCircle(currentPos, circleCenter, radius, 0.1);
            objPoint->setPosition((projectedPoint));
        }
    }
}
// Circle.cpp
std::string Circle::getStatus() const {
    std::ostringstream oss;
    oss << "Circle: center(" << center.x << ", " << center.y
        << "), radius: " << radius
        << ", outline color: (" << static_cast<int>(outlineColor.r) << ", "
        << static_cast<int>(outlineColor.g) << ", "
        << static_cast<int>(outlineColor.b) << ")";
    return oss.str();
}


