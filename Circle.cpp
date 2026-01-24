#include "Circle.h"
#include "ObjectPoint.h"
#include "Constants.h"
#include "Transforms.h"
#include <cmath>
#include <iostream>

// Constructor
Circle::Circle(Point *centerPoint, double radius, const sf::Color &color)
    : GeometricObject(ObjectType::Circle, color), m_centerPoint(centerPoint), m_radius(radius),
      m_outlineColor(color), m_fillColor(sf::Color(color.r, color.g, color.b, 50)) {
  updateSFMLShape();
}

// Destructor
Circle::~Circle() {
  // Clean up child ObjectPoints
  for (auto &wp : m_hostedObjectPoints) {
    if (auto sp = wp.lock()) {
      sp->clearHost();
    }
  }
}

// Factory method
std::shared_ptr<Circle> Circle::create(Point *centerPoint, double radius,
                                       const sf::Color &color) {
  auto circle = std::make_shared<Circle>(centerPoint, radius, color);
  return circle;
}

// Geometry accessors
Point_2 Circle::getCenterPoint() const {
  if (m_centerPoint) {
    return m_centerPoint->getCGALPosition();
  }
  return Point_2(0, 0);
}

Circle_2 Circle::getCGALCircle() const {
  try {
    Point_2 center = getCenterPoint();
    return Circle_2(center, m_radius * m_radius);
  } catch (const std::exception &e) {
    std::cerr << "Error creating CGAL circle: " << e.what() << std::endl;
    return Circle_2(Point_2(0, 0), 1.0);
  }
}

// Setters
void Circle::setRadius(double newRadius) {
  if (newRadius > 0 && std::isfinite(newRadius)) {
    m_radius = newRadius;
    updateSFMLShape();
    updateHostedPoints();
  }
}

void Circle::setCenter(const Point_2 &newCenter) {
  if (m_centerPoint && CGAL::is_finite(newCenter.x()) && CGAL::is_finite(newCenter.y())) {
    m_centerPoint->setCGALPosition(newCenter);
    // updateSFMLShape and updateHostedPoints will be called via observer notification
  }
}

void Circle::setCenterPointObject(Point *newCenterPoint) {
  if (newCenterPoint) {
    m_centerPoint = newCenterPoint;
    updateSFMLShape();
    updateHostedPoints();
  }
}

// GeometricObject overrides
void Circle::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!m_visible && !forceVisible) return;
  
  // Clone and scale main circle
  sf::CircleShape circleShape = m_sfmlShape;
  
  float baseOutlineThickness = 1.0f;
  if (isSelected()) baseOutlineThickness = 3.0f;
  else if (isHovered()) baseOutlineThickness = 3.0f;
  
  circleShape.setOutlineThickness(baseOutlineThickness * scale);

  // GHOST MODE
  if (!m_visible && forceVisible) {
      sf::Color ghostFill = circleShape.getFillColor();
      ghostFill.a = 50;
      circleShape.setFillColor(ghostFill);
      
      sf::Color ghostOutline = circleShape.getOutlineColor();
      ghostOutline.a = 50;
      circleShape.setOutlineColor(ghostOutline);
  }

  window.draw(circleShape);

  // Clone and scale center visual
  sf::CircleShape centerShape = m_centerVisual;
  float baseCenterRadius = 3.0f; // Corresponds to Constants::CIRCLE_CENTER_VISUAL_RADIUS
  
  // Recalculate position to handle scaling from center properly
  // We use the center point directly to be precise
  Point_2 center = getCenterPoint();
  auto sfmlCenter = Point::cgalToSFML(center);
  
  centerShape.setRadius(baseCenterRadius * scale);
  centerShape.setOrigin(baseCenterRadius * scale, baseCenterRadius * scale);
  centerShape.setPosition(sfmlCenter);
  
  // GHOST MODE for center
  if (!m_visible && forceVisible) {
    sf::Color ghostCenter = centerShape.getFillColor();
    ghostCenter.a = 50;
    centerShape.setFillColor(ghostCenter);
  }

  window.draw(centerShape);
}

bool Circle::contains(const sf::Vector2f &worldPos, float tolerance) const {
  if (!isValid()) return false;

  try {
    Point_2 center = getCenterPoint();
    auto sfmlCenter = Point::cgalToSFML(center);
    float distToCenter = std::sqrt((worldPos.x - sfmlCenter.x) * (worldPos.x - sfmlCenter.x) +
                                   (worldPos.y - sfmlCenter.y) * (worldPos.y - sfmlCenter.y));

    // Check if on circumference (within tolerance band)
    float circumferenceDistance = std::abs(distToCenter - static_cast<float>(m_radius));
    return circumferenceDistance <= tolerance;
  } catch (const std::exception &e) {
    std::cerr << "Error in Circle::contains: " << e.what() << std::endl;
    return false;
  }
}

bool Circle::isValid() const {
  if (!m_centerPoint) return false;
  Point_2 center = m_centerPoint->getCGALPosition();
  return m_radius > 0 && std::isfinite(m_radius) && 
         CGAL::is_finite(center.x()) && CGAL::is_finite(center.y());
}

void Circle::update() {
  // Called when observed Point (center) changes
  if (!isValid()) {
    return;
  }
  updateSFMLShape();
  updateHostedPoints();
}

sf::FloatRect Circle::getGlobalBounds() const {
  return m_sfmlShape.getGlobalBounds();
}

void Circle::translate(const Vector_2 &offset) {
  if (m_centerPoint) {
    Point_2 currentCenter = m_centerPoint->getCGALPosition();
    Point_2 newCenter = currentCenter + offset;
    setCenter(newCenter);
  }
}

void Circle::setPosition(const sf::Vector2f &newSfmlPos) {
  try {
    Point_2 newCenter = Point::sfmlToCGAL(newSfmlPos);
    setCenter(newCenter);
  } catch (const std::exception &e) {
    std::cerr << "Error setting position: " << e.what() << std::endl;
  }
}

void Circle::setSelected(bool sel) {
  GeometricObject::setSelected(sel);
  updateSFMLShape();
}

void Circle::setHovered(bool hov) {
  GeometricObject::setHovered(hov);
  updateSFMLShape();
}

void Circle::clearCenterPoint() {
  m_centerPoint = nullptr;
  m_isValid = false;
}

// Interaction helpers
bool Circle::isCenterPointHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  if (!isValid()) return false;

  try {
    Point_2 center = getCenterPoint();
    auto sfmlCenter = Point::cgalToSFML(center);
    float dist = std::sqrt((worldPos_sfml.x - sfmlCenter.x) * (worldPos_sfml.x - sfmlCenter.x) +
                           (worldPos_sfml.y - sfmlCenter.y) * (worldPos_sfml.y - sfmlCenter.y));
    return dist <= tolerance;
  } catch (const std::exception &e) {
    std::cerr << "Error in isCenterPointHovered: " << e.what() << std::endl;
    return false;
  }
}

bool Circle::isCircumferenceHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  if (!isValid()) return false;

  try {
    Point_2 center = getCenterPoint();
    auto sfmlCenter = Point::cgalToSFML(center);
    float distToCenter = std::sqrt((worldPos_sfml.x - sfmlCenter.x) * (worldPos_sfml.x - sfmlCenter.x) +
                                   (worldPos_sfml.y - sfmlCenter.y) * (worldPos_sfml.y - sfmlCenter.y));
    float circumferenceDistance = std::abs(distToCenter - static_cast<float>(m_radius));
    return circumferenceDistance <= tolerance;
  } catch (const std::exception &e) {
    std::cerr << "Error in isCircumferenceHovered: " << e.what() << std::endl;
    return false;
  }
}

Point_2 Circle::projectOntoCircumference(const Point_2 &p) const {
  try {
    Point_2 center = getCenterPoint();
    Vector_2 toPoint = p - center;
    double distToCenter = std::sqrt(CGAL::to_double(toPoint.squared_length()));

    if (distToCenter < Constants::EPSILON) {
      // Point is at center, project to circumference (arbitrary angle)
      return center + Vector_2(m_radius, 0);
    }

    // Normalize and scale to radius
    Vector_2 normalized = toPoint / distToCenter;
    return center + normalized * m_radius;
  } catch (const std::exception &e) {
    std::cerr << "Error in projectOntoCircumference: " << e.what() << std::endl;
    return p;
  }
}

// Child ObjectPoint management methods (addChildPoint, removeChildPoint, updateHostedPoints)
// are now inherited from GeometricObject.



// Helper - Update SFML shape
void Circle::updateSFMLShape() {
  if (!isValid()) return;

  try {
    Point_2 center = getCenterPoint();
    auto sfmlCenter = Point::cgalToSFML(center);
    float sfmlRadius = static_cast<float>(m_radius);

    // Update main circle
    m_sfmlShape.setPointCount(120);
    m_sfmlShape.setRadius(sfmlRadius);
    m_sfmlShape.setPosition(sfmlCenter.x - sfmlRadius, sfmlCenter.y - sfmlRadius);
    m_sfmlShape.setFillColor(m_fillColor);

    // Set outline based on state
    if (isSelected()) {
      m_sfmlShape.setOutlineColor(Constants::SELECTION_COLOR);
      m_sfmlShape.setOutlineThickness(Constants::SELECTION_THICKNESS_CIRCLE);
    } else if (isHovered()) {
      m_sfmlShape.setOutlineColor(Constants::HOVER_COLOR);
      m_sfmlShape.setOutlineThickness(Constants::HOVER_THICKNESS_CIRCLE);
    } else {
      m_sfmlShape.setOutlineColor(m_outlineColor);
      m_sfmlShape.setOutlineThickness(1.0f);
    }

    // Update center visual
    float centerRadius = Constants::CIRCLE_CENTER_VISUAL_RADIUS;
    m_centerVisual.setRadius(centerRadius);
    m_centerVisual.setPosition(sfmlCenter.x - centerRadius, sfmlCenter.y - centerRadius);
    m_centerVisual.setFillColor(isSelected() ? Constants::SELECTION_COLOR :
                                 isHovered() ? Constants::HOVER_COLOR :
                                 Constants::POINT_DEFAULT_COLOR);
    m_centerVisual.setOutlineThickness(0);

  } catch (const std::exception &e) {
    std::cerr << "Error in updateSFMLShape: " << e.what() << std::endl;
  }
}

std::vector<Point_2> Circle::getInteractableVertices() const {
  std::vector<Point_2> result;
  if (isValid()) {
    result.push_back(getCenterPoint());
  }
  return result;
}

bool Circle::getClosestPointOnPerimeter(const Point_2 &query, Point_2 &outPoint) const {
  if (!isValid()) {
    return false;
  }

  try {
    outPoint = projectOntoCircumference(query);
    return true;
  } catch (...) {
    return false;
  }
}
