#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message("CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif

#include "Circle.h"
#include "CGALSafeUtils.h"
#include "Constants.h"
#include "ObjectPoint.h"  // Include ObjectPoint.h to access its clearHost method
#include "Point.h"        // For Point class
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>  // For Point_2 constructor from Point
#include <CGAL/Origin.h>                                       // For CGAL::ORIGIN
#include <CGAL/number_utils.h>
#include <algorithm>  // For std::remove
#include <cmath>      // For std::sqrt, std::abs
#include <iostream>   // For debugging

// Constructor with outline and fill colors (full implementation)
Circle::Circle(const Point_2 &center_cgal, double radius, const sf::Color &outlineColor,
               const sf::Color &fillColor)
    : GeometricObject(ObjectType::Circle, outlineColor),
      m_centerPointObject(std::make_unique<Point>(center_cgal, 1.0f, outlineColor)),
      m_radius(std::max(radius, static_cast<double>(Constants::MIN_CIRCLE_RADIUS))),
      m_outlineColor(outlineColor),
      m_fillColor(fillColor) {
  std::cout << "=== Circle Constructor Debug ===" << std::endl;
  std::cout << "Circle address: " << this << std::endl;
  std::cout << "Center: (" << CGAL::to_double(center_cgal.x()) << ", "
            << CGAL::to_double(center_cgal.y()) << ")" << std::endl;
  std::cout << "Radius: " << radius << std::endl;

  // Test if this will be manageable by shared_ptr
  std::cout << "Constructor completed. Object ready for shared_ptr management." << std::endl;
  std::cout << "=== End Circle Constructor Debug ===" << std::endl;

  // Debug the parameters
  std::cout << "Creating circle with center at (" << CGAL::to_double(center_cgal.x()) << ", "
            << CGAL::to_double(center_cgal.y()) << ") and radius " << radius << std::endl;

  updateCGALCircle();
  updateSFMLShape();
}

// Constructor with Point* center (full implementation)
Circle::Circle(Point *centerPoint, double radius, const sf::Color &outlineColor,
               const sf::Color &fillColor)
    : GeometricObject(ObjectType::Circle, outlineColor),
      m_centerPointObject(nullptr),
      m_radius(std::max(radius, static_cast<double>(Constants::MIN_CIRCLE_RADIUS))),
      m_outlineColor(outlineColor),
      m_fillColor(fillColor) {
  try {
    if (!centerPoint) {
      throw std::invalid_argument(
          "Center point cannot be null for Circle "
          "constructor with Point*");
    }

    m_centerPointObject =
        std::make_unique<Point>(centerPoint->getCGALPosition(), 1.0f, centerPoint->getColor());

    updateCGALCircle();
    updateSFMLShape();
  } catch (const std::exception &e) {
    std::cerr << "Error in Circle constructor: " << e.what() << std::endl;
    throw;  // Re-throw the exception
  }
}
void Circle::setColor(const sf::Color &color) {
  m_color = color;
  // If Circle has its own m_fillColor, set it:
  // m_fillColor = color;
  updateSFMLShape();  // Update the shape to reflect color changes
}

// Single color constructor (simplified version for backward compatibility)
Circle::Circle(const Point_2 &center, double radius, const sf::Color &color)
    : GeometricObject(ObjectType::Circle, color),
      m_centerPointObject(std::make_unique<Point>(center, 1.0f, color)),
      m_radius(std::max(radius, static_cast<double>(Constants::MIN_CIRCLE_RADIUS))),
      m_outlineColor(color),
      m_fillColor(Constants::CIRCLE_DEFAULT_FILL_COLOR) {
  // Debug the parameters
  std::cout << "Creating circle with center at (" << CGAL::to_double(center.x()) << ", "
            << CGAL::to_double(center.y()) << ") and radius " << radius << std::endl;

  // Initialize the CGAL circle
  updateCGALCircle();

  // Initialize the SFML circle shape
  m_sfmlShape.setRadius(static_cast<float>(m_radius));
  m_sfmlShape.setOrigin(static_cast<float>(m_radius), static_cast<float>(m_radius));

  // Set position
  try {
    sf::Vector2f center_sfml = Point::cgalToSFML(center);
    m_sfmlShape.setPosition(center_sfml);
  } catch (const std::exception &e) {
    std::cerr << "Error setting circle position: " << e.what() << std::endl;
    m_sfmlShape.setPosition(0, 0);  // Default position
  }

  // Configure appearance
  m_sfmlShape.setFillColor(m_fillColor);
  m_sfmlShape.setOutlineThickness(Constants::CIRCLE_OUTLINE_THICKNESS);
  m_sfmlShape.setOutlineColor(m_outlineColor);

  // Initialize the center visual
  m_centerVisual.setRadius(Constants::CIRCLE_CENTER_VISUAL_RADIUS);
  m_centerVisual.setOrigin(Constants::CIRCLE_CENTER_VISUAL_RADIUS,
                           Constants::CIRCLE_CENTER_VISUAL_RADIUS);
  m_centerVisual.setPosition(m_sfmlShape.getPosition());
  m_centerVisual.setFillColor(Constants::CIRCLE_CENTER_VISUAL_COLOR);
}

// Destructor
Circle::~Circle() {
  std::cout << "=== Circle Destructor Debug ===" << std::endl;
  std::cout << "Circle at address " << this << " is being destroyed" << std::endl;
  std::cout << "Radius was: " << m_radius << std::endl;
  if (m_centerPointObject) {
    Point_2 center = m_centerPointObject->getCGALPosition();
    std::cout << "Center was: (" << CGAL::to_double(center.x()) << ", "
              << CGAL::to_double(center.y()) << ")" << std::endl;
  } else {
    std::cout << "No center point object" << std::endl;
  }
  std::cout << "Number of hosted ObjectPoints: " << m_hostedObjectPoints.size() << std::endl;
  std::cout << "=== End Circle Destructor Debug ===" << std::endl;

  // Cleanup, if any, handled by unique_ptr for m_centerPointObject
  // and other members with automatic storage duration or smart pointers.
}
std::shared_ptr<Circle> Circle::create(const Point_2 &center, double radius,
                                       const sf::Color &color) {
  auto circle = std::make_shared<Circle>(center, radius, color);
  std::cout << "Circle::create - Created circle at: " << circle.get() << std::endl;
  return circle;
}

std::shared_ptr<Circle> Circle::create(const Point_2 &center, double radius,
                                       const sf::Color &outlineColor, const sf::Color &fillColor) {
  auto circle = std::make_shared<Circle>(center, radius, outlineColor, fillColor);
  std::cout << "Circle::create - Created circle at: " << circle.get() << std::endl;
  return circle;
}
// --- GeometricObject Overrides ---

void Circle::draw(sf::RenderWindow &window) const {
  // Draw the circle outline
  sf::Color currentOutlineColor = m_outlineColor;  // Use m_outlineColor
  if (m_selected) {
    currentOutlineColor = Constants::SELECTION_COLOR_CIRCLE_OUTLINE;  // Use specific selection
                                                                      // outline
  } else if (m_hovered) {
    currentOutlineColor = Constants::HOVER_COLOR_CIRCLE_OUTLINE;  // Use specific hover outline
  }

  // Create a non-const copy of the shape that can be modified
  sf::CircleShape tempShape =
      m_sfmlShape;  // m_sfmlShape now has fill color set by updateSFMLShape()
  tempShape.setOutlineColor(currentOutlineColor);

  // Adjust outline thickness for selection/hover
  if (m_selected) {
    tempShape.setOutlineThickness(Constants::SELECTION_THICKNESS_CIRCLE);
  } else if (m_hovered) {
    tempShape.setOutlineThickness(Constants::HOVER_THICKNESS_CIRCLE);
  } else {
    tempShape.setOutlineThickness(Constants::CIRCLE_OUTLINE_THICKNESS);
  }

  window.draw(tempShape);

  // Always draw the center point with enhanced visibility
  drawCenterPoint(window);
}

void Circle::drawCenterPoint(sf::RenderWindow &window) const {
  // Ensure m_centerPointObject is valid before trying to use it
  if (!m_centerPointObject) {
    return;  // Or handle error appropriately
  }

  // Define a minimum and maximum size for the center visual
  const float minSize = Constants::MIN_CENTER_POINT_VISUAL_RADIUS;
  const float maxSize = Constants::MAX_CENTER_POINT_VISUAL_RADIUS;

  // Calculate desired size based on circle radius, but clamp it
  float centerPointSize = std::max<float>(static_cast<float>(m_radius * 0.1f), minSize);
  centerPointSize = std::min<float>(centerPointSize, maxSize);

  // Create a circle shape for the center point
  sf::CircleShape centerPoint(centerPointSize);

  // Get center point in SFML coordinates
  sf::Vector2f sfmlCenter = Point::cgalToSFML(m_cgalCircle.center());

  // Position the center point (centered at the point)
  centerPoint.setPosition(sfmlCenter.x - centerPointSize, sfmlCenter.y - centerPointSize);

  // Set fill color based on circle state
  sf::Color fillColor;
  if (m_interactionMode == CircleInteractionMode::DragCenter) {
    fillColor = Constants::SELECTION_COLOR;  // Highlight when dragging center
  } else if (m_selected) {
    fillColor = Constants::SELECTION_COLOR;  // Highlight when circle is selected
  } else {
    fillColor = Constants::CIRCLE_CENTER_COLOR;  // Default color for center point
  }

  centerPoint.setFillColor(fillColor);

  // Draw the center point
  window.draw(centerPoint);
}

void Circle::updateCGALCircle() {
  if (m_centerPointObject) {
    m_cgalCircle =
        CGAL::Circle_2<Kernel>(m_centerPointObject->getCGALPosition(), m_radius * m_radius);
  } else {
    // Fallback: if m_centerPointObject is somehow null, update based on
    // existing m_cgalCircle center. This assumes m_cgalCircle was validly
    // initialized or set before.
    m_cgalCircle = CGAL::Circle_2<Kernel>(m_cgalCircle.center(), m_radius * m_radius);
  }
}

void Circle::updateSFMLShape() {
  if (!m_centerPointObject) return;

  Point_2 center_cgal = m_centerPointObject->getCGALPosition();
  sf::Vector2f center_sfml = Point::cgalToSFML(center_cgal);
  float radius_sfml = static_cast<float>(m_radius);
  if (m_isValid) {
    m_sfmlShape.setRadius(radius_sfml);
    m_sfmlShape.setOrigin(radius_sfml, radius_sfml);  // Center the origin
    m_sfmlShape.setPosition(center_sfml);
    m_sfmlShape.setFillColor(m_fillColor);        // Set the fill color
    
    // VISUAL FEEDBACK: Change outline color based on selection/hover state
    sf::Color outlineColor = m_outlineColor;
    float outlineThickness = Constants::CIRCLE_OUTLINE_THICKNESS;
    
    if (isSelected()) {
      outlineColor = Constants::SELECTION_COLOR;
      outlineThickness = Constants::SELECTION_THICKNESS_CIRCLE;
    } else if (isHovered()) {
      outlineColor = Constants::HOVER_COLOR;
      outlineThickness = Constants::HOVER_THICKNESS_CIRCLE;
    }
    
    m_sfmlShape.setOutlineColor(outlineColor);
    m_sfmlShape.setOutlineThickness(outlineThickness);
    m_sfmlShape.setPointCount(100);  // Make it smooth

    // VISUAL FEEDBACK: Update center point visual with hover/selection colors
    m_centerVisual.setRadius(Constants::CIRCLE_CENTER_VISUAL_RADIUS);
    m_centerVisual.setOrigin(Constants::CIRCLE_CENTER_VISUAL_RADIUS,
                             Constants::CIRCLE_CENTER_VISUAL_RADIUS);
    m_centerVisual.setPosition(center_sfml);
    
    // Change center color based on state
    sf::Color centerColor = Constants::CIRCLE_CENTER_VISUAL_COLOR;
    if (isSelected()) {
      centerColor = Constants::SELECTION_COLOR;
    } else if (isHovered()) {
      centerColor = Constants::HOVER_COLOR;
    }
    
    m_centerVisual.setFillColor(centerColor);
  }
}
void Circle::addChildPoint(std::shared_ptr<ObjectPoint> objPoint) {
  if (!objPoint) {  // Added a check for null objPoint
    return;
  }

  // Check if the point is already in our hosted list
  auto it = std::find_if(m_hostedObjectPoints.begin(), m_hostedObjectPoints.end(),
                         [&objPoint](const std::weak_ptr<ObjectPoint> &weakPtr) {
                           if (auto sharedPtr = weakPtr.lock()) {
                             return sharedPtr.get() == objPoint.get();
                           }
                           return false;
                         });

  if (it == m_hostedObjectPoints.end()) {
    // Store as weak_ptr to avoid circular references
    m_hostedObjectPoints.push_back(std::weak_ptr<ObjectPoint>(objPoint));

    // --- BEGIN ENHANCED DEBUG ---
    std::cout << "Circle::addChildPoint: 'this' (Circle instance) pointer is " << this << std::endl;
    std::cout << "Circle::addChildPoint: Object type verification - this is a Circle: "
              << (this != nullptr) << std::endl;

    if (this) {  // Basic null check for 'this'
      // Check if this object inherits from enable_shared_from_this properly
      std::cout << "Circle::addChildPoint: Attempting to check enable_shared_from_this state..."
                << std::endl;

      std::weak_ptr<Circle> self_weak_check;
      try {
        self_weak_check = this->weak_from_this();  // Get the internal weak_ptr
        std::cout << "Circle::addChildPoint: weak_from_this() succeeded" << std::endl;
        std::cout << "Circle::addChildPoint: weak_from_this().expired() = "
                  << self_weak_check.expired() << std::endl;
        std::cout << "Circle::addChildPoint: weak_from_this().use_count() = "
                  << self_weak_check.use_count() << std::endl;

        if (self_weak_check.expired()) {
          std::cout << "Circle::addChildPoint: WARNING - weak_from_this() indicates no shared_ptr "
                       "owns this object!"
                    << std::endl;
          std::cout << "Circle::addChildPoint: This suggests enable_shared_from_this was never "
                       "properly initialized"
                    << std::endl;
          std::cout << "Circle::addChildPoint: or all shared_ptrs have been destroyed" << std::endl;
        }

      } catch (const std::bad_weak_ptr &bwp_ex) {
        std::cerr << "Circle::addChildPoint: EXCEPTION trying to call weak_from_this(): "
                  << bwp_ex.what() << std::endl;
        std::cerr
            << "Circle::addChildPoint: This indicates enable_shared_from_this was never initialized"
            << std::endl;
        // This itself indicates a problem with enable_shared_from_this setup or 'this' being
        // invalid
      }
    } else {
      std::cerr << "Circle::addChildPoint: CRITICAL ERROR - 'this' is nullptr." << std::endl;
    }
    // --- END ENHANCED DEBUG ---

    try {
      std::cout << "Circle::addChildPoint: About to call shared_from_this()..." << std::endl;
      // Use the generic setHost method instead
      auto selfPtr = std::static_pointer_cast<GeometricObject>(
          shared_from_this());  // This is where it crashes
      std::cout << "Circle::addChildPoint: shared_from_this() succeeded, selfPtr.use_count() = "
                << selfPtr.use_count() << std::endl;
      objPoint->setHost(selfPtr, ObjectType::Circle);
      std::cout << "Circle::addChildPoint: setHost() completed successfully" << std::endl;
    } catch (const std::bad_weak_ptr &e) {
      std::cerr << "Circle::addChildPoint: CAUGHT std::bad_weak_ptr on shared_from_this(): "
                << e.what() << std::endl;
      std::cerr << "  Circle 'this' address: " << this << std::endl;
      std::cerr << "  This means the Circle object is not currently managed by any shared_ptr"
                << std::endl;
      std::cerr << "  or the enable_shared_from_this mechanism was never properly initialized"
                << std::endl;
      // It's often good to re-throw to signal the failure upstream unless you can recover.
      throw;
    } catch (const std::exception &e) {
      std::cerr << "Circle::addChildPoint: CAUGHT std::exception: " << e.what() << std::endl;
      throw;
    }
  }
}

void Circle::removeChildPoint(std::shared_ptr<ObjectPoint> objPoint) {
  if (!objPoint) return;

  // Find and remove the point from our hosted list
  auto it = std::remove_if(m_hostedObjectPoints.begin(), m_hostedObjectPoints.end(),
                           [&objPoint](const std::weak_ptr<ObjectPoint> &weakPtr) {
                             if (auto sharedPtr = weakPtr.lock()) {
                               return sharedPtr.get() == objPoint.get();
                             }
                             // Also remove expired weak_ptrs
                             return true;
                           });

  if (it != m_hostedObjectPoints.end()) {
    // Clear the host from the point before erasing
    objPoint->clearHost();
    m_hostedObjectPoints.erase(it, m_hostedObjectPoints.end());
  }
}

std::vector<std::shared_ptr<ObjectPoint>> Circle::getHostedObjectPoints() const {
  std::vector<std::shared_ptr<ObjectPoint>> activePoints;

  for (const auto &weakPtr : m_hostedObjectPoints) {
    if (auto sharedPtr = weakPtr.lock()) {
      activePoints.push_back(sharedPtr);
    }
  }

  return activePoints;
}

// Alternative version that takes raw pointer for backward compatibility
void Circle::removeChildPoint(ObjectPoint *point) {
  if (!point) return;

  // Find and remove the point from our hosted list
  auto it = std::remove_if(m_hostedObjectPoints.begin(), m_hostedObjectPoints.end(),
                           [point](const std::weak_ptr<ObjectPoint> &weakPtr) {
                             if (auto sharedPtr = weakPtr.lock()) {
                               return sharedPtr.get() == point;
                             }
                             // Also remove expired weak_ptrs
                             return true;
                           });

  if (it != m_hostedObjectPoints.end()) {
    // Clear the host from the point before erasing
    point->clearHost();
    m_hostedObjectPoints.erase(it, m_hostedObjectPoints.end());
  }
}
//
void Circle::cleanupExpiredChildPoints() {
  auto it =
      std::remove_if(m_hostedObjectPoints.begin(), m_hostedObjectPoints.end(),
                     [](const std::weak_ptr<ObjectPoint> &weakPtr) { return weakPtr.expired(); });

  m_hostedObjectPoints.erase(it, m_hostedObjectPoints.end());
}
void Circle::removeChildPointByRawPointer(ObjectPoint *rawPtr) {
  if (!rawPtr) return;

  // Remove using raw pointer comparison to avoid shared_ptr issues during
  // destruction Assuming Circle has a similar collection like Line's
  // m_hostedObjectPoints
  m_hostedObjectPoints.erase(
      std::remove_if(m_hostedObjectPoints.begin(), m_hostedObjectPoints.end(),
                     [rawPtr](const std::weak_ptr<ObjectPoint> &weak_pt) {
                       if (auto pt = weak_pt.lock()) {
                         return pt.get() == rawPtr;
                       }
                       return true;  // Remove expired weak_ptrs too
                     }),
      m_hostedObjectPoints.end());

  std::cout << "Circle::removeChildPointByRawPointer: Removed ObjectPoint " << rawPtr << std::endl;
}
//
void Circle::updateHostedPoints() {
  // Clean up expired pointers first
  cleanupExpiredChildPoints();

  // Notify remaining valid points
  notifyObjectPoints();
}

bool Circle::contains(const sf::Vector2f &worldPos, float tolerance) const {
  if (m_isLocked) return false;

  Point_2 worldPos_cgal = Point::sfmlToCGAL(worldPos);
  Point_2 center_cgal = m_cgalCircle.center();

  Kernel::FT dist_sq_to_center = CGAL::squared_distance(center_cgal, worldPos_cgal);
  double dist_to_center = std::sqrt(CGAL::to_double(dist_sq_to_center));

  // Check if the point is close to the circumference
  return std::abs(dist_to_center - m_radius) <= tolerance;
}

bool Circle::isCenterPointHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  // Get center position in SFML coordinates
  sf::Vector2f center_sfml = Point::cgalToSFML(m_cgalCircle.center());

  // Calculate distance
  float dx = worldPos_sfml.x - center_sfml.x;
  float dy = worldPos_sfml.y - center_sfml.y;
  float distSq = dx * dx + dy * dy;

  return distSq <= (tolerance * tolerance);
}
bool Circle::isCircumferenceHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  if (m_isLocked) return false;

  Point_2 worldPos_cgal = Point::sfmlToCGAL(worldPos_sfml);
  Point_2 center_cgal = m_cgalCircle.center();

  Kernel::FT dist_sq_to_center = CGAL::squared_distance(center_cgal, worldPos_cgal);
  double dist_to_center = std::sqrt(CGAL::to_double(dist_sq_to_center));

  // Check if the point is close to the circumference
  // This is essentially the same logic as your current Circle::contains
  return std::abs(dist_to_center - m_radius) <= tolerance;
}

void Circle::notifyObjectPoints() {
  for (auto it = m_hostedObjectPoints.begin(); it != m_hostedObjectPoints.end();) {
    if (auto objPoint = it->lock()) {
      try {
        objPoint->updatePositionFromHost();
        ++it;
      } catch (const std::exception &e) {
        std::cerr << "Error updating ObjectPoint position during notification: " << e.what()
                  << std::endl;
        ++it;
      }
    } else {
      // Remove expired weak_ptr
      it = m_hostedObjectPoints.erase(it);
    }
  }
}
// projectOntoCircumference: return the point on the rim closest to clickPos
// --- Utility ---
Point_2 Circle::projectOntoCircumference(const Point_2 &p) const {
  if (!m_centerPointObject) {
    std::cerr << "Error: projectOntoCircumference called with no center point object." << std::endl;
    return p;  // Return original point if no center
  }

  Point_2 center = m_centerPointObject->getCGALPosition();

  // Validate center coordinates are finite
  bool center_is_finite = true;
  try {
    double cx = CGAL::to_double(center.x());
    double cy = CGAL::to_double(center.y());
    if (!std::isfinite(cx) || !std::isfinite(cy)) {
      center_is_finite = false;
    }
  } catch (const std::exception &e) {
    center_is_finite = false;
    std::cerr << "Error converting center coordinates: " << e.what() << std::endl;
  }

  if (!center_is_finite) {
    std::cerr << "Error: Circle center is not finite in projectOntoCircumference." << std::endl;
    return (std::isfinite(m_radius) && m_radius > Constants::EPSILON) ? Point_2(m_radius, 0) : p;
  }

  // Validate input point p coordinates are finite
  bool p_is_finite = true;
  try {
    double px = CGAL::to_double(p.x());
    double py = CGAL::to_double(p.y());
    if (!std::isfinite(px) || !std::isfinite(py)) {
      p_is_finite = false;
    }
  } catch (const std::exception &e) {
    p_is_finite = false;
    std::cerr << "Error converting point coordinates: " << e.what() << std::endl;
  }

  if (!p_is_finite) {
    std::cerr << "Error: Input point p is not finite in projectOntoCircumference." << std::endl;
    return (std::isfinite(m_radius) && m_radius > Constants::EPSILON)
               ? Point_2(center.x() + m_radius, center.y())
               : p;
  }

  // Validate radius
  if (!std::isfinite(m_radius) || m_radius < Constants::EPSILON) {
    std::cerr << "Warning: Invalid radius (" << m_radius << ") in projectOntoCircumference."
              << std::endl;
    return p;
  }

  Vector_2 vec(center, p);

  // Validate vector from center to p
  bool vec_components_finite = true;
  try {
    double vx_dbl = CGAL::to_double(vec.x());
    double vy_dbl = CGAL::to_double(vec.y());
    if (!std::isfinite(vx_dbl) || !std::isfinite(vy_dbl)) {
      vec_components_finite = false;
      std::cerr << "Error: Vector components not finite in projectOntoCircumference." << std::endl;
      return Point_2(center.x() + m_radius, center.y());
    }
  } catch (const std::exception &e) {
    std::cerr << "Error checking vector components in "
              << "projectOntoCircumference: " << e.what() << std::endl;
    vec_components_finite = false;
  }

  if (!vec_components_finite) {
    std::cerr << "Error: Vector from center to p has non-finite components in "
                 "projectOntoCircumference."
              << std::endl;
    return Point_2(center.x() + m_radius, center.y());
  }

  // Calculate squared length
  Kernel::FT sq_len_ft = vec.squared_length();

  // Validate squared length
  if (!CGAL::is_finite(sq_len_ft)) {  // CGAL::is_finite should work for Kernel::FT
    std::cerr << "Error: Vector squared length is not finite in "
                 "projectOntoCircumference."
              << std::endl;
    return Point_2(center.x() + m_radius, center.y());
  }

  // Handle case where p is at the center of the circle
  if (CGAL::is_zero(sq_len_ft)) {
    return Point_2(center.x() + Kernel::FT(m_radius), center.y());
  }

  // Convert to double for scaling operations
  double dist_dbl;
  try {
    dist_dbl = std::sqrt(CGAL::to_double(sq_len_ft));
    if (!std::isfinite(dist_dbl) || dist_dbl < Constants::EPSILON) {
      std::cerr << "Error: Calculated double distance 'dist_dbl' (" << dist_dbl
                << ") is not finite or near zero in projectOntoCircumference. "
                   "Defaulting to fallback."
                << std::endl;
      return Point_2(center.x() + m_radius, center.y());
    }
  } catch (const std::exception &e) {
    std::cerr << "Error converting squared length to double: " << e.what() << std::endl;
    return Point_2(center.x() + m_radius, center.y());
  }

  double scale_factor = m_radius / dist_dbl;
  if (!std::isfinite(scale_factor)) {
    std::cerr << "Error: Calculated scale_factor (" << scale_factor
              << ") is not finite in projectOntoCircumference. Defaulting to "
                 "fallback."
              << std::endl;
    return Point_2(center.x() + m_radius, center.y());
  }

  Vector_2 scaled_vec = vec * scale_factor;

  // Manual finiteness check for scaled_vec
  bool scaled_vec_components_finite = true;
  try {
    double svx_dbl = CGAL::to_double(scaled_vec.x());
    double svy_dbl = CGAL::to_double(scaled_vec.y());
    if (!std::isfinite(svx_dbl) || !std::isfinite(svy_dbl)) {
      scaled_vec_components_finite = false;
    }
  } catch (const CGAL::Uncertain_conversion_exception
               &ex) {  // Use different exception variable name if 'e' is used above
    std::cerr << "Warning: Uncertain conversion of scaled_vec components to "
                 "double: "
              << ex.what() << std::endl;
    scaled_vec_components_finite = false;
  } catch (const std::exception &ex) {
    std::cerr << "Warning: Exception converting scaled_vec components to double: " << ex.what()
              << std::endl;
    scaled_vec_components_finite = false;
  }

  if (!scaled_vec_components_finite) {
    std::cerr << "Error: Scaled vector has non-finite components in "
                 "projectOntoCircumference."
              << std::endl;
    return Point_2(center.x() + m_radius, center.y());
  }

  Point_2 result_point = center + scaled_vec;

  // Manual finiteness check for result_point
  bool result_point_components_finite = true;
  try {
    double rpx_dbl = CGAL::to_double(result_point.x());
    double rpy_dbl = CGAL::to_double(result_point.y());
    if (!std::isfinite(rpx_dbl) || !std::isfinite(rpy_dbl)) {
      result_point_components_finite = false;
    }
  } catch (const CGAL::Uncertain_conversion_exception &ex) {
    std::cerr << "Warning: Uncertain conversion of result_point components "
                 "to double: "
              << ex.what() << std::endl;
    result_point_components_finite = false;
  } catch (const std::exception &ex) {
    std::cerr << "Warning: Exception converting result_point components to double: " << ex.what()
              << std::endl;
    result_point_components_finite = false;
  }

  if (!result_point_components_finite) {
    std::cerr << "Error: Resulting projected point is not finite in "
                 "projectOntoCircumference."
              << std::endl;
    return Point_2(center.x() + m_radius, center.y());
  }
  return result_point;
}

void Circle::dragCircumferencePoint(const Point_2 &dragPoint) {
  if (m_isLocked || !m_centerPointObject) {
    return;
  }

  try {
    Point_2 center = m_centerPointObject->getCGALPosition();
    // Calculate squared distance first for precision with CGAL types
    Kernel::FT squared_radius_ft = CGAL::squared_distance(center, dragPoint);

    // Convert to double safely
    double new_radius =
        CGALSafeUtils::safe_sqrt(squared_radius_ft, "Circle::dragCircumferencePoint", m_radius);

    // Apply the radius update with proper bounds checking
    setRadius(std::max(new_radius, static_cast<double>(Constants::MIN_CIRCLE_RADIUS)));
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "CGAL conversion error in dragCircumferencePoint: " << e.what() << std::endl;
    // Continue with current radius
  } catch (const std::exception &e) {
    std::cerr << "Error in dragCircumferencePoint: " << e.what() << std::endl;
    // Continue with current radius
  }
}
void Circle::setRadius(double newRadius) {
  if (m_isLocked) return;
  m_radius = std::max(newRadius, static_cast<double>(Constants::MIN_CIRCLE_RADIUS));
  updateCGALCircle();
  updateSFMLShape();
  notifyObjectPoints();  // Notify points about radius change
}

void Circle::setFillColor(const sf::Color &color) {
  m_fillColor = color;
  updateSFMLShape();  // Update the visual representation
  // notifyObservers(); // If observers need to know about fill color changes
}

sf::Color Circle::getFillColor() const { return m_fillColor; }

sf::Color Circle::getOutlineColor() const { return m_outlineColor; }

// --- GeometricObject Overrides ---

void Circle::setSelected(bool sel) {
  if (m_selected == sel) return;
  m_selected = sel;
  updateSFMLShape();
}

void Circle::setHovered(bool hoveredStatus) {
  if (m_hovered == hoveredStatus)  // Fixed: changed m_isHovered to m_hovered
    return;
  m_hovered = hoveredStatus;  // Fixed: changed m_isHovered to m_hovered
  updateSFMLShape();
}

void Circle::update() {
  std::cout << "Circle::update() called - center: (";

  if (m_centerPointObject) {
    Point_2 center = m_centerPointObject->getCGALPosition();
    std::cout << CGAL::to_double(center.x()) << ", " << CGAL::to_double(center.y());
  } else {
    std::cout << "no center point object";
  }

  std::cout << "), radius: " << m_radius << std::endl;

  // Update CGAL representation based on m_centerPointObject and m_radius
  updateCGALCircle();
  // Update SFML representation based on CGAL circle
  updateSFMLShape();
  // Notify dependent objects like hosted ObjectPoints
  notifyObjectPoints();
}

sf::FloatRect Circle::getGlobalBounds() const {
  // Ensure m_sfmlShape is up-to-date before getting its bounds
  // const_cast<Circle*>(this)->updateSFMLShape(); // Risky if not careful,
  // prefer ensuring update elsewhere or making SFML shape update on demand. A
  // safer way is to ensure update() is called when properties change. For
  // now, assume m_sfmlShape is reasonably current.
  return m_sfmlShape.getGlobalBounds();
}

Point_2 Circle::getCGALPosition() const { return m_cgalCircle.center(); }

CGAL::Circle_2<CGAL::Epeck> Circle::getCGALCircle() const { return m_cgalCircle; }

// --- Modification Methods ---
void Circle::moveCenter(const Point_2 &newCenter) {
  if (m_isLocked) return;

  if (m_centerPointObject) {
    m_centerPointObject->setCGALPosition(newCenter);
    // The Point's setCGALPosition should trigger its own update,
    // and if Circle observes Point, Circle::update will be called.
    // If not observing, then Circle needs to update itself here.
    updateCGALCircle();    // Ensure CGAL circle is updated
    updateSFMLShape();     // Ensure SFML shape is updated
    notifyObjectPoints();  // Notify hosted points
  }
}

void Circle::setCGALPosition(const Point_2 &newPos) {
  if (m_isLocked) return;

  if (m_centerPointObject) {
    m_centerPointObject->setCGALPosition(newPos);
    // The update method of Circle (or observation mechanism if re-added)
    // will handle updating m_cgalCircle and m_sfmlShape.
    // For now, explicitly update:
    updateCGALCircle();
    updateSFMLShape();
    notifyObjectPoints();
  } else {
    // This case (no m_centerPointObject) should ideally not happen if
    // constructors always create it.
    // If it can happen, then update m_cgalCircle directly:
    m_cgalCircle = Circle_2(newPos, m_radius * m_radius);
    updateSFMLShape();
    notifyObjectPoints();
  }
}

void Circle::setLocked(bool lockStatus) { m_isLocked = lockStatus; }

bool Circle::isLocked() const { return m_isLocked; }

void Circle::translateWithDependents(const Vector_2 &offset) {
  // Store the current attachment parameters of any hosted object points
  std::vector<std::pair<std::shared_ptr<ObjectPoint>, double>> pointAngles;

  // For each object point attached to this circle, save its current angle
  for (const auto &weakPtr : m_hostedObjectPoints) {
    if (auto objPoint = weakPtr.lock()) {
      pointAngles.push_back({objPoint, objPoint->getAngleOnCircle()});
    }
  }

  // Now translate the circle itself
  translate(offset);

  // Finally, update each object point's position
  for (auto &[objPoint, angle] : pointAngles) {
    if (objPoint && objPoint->getHostObject() == this) {
      try {
        objPoint->updatePositionFromHost();
      } catch (const std::exception &e) {
        std::cerr << "Error updating object point position: " << e.what() << std::endl;
      }
    }
  }
}

void Circle::translate(const Vector_2 &offset) {
  if (m_isLocked) return;  // Don't move if locked

  if (m_centerPointObject) {
    // Get current position
    Point_2 currentPos = m_centerPointObject->getCGALPosition();
    // Calculate new position
    Point_2 newPos = Point_2(currentPos.x() + offset.x(), currentPos.y() + offset.y());
    // Update position
    m_centerPointObject->setCGALPosition(newPos);
  } else {
    // If there's no center point object, update the CGAL circle directly
    Point_2 currentCenter = m_cgalCircle.center();
    Point_2 newCenter(currentCenter.x() + offset.x(), currentCenter.y() + offset.y());
    m_cgalCircle = CGAL::Circle_2<CGAL::Epeck>(newCenter, m_radius * m_radius);
    updateSFMLShape();
    notifyObjectPoints();
  }
}

void Circle::setPosition(const sf::Vector2f &newSfmlPos) {
  if (m_isLocked) return;  // Don't move if locked

  // Convert SFML position to CGAL position
  Point_2 newCgalPos = Point::sfmlToCGAL(newSfmlPos);

  if (m_centerPointObject) {
    m_centerPointObject->setCGALPosition(newCgalPos);
    // The update will be triggered by the center point
  } else {
    // If there's no center point object, update the CGAL circle directly
    Point_2 newCgalPos = Point::sfmlToCGAL(newSfmlPos);
    m_cgalCircle = Circle_2(newCgalPos, m_radius * m_radius);
    updateSFMLShape();
    notifyObjectPoints();
  }
}

// --- Interaction ---
CircleInteractionMode Circle::handleInteractionEvent(const sf::Event &event,
                                                     const Point_2 &worldPos_cgal,
                                                     float tolerance) {
  // sf::Vector2f worldPos_sfml = Point::cgalToSFML(worldPos_cgal); // Line
  // 378: Commented out due to unused variable warning. Uncomment if needed.
  sf::Vector2f worldPos_sfml_for_use;  // Declare a new variable if needed for
                                       // specific calls Or, pass worldPos_cgal
                                       // and convert inside specific methods
                                       // like isCenterPointHovered if they
                                       // only take sf::Vector2f

  if (event.type == sf::Event::MouseButtonPressed) {
    // Example: if isCenterPointHovered needs sf::Vector2f:
    worldPos_sfml_for_use = Point::cgalToSFML(worldPos_cgal);
    if (isCenterPointHovered(worldPos_sfml_for_use, tolerance)) {
      m_interactionMode = CircleInteractionMode::DragCenter;
      m_lastDragPos_cgal = worldPos_cgal;  // Store CGAL position for dragging
      return m_interactionMode;
    }
    // Example: if contains needs sf::Vector2f:
    // worldPos_sfml_for_use = Point::cgalToSFML(worldPos_cgal); // Already
    // converted if needed above
    if (this->contains(worldPos_sfml_for_use, tolerance)) {
      // Ensure it's not the center to differentiate resize from center drag
      // This check should use CGAL points for precision with geometric
      // properties
      if (CGAL::squared_distance(m_cgalCircle.center(), worldPos_cgal) >
          (tolerance * tolerance)) {  // A small buffer to avoid issues if
                                      // tolerance is very small
        m_interactionMode = CircleInteractionMode::Resize;
        m_lastDragPos_cgal = worldPos_cgal;  // Store CGAL position
        return m_interactionMode;
      }
    }
  } else if (event.type == sf::Event::MouseMoved) {
    if (m_interactionMode == CircleInteractionMode::DragCenter) {
      moveCenter(worldPos_cgal);
      m_lastDragPos_cgal = worldPos_cgal;
      return m_interactionMode;
    } else if (m_interactionMode == CircleInteractionMode::Resize) {
      dragCircumferencePoint(worldPos_cgal);
      m_lastDragPos_cgal = worldPos_cgal;
      return m_interactionMode;
    }
  } else if (event.type == sf::Event::MouseButtonReleased) {
    m_interactionMode = CircleInteractionMode::None;
    return m_interactionMode;
  }
  return CircleInteractionMode::None;
}

// --- Private Helper Methods ---
