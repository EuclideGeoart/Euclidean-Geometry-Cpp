#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message("CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif

#include "Point.h"
#include "CGALSafeUtils.h"
#include "Constants.h"
#include "Line.h"
#include "PointUtils.h" // Added PointUtils.h
#include "QuickProfiler.h"
#include "Transforms.h"                                        // For toSFMLVector, toCGALPoint
//#include "LabelManager.h"                                // For font size
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>  // For Point_2
#include <CGAL/number_utils.h>                                 // For CGAL::to_double
#include <cmath>                                               // For std::sqr
#include <iostream>

using namespace CGALSafeUtils;

// Definiing Static Font Member
const sf::Font* Point::commonFont = nullptr;

// void debug_cgal_point(const Point_2 &pt, const std::string &point_name,
//                       const std::string &context);
// --- Constructors ---
Point::Point(float initialZoomFactor)
    : GeometricObject(ObjectType::Point, Constants::POINT_DEFAULT_COLOR),
      m_cgalPosition(safe_zero_ft(), safe_zero_ft()),
      m_fillColor(Constants::POINT_FILL_COLOR),
      m_outlineColor(Constants::POINT_DEFAULT_COLOR),
      m_outlineThickness(Constants::POINT_OUTLINE_THICKNESS),
      m_isDragging(false),
      m_isIntersectionPoint(false),
      m_isInitialized(true),
      m_desiredScreenRadius(Constants::POINT_DRAW_RADIUS_SCREEN_PIXELS) {
  m_radius = m_desiredScreenRadius / (initialZoomFactor > 0 ? initialZoomFactor : 1.0f);
  initializeShape();
}

// ctor from CGAL
Point::Point(const Point_2 &cgalPos, float initialZoomFactor, const sf::Color &fillColor,
             const sf::Color &outlineColor)
    : GeometricObject(ObjectType::Point, fillColor),
      m_cgalPosition(cgalPos),
      m_fillColor(fillColor),
      m_outlineColor(outlineColor),
      m_outlineThickness(Constants::POINT_OUTLINE_THICKNESS),
      m_isDragging(false),
      m_isIntersectionPoint(false),
      m_isInitialized(true),
      m_desiredScreenRadius(Constants::POINT_DRAW_RADIUS_SCREEN_PIXELS) {
  m_radius = m_desiredScreenRadius / (initialZoomFactor > 0 ? initialZoomFactor : 1.0f);
  initializeShape();
}

// ctor from SFML
Point::Point(const sf::Vector2f &sfmlPos, float initialZoomFactor, const sf::Color &fillColor,
             const sf::Color &outlineColor)
    : GeometricObject(ObjectType::Point, fillColor),
      m_cgalPosition(sfmlToCGAL(sfmlPos)),
      m_fillColor(fillColor),
      m_outlineColor(outlineColor),
      m_outlineThickness(Constants::POINT_OUTLINE_THICKNESS),
      m_isDragging(false),
      m_isIntersectionPoint(false),
      m_isInitialized(true),
      m_desiredScreenRadius(Constants::POINT_DRAW_RADIUS_SCREEN_PIXELS) {
  m_radius = m_desiredScreenRadius / (initialZoomFactor > 0 ? initialZoomFactor : 1.0f);
  initializeShape();
}

// NEW Constructor with CGAL point, color, and ID
Point::Point(const Point_2 &cgal_point, float initialZoomFactor, const sf::Color &fillColor,
             unsigned int id, const sf::Color &outlineColor)
    : GeometricObject(ObjectType::Point, fillColor, cgal_point, id),
      m_cgalPosition(cgal_point),
      m_fillColor(fillColor),
      m_outlineColor(outlineColor),
      m_outlineThickness(Constants::POINT_OUTLINE_THICKNESS),
      m_isDragging(false),
      m_isIntersectionPoint(false),
      m_isInitialized(true),
      m_desiredScreenRadius(Constants::POINT_DRAW_RADIUS_SCREEN_PIXELS) {
  m_radius = m_desiredScreenRadius / (initialZoomFactor > 0 ? initialZoomFactor : 1.0f);
  if (!CGAL::is_finite(cgal_point.x()) || !CGAL::is_finite(cgal_point.y())) {
    std::cerr << "Point Constructor WARNING: CGAL point has non-finite "
                 "coordinates! ID: "
              << getID() << std::endl;
    debug_cgal_point(cgal_point, "ctor_input_point_with_id", "PointCtorWithID");
  }
  initializeShape();
  if (Constants::DEBUG_POINT_DRAWING) {
    std::cout << "Point Constructor (Point_2, Color, ID): ID " << getID() << " at ("
              << CGAL::to_double(m_cgalPosition.x()) << ", " << CGAL::to_double(m_cgalPosition.y())
              << ")" << std::endl;
  }
}
// Implement a safe destructor for Point
Point::~Point() {
  if (Constants::LIFECYCLE) {
    std::cout << "Point::~Point: Destroying point " << this << " with ID " << getID()
              << std::endl;
  }
  try {
    std::cerr << "Point::~Point: Starting destruction of point " << this << " with ID " << getID()
              << std::endl;

    // CRITICAL: Just clear the list without notifying lines
    // Any attempt to access or notify the connected lines during destruction
    // could lead to circular destruction and crashes
    if (!m_connectedLines.empty()) {
      std::cerr << "Point::~Point: Clearing " << m_connectedLines.size()
                << " connected lines without notification" << std::endl;
      m_connectedLines.clear();
    }

    std::cerr << "Point::~Point: Completed destruction of point " << this << std::endl;
  } catch (...) {
    // Suppress all exceptions during destruction
    std::cerr << "Point::~Point: Caught exception during destruction" << std::endl;
  }
}

void Point::initializeShape() {
  m_sfmlShape.setRadius(this->m_radius);  // Use the calculated m_radius
  m_sfmlShape.setOrigin(this->m_radius,
                        this->m_radius);        // Use the calculated m_radius
  m_sfmlShape.setFillColor(this->m_fillColor);  // Use m_fillColor for consistency
  m_sfmlShape.setOutlineThickness(Constants::POINT_OUTLINE_THICKNESS);
  m_sfmlShape.setOutlineColor(isLocked() ? Constants::LOCKED_COLOR
                                         : Constants::POINT_DEFAULT_COLOR);
  // Consider if updateSFMLShape() should be called here to ensure all states
  // are immediately reflected, though it might be redundant if constructors set
  // states that updateSFMLShape also handles. For now, ensuring m_fillColor is
  // used is the primary fix for initial color.
}
// Clean up connected lines
void Point::cleanupConnectedLines() {
  // Simply clear the lines without attempting any notification
  // This prevents circular references during destruction
  m_connectedLines.clear();
}

void Point::setColor(const sf::Color &color) {
  // NO CALL to GeometricObject::setColor here
  this->m_color = color;      // Set the inherited m_color from GeometricObject
  this->m_fillColor = color;  // Set Point's own fill color
  // this->m_outlineColor = color; // Optionally set outline color too
  this->updateSFMLShape();  // Update the SFML shape
}

void Point::setFillColor(const sf::Color &fillColor) {
  m_fillColor = fillColor;
  updateSFMLShape();
}

void Point::setOutlineColor(const sf::Color &outlineColor) {
  m_outlineColor = outlineColor;
  updateSFMLShape();
}

void Point::setIsValid(bool valid) {
  m_isValid = valid;
  updateSFMLShape();
}

sf::Color Point::getFillColor() const { return m_fillColor; }
sf::Color Point::getOutlineColor() const { return m_outlineColor; }

// --- SFML and CGAL Conversions ---
// --- Utility Conversions ---
// Static, so they can be called as Point::cgalToSFML(...)
sf::Vector2f Point::cgalToSFML(
    const Point_2 &p) {  // Removed const from function signature, as it's static
  return sf::Vector2f(static_cast<float>(CGAL::to_double(p.x())),
                      static_cast<float>(CGAL::to_double(p.y())));
}

// Static, so they can be called as Point::sfmlToCGAL(...)
Point_2 Point::sfmlToCGAL(
    const sf::Vector2f &p_sfml) {  // Removed const from function signature, as it's static
  return Point_2(FT(p_sfml.x), FT(p_sfml.y));  // Use FT for construction
}

void Point::setCGALPosition(const Point_2 &newPos) {
  QUICK_PROFILE("Point::setCGALPosition");
  
  try {
    if (Constants::DEBUG_POINT_UPDATE) {
      std::cout << "Point " << this << " setCGALPosition: New CGAL Pos: ("
                << CGAL::to_double(newPos.x()) << ", " << CGAL::to_double(newPos.y())
                << "), Finite: " << (CGAL::is_finite(newPos.x()) && CGAL::is_finite(newPos.y()))
                << std::endl;
    }

    if (!CGAL::is_finite(newPos.x()) || !CGAL::is_finite(newPos.y())) {
      std::cerr << "Point " << this
                << " setCGALPosition: Attempted to set non-finite CGAL position. "
                   "Marking invalid."
                << std::endl;
      m_isValid = false;
      m_cgalPosition = newPos;
      updateSFMLShape();
      return;
    }

    m_cgalPosition = newPos;
    m_isValid = true;
    updateSFMLShape();  // Always update visual immediately

    // PERFORMANCE FIX: Only update constraints if not deferred
    if (!m_deferConstraintUpdates) {
      QUICK_PROFILE("Point::updateConnectedLines");
      updateConnectedLines();
      updateHostedPoints(); // Notify hosted points and general dependents
    }
  } catch (const std::exception& e) {
    std::cerr << "Point::setCGALPosition - Exception: " << e.what() << std::endl;
    m_isValid = false;
  } catch (...) {
    std::cerr << "Point::setCGALPosition - Unknown exception" << std::endl;
    m_isValid = false;
  }
}
void Point::updateSFMLShape() {
  if (Constants::DEBUG_POINT_UPDATE) {
    std::cout << "Point " << this << " updateSFMLShape: m_isValid=" << m_isValid
              << " m_radius=" << m_radius;
    if (m_isValid) {
      std::cout << ", Current CGAL Pos: (" << CGAL::to_double(m_cgalPosition.x()) << ", "
                << CGAL::to_double(m_cgalPosition.y()) << ")";
    }
    std::cout << std::endl;
  }

  if (!m_isValid) {
    // Set to a default "invalid" appearance
    m_sfmlShape.setFillColor(sf::Color::Transparent);
    m_sfmlShape.setOutlineColor(sf::Color::Red);
    m_sfmlShape.setOutlineThickness(1.0f / Constants::CURRENT_ZOOM);  // Minimal thickness
    m_sfmlShape.setRadius(Constants::POINT_DRAW_RADIUS_SCREEN_PIXELS / Constants::CURRENT_ZOOM);
    m_sfmlShape.setOrigin(m_sfmlShape.getRadius(), m_sfmlShape.getRadius());
    m_sfmlShape.setPosition(-10000, -10000);  // Off-screen
    return;
  }

  try {
    sf::Vector2f sfmlPos = cgalToSFML(m_cgalPosition);
    if (std::isnan(sfmlPos.x) || std::isinf(sfmlPos.x) || std::isnan(sfmlPos.y) ||
        std::isinf(sfmlPos.y)) {
      std::cerr << "Point " << this
                << " updateSFMLShape: Non-finite SFML position. Marking invalid." << std::endl;
      m_isValid = false;  // Mark as invalid if conversion results in non-finite
      // Call recursively to set invalid appearance
      updateSFMLShape();
      return;
    }

    m_sfmlShape.setPosition(sfmlPos);
    m_sfmlShape.setRadius(m_radius);
    m_sfmlShape.setOrigin(m_radius, m_radius);

    sf::Color currentFill = m_fillColor;        // Default fill color
    sf::Color currentOutline = m_outlineColor;  // Default outline color
    // Base outline thickness in world units, draw() handles screen scaling for it.
    // float currentOutlineThickness = m_outlineThickness;

    if (isSelected()) {
      currentFill = Constants::SELECTION_COLOR_POINT_FILL;
      currentOutline = Constants::SELECTION_COLOR_POINT_OUTLINE;
    } else if (isHovered()) {                          // Use the standardized isHovered()
      currentFill = Constants::HOVER_UNIVERSAL_COLOR;  // Universal fill for hover
      currentOutline = Constants::HOVER_COLOR_POINT_OUTLINE;
    } else if (isLocked()) {
      currentFill = Constants::LOCKED_COLOR;
      currentOutline = sf::Color::Black;  // Or Constants::LOCKED_OUTLINE_COLOR
    }
    // If it's an intersection point and not selected/hovered/locked, apply its specific color
    else if (m_isIntersectionPoint) {
      currentFill = Constants::INTERSECTION_POINT_COLOR;
      currentOutline = Constants::POINT_DEFAULT_COLOR;  // Or a default outline
    }

    m_sfmlShape.setFillColor(currentFill);
    m_sfmlShape.setOutlineColor(currentOutline);
    // m_sfmlShape's outline thickness is the base world thickness.
    // The draw() method will scale this for screen consistency.
    m_sfmlShape.setOutlineThickness(m_outlineThickness);

  } catch (const std::exception &e) {
    std::cerr << "Point " << this << " updateSFMLShape: Exception: " << e.what()
              << ". Marking point invalid." << std::endl;
    m_isValid = false;
    // Call recursively to set invalid appearance
    updateSFMLShape();
  }
}
/* void Point::updateSFMLShape() {
  try {
    // Safety check for infinite or NaN values
    if (!isValid()) {
      m_sfmlShape.setPosition(0, 0); // Default position
      m_sfmlShape.setRadius(Constants::POINT_RADIUS);
      m_sfmlShape.setOrigin(Constants::POINT_RADIUS, Constants::POINT_RADIUS);
      m_sfmlShape.setFillColor(m_color);
      return;
    }

    sf::Vector2f pos_sfml;
    try {
      // Get position
      Point_2 pos_cgal = getCGALPosition();

      // Convert to SFML
      pos_sfml.x = static_cast<float>(CGAL::to_double(pos_cgal.x()));
      pos_sfml.y = static_cast<float>(CGAL::to_double(pos_cgal.y()));

      // Check for NaN or infinity
      if (std::isnan(pos_sfml.x) || std::isinf(pos_sfml.x) ||
          std::isnan(pos_sfml.y) || std::isinf(pos_sfml.y)) {
        std::cerr << "Warning: Point has non-finite coordinates" << std::endl;
        pos_sfml.x = 0;
        pos_sfml.y = 0;
      }
    } catch (const std::exception &e) {
      std::cerr << "Error converting point coords: " << e.what() << std::endl;
      pos_sfml.x = 0;
      pos_sfml.y = 0;
    }

    // Update SFML circle shape
    m_sfmlShape.setPosition(pos_sfml);
    m_sfmlShape.setRadius(Constants::POINT_RADIUS);
    m_sfmlShape.setOrigin(Constants::POINT_RADIUS, Constants::POINT_RADIUS);

    // Set the fill color based on state
    if (m_selected) {
      m_sfmlShape.setFillColor(Constants::SELECTION_COLOR);
    } else if (m_hovered) {
      m_sfmlShape.setFillColor(Constants::HOVER_COLOR);
    } else {
      m_sfmlShape.setFillColor(m_color);
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception in Point::updateSFMLShape: " << e.what()
              << std::endl;
    // Set default values to avoid having an invalid shape
    m_sfmlShape.setPosition(0, 0);
    m_sfmlShape.setRadius(Constants::POINT_RADIUS);
    m_sfmlShape.setOrigin(Constants::POINT_RADIUS, Constants::POINT_RADIUS);
    m_sfmlShape.setFillColor(m_color);
  }
} */

// Ensure this implementation exists
void Point::updateSFMLShape(const sf::Vector2f &position) {
  m_sfmlShape.setRadius(m_radius);
  m_sfmlShape.setOrigin(m_radius, m_radius);
  m_sfmlShape.setPosition(position);

  sf::Color currentFill = m_fillColor;
  sf::Color currentOutline = m_outlineColor;

  if (isLocked()) {
    currentFill = Constants::LOCKED_COLOR;
    currentOutline = sf::Color::Black;
  } else if (isSelected()) {
    currentFill = Constants::SELECTION_COLOR_POINT_FILL;
    currentOutline = Constants::SELECTION_COLOR_POINT_OUTLINE;
  } else if (isHovered()) {
    currentFill = Constants::HOVER_UNIVERSAL_COLOR;
    currentOutline = Constants::HOVER_COLOR_POINT_OUTLINE;
  } else if (m_isIntersectionPoint) {
    currentFill = Constants::INTERSECTION_POINT_COLOR;
    currentOutline = Constants::POINT_DEFAULT_COLOR;
  }

  m_sfmlShape.setFillColor(currentFill);
  m_sfmlShape.setOutlineColor(currentOutline);
  m_sfmlShape.setOutlineThickness(m_outlineThickness);
}

void Point::translate(const Vector_2 &offset) {
  if (!m_isInitialized) {
    std::cerr << "Point::translate: Point not initialized. Translation aborted." << std::endl;
    return;
  }
  if (!CGAL::is_finite(offset.x()) || !CGAL::is_finite(offset.y())) {
    std::cerr << "Point::translate: Invalid (non-finite) offset vector. "
                 "Translation aborted."
              << std::endl;
    return;
  }
  if (!CGAL::is_finite(m_cgalPosition.x()) || !CGAL::is_finite(m_cgalPosition.y())) {
    std::cerr << "Point::translate: Current position is non-finite. Attempting "
                 "to reset to (0,0) before translation."
              << std::endl;
    m_cgalPosition = Point_2(safe_zero_ft(), safe_zero_ft());  // Attempt recovery
    // If recovery isn't desired, simply return or throw.
  }

  try {
    Point_2 newPosition = m_cgalPosition + offset;  // This is the CGAL operation
    if (!CGAL::is_finite(newPosition.x()) || !CGAL::is_finite(newPosition.y())) {
      std::cerr << "Point::translate: Translation resulted in non-finite "
                   "position. Aborting update."
                << std::endl;
      return;
    }
    m_cgalPosition = newPosition;
    updateSFMLShape();  // <— unified
    updateConnectedLines();
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "Point::translate: Uncertain conversion during translation: " << e.what()
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Point::translate: Exception during translation: " << e.what() << std::endl;
  }
}

sf::FloatRect Point::getGlobalBounds() const { return m_sfmlShape.getGlobalBounds(); }

// --- GeometricObject Overrides ---
void Point::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!isVisible() && !forceVisible) return;
  if (!isValid()) return;  // Don't draw if invalid

  if (Constants::DEBUG_POINT_DRAWING) {
    std::cout << "Point::draw: ID " << getID() << " CGAL Pos=("
              << CGAL::to_double(m_cgalPosition.x()) << "," << CGAL::to_double(m_cgalPosition.y())
              << ")"
              << " World Radius: " << m_radius << " IsHovered: " << isHovered() << std::endl;
  }

  // m_sfmlShape should already have its correct colors and base properties
  sf::CircleShape pointToDraw = m_sfmlShape;  // Make a copy to modify for drawing

  pointToDraw.setPosition(cgalToSFML(m_cgalPosition));
  
  // Apply visual invariance: scale screen pixels to world units
  float currentRadius = m_desiredScreenRadius * scale;
  pointToDraw.setRadius(currentRadius);
  pointToDraw.setOrigin(currentRadius, currentRadius);

  // Outline thickness scaling
  float baseScreenOutlineThickness = Constants::POINT_OUTLINE_THICKNESS;

  if (isSelected()) {
    baseScreenOutlineThickness = Constants::SELECTION_THICKNESS_POINT;
  } else if (isHovered()) {
    baseScreenOutlineThickness = Constants::HOVER_THICKNESS_POINT;
  }

  pointToDraw.setOutlineThickness(baseScreenOutlineThickness * scale);

  // GHOST MODE: Apply transparency if hidden but forced visible
  if (!isVisible() && forceVisible) {
      sf::Color ghostFill = pointToDraw.getFillColor();
      ghostFill.a = 50; // Faint alpha
      pointToDraw.setFillColor(ghostFill);
      
      sf::Color ghostOutline = pointToDraw.getOutlineColor();
      ghostOutline.a = 50;
      pointToDraw.setOutlineColor(ghostOutline);
  }

  // Selection Halo (Selection PRIORITY: Only show when selected)
  if (isSelected()) {
      drawHalo(window, 10.0f);
  }

  window.draw(pointToDraw);


}

void Point::drawLabel(sf::RenderWindow &window, const sf::View &worldView) const {
  if (!m_visible) return;
  if (!getShowLabel() || getLabelMode() == LabelMode::Hidden) return;
  drawLabelExplicit(window, worldView);
}

void Point::drawLabelExplicit(sf::RenderWindow &window, const sf::View &worldView) const {
  if (!m_visible) return;
  if (!isValid()) return;

  std::string labelStr = "";
  switch (getLabelMode()) {
      case LabelMode::Name: labelStr = getLabel(); break;
      case LabelMode::Value: {
          // Coordinates - Use precision? Fixed to int for now to match angle
          labelStr = "(" + std::to_string(static_cast<int>(std::round(CGAL::to_double(m_cgalPosition.x())))) + ", " + 
                           std::to_string(static_cast<int>(std::round(CGAL::to_double(m_cgalPosition.y())))) + ")";
          break;
      }
      case LabelMode::NameAndValue: {
           labelStr = getLabel() + (getLabel().empty() ? "" : " = ") + "(" + std::to_string(static_cast<int>(std::round(CGAL::to_double(m_cgalPosition.x())))) + ", " + 
                           std::to_string(static_cast<int>(std::round(CGAL::to_double(m_cgalPosition.y())))) + ")";
           break;
      }
      case LabelMode::Caption: labelStr = getCaption(); break;
      default: break;
  }

  if (labelStr.empty()) return;

  // 1. World -> Screen conversion
  sf::Vector2f worldPos = cgalToSFML(m_cgalPosition);
  sf::Vector2i screenPos = window.mapCoordsToPixel(worldPos, worldView);

  // 2. Text setup
  sf::Text text;
  text.setFont(LabelManager::instance().getSelectedFont());
  text.setString(m_label);
  text.setCharacterSize(LabelManager::instance().getFontSize()); // Use global font size
  text.setFillColor(Constants::AXIS_LABEL_COLOR);
  
  // 3. Position (Screen Space)
  sf::Vector2f finalPos(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
  
  // Apply visual offset (m_labelOffset is treated as screen pixels)
  // Default offset is usually (10, -10)
  finalPos += m_labelOffset; 
  
  // 4. Pixel Snapping for sharpness
  finalPos.x = std::round(finalPos.x);
  finalPos.y = std::round(finalPos.y);
  
  text.setPosition(finalPos);
  
  // 5. Draw (Caller ensures view is DefaultView)
  window.draw(text);
}
bool Point::contains(const sf::Vector2f &worldPos_sfml,
                     float tolerance) const {  // Ensure tolerance is used
  if (!m_isInitialized) return false;
  // If tolerance is meant to be POINT_INTERACTION_RADIUS by default,
  // it's handled by the header. If it's passed explicitly, use the passed
  // value. The default in the header is now POINT_INTERACTION_RADIUS.
  sf::Vector2f shapePos_sfml = m_sfmlShape.getPosition();
  sf::Vector2f diff = worldPos_sfml - shapePos_sfml;
  float distSq = diff.x * diff.x + diff.y * diff.y;
  // Use the provided tolerance, which defaults to POINT_INTERACTION_RADIUS
  return distSq <= tolerance * tolerance;
}

void Point::setSelected(bool sel) {
  GeometricObject::setSelected(sel);
  updateSFMLShape();  // Always update the shape when selection changes
}

void Point::setHovered(bool hover) {
  if (!isValid()) {
    if (isHovered()) {  // If it thought it was hovered but is invalid, ensure it's unhovered
      GeometricObject::setHovered(false);
    }
    return;
  }

  GeometricObject::setHovered(hover);
  updateSFMLShape();  // Update visuals based on new hover state
}

void Point::setLocked(bool lockStatus) {
    GeometricObject::setLocked(lockStatus);
    updateSFMLShape();
}

bool Point::isLocked() const {
    return GeometricObject::isLocked();
}

void Point::setVisible(bool v) {
    GeometricObject::setVisible(v);
    updateSFMLShape();
}

// --- Position Management ---
Point_2 Point::getCGALPosition() const {
  if (!m_isInitialized) {
    // std::cerr << "Warning: Accessing position of uninitialized Point.
    // Returning (0,0)." << std::endl;
    return Point_2(safe_zero_ft(), safe_zero_ft());
  }
  return m_cgalPosition;
}

sf::Vector2f Point::getSFMLPosition() const {
  if (!m_isInitialized) {
    // std::cerr << "Warning: Accessing SFML position of uninitialized Point.
    // Returning (0,0)." << std::endl;
    return sf::Vector2f(0.f, 0.f);
  }
  return cgalToSFML(m_cgalPosition);
}

/* void Point::setCGALPosition(const Point_2 &newCgalPos) {
  try {
    // Validate position early
    [[maybe_unused]] double px = CGAL::to_double(newCgalPos.x());
    [[maybe_unused]] double py = CGAL::to_double(newCgalPos.y());

    // If we make it here, the position is valid for conversion
    m_cgalPosition = newCgalPos;
    // Update the SFML position based on the new CGAL position
    updateSFMLPosition();
    // Update all connected lines (and other objects if needed)
    updateConnectedLines();
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "Point::setCGALPosition: Uncertain conversion with position
("
              << "could not convert to double): " << e.what() << std::endl;
    // Don't update position if invalid
  } catch (const std::exception &e) {
    std::cerr << "Point::setCGALPosition: Exception: " << e.what() <<
std::endl;
  }
} */

void Point::setPosition(const sf::Vector2f &newSfmlPos) { setCGALPosition(sfmlToCGAL(newSfmlPos)); }

sf::Color Point::getColor() const { return m_color; }

// --- Point Specific Methods ---

void Point::dragTo(const sf::Vector2f &targetSfmlPos) {
  if (isLocked() || !m_isInitialized) return;
  setPosition(targetSfmlPos);  // This will call setCGALPosition ->
                               // updateConnectedLines
}
void Point::addConnectedLine(std::weak_ptr<Line> line) {
  // Check if line already exists
  for (const auto &weak_line : m_connectedLines) {
    if (auto existing_line = weak_line.lock()) {
      if (auto new_line = line.lock()) {
        if (existing_line.get() == new_line.get()) {
          return;  // Already exists
        }
      }
    }
  }
  m_connectedLines.push_back(line);
}

void Point::removeConnectedLine(Line *line) {
  if (!line) return;

  m_connectedLines.erase(std::remove_if(m_connectedLines.begin(), m_connectedLines.end(),
                                        [line](const std::weak_ptr<Line> &weak_line) {
                                          if (auto line_ptr = weak_line.lock()) {
                                            return line_ptr.get() == line;
                                          }
                                          return true;  // Remove expired weak_ptrs
                                        }),
                         m_connectedLines.end());
}

const std::vector<std::weak_ptr<Line>> &Point::getConnectedLines() const {
  return m_connectedLines;
}

void Point::updateConnectedLines() {
  if (!m_isInitialized) return;

  // std::cout << "Point " << getID() << " updating " << m_connectedLines.size() << " connected lines" << std::endl;

  for (const auto &weakLine : m_connectedLines) {
    if (auto line = weakLine.lock()) {
      try {
        line->setExternallyMovedEndpoint(this);
        line->update();
      } catch (const std::exception &e) {
        std::cerr << "Point::updateConnectedLines: Exception: " << e.what() << std::endl;
      }
    }
  }
}

void Point::notifyConnectedLines() {
  // Clean expired weak_ptrs first
  m_connectedLines.erase(std::remove_if(m_connectedLines.begin(), m_connectedLines.end(),
                                        [](const std::weak_ptr<Line> &wp) { return wp.expired(); }),
                         m_connectedLines.end());

  for (const auto &weakLine : m_connectedLines) {
    if (auto line = weakLine.lock()) {
      if (line->isValid()) {
        line->updateCGALLine();
      }
    }
  }
}
// --- Other Methods ---

bool Point::isIntersectionPoint() const { return m_isIntersectionPoint; }

// Add these transformation methods
void Point::transform(const CoordinateTransform &transform_matrix) {
  if (!m_isInitialized) return;
  try {
    setCGALPosition(m_cgalPosition.transform(transform_matrix));
  } catch (const std::exception &e) {
    std::cerr << "Point::transform: Exception: " << e.what() << std::endl;
  }
}

void Point::scale(float factor) {
  if (!m_isInitialized) return;
  // Scaling a point relative to origin (0,0).
  // If scaling relative to another center is needed, adjust logic.
  try {
    Vector_2 vec_to_origin(m_cgalPosition.x(), m_cgalPosition.y());
    vec_to_origin = vec_to_origin * FT(factor);
    setCGALPosition(Point_2(vec_to_origin.x(), vec_to_origin.y()));
  } catch (const std::exception &e) {
    std::cerr << "Point::scale: Exception: " << e.what() << std::endl;
  }
}

void Point::rotate(float angleRadians) {
  if (!m_isInitialized) return;
  // Rotating a point relative to origin (0,0).
  // If rotation relative to another center is needed, translate, rotate,
  // translate back.
  try {
    Aff_transformation_2 rotation(CGAL::ROTATION, std::sin(angleRadians), std::cos(angleRadians));
    setCGALPosition(m_cgalPosition.transform(rotation));
  } catch (const std::exception &e) {
    std::cerr << "Point::rotate: Exception: " << e.what() << std::endl;
  }
}

// Add this debugging function to Point class implementation
void Point::update() {
  // Update the SFML representation based on the CGAL position
  updateSFMLShape();

  // Update any connected objects if needed
  // We can implement this more thoroughly if needed later
  for (const auto &weakLine : m_connectedLines) {
    if (auto line = weakLine.lock()) {
      try {
        line->update();
      } catch (const std::exception &e) {
        std::cerr << "Error updating connected line from point: " << e.what() << std::endl;
      }
    }
  }
}

void Point::setRadius(float screenRadius) {
  m_desiredScreenRadius = screenRadius;
  // Update m_radius based on current zoom (if accessible) or just let updateZoomFactor handle it later?
  // Ideally we want immediate feedback.
  if (Constants::CURRENT_ZOOM > 0) {
      m_radius = m_desiredScreenRadius / Constants::CURRENT_ZOOM;
  } else {
      m_radius = m_desiredScreenRadius; 
  }
  updateSFMLShape();
}
