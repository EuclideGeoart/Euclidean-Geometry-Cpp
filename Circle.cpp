#include "Circle.h"
#include "ObjectPoint.h"
#include "Constants.h"
#include "PointUtils.h"
#include "Transforms.h"
#include "Line.h"
#include <cmath>
#include <iostream>

// Constructor
Circle::Circle(Point *centerPoint, std::shared_ptr<Point> radiusPoint, double radius,
               const sf::Color &color)
    : GeometricObject(ObjectType::Circle, color), m_centerPoint(centerPoint),
      m_radiusPoint(radiusPoint), m_radius(radius), m_outlineColor(sf::Color::Black),
      m_fillColor(sf::Color::Transparent) {
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
std::shared_ptr<Circle> Circle::create(Point *centerPoint, std::shared_ptr<Point> radiusPoint,
                                       double radius, const sf::Color &color) {
  auto circle = std::make_shared<Circle>(centerPoint, radiusPoint, radius, color);
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

void Circle::setRadiusPoint(std::shared_ptr<Point> pt) {
    if (pt) {
        m_radiusPoint = pt;
        // Recalculate radius based on new point?
        // Actually, if we just swap the point object, the radius should stay defined by the DISTANCE to that point.
        // We should probably update the radius to match the new point's position relative to center?
        // Or should we move the new point to match current radius?
        // The Detach tool creates a CLONE at the SAME position.
        // So distance is same.
        updateSFMLShape();
        updateHostedPoints(); // In case any object points depend on radius?
    }
}

void Circle::set3PointDefinition(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2, std::shared_ptr<Point> p3) {
    m_p1 = p1;
    m_p2 = p2;
    m_p3 = p3;
    m_is3PointCircle = (p1 && p2 && p3 != nullptr);

    if (m_is3PointCircle && m_selfHandle) {
        if (p1) p1->addDependent(shared_from_this());
        if (p2) p2->addDependent(shared_from_this());
        if (p3) p3->addDependent(shared_from_this());
        update(); // Force initial calculation
    }
}

// GeometricObject overrides
void Circle::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
    if (!isVisible() && !forceVisible) return;

    // --- SETUP COLORS ---
    sf::Color fillColor = m_fillColor;
    sf::Color outlineColor = m_outlineColor;
    
    // Pixel-perfect thickness (Integers for sharpness)
    // Pixel-perfect thickness (Integers for sharpness)
    float baseThickness = m_thickness;
    if (isSelected()) {
        outlineColor = Constants::SELECTION_COLOR;
        baseThickness += 2.0f;
    } else if (isHovered()) {
        outlineColor = Constants::HOVER_COLOR;
        baseThickness += 1.0f;
    }
    float pixelThickness = std::round(baseThickness);
    if (pixelThickness < 1.0f) pixelThickness = 1.0f;

    if (!m_visible && forceVisible) {
        fillColor.a = 50;
        outlineColor.a = 50;
    }

    // --- COORDINATES ---
    Point_2 centerCgal = getCenterPoint();
    sf::Vector2f center(static_cast<float>(CGAL::to_double(centerCgal.x())), 
                        static_cast<float>(CGAL::to_double(centerCgal.y())));
    
    // Linear Radius
    float radius = static_cast<float>(m_radius); 

    // --- HELPER: SHARP LINE DRAWING ---
    // Defined here so BOTH Semicircle and Circle can use it
    auto drawSharpLine = [&](sf::Vector2f w1, sf::Vector2f w2) {
        sf::Vector2i sp1 = window.mapCoordsToPixel(w1);
        sf::Vector2i sp2 = window.mapCoordsToPixel(w2);
        
        sf::Vector2f d = sf::Vector2f(sp2 - sp1);
        float len = std::sqrt(d.x*d.x + d.y*d.y);
        if (len < 0.1f) return;
        
        sf::Vector2f norm(-d.y/len, d.x/len);
        sf::Vector2f off = norm * (pixelThickness * 0.5f);
        
        sf::VertexArray q(sf::Quads, 4);
        // Map back to world coords for drawing
        q[0].position = window.mapPixelToCoords(sf::Vector2i(sf::Vector2f(sp1) + off));
        q[1].position = window.mapPixelToCoords(sf::Vector2i(sf::Vector2f(sp2) + off));
        q[2].position = window.mapPixelToCoords(sf::Vector2i(sf::Vector2f(sp2) - off));
        q[3].position = window.mapPixelToCoords(sf::Vector2i(sf::Vector2f(sp1) - off));
        
        for(int k=0; k<4; ++k) q[k].color = outlineColor;
        window.draw(q);
    };

    int segments = 60; // Smoothness

    // =========================================================
    // CASE 1: SEMICIRCLE
    // =========================================================
    if (m_isSemicircle) {
        // 1. Calculate Start/End Angles
        Point_2 startCgal = m_semicircleStart;
        Point_2 endCgal   = m_semicircleEnd;
        
        sf::Vector2f pStart(static_cast<float>(CGAL::to_double(startCgal.x())), static_cast<float>(CGAL::to_double(startCgal.y())));
        sf::Vector2f pEnd(static_cast<float>(CGAL::to_double(endCgal.x())), static_cast<float>(CGAL::to_double(endCgal.y())));

        double ang1 = std::atan2(pStart.y - center.y, pStart.x - center.x);
        double ang2 = std::atan2(pEnd.y - center.y, pEnd.x - center.x);

        // Ensure we draw CCW from Start to End
        if (ang2 < ang1) ang2 += 2 * M_PI;

        // 2. Draw Fill (Triangle Fan)
        sf::VertexArray fan(sf::TriangleFan, segments + 2);
        fan[0].position = center;
        fan[0].color = fillColor;

        for (int i = 0; i <= segments; ++i) {
            double t = static_cast<double>(i) / segments;
            double currentAng = ang1 + t * (ang2 - ang1);
            
            float px = center.x + radius * std::cos(currentAng);
            float py = center.y + radius * std::sin(currentAng);
            
            fan[i+1].position = sf::Vector2f(px, py);
            fan[i+1].color = fillColor;
        }
        window.draw(fan);

        // 3. Draw Outline (Using Sharp Lines with Style Support)
        sf::Vector2f prevPos = fan[1].position;
        for (int i = 1; i <= segments; ++i) {
            sf::Vector2f currPos = fan[i+1].position;
            if (m_lineStyle == LineStyle::Solid) {
                drawSharpLine(prevPos, currPos);
            } else {
                GeometricObject::drawStyledLine(window, prevPos, currPos, m_lineStyle, pixelThickness, outlineColor);
            }
            prevPos = currPos;
        }
        // Draw Diameter
        if (m_lineStyle == LineStyle::Solid) {
            drawSharpLine(pStart, pEnd);
        } else {
            GeometricObject::drawStyledLine(window, pStart, pEnd, m_lineStyle, pixelThickness, outlineColor);
        }
    }
    // =========================================================
    // CASE 2: NORMAL CIRCLE & 3-POINT CIRCLE
    // =========================================================
    else {
        // 1. Draw Fill (Standard SFML Circle is fine for fill)
        sf::CircleShape circleShape(radius);
        circleShape.setOrigin(radius, radius);
        circleShape.setPosition(center);
        circleShape.setFillColor(fillColor);
        circleShape.setOutlineThickness(0); // We draw outline manually for sharpness
        window.draw(circleShape);
 
        // 2. Draw Sharp Outline Ring with Style Support
        double dAng = 2 * M_PI / segments;
        sf::Vector2f prev(center.x + radius, center.y);
        
        for (int i = 1; i <= segments; ++i) {
            double ang = i * dAng;
            sf::Vector2f curr(center.x + radius * std::cos(ang), center.y + radius * std::sin(ang));
            
            if (m_lineStyle == LineStyle::Solid) {
                drawSharpLine(prev, curr);
            } else {
                GeometricObject::drawStyledLine(window, prev, curr, m_lineStyle, pixelThickness, outlineColor);
            }
            prev = curr;
        }
    }

    // =========================================================
    // CENTER VISUAL (Draw last to be on top)
    // =========================================================
    sf::CircleShape centerShape;
    float baseCenterRadius = 3.0f; // 3 pixels screen size
    float scaledRadius = baseCenterRadius * scale; 
    
    centerShape.setRadius(scaledRadius);
    centerShape.setOrigin(scaledRadius, scaledRadius);
    centerShape.setPosition(center);
    centerShape.setFillColor(m_outlineColor); // Use outline color for the dot

    if (!m_visible && forceVisible) {
        sf::Color ghost = centerShape.getFillColor();
        ghost.a = 50;
        centerShape.setFillColor(ghost);
    }
    window.draw(centerShape);
}

void Circle::drawLabel(sf::RenderWindow& window, const sf::View& worldView) const {
  if (!isVisible() || !getShowLabel() || getLabelMode() == LabelMode::Hidden || !Point::commonFont) return;
  if (!isValid()) return;

  Point_2 centerCgal = getCenterPoint();
  double r = m_radius;
  
  // Position label on the ring at 45 degrees
  double angle = 0.785398; // pi/4
  Point_2 labelPosCgal(centerCgal.x() + FT(r * std::cos(angle)), centerCgal.y() + FT(r * std::sin(angle)));
  
  sf::Vector2f labelPos(static_cast<float>(CGAL::to_double(labelPosCgal.x())), 
                        static_cast<float>(CGAL::to_double(labelPosCgal.y())));

  sf::Vector2i screenPos = window.mapCoordsToPixel(labelPos, worldView);

  std::string labelStr = "";
  switch (getLabelMode()) {
    case LabelMode::Name: labelStr = getLabel(); break;
    case LabelMode::Value: {
      // Area = pi * r^2
      double area = 3.141592653589793 * r * r;
      labelStr = std::to_string(static_cast<int>(std::round(area)));
      break;
    }
    case LabelMode::NameAndValue: {
      labelStr = getLabel();
      double area = 3.141592653589793 * r * r;
      labelStr += (labelStr.empty() ? "" : " = ") + std::to_string(static_cast<int>(std::round(area)));
      break;
    }
    case LabelMode::Caption: labelStr = getCaption(); break;
    default: break;
  }

  if (labelStr.empty()) return;

  sf::Text text;
  text.setFont(*Point::commonFont);
  text.setString(sf::String::fromUtf8(labelStr.begin(), labelStr.end()));
  text.setCharacterSize(LabelManager::instance().getFontSize());
  
  sf::Color textColor = m_color;
  // Fallback for very bright colors on white background
  if (textColor.r > 200 && textColor.g > 200 && textColor.b > 200) textColor = sf::Color::Black;
  text.setFillColor(textColor);

  sf::Vector2f finalPos(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
  
  // Offset slightly outward from center
  sf::Vector2f centerSfml(static_cast<float>(CGAL::to_double(centerCgal.x())), 
                          static_cast<float>(CGAL::to_double(centerCgal.y())));
  sf::Vector2f dir = labelPos - centerSfml;
  float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  if (len > 1e-6f) {
      finalPos += (dir / len) * 15.0f; // 15 pixels outward
  }

  text.setPosition(std::round(finalPos.x), std::round(finalPos.y));
  sf::FloatRect bounds = text.getLocalBounds();
  text.setOrigin(bounds.width / 2.0f, bounds.height / 2.0f);

  window.draw(text);
}

bool Circle::contains(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  if (!isValid() || !m_visible) return false;

  try {
    Point_2 center = getCenterPoint();
    auto sfmlCenter = Point::cgalToSFML(center);
    double dx = worldPos_sfml.x - sfmlCenter.x;
    double dy = worldPos_sfml.y - sfmlCenter.y;
    double distSq = dx * dx + dy * dy;
    double dist = std::sqrt(distSq);

    // 1. Distance Check (Ring Check)
    // Check if point is near the circumference
    if (std::abs(dist - m_radius) > tolerance) {
      return false;
    }

    // 2. Semicircle Angle Check
    if (m_isSemicircle) {
      double mouseAngle = std::atan2(dy, dx);

      Point_2 p1 = m_semicircleStart;
      Point_2 p2 = m_semicircleEnd;
      
      // Calculate basis angles relative to center
      auto cgalCenter = getCenterPoint();
      double angStart = std::atan2(CGAL::to_double(p1.y()) - CGAL::to_double(cgalCenter.y()), 
                                   CGAL::to_double(p1.x()) - CGAL::to_double(cgalCenter.x()));
      double angEnd   = std::atan2(CGAL::to_double(p2.y()) - CGAL::to_double(cgalCenter.y()), 
                                   CGAL::to_double(p2.x()) - CGAL::to_double(cgalCenter.x()));

      // Normalize CCW order (End must be > Start, wrapping around 2PI if needed)
      if (angEnd < angStart) angEnd += 2 * M_PI;

      // Normalize mouse angle to be relative to Start
      double angleRel = mouseAngle - angStart;
      // Wrap angleRel to [0, 2PI) range relative to start
      while (angleRel < 0) angleRel += 2 * M_PI;
      while (angleRel >= 2 * M_PI) angleRel -= 2 * M_PI;
      
      // Allow for small tolerance in angle if close to ends, but strict logic:
      // If the relative angle is beyond the span, we are on the "invisible" part of the circle
      double span = angEnd - angStart;
      if (angleRel > span) {
        return false;
      }
    }

    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

bool Circle::isValid() const {
  if (m_isSemicircle) {
    if (!m_diameterP1 || !m_diameterP2) return false;
    if (!m_diameterP1->isValid() || !m_diameterP2->isValid()) return false;
    Point_2 p1 = m_diameterP1->getCGALPosition();
    Point_2 p2 = m_diameterP2->getCGALPosition();
    double distSq = CGAL::to_double(CGAL::squared_distance(p1, p2));
    if (!std::isfinite(distSq) || distSq <= 0.0) return false;
    Point_2 center((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);
    return CGAL::is_finite(center.x()) && CGAL::is_finite(center.y());
  }

  if (m_is3PointCircle) {
    if (!m_p1 || !m_p2 || !m_p3) return false;
    if (!m_p1->isValid() || !m_p2->isValid() || !m_p3->isValid()) return false;
    Point_2 a = m_p1->getCGALPosition();
    Point_2 b = m_p2->getCGALPosition();
    Point_2 c = m_p3->getCGALPosition();
    if (CGAL::collinear(a, b, c)) return false;
    try {
        Point_2 center = CGAL::circumcenter(a, b, c);
        return CGAL::is_finite(center.x()) && CGAL::is_finite(center.y());
    } catch (...) {
        return false;
    }
  }

  if (!m_centerPoint) return false;
  if (m_radiusPoint && !m_radiusPoint->isValid()) return false;
  Point_2 center = m_centerPoint->getCGALPosition();
  double radiusToCheck = m_radius;
  if (m_radiusPoint && m_radiusPoint->isValid()) {
    radiusToCheck = std::sqrt(CGAL::to_double(
        CGAL::squared_distance(center, m_radiusPoint->getCGALPosition())));
  }
  return radiusToCheck > 0 && std::isfinite(radiusToCheck) &&
         CGAL::is_finite(center.x()) && CGAL::is_finite(center.y());
}

void Circle::update() {
  // 1. RESURRECTION LOGIC: Check parent validity first
  // For standard circles (with center point)
  if (!isDependent() && !m_isSemicircle) {
    bool parentsAreValid = true;
    if (!m_centerPoint || !m_centerPoint->isValid()) {
      parentsAreValid = false;
    }
    
    // APPLY STATE (Persistence)
    if (parentsAreValid) {
      if (!this->isVisible() && !hasVisibilityUserOverride()) {
        this->setVisible(true); // RESURRECTION
      }
    } else {
      // If parents are invalid, we MUST be hidden.
      if (this->isVisible()) {
        this->setVisible(false); // HIDE
      }
      return; // STOP calculation if parents are gone/invalid
    }
  }

  // 2. Dependent Shape Logic (Transforms)
  if (isDependent()) {
    // This function handles Position, Radius (including Dilation), and Visibility
    updateDependentShape(); 
    
    // FIX: Force Radius Sync for Rigid Transformations (Rotate, Reflect, Translate)
    // We EXCLUDE Dilate and ReflectCircle because they have their own radius math.
    if (m_transformType != TransformationType::Dilate && 
        m_transformType != TransformationType::ReflectCircle) {
        
        auto parent = m_parentSource.lock();
        // Ensure parent exists and is actually a Circle
        if (parent && parent->getType() == ObjectType::Circle) {
            // Use static_pointer_cast since we checked getType()
            auto parentCircle = std::static_pointer_cast<Circle>(parent);
            setRadius(parentCircle->getRadius());
        }
    }

    // Note: updateDependentShape() already calls updateSFMLShape(), 
    // but calling it again here ensures the visual mesh is definitely in sync.
    updateSFMLShape();
    updateHostedPoints();
    return;
  }

  // 3. Semicircle Logic (Independent)
  if (m_isSemicircle && m_diameterP1 && m_diameterP2) {
    // RESURRECTION LOGIC for semicircles
    bool semicircleParentsValid = m_diameterP1->isValid() && m_diameterP2->isValid();
    
    if (semicircleParentsValid) {
      if (!this->isVisible() && !hasVisibilityUserOverride()) {
        this->setVisible(true); // RESURRECTION
      }
      
      Point_2 p1 = m_diameterP1->getCGALPosition();
      Point_2 p2 = m_diameterP2->getCGALPosition();

      // Calculate Center
      Point_2 newCenter((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);
      if (m_centerPoint) {
        m_centerPoint->setCGALPosition(newCenter);
      }

      // Calculate Radius
      double distSq = CGAL::to_double(CGAL::squared_distance(p1, p2));
      if (!std::isfinite(distSq) || distSq <= 0.0) return;
      m_radius = std::sqrt(distSq) * 0.5;

      m_semicircleStart = p1;
      m_semicircleEnd = p2;
    } else {
      if (this->isVisible()) {
        this->setVisible(false); // HIDE
      }
      return;
    }
  }

  // 4. 3-Point Circle Logic
  if (m_is3PointCircle && m_p1 && m_p2 && m_p3) {
    bool pValid = m_p1->isValid() && m_p2->isValid() && m_p3->isValid();
    if (pValid) {
        Point_2 a = m_p1->getCGALPosition();
        Point_2 b = m_p2->getCGALPosition();
        Point_2 c = m_p3->getCGALPosition();
        if (!CGAL::collinear(a, b, c)) {
          if (!this->isVisible() && !hasVisibilityUserOverride()) this->setVisible(true);

            Point_2 center = CGAL::circumcenter(a, b, c);
            if (m_centerPoint) {
                m_centerPoint->setCGALPosition(center);
            }
            m_radius = std::sqrt(CGAL::to_double(CGAL::squared_distance(center, a)));
        } else {
            if (this->isVisible()) this->setVisible(false);
        }
    } else {
        if (this->isVisible()) this->setVisible(false);
        return;
    }
  }

  // 5. Standard Logic - Always recalculate geometry after resurrection
  updateSFMLShape();
  updateHostedPoints();
}

sf::FloatRect Circle::getGlobalBounds() const {
  return m_sfmlShape.getGlobalBounds();
}

void Circle::translate(const Vector_2 &offset) {
  if (m_isSemicircle) {
    if (m_diameterP1) m_diameterP1->translate(offset);
    if (m_diameterP2) m_diameterP2->translate(offset);
    return;
  }

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
  return contains(worldPos_sfml, tolerance);
  // if (!isValid()) return false;

  // try {
  //   Point_2 center = getCenterPoint();
  //   auto sfmlCenter = Point::cgalToSFML(center);
  //   float distToCenter = std::sqrt((worldPos_sfml.x - sfmlCenter.x) * (worldPos_sfml.x - sfmlCenter.x) +
  //                                  (worldPos_sfml.y - sfmlCenter.y) * (worldPos_sfml.y - sfmlCenter.y));
  //   float circumferenceDistance = std::abs(distToCenter - static_cast<float>(m_radius));
  //   return circumferenceDistance <= tolerance;
  // } catch (const std::exception &e) {
  //   std::cerr << "Error in isCircumferenceHovered: " << e.what() << std::endl;
  //   return false;
  // }
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
    if (m_radiusPoint && m_radiusPoint->isValid()) {
      m_radius = std::sqrt(CGAL::to_double(
          CGAL::squared_distance(center, m_radiusPoint->getCGALPosition())));
    }
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

void Circle::updateDependentShape() {
  auto parent = m_parentSource.lock();
  if (!parent || !parent->isValid()) {
    setVisible(false);
    return;
  }

  // Helper lambda for Point transform
  auto transformPoint = [&](const Point_2& p) -> std::optional<Point_2> {
    auto aux = m_auxObject.lock();
    switch (m_transformType) {
      case TransformationType::Translate: {
        Vector_2 delta = m_translationVector;
        if (aux && (aux->getType() == ObjectType::Line || aux->getType() == ObjectType::LineSegment ||
                    aux->getType() == ObjectType::Ray || aux->getType() == ObjectType::Vector)) {
           auto line = std::dynamic_pointer_cast<Line>(aux);
           if (line && line->isValid()) delta = line->getEndPoint() - line->getStartPoint();
        }
        return p + delta;
      }
      case TransformationType::Reflect: {
        if (aux && (aux->getType() == ObjectType::Line || aux->getType() == ObjectType::LineSegment || aux->getType() == ObjectType::Ray)) {
             auto line = std::dynamic_pointer_cast<Line>(aux);
             if (!line || !line->isValid()) return std::nullopt;
             Point_2 a = line->getStartPoint();
             Point_2 b = line->getEndPoint();
             Vector_2 ab = b - a;
             if (CGAL::to_double(ab.squared_length()) < 1e-12) return p;
             Vector_2 ap = p - a;
             FT t = (ap * ab) / ab.squared_length();
             Point_2 h = a + ab * t;
             return p + (h - p) * FT(2.0);
        }
        return std::nullopt;
      }
      case TransformationType::ReflectPoint: {
        auto center = std::dynamic_pointer_cast<Point>(aux);
        if (!center || !center->isValid()) return std::nullopt;
        Point_2 c = center->getCGALPosition();
        return c + (c - p);
      }
      case TransformationType::ReflectCircle: {
        auto circle = std::dynamic_pointer_cast<Circle>(aux);
        if (!circle || !circle->isValid()) return std::nullopt;
        Point_2 o = circle->getCenterPoint();
        double r = circle->getRadius();
        Vector_2 op = p - o;
        double opLenSq = CGAL::to_double(op.squared_length());
        if (opLenSq < 1e-12) return std::nullopt;
        double scale = (r * r) / opLenSq;
        return o + op * FT(scale);
      }
      case TransformationType::Rotate: {
        auto center = std::dynamic_pointer_cast<Point>(aux);
        if (!center || !center->isValid()) return std::nullopt;
        Point_2 c = center->getCGALPosition();
        double rad = m_transformValue * 3.14159265358979323846 / 180.0;
        double s = std::sin(rad);
        double c_val = std::cos(rad);
        double dx = CGAL::to_double(p.x() - c.x());
        double dy = CGAL::to_double(p.y() - c.y());
        return Point_2(c.x() + FT(dx * c_val - dy * s),
                       c.y() + FT(dx * s + dy * c_val));
      }
      case TransformationType::Dilate: {
        auto center = std::dynamic_pointer_cast<Point>(aux);
        if (!center || !center->isValid()) return std::nullopt;
        Point_2 c = center->getCGALPosition();
        double scale = m_transformValue;
        return c + (p - c) * FT(scale);
      }
      default: return std::nullopt;
    }
  };

  auto sourceCircle = std::dynamic_pointer_cast<Circle>(parent);
  if (!sourceCircle || !sourceCircle->isValid()) return;

  Point_2 center = sourceCircle->getCenterPoint();
  double originalRadius = sourceCircle->getRadius();
  
  auto newCenter = transformPoint(center);
  
  if (newCenter) {
      setCenter(*newCenter);
      
      // Radius Update
      if (m_transformType == TransformationType::Dilate) {
          setRadius(std::abs(originalRadius * m_transformValue));
      } else if (m_transformType == TransformationType::ReflectCircle) {
           // Inversion Radius check
           Point_2 edgePt = center + Vector_2(FT(originalRadius), FT(0));
           auto newEdgePt = transformPoint(edgePt);
           if (newEdgePt) {
               double d = std::sqrt(CGAL::to_double(CGAL::squared_distance(*newCenter, *newEdgePt)));
               setRadius(d);
           }
      } else {
          setRadius(originalRadius);
      }
  }

  updateSFMLShape();
  if (!hasVisibilityUserOverride()) {
    setVisible(sourceCircle->isVisible());
  }
}