#ifndef CIRCLE_H
#define CIRCLE_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif
#include "CharTraitsFix.h"
#include <string>

#include "CompilerFixes.h"
#include "ForwardDeclarations.h"
#include "GeometricObject.h"
#include "ObjectType.h"  // For DragMode enum
#include "Point.h"       // Ensure Point is included for m_centerPointObject
#include "Types.h"       // For Point_2
#include <SFML/Graphics.hpp>
#include <memory>  // For std::unique_ptr
#include <vector>

#include "Point.h"  // Ensure Point is included for Point::sfmlToCGAL etc.

// Enum for circle interaction modes
enum class CircleInteractionMode {
  None,
  DragCenter,
  Resize  // Added for resizing by dragging circumference
};

class Circle : public GeometricObject, public std::enable_shared_from_this<Circle> {
 public:
  // Constructor with outline and fill colors
  Circle(const Point_2 &center, double radius, const sf::Color &outlineColor,
         const sf::Color &fillColor);

  // Constructor with Point* center
  Circle(Point *centerPoint, double radius, const sf::Color &outlineColor,
         const sf::Color &fillColor);
  Circle(const Point_2 &center, double radius, const sf::Color &color);
  ~Circle();
  // factory method
  static std::shared_ptr<Circle> create(const Point_2 &center, double radius,
                                        const sf::Color &color = sf::Color::Blue);
  static std::shared_ptr<Circle> create(const Point_2 &center, double radius,
                                        const sf::Color &outlineColor, const sf::Color &fillColor);
  // --- GeometricObject Overrides ---
  void draw(sf::RenderWindow &window) const override;
  bool contains(const sf::Vector2f &worldPos, float tolerance) const override;
  void setSelected(bool sel) override;
  void setHovered(bool hoveredStatus) override;
  ObjectType getType() const override { return ObjectType::Circle; }
  void update() override;
  sf::FloatRect getGlobalBounds() const override;
  Point_2 getCGALPosition() const override;
  Point_2 getCenterPoint() const { return m_centerPointObject->getCGALPosition(); }
  double getRadius() const { return m_radius; }
  Circle_2 getCGALCircle() const;  // Changed from CGAL_Circle_2
  void setRadius(double newRadius);
  void setFillColor(const sf::Color &color);  // New method
  sf::Color getFillColor() const;             // New method
  sf::Color getOutlineColor() const;  // Renamed from getColor or new if m_color was for outline
  void setColor(const sf::Color &color) override;
  // Implement pure virtual functions from GeometricObject
  void setCGALPosition(const Point_2 &newPos) override;
  void translate(const Vector_2 &offset) override;
  void setPosition(const sf::Vector2f &newSfmlPos) override;

  // --- Hosted ObjectPoint Management ---
  void addChildPoint(std::shared_ptr<ObjectPoint> point);
  void removeChildPoint(std::shared_ptr<ObjectPoint> point);
  void removeChildPoint(ObjectPoint *point);  // Backward compatibility overload
  void removeChildPointByRawPointer(ObjectPoint *rawPtr);
  std::vector<std::shared_ptr<ObjectPoint>> getHostedObjectPoints() const;
  void updateHostedPoints();
  void notifyObjectPoints();
  void cleanupExpiredChildPoints();
  // --- Interaction ---
  CircleInteractionMode handleInteractionEvent(
      const sf::Event &ev, const Point_2 &worldPos_cgal,
      float tolerance);  // Modified: Added tolerance, removed lastPos_cgal
  bool isCenterPointHovered(const sf::Vector2f &worldPos_sfml, float tolerance) const;
  bool isCircumferenceHovered(const sf::Vector2f &worldPos_sfml,  // Add this line
                              float tolerance) const;             // Add this line
  // --- Utility ---
  Point_2 projectOntoCircumference(const Point_2 &p) const;  // Added method

  // Observer pattern
  // void update(Subject &subject) override; // REMOVE THIS LINE

  // Lock status
  void setLocked(bool lockStatus);
  bool isLocked() const;

  // Add this method declaration
  void translateWithDependents(const Vector_2 &offset);

  // Add methods to support proper circle manipulation
  void moveCenter(const Point_2 &newCenter);
  void dragCircumferencePoint(const Point_2 &dragPoint);

  // Override isValid to check Circle integrity
  /* bool isValid() const override {
    try {
      // Access center and radius to check validity
      auto center = getCenterPoint();
      center.x();
      center.y();
      return getRadius() > 0;
    } catch (...) {
      return false;
    }
  } */
  bool isValid() const override {
    // Check if radius is finite and positive
    if (m_radius <= 0 || !std::isfinite(m_radius)) {
      return false;
    }

    // Check if center point is valid
    try {
      Point_2 center = getCenterPoint();
      return CGAL::is_finite(center.x()) && CGAL::is_finite(center.y());
    } catch (const std::exception &e) {
      std::cerr << "Error checking circle validity: " << e.what() << std::endl;
      return false;
    }
  }
  // Add or modify these methods:
  void drawCenterPoint(sf::RenderWindow &window) const;

 private:
  Point_2 m_centerPoint;
  std::unique_ptr<Point> m_centerPointObject;  // Circle owns its center Point object
  double m_radius;                             // Radius as double
  Circle_2 m_cgalCircle;                       // CGAL representation
  sf::Color m_outlineColor;                    // Renamed from m_color to be specific
  sf::Color m_fillColor;                       // New member for fill color

  sf::CircleShape m_sfmlShape;
  sf::CircleShape m_centerVisual;  // Visual representation of the center point
  // Interaction state members
  bool m_isDragging = false;
  CircleInteractionMode m_interactionMode =
      CircleInteractionMode::None;  // Member to track current interaction
  Point_2 m_lastDragPos_cgal;

  std::vector<std::weak_ptr<ObjectPoint>> m_hostedObjectPoints;
  std::vector<ObjectPoint *> m_childPoints;  // Stores raw pointers to ObjectPoints on this circle

  DragMode m_currentDragInteraction = DragMode::None;

  bool m_isLocked = false;

  void updateSFMLShape();
  void updateCGALCircle();  // Call when center or radius changes
  // Single color constructor (for backward compatibility)
};

#endif  // CIRCLE_H
