#pragma once  // Use pragma once for modern include guard
#include <SFML/Graphics/Color.hpp>
#ifndef POINT_H
#define POINT_H

#include "CharTraitsFix.h"  // Ensure this is very early
#include <string>           // Ensure standard string is included very early

#include "Constants.h"
#include "ForwardDeclarations.h"  // For ObjectType enum and Line
#include "GeometricObject.h"
#include "ObjectType.h"  // For ObjectType enum
#include "QuickProfiler.h"
#include "Transforms.h"
#include "Types.h"  // For Point_2
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>  // For connectedLines

// Forward declaration for Line to resolve circular dependency
class Line;

class Point : public GeometricObject, public std::enable_shared_from_this<Point> {
 public:
  // --- Constructors ---
  Point(float initialZoomFactor = 1.0f);  // Default constructor needs initial zoom
  Point(const Point_2 &cgalPos,
        float initialZoomFactor,  // Add initialZoomFactor
        const sf::Color &fillColor = Constants::POINT_FILL_COLOR,
        const sf::Color &outlineColor = Constants::POINT_DEFAULT_COLOR);
  Point(const sf::Vector2f &sfmlPos,
        float initialZoomFactor,  // Add initialZoomFactor
        const sf::Color &fillColor = Constants::POINT_FILL_COLOR,
        const sf::Color &outlineColor = Constants::POINT_DEFAULT_COLOR);
  Point(const Point_2 &cgal_point,
        float initialZoomFactor,  // Add initialZoomFactor
        const sf::Color &fillColor, unsigned int id,
        const sf::Color &outlineColor = Constants::POINT_DEFAULT_COLOR);

  // --- Destructor ---
  // Destructor should be virtual to ensure proper cleanup of derived classes
  virtual ~Point();
  // --- SFML and CGAL Conversions ---
  static sf::Vector2f cgalToSFML(const Point_2 &p);
  static Point_2 sfmlToCGAL(const sf::Vector2f &p);

  // --- GeometricObject Overrides ---
  virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
  void drawLabel(sf::RenderWindow &window, const sf::View &worldView) const override;
  void drawLabelExplicit(sf::RenderWindow &window, const sf::View &worldView) const;
  bool contains(const sf::Vector2f &worldPos,
                float tolerance = Constants::POINT_INTERACTION_RADIUS) const override;
  void setSelected(bool sel) override;
  ObjectType getType() const override { return ObjectType::Point; }
  void update() override;  // Implement if needed, or leave empty
  void setHovered(bool hoveredStatus) override;
  sf::FloatRect getGlobalBounds() const override;

  void setCGALPosition(const Point_2 &newPos) override;
  virtual void setPosition(const sf::Vector2f &newSfmlPos) override;
  sf::Color getColor() const;  // Added getColor method

  // --- Point Specific Methods ---
  Point_2 getCGALPosition() const override;  // Override from GeometricObject

  virtual void dragTo(const sf::Vector2f &targetSfmlPos);

  sf::Vector2f getSFMLPosition() const;

  // Lock status
  void setLocked(bool lockStatus) override;
  bool isLocked() const override;

  void setVisible(bool v) override;
  bool isVisible() const override { return GeometricObject::isVisible(); }
  void setIsValid(bool valid);

  // Intersection status
  void setIntersectionPoint(bool isInter) { m_isIntersectionPoint = isInter; }
  bool isIntersectionPoint() const;
  //void setIsIntersectionPoint(bool isIntersection) { m_isIntersectionPoint = isIntersection; }

  // Connected lines management
  void addConnectedLine(std::weak_ptr<Line> line);
  void removeConnectedLine(Line *line);
  const std::vector<std::weak_ptr<Line>> &getConnectedLines() const;
  void updateConnectedLines();
  void setDeferConstraintUpdates(bool defer) { m_deferConstraintUpdates = defer; }
  bool isDeferringConstraintUpdates() const { return m_deferConstraintUpdates; }
  inline void forceConstraintUpdate() {
    QUICK_PROFILE("Point::forceConstraintUpdate");
    m_deferConstraintUpdates = false;
    updateConnectedLines();
  }

  // lock() and unlock() inherited from GeometricObject
  void updateZoomFactor(float newZoomFactor);

  void translate(const Vector_2 &offset) override;
  void setColor(const sf::Color &color) override;
  void setFillColor(const sf::Color &fillColor);
  void setOutlineColor(const sf::Color &outlineColor);
  sf::Color getFillColor() const;
  sf::Color getOutlineColor() const;
  
  // Point Size Control
  void setRadius(float screenRadius);
  float getRadius() const { return m_desiredScreenRadius; }

  unsigned int getID() const override { return m_id; }
  void cleanupConnectedLines();
  void notifyConnectedLines();
  // Add these transformation method declarations
  void transform(const CoordinateTransform &transform);
  void scale(float factor);
  void rotate(float angleRadians);

  // Add this method to check if the point is properly initialized
  bool isInitialized() const {
    try {
      // Test if we can access the CGAL position without errors
      Point_2 pos = getCGALPosition();
      return CGAL::is_finite(pos.x()) && CGAL::is_finite(pos.y());
    } catch (...) {
      return false;
    }
  }

  // Add this to the base class if not already present
  // bool isValid() const override { return isInitialized(); }
  bool isValid() const override {
    // Basic validity check for Point
    try {
      if (!m_isValid) {
        return false;
      }
      // Check if position is finite
      const Point_2 &pos = getCGALPosition();
      bool isFiniteX = CGAL::is_finite(pos.x());
      bool isFiniteY = CGAL::is_finite(pos.y());
      return isFiniteX && isFiniteY;
    } catch (const std::exception &e) {
      std::cerr << "Exception in Point::isValid: " << e.what() << std::endl;
      return false;
    }
  }

 protected:  // Changed from private for m_cgalPosition and m_sfmlShape
  void initializeShape();

  // Make both variations of updateSFMLShape available in the base class
  virtual void updateSFMLShape();  // No-param version updates from internal state
  virtual void updateSFMLShape(
      const sf::Vector2f &position);  // With-param version updates with explicit position

  Point_2 m_cgalPosition;
  sf::CircleShape m_sfmlShape;

 private:
  float m_radius;  // Radius for the point
  float m_desiredScreenRadius;
  sf::Color m_fillColor;
  sf::Color m_outlineColor;
  float m_outlineThickness;
  // void updateSFMLPosition(); // Removed, replaced by updateSFMLShape
  bool m_isHovered;
  bool m_isDragging;
  bool m_isIntersectionPoint;
  bool m_isInitialized;
  bool m_deferConstraintUpdates;
  bool m_createdWithShape = false;
  std::vector<std::weak_ptr<Line>> m_connectedLines;  // Lines connected to this point

  // --- Labeling ---
  std::string m_label;
  
public: // Public for initialization in GeometryEditor
    static const sf::Font* commonFont; // Shared font pointer

public:
    void setLabel(const std::string& label) { m_label = label; }
    std::string getLabel() const { return m_label; }
    void setShowLabel(bool show) override { GeometricObject::setShowLabel(show); }
    bool getShowLabel() const override { return GeometricObject::getShowLabel(); }
    void setLabelOffset(const sf::Vector2f& offset) override { GeometricObject::setLabelOffset(offset); }
    const sf::Vector2f& getLabelOffset() const override { return GeometricObject::getLabelOffset(); }
    void setDependent(bool dependent) override { GeometricObject::setDependent(dependent); }
    bool isDependent() const override { return GeometricObject::isDependent(); }
    void setCreatedWithShape(bool created) { m_createdWithShape = created; }
    bool isCreatedWithShape() const { return m_createdWithShape; }
};

#endif  // POINT_H