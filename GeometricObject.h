#ifndef GEOMETRIC_OBJECT_H
#define GEOMETRIC_OBJECT_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include <SFML/Graphics.hpp>
#include <memory>
#include <string>
#include <vector>

#include "CharTraitsFix.h"
#include "Constants.h"
#include "ForwardDeclarations.h"  // For ObjectType enum
#include "ObjectType.h"
#include "Types.h"  // For Point_2 and other CGAL types


// Abstract base class for all geometric objects
class GeometricObject {
 public:
  // Add constructor that takes ObjectType and color
  GeometricObject(ObjectType type, const sf::Color& color) : m_type(type), m_color(color), m_selected(false), m_hovered(false), m_isValid(true) {}

  // NEW constructor to handle Point_2 and ID
  GeometricObject(ObjectType type, const sf::Color& color, const Point_2& cgal_pos, unsigned int id);
  GeometricObject(ObjectType type, const sf::Color& color, unsigned int id);
  virtual ~GeometricObject() = default;

  // Object properties
  virtual unsigned int getID() const { return m_id; }
  void setID(unsigned int id) { m_id = id; }
  virtual ObjectType getType() const { return m_type; }

  // Geometry operations
  virtual void draw(sf::RenderWindow& window, float scale, bool forceVisible = false) const = 0;
  // NEW: Screen-space label rendering
  virtual void drawLabel(sf::RenderWindow& window, const sf::View& worldView) const {}
  virtual bool contains(const sf::Vector2f& worldPos, float tolerance) const = 0;
  virtual sf::FloatRect getGlobalBounds() const = 0;
  virtual void update() {
    if (isDependent()) {
      updateDependentShape();
    }
  }
  virtual void updateDependentShape() {}

  // Position access
  virtual Point_2 getCGALPosition() const = 0;
  virtual void setCGALPosition(const Point_2& newPos) = 0;
  virtual void setPosition(const sf::Vector2f& newSfmlPos) = 0;

  // NEW: Label Anchor for dragging/positioning
  virtual sf::Vector2f getLabelAnchor(const sf::View& view) const {
    (void)view;
    Point_2 p = getCGALPosition();
    return sf::Vector2f((float)CGAL::to_double(p.x()), (float)CGAL::to_double(p.y()));
  }

  virtual void setColor(const sf::Color& color);
  virtual void translate(const Vector_2& offset) { (void)offset; }  // No default implementation
  void move(const Vector_2& delta);

  // Point/Edge provider interface for generic anchor point detection
  // Shapes override these to expose their vertices and edges to all tools
  virtual std::vector<Point_2> getInteractableVertices() const { return {}; }
  virtual std::vector<Segment_2> getEdges() const { return {}; }
  virtual std::vector<Segment_2> getBoundarySegments() const { return getEdges(); }
  virtual bool getClosestPointOnPerimeter(const Point_2& query, Point_2& outPoint) const;

  // virtual int getIDIfAvailable() const { return m_id; }
  //  Selection/hover state
  virtual void setSelected(bool selected);
  virtual bool isSelected() const;
  virtual void setHovered(bool hovered);
  virtual bool isHovered() const;

  virtual void setHoveredEdge(int edgeIndex) { m_hoveredEdgeIndex = edgeIndex; }
  virtual int getHoveredEdge() const { return m_hoveredEdgeIndex; }

  // Visibility
  virtual void setVisible(bool visible) { m_visible = visible; }
  virtual bool isVisible() const { return m_visible; }
  virtual void setVisibilityUserOverride(bool override) { m_visibilityUserOverride = override; }
  virtual bool hasVisibilityUserOverride() const { return m_visibilityUserOverride; }
  virtual void clearVisibilityUserOverride() { m_visibilityUserOverride = false; }

  // Label controls
  virtual void setShowLabel(bool show) { m_showLabel = show; }
  virtual bool getShowLabel() const { return m_showLabel; }
  virtual void setHasUserOverride(bool override) { m_hasUserOverride = override; }
  virtual bool hasUserOverride() const { return m_hasUserOverride; }
  virtual void clearUserOverride() { m_hasUserOverride = false; }
  virtual sf::FloatRect getLabelBounds(const sf::View& view) const {
    (void)view;
    return sf::FloatRect();
  }
  virtual void setLabelOffset(const sf::Vector2f& offset) { m_labelOffset = offset; }
  virtual const sf::Vector2f& getLabelOffset() const { return m_labelOffset; }

  // Label content
  virtual void setLabel(const std::string& label) { m_label = label; }
  virtual std::string getLabel() const { return m_label; }
  virtual void setLabelMode(LabelMode mode) { m_labelMode = mode; }
  virtual LabelMode getLabelMode() const { return m_labelMode; }
  virtual void setCaption(const std::string& caption) { m_caption = caption; }
  virtual std::string getCaption() const { return m_caption; }

  // Thickness (used by UI slider for line rendering)
  virtual void setThickness(float thickness) { m_thickness = thickness; }
  virtual float getThickness() const { return m_thickness; }

  // Vertex handle size (used by shapes with draggable vertices)
  virtual void setVertexHandleSize(float size) { m_vertexHandleSize = size; }
  virtual float getVertexHandleSize() const { return m_vertexHandleSize; }

  // Locking
  virtual void setLocked(bool locked);
  virtual bool isLocked() const;

  // Resizability
  virtual bool isResizable() const { return true; }
  virtual void lock() { setLocked(true); }
  virtual void unlock() { setLocked(false); }
  virtual void setDependent(bool dependent) { m_isDependent = dependent; }
  virtual bool isDependent() const { return m_isDependent; }
  void setDecoration(DecorationType t) { m_decoration = t; }
  DecorationType getDecoration() const { return m_decoration; }

  // Transformation support
  void setTransformType(TransformationType t) { m_transformType = t; }
  TransformationType getTransformType() const { return m_transformType; }
  void setTranslationVector(const Vector_2& translation) { m_translationVector = translation; }
  Vector_2 getTranslationVector() const { return m_translationVector; }
  void setTransformValue(double val) { m_transformValue = val; }
  double getTransformValue() const { return m_transformValue; }
  void setParentSourceID(unsigned int id) { m_parentSourceID = id; }
  unsigned int getParentSourceID() const { return m_parentSourceID; }
  void setAuxObjectID(unsigned int id) { m_auxObjectID = id; }
  unsigned int getAuxObjectID() const { return m_auxObjectID; }
  void setAuxObject(std::shared_ptr<GeometricObject> aux);
  std::shared_ptr<GeometricObject> getParentSource() const { return m_parentSource.lock(); }

  std::shared_ptr<GeometricObject> getAuxObject() const { return m_auxObject.lock(); }
  void setParentSource(std::shared_ptr<GeometricObject> parent) { m_parentSource = parent; }

  virtual void relinkTransformation(std::shared_ptr<GeometricObject> parent,
                                    std::shared_ptr<GeometricObject> aux = nullptr,
                                    std::shared_ptr<GeometricObject> aux2 = nullptr) {
    restoreTransformation(parent, aux, m_transformType);
    // Note: aux2 handling is extended in derived classes like Angle
  }

  virtual void restoreTransformation(std::shared_ptr<GeometricObject> parent, std::shared_ptr<GeometricObject> aux, TransformationType type) {
    m_parentSource = parent;
    setAuxObject(aux);
    m_transformType = type;

    // CRITICAL: Re-establish observer pattern so parent updates trigger child updates
    if (parent && m_selfHandle) {
      parent->addDependent(m_selfHandle);
    }

    // Default implementation stores metadata. Derived classes (like ReflectPoint) override this.
  }

  // Dependent notification system
  virtual void addDependent(std::shared_ptr<GeometricObject> obj);
  virtual void removeDependent(GeometricObject* obj);
  virtual void notifyDependents();

  // Added validation method that can be overridden by derived classes
  virtual bool isValid() const { return m_isValid; }

  // Added getColor for serialization support
  virtual sf::Color getColor() const { return m_color; }

  // Generic Halo Helper
  virtual void drawHalo(sf::RenderTarget& target, float radius) const;

  // Hosted ObjectPoint management (Generic for all shapes)
  virtual void addChildPoint(std::shared_ptr<ObjectPoint> point);
  virtual void removeChildPoint(ObjectPoint* point);
  virtual void updateHostedPoints();
  const std::vector<std::weak_ptr<ObjectPoint>>& getHostedObjectPoints() const { return m_hostedObjectPoints; }

 protected:
  std::vector<std::weak_ptr<ObjectPoint>> m_hostedObjectPoints;
  // --- Original State Members ---
  ObjectType m_type;
  sf::Color m_color;
  unsigned int m_id;
  // Point_2 m_cgalPositionIfBase; // If Point_2 is stored in the base for all
  // objects
  // objects

  bool m_selected = false;
  bool m_hovered = false;
  int m_hoveredEdgeIndex = -1;
  bool m_isValid = true;

  // --- New Metadata (Moved to end for layout stability) ---
  bool m_visible = true;
  bool m_visibilityUserOverride = false;  // Tracks if user manually toggled visibility
  bool m_locked = false;
  bool m_isDependent = false;
  bool m_showLabel = true;
  bool m_hasUserOverride = false;  // Tracks if user manually toggled label via context menu
  sf::Vector2f m_labelOffset = {0.f, 0.f};
  std::string m_label = "";
  LabelMode m_labelMode = LabelMode::Name;
  std::string m_caption = "";
  float m_thickness = Constants::LINE_THICKNESS_DEFAULT;
  float m_vertexHandleSize = 4.0f;
  DecorationType m_decoration = DecorationType::None;

  std::vector<std::weak_ptr<GeometricObject>> m_dependents;
  std::shared_ptr<GeometricObject> m_selfHandle;
  TransformationType m_transformType = TransformationType::None;
  unsigned int m_parentSourceID = 0;
  unsigned int m_auxObjectID = 0;
  Vector_2 m_translationVector = Vector_2(0, 0);
  double m_transformValue = 0.0;
  std::weak_ptr<GeometricObject> m_parentSource;
  std::weak_ptr<GeometricObject> m_auxObject;
};

#endif  // GEOMETRIC_OBJECT_H