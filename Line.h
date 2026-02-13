#pragma once // Use pragma once for modern include guard
#ifndef LINE_H
#define LINE_H

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

#include "CharTraitsFix.h" // Ensure this is very early
#include "Constants.h"
#include "ForwardDeclarations.h" // For ObjectType, Point, ObjectPoint
#include "GeometricObject.h"
#include "ObjectType.h"
#include "Point.h"      // For Point*
#include "Transforms.h" // For CoordinateTransform
#include "Types.h"      // For Point_2, Line_2
#include <SFML/Graphics.hpp>
#include <algorithm> // For std::remove
#include <chrono>    // Add include for time-based debounce
#include <map>
#include <memory>
#include <set> // Add this for std::set used in maintainConstraints
#include <stdexcept>
#include <string>
#include <thread> // For thread_local
#include <vector> // For std::vector

// Forward declarations
class Point;
class ObjectPoint;
class Object; // Added forward declaration for Object

// Line render style struct for applying styles to lines
struct LineRenderStyle {
  sf::Color color;
  float thickness;
  bool dashed;
  float gapLength;
  bool arrowheadsEnabled;
  float arrowheadSize;
  bool curved;
  float curveRadius;
  bool smooth;
  bool spline;
  float knotSpacing;
  bool showControlPoints;
  bool showTangents;
  bool showNormals;
  bool showIntersectionPoints;
};

class Line : public GeometricObject, public std::enable_shared_from_this<Line> {
public:
  // Constructors
  Line(std::shared_ptr<Point> start, std::shared_ptr<Point> end,
       bool isSegment = false,
       const sf::Color &color = Constants::LINE_DEFAULT_COLOR);
  Line(const Point_2 &start, const Point_2 &end, bool isSegment = false,
       const sf::Color &color = Constants::LINE_DEFAULT_COLOR);
  Line(std::shared_ptr<Point> start, std::shared_ptr<Point> end, bool isSegment,
       const sf::Color &color, unsigned int id);
  ~Line() override;
  //---- enum class for LineType ----
  enum class LineType { Infinite, Segment, Ray, Vector };
  // --- GeometricObject Overrides ---
  virtual void draw(sf::RenderWindow &window, float scale, bool forceVisible = false) const override;
  virtual void drawLabel(sf::RenderWindow &window, const sf::View &worldView) const override;
  bool
  contains(const sf::Vector2f &worldPos_sfml,
           float tolerance = Constants::LINE_INTERACTION_RADIUS) const override;
  void setSelected(bool sel) override;
  void setHovered(bool hoveredStatus) override;
  sf::FloatRect getGlobalBounds() const override;
  void update() override;
  void updateDependentShape() override;
  void translate(const Vector_2 &offset) override;
  Point_2 getCGALPosition() const override;
  void setCGALPosition(const Point_2 &newPos) override;
  void setPosition(const sf::Vector2f &newSfmlPos) override;
  ObjectType getType() const override {
    if (m_lineType == LineType::Ray) return ObjectType::Ray;
    if (m_lineType == LineType::Vector) return ObjectType::Vector;
    return m_isSegment ? ObjectType::LineSegment : ObjectType::Line;
  }
  std::vector<Segment_2> getEdges() const override;
  std::vector<Segment_2> getBoundarySegments() const override;
  bool getClosestPointOnPerimeter(const Point_2 &query, Point_2 &outPoint) const override;

  // Label anchor for movable labels
  sf::Vector2f getLabelAnchor(const sf::View &view) const override;

  // --- Line Specific Methods ---
  Point *getStartPointObject() const { return m_startPoint.get(); }
  Point *getEndPointObject() const { return m_endPoint.get(); }
  std::shared_ptr<Point> getStartPointObjectShared() const { return m_startPoint; }
  std::shared_ptr<Point> getEndPointObjectShared() const { return m_endPoint; }
  Point_2 getStartPoint() const;
  Point_2 getEndPoint() const;
  Line_2 getCGALLine() const; // Throws if endpoints are null or coincident
  bool isSegment() const { return m_isSegment; }
  void setIsSegment(bool segment);
  void setLineType(LineType type) { m_lineType = type; if(type == LineType::Segment || type == LineType::Vector) m_isSegment = true; else m_isSegment = false; }
  LineType getLineType() const { return m_lineType; }
  // Vector_2 getDirection() const;
  Direction_2 getDirection() const;
  Point *getStartPointPtr() const { return m_startPoint.get(); }
  Point *getEndPointPtr() const { return m_endPoint.get(); }
  bool getIsUnderDirectManipulation() const {
    return m_isUnderDirectManipulation;
  }

  // --- Constraint Methods ---
  bool isSelected() const override { return m_selected; }
  bool isParallelLine() const { return m_isParallelLine; }
  bool isPerpendicularLine() const { return m_isPerpendicularLine; }
  void setAsParallelLine(std::shared_ptr<GeometricObject> refObj, int edgeIndex,
                         const Vector_2 &referenceDirection);
  void setAsPerpendicularLine(std::shared_ptr<GeometricObject> refObj, int edgeIndex,
                              const Vector_2 &referenceDirection);
  bool maintainConstraints();  // Returns true if geometry changed
  void forceConstraintUpdate();
  
  // Constraint reference getters for serialization
  std::shared_ptr<GeometricObject> getConstraintRefObject() const {
    return m_constraintRefObject.lock();
  }
  int getConstraintRefEdgeIndex() const { return m_constraintRefEdgeIndex; }



  // --- Endpoint Manipulation ---
  void setPoints(std::shared_ptr<Point> start, std::shared_ptr<Point> end);
  void moveEndpointToStart(const Point_2 &newPos);
  void moveEndpointToEnd(const Point_2 &newPos);

  // --- Hosted ObjectPoint Management ---
  // addChildPoint, removeChildPoint, updateHostedPoints inherited from GeometricObject
  void notifyObjectPoints(); // Kept for backward compatibility, calls updateHostedPoints
  // const std::vector<std::weak_ptr<ObjectPoint>> &getHostedObjectPoints() const; // Inherited
  
  void clearEndpointIfMatches(Point *p); // Specific to Line endpoints
  
  // --- Internal Update Methods ---

  // --- Internal Update Methods ---
  virtual void updateSFMLShape();
  void updateCGALLine();
  void validate() const; // For debugging
  void setIsUnderDirectManipulation(bool isManipulated);
  void setAsConstructionLine();
  bool isConstructionLine() const;
  void setColor(const sf::Color &color) override;

  // --- Visibility Methods ---
  void toggleVisibility();
  bool isVisible() const override;
  void setHidden(bool hidden);
  bool isHidden() const;
  void setAxis(bool axis) { m_isAxis = axis; }
  bool isAxis() const { return m_isAxis; }

  // --- Snapping Methods ---
  void setSnapToGrid(bool snap);
  bool isSnappedToGrid() const;
  void setGridSnapInterval(float interval);
  float getGridSnapInterval() const;
  void setEndpointSnapEnabled(bool enabled);
  bool isEndpointSnapEnabled() const;
  void setMidpointSnapEnabled(bool enabled);
  bool isMidpointSnapEnabled() const;
  void setSnapDistance(float distance);
  float getSnapDistance() const;

  // --- Display Options ---
  void setShowEndpoints(bool show);
  bool areEndpointsShown() const;
  void setEndpointSize(float size);
  float getEndpointSize() const;
  void setShowMidpoints(bool show);
  bool areMidpointsShown() const;
  void setMidpointSize(float size);
  float getMidpointSize() const;
  // Label offset management inherited from GeometricObject

  // --- Line Style Methods ---
  void setLineRenderStyle(LineRenderStyle style);
  LineRenderStyle getLineRenderStyle() const;
  void setDashed(bool dashed);
  bool isDashed() const;
  void setDashLength(float length);
  float getDashLength() const;
  void setGapLength(float length);
  float getGapLength() const;

  // --- Arrowhead Methods ---
  void setArrowheadsEnabled(bool enabled);
  bool areArrowheadsEnabled() const;
  void setArrowheadSize(float size);
  float getArrowheadSize() const;
  void setArrowheadDistance(float distance);
  float getArrowheadDistance() const;

  // --- Curve Methods ---
  void setCurved(bool curved);
  bool isCurved() const;
  void setCurveRadius(float radius);
  float getCurveRadius() const;
  void setSmooth(bool smooth);
  bool isSmooth() const;
  void setSpline(bool spline);
  bool isSpline() const;
  void setKnotSpacing(float spacing);
  float getKnotSpacing() const;

  // --- Control Point Methods ---
  void setShowControlPoints(bool show);
  bool areControlPointsShown() const;
  void setControlPointSize(float size);
  float getControlPointSize() const;
  void setControlPointColor(const sf::Color &color);
  const sf::Color &getControlPointColor() const;
  void setControlPointOutlineColor(const sf::Color &color);
  const sf::Color &getControlPointOutlineColor() const;
  void setControlPointOutlineThickness(float thickness);
  float getControlPointOutlineThickness() const;

  // --- Tangent Methods ---
  void setShowTangents(bool show);
  bool areTangentsShown() const;
  void setTangentLength(float length);
  float getTangentLength() const;
  void setTangentColor(const sf::Color &color);
  const sf::Color &getTangentColor() const;
  void setTangentOutlineColor(const sf::Color &color);
  const sf::Color &getTangentOutlineColor() const;
  void setTangentOutlineThickness(float thickness);
  float getTangentOutlineThickness() const;

  // --- Normal Methods ---
  void setShowNormals(bool show);
  bool areNormalsShown() const;
  void setNormalLength(float length);
  float getNormalLength() const;
  void setNormalColor(const sf::Color &color);
  const sf::Color &getNormalColor() const;
  void setNormalOutlineColor(const sf::Color &color);
  const sf::Color &getNormalOutlineColor() const;
  void setNormalOutlineThickness(float thickness);
  float getNormalOutlineThickness() const;

  // --- Intersection Point Methods ---
  void setShowIntersectionPoints(bool show);
  bool areIntersectionPointsShown() const;
  void setIntersectionPointSize(float size);
  float getIntersectionPointSize() const;
  void setIntersectionPointColor(const sf::Color &color);
  const sf::Color &getIntersectionPointColor() const;
  void setIntersectionPointOutlineColor(const sf::Color &color);
  const sf::Color &getIntersectionPointOutlineColor() const;
  void setIntersectionPointOutlineThickness(float thickness);
  float getIntersectionPointOutlineThickness() const;

  // --- General State Methods ---
  void setShow(bool show);
  bool isShown() const;
  void setVisible(bool visible) override;
  void setLocked(bool locked) override;
  bool isLocked() const override;
  void setID(int id);
  // int getID() const;
  void setParent(Object *parent);
  Object *getParent() const;
  void setUserData(void *data);
  void *getUserData() const;

  // --- Data Methods ---
  void setData(const std::string &key, const std::string &value);
  std::string getData(const std::string &key) const;
  void setColorByName(const std::string &colorName);
  void setThicknessByName(const std::string &thicknessName);
  void setStyleByName(const std::string &styleName);
  void setArrowheadByName(const std::string &arrowheadName);
  void setCurvatureByName(const std::string &curvatureName);
  void setVisibilityByName(const std::string &visibilityName);
  void setSnapToGridByName(const std::string &snapName);
  void setEndpointSnapEnabledByName(const std::string &enabledName);
  void setMidpointSnapEnabledByName(const std::string &enabledName);
  void setDashedByName(const std::string &dashedName);
  void setArrowheadsEnabledByName(const std::string &enabledName);
  void setCurvedByName(const std::string &curvedName);
  void setSmoothByName(const std::string &smoothName);
  void setSplineByName(const std::string &splineName);
  void setShowControlPointsByName(const std::string &showName);
  void setShowTangentsByName(const std::string &showName);
  void setShowNormalsByName(const std::string &showName);
  void setShowIntersectionPointsByName(const std::string &showName);
  void setDataByName(const std::string &key, const std::string &value);

  // --- Style Application ---
  void applyStyle(const LineRenderStyle &style);

  // --- Serialization Methods ---
  void loadFromStream(std::istream &is);
  void saveToStream(std::ostream &os) const;

  // --- Transformation Methods ---
  void transform(const CoordinateTransform &transform);
  void scale(float factor);
  void rotate(float angle);
  void translateWithDependents(const Vector_2 &offset);

  // void updateHostedPoints(); // Inherited from GeometricObject

  // Constraint observer methods
  void addConstraintObserver(std::shared_ptr<Line> observer);
  void removeConstraintObserver(std::shared_ptr<Line> observer);
  void clearConstraintReference(
      Line *disappearingRef); // Called by a disappearing reference
  void calculateCanonicalLength();
  void initializeSFMLShape();
  void registerWithEndpoints();
  // void removeChildPointByRawPointer(ObjectPoint *rawPtr); // Removed, use Generic base method
  void prepareForDestruction();
  // Safety utility methods
  Vector_2 getSafeDirection() const;
  void attemptDataRepair();

  // Add a method to notify observers about changes
  void notifyObservers();

  // Override isValid to check for valid endpoints
  bool isValid() const override {
    return (m_startPoint != nullptr && m_endPoint != nullptr);
  }

  void safeResetEndpoints() {
    // first try to notify connected Points, but only if we really live under a
    // shared_ptr
    try {
      auto self = this->shared_from_this();
      if (m_startPoint) {
        m_startPoint->removeConnectedLine(self.get());
      }
      if (m_endPoint) {
        m_endPoint->removeConnectedLine(self.get());
      }
    } catch (const std::bad_weak_ptr &) {
      // no shared_ptr owns us (e.g. preview‐line) ⇒ skip notification
    }

    // now clear without notification
    m_startPoint.reset();
    m_endPoint.reset();
  }

  void setTemporaryPreviewPoints(std::shared_ptr<Point> p1,
                                 std::shared_ptr<Point> p2);
  
  void clearTemporaryPreviewPoints() {
      m_tempPreviewPoint1.reset();
      m_tempPreviewPoint2.reset();
  }
  
  void setExternallyMovedEndpoint(Point *p);
  // Internal method to notify observers
  void notifyConstraintObserversOfChange();

  void setDeferSFMLUpdates(bool defer);
  void forceSFMLUpdate();
  bool isDeferringSFMLUpdates() const { return m_deferSFMLUpdates; }
  void clearConstraintReference(std::shared_ptr<Line> lineBeingDestroyed);
private:
  LineType m_lineType;
  std::shared_ptr<Point> m_startPoint;
  std::shared_ptr<Point> m_endPoint;

  // std::vector<std::weak_ptr<ObjectPoint>> m_hostedObjectPoints; // Inherited from GeometricObject
  bool m_isSegment;
  Line_2 m_cgalLine;
  sf::VertexArray m_sfmlShape;
  Kernel::FT m_canonicalLength;
  bool m_isUpdating = false;           // Flag to prevent re-entrant updates
  bool m_isUpdatingInternally = false; // Flag for internal updates
  bool m_wasUpdatingAtEntry = false;
  bool m_isUnderDirectManipulation = false;
  // Constraint properties
  bool m_isParallelLine = false;
  bool m_isPerpendicularLine = false;
  std::weak_ptr<GeometricObject> m_constraintRefObject; // Check generic object for constraint
  int m_constraintRefEdgeIndex = -1; // -1 = object itself (if line), >=0 = specific edge index
  //--- preview Points ---
  std::shared_ptr<Point> m_tempPreviewP1;
  std::shared_ptr<Point> m_tempPreviewP2;

  std::vector<std::weak_ptr<Line>>
      m_constraintObservers; // Lines that are constrained by *this* line

  // Additional properties from Line.cpp methods
  bool m_isConstructionLine = false;
  bool m_isAxis = false;
  bool m_snapToGrid = true;
  float m_gridSnapInterval = 10.0f;
  bool m_endpointSnapEnabled = true;
  bool m_midpointSnapEnabled = true;
  float m_snapDistance = 10.0f;
  bool m_showEndpoints = true;
  float m_endpointSize = 5.0f;
  bool m_showMidpoints = false;
  float m_midpointSize = 3.0f;
  LineRenderStyle m_lineRenderStyle;
  bool m_dashed = false;
  float m_dashLength = 10.0f;
  float m_gapLength = 5.0f;
  bool m_arrowheadsEnabled = false;
  float m_arrowheadSize = 10.0f;
  float m_arrowheadDistance = 0.0f;
  bool m_curved = false;
  float m_curveRadius = 25.0f;
  bool m_smooth = false;
  bool m_spline = false;
  float m_knotSpacing = 50.0f;
  bool m_showControlPoints = false;
  float m_controlPointSize = 4.0f;
  sf::Color m_controlPointColor = sf::Color::White;
  sf::Color m_controlPointOutlineColor = sf::Color::Black;
  float m_controlPointOutlineThickness = 1.0f;
  bool m_showTangents = false;
  float m_tangentLength = 50.0f;
  sf::Color m_tangentColor = sf::Color::Green;
  sf::Color m_tangentOutlineColor = sf::Color::Black;
  float m_tangentOutlineThickness = 1.0f;
  bool m_showNormals = false;
  float m_normalLength = 50.0f;
  sf::Color m_normalColor = sf::Color::Red;
  sf::Color m_normalOutlineColor = sf::Color::Black;
  float m_normalOutlineThickness = 1.0f;
  bool m_showIntersectionPoints = false;
  float m_intersectionPointSize = 5.0f;
  sf::Color m_intersectionPointColor = sf::Color::Red;
  sf::Color m_intersectionPointOutlineColor = sf::Color::Black;
  float m_intersectionPointOutlineThickness = 1.0f;
  Vector_2 m_constraintDirection =
      Vector_2(0, 0); // Direction for parallel/perpendicular
  bool m_show = true;
  Object *m_parent = nullptr;
  void *m_userData = nullptr;
  std::map<std::string, std::string> m_data;
  // float m_thickness = Constants::LINE_THICKNESS;  // REMOVED: Use inherited m_thickness from GeometricObject

  // Prevent re‐entrant maintainConstraints calls
  bool m_inConstraint = false;
  Point *m_externallyMovedEndpoint = nullptr;
  static thread_local std::set<Line *> s_updatingLines;
  bool m_deferSFMLUpdates = false;
  bool m_pendingSFMLUpdate = false;
  std::shared_ptr<Point> m_tempPreviewPoint1;
  std::shared_ptr<Point> m_tempPreviewPoint2;
};

#endif // LINE_H