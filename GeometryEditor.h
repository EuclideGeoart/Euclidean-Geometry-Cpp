#pragma once

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#warning "CGAL_USE_SSE2 was defined, now undefined locally for testing"
#endif

// Attempt to fix char_traits issues by including these first globally.
#include <string>  // Ensure standard string is included very early

#include "CharTraitsFix.h"  // For char_traits<unsigned int> specialization

// First include CompilerFixes.h to set up the necessary defines
#include "CompilerFixes.h"  // For specific compiler workarounds

// Forward declarations for enums and potentially classes to break circular
// dependencies
#include "ForwardDeclarations.h"  // Contains DragMode, EndpointSelection, ObjectType etc.

// Then include standard library headers
#include <memory>    // For std::unique_ptr, std::make_unique
#include <map>
#include <optional>  // For std::optional
#include <vector>    // For std::vector

// Next include SFML
#include <SFML/Graphics.hpp>  // Core SFML graphics, window, events

// Include Types.h which contains CGAL typedefs
#include "Types.h"  // Your CGAL type definitions (Point_2, Line_2, etc.)

// Then include your own project headers
#include "Circle.h"
#include "CommandSystem.h"    // For undo/redo functionality
#include "GUI.h"              // GUI class for buttons, messages
#include "GeometricObject.h"  // Base class for all geometric entities
#include "Grid.h"             // Visual grid
#include "Intersection.h"
#include "Line.h"
#include "ObjectPoint.h"  // Points that are part of other objects (e.g., line endpoints)
#include "Point.h"        // Free-standing points
#include "Rectangle.h"    // Axis-aligned and rotatable rectangles
#include "Polygon.h"      // User-drawn polygons
#include "RegularPolygon.h"  // Regular N-sided polygons
#include "Triangle.h"        // General triangles
#include "Angle.h"           // Angle measurement
#include "PointUtils.h"      // For EdgeHitResult and anchor point detection
#include "TextLabel.h"
#include "SymbolPalette.h"
#include "TextEditorDialog.h"

// Note: Headers like "HandleEvents.h" (if it was a .h file for functions) are
// not typically included if GeometryEditor itself implements the event handling
// logic by calling functions from a HandleEvents.cpp. If HandleEvents.cpp
// contains free functions that GeometryEditor.cpp calls, then no .h inclusion
// is needed here.
/**
 * @brief Looks for a geometric object at the given world position.
 *
 * Iterates through specified object types (or all if allowedTypes is empty)
 * and returns the first object found within the tolerance.
 * Priority can be given (e.g., points over lines).
 *
 * @param worldPos_sfml The SFML world coordinates to check.
 * @param tolerance The click tolerance radius.
 * @param allowedTypes A list of ObjectTypes to consider. If empty, all relevant
 * types are checked.
 * @return GeometricObject* Pointer to the found object, or nullptr if no object
 * is found.
 */
class GeometryEditor {
  // Generic object insertion
  
 public:
  GeometryEditor();
  ~GeometryEditor();  // Add destructor declaration
  void run();

  void addObject(const std::shared_ptr<GeometricObject>& obj);

  // --- Core Components (public for now, consider access levels later if
  // needed) ---
  sf::ContextSettings settings;  // SFML Context settings (AA, depth, stencil) -
                                 // DECLARED BEFORE WINDOW
  sf::RenderWindow window;
  sf::View drawingView;           // View for the main drawing canvas
  sf::View guiView;               // View for the GUI elements (usually static or scaled
                                  // differently)
  GUI gui;                        // Manages GUI elements and interactions
  Grid grid;                      // Manages the background grid
  CommandManager commandManager;  // Manages undo/redo operations

  // --- Object Storage ---
  // Using separate vectors for different types of geometric objects
  std::vector<std::shared_ptr<Point>> points;  // Free-standing points
  // Points attached to other objects
  std::vector<std::shared_ptr<ObjectPoint>> ObjectPoints;
  std::vector<std::shared_ptr<Line>> lines;
  std::vector<std::shared_ptr<Circle>> circles;
  std::vector<std::shared_ptr<Rectangle>> rectangles;  // Axis-aligned and rotatable rectangles
  std::vector<std::shared_ptr<Polygon>> polygons;      // User-drawn polygons
  std::vector<std::shared_ptr<RegularPolygon>> regularPolygons;  // Regular N-sided polygons
  std::vector<std::shared_ptr<Triangle>> triangles;              // General triangles
  std::vector<std::shared_ptr<Angle>> angles;                    // Angle measurements
  std::vector<std::shared_ptr<TextLabel>> textLabels;            // Text/LaTeX labels


  // ---Color Management---
  void setCurrentColor(sf::Color color) { currentColor = color; }

  sf::Color getCurrentColor() const { return currentColor; }
  void changeSelectedObjectColor(sf::Color newColor);

  std::vector<std::shared_ptr<Point>> getAllPoints() const;

  float currentThickness = Constants::LINE_THICKNESS_DEFAULT;
  float currentPointSize = 4.0f;
  float guiFontSize = 13.0f;        // Font size for ImGui interface
  float drawingFontSize = 18.0f;    // Font size for drawable object labels
  
  GUI& getGUI() { return gui; }

  // Consider: std::vector<std::unique_ptr<GeometricObject>> allObjects;
  // This could simplify rendering and some generic operations if all objects
  // are added here too.

  // --- Interaction State ---
  sf::Vector2f lastMousePos_sfml;                   // Last known mouse position in SFML world
                                                    // coordinates (drawingView)
  ObjectType m_currentToolType = ObjectType::None;  
  DragMode dragMode = DragMode::None;               // Current dragging operation type
  bool isDragging = false;  // General flag indicating an active drag operation
  int activeVertexIndex = -1;  // For shape vertex dragging
  GeometricObject *activeVertexShape = nullptr;  // Shape owning the active vertex

  // Label Visibility Control
  enum class LabelVisibilityMode {
    All,
    PointsOnly,
    None
  };
  LabelVisibilityMode m_labelVisibility = LabelVisibilityMode::PointsOnly;
  
  // Getter/Setter for Label Policy (encapsulation)
  LabelVisibilityMode getLabelVisibilityMode() const { return m_labelVisibility; }
  void setLabelVisibilityMode(LabelVisibilityMode mode, bool clearUserOverrides = true);
  
  // Apply label policy to a specific object (respects user overrides)
  void applyLabelPolicy(GeometricObject* obj);
  
  // Enforce label policy on ALL existing objects (for startup/load/mode change)
  void enforceLabelPolicyOnAll(bool clearUserOverrides = false);

  // For Line interactions (primarily used by event handling logic)
  EndpointSelection m_selectedEndpoint =
      EndpointSelection::None;  // Which endpoint of a line is being dragged
  sf::Vector2f dragStart_sfml;  // For object translation, stores the initial
                                // mouse click position (SFML world coords)
  std::map<unsigned int, Point_2> m_dragStartPositions;  // Point ID -> position snapshot at drag start

  // For general object selection
  GeometricObject *selectedObject =
      nullptr;  // Pointer to the currently selected geometric object
                // (raw pointer, object owned by one of the vectors above)
  std::vector<GeometricObject *> selectedObjects;  // Multi-selection list (raw pointers)
  GeometricObject *hoveredObject = nullptr;  // Object currently under mouse for hover feedback
    int hoveredVertexIndex = -1;
    GeometricObject *hoveredVertexShape = nullptr;

  // Label UI state
  bool showGlobalLabels = true;
  bool isDraggingLabel = false;
  GeometricObject *labelDragObject = nullptr;
  sf::Vector2f labelDragGrabOffset = sf::Vector2f(0.f, 0.f);
  int labelDragVertexIndex = -1;  // For shapes with multiple vertex labels (Rectangle, etc.)

  // Text editing UI state
  bool isTextEditing = false;
  bool isEditingExistingText = false; // Flag for TextEditorDialog saves
  std::shared_ptr<TextLabel> textEditingLabel = nullptr;
  std::string textEditBuffer;
  bool textEditIsRich = false;
  float textEditFontSize = 18.0f;
  size_t textCursorIndex = 0;
  sf::Clock textCursorBlinkClock;
  bool showSymbolPalette = false;
  SymbolPalette textSymbolPalette;
  sf::Vector2f textPalettePosition{0.f, 0.f};
  bool useImGuiSymbolPalette = true;
  std::string textPaletteSearch;

  // Text editor dialog
  TextEditorDialog textEditorDialog;

  // Text box creation state
  bool isCreatingTextBox = false;
  sf::Vector2f textBoxStart_sfml;
  sf::Vector2f textBoxCurrent_sfml;
  sf::RectangleShape textBoxPreviewShape;
  float textBoxStartScale = 1.0f;
    
  // Edge hover tracking for universal snapping
  std::optional<EdgeHitResult> m_hoveredEdge;  // Currently hovered edge (if any)
  
  // Angle Resizing
  bool isResizingAngle = false;
  
  GeometricObject *lookForObjectAt(const sf::Vector2f &worldPos_sfml, float tolerance,
                                   const std::vector<ObjectType> &allowedTypes = {});

  // If you prefer to pass initializer_list directly from the call site:
  GeometricObject *lookForObjectAt(const sf::Vector2f &worldPos_sfml, float tolerance,
                                   std::initializer_list<ObjectType> allowedTypes);
  // Tool Hint for UI
  std::string m_toolHint = "Welcome! Select a tool to begin.";
  void setToolHint(const std::string& hint) { m_toolHint = hint; }
  std::string getToolHint() const { return m_toolHint; }

  void setGUIMessage(const std::string &message);
  // --- Selection Box State ---
  bool isDrawingSelectionBox = false;
  sf::Vector2f selectionBoxStart_sfml;
  sf::RectangleShape selectionBoxShape;
  sf::Vector2f potentialSelectionBoxStart_sfml;
  // --- Object Creation State (primarily used by event handling logic) ---
  // Line Creation
  std::shared_ptr<Point> lineCreationPoint1 = nullptr;  // First point selected/created for
                                                        // a new line (shared pointer)

  // Circle Creation
  bool isCreatingCircle = false;          // Flag indicating if a circle is currently being created
  Point_2 createStart_cgal;               // CGAL coordinates of the circle's center during
                                          // creation
  std::shared_ptr<Circle> previewCircle;  // For visual feedback during circle creation

  // Semicircle Creation
  bool isCreatingSemicircle = false;
  std::shared_ptr<Point> semicirclePoint1 = nullptr;
  std::shared_ptr<Point> semicirclePoint2 = nullptr;

  // Circle (3 Points) Creation
  bool isCreatingCircle3P = false;
  std::vector<Point_2> circle3PPoints;
  std::vector<std::shared_ptr<Point>> circle3PPointObjects;

  // Perpendicular Bisector tool state
  bool isCreatingPerpendicularBisector = false;
  std::shared_ptr<Point> perpBisectorP1 = nullptr;
  std::shared_ptr<Point> perpBisectorP2 = nullptr;
  std::shared_ptr<Line> perpBisectorLineRef = nullptr; // optional line-segment input

  // Angle Bisector tool state
  bool isCreatingAngleBisector = false;
  std::vector<std::shared_ptr<Point>> angleBisectorPoints; // expecting up to 3 points A,B,C (B vertex)
  std::shared_ptr<Line> angleBisectorLine1 = nullptr;
  std::shared_ptr<Line> angleBisectorLine2 = nullptr;

  // Tangent tool state
  bool isCreatingTangent = false;
  std::shared_ptr<Point> tangentAnchorPoint = nullptr;
  std::shared_ptr<Circle> tangentCircle = nullptr;

  // Rectangle Creation
  bool isCreatingRectangle = false;       // Flag indicating if a rectangle is being created
  bool isCreatingRotatableRectangle = false;  // Flag for rotatable rectangle mode
  Point_2 rectangleCorner1;               // First corner of rectangle
  Point_2 rectangleCorner2;               // Second corner for axis-aligned, or adj point for rotatable
  std::shared_ptr<Point> rectangleCorner1Point = nullptr;  // First corner point (shared)
  std::shared_ptr<Point> rectangleCorner2Point = nullptr;  // Second corner point (shared)
  std::shared_ptr<Rectangle> previewRectangle;  // Preview during creation

  // Polygon Creation
  bool isCreatingPolygon = false;         // Flag indicating if a polygon is being created
  std::vector<Point_2> polygonVertices;   // Vertices being added to polygon
  std::vector<std::shared_ptr<Point>> polygonVertexPoints; // Shared pointers for topological linking
  std::shared_ptr<Polygon> previewPolygon;  // Preview during creation

  // Regular Polygon Creation
  bool isCreatingRegularPolygon = false;  // Flag indicating if a regular polygon is being created
  int regularPolygonPhase = 0;            // 0: awaiting first vertex, 1: awaiting second vertex
  Point_2 regularPolygonCenter;           // Center point of regular polygon
  Point_2 regularPolygonFirstVertex;      // First vertex (defines radius)
  std::shared_ptr<Point> regularPolygonCenterPoint = nullptr;      // Shared pointer for center
  std::shared_ptr<Point> regularPolygonFirstVertexPoint = nullptr; // Shared pointer for first vertex
  Point_2 regularPolygonEdgeStart;        // Edge mode: first edge vertex
  Point_2 regularPolygonEdgeEnd;          // Edge mode: second edge vertex
  std::shared_ptr<Point> regularPolygonEdgeStartPoint = nullptr;   // Edge mode start point
  std::shared_ptr<Point> regularPolygonEdgeEndPoint = nullptr;     // Edge mode end point
  int regularPolygonNumSides = 6;         // Default to hexagon
  bool showRegularPolygonSidesPopup = false;
  std::shared_ptr<RegularPolygon> previewRegularPolygon;  // Preview during creation

  // Compass Tool State
  std::vector<GeometricObject*> m_compassSelection;
  std::shared_ptr<Point> m_previewCompassCenter;

  // Triangle Creation
  bool isCreatingTriangle = false;        // Flag indicating if a triangle is being created
  std::vector<Point_2> triangleVertices;  // Vertices being added to triangle (up to 3)
  std::vector<std::shared_ptr<Point>> triangleVertexPoints; // Shared pointers for topological linking
  std::shared_ptr<Triangle> previewTriangle;  // Preview during creation

  // Angle Creation
  std::shared_ptr<Point> anglePointA = nullptr;
  std::shared_ptr<Point> angleVertex = nullptr;
  std::shared_ptr<Point> anglePointB = nullptr;
  std::shared_ptr<Line> angleLine1 = nullptr;
  std::shared_ptr<Line> angleLine2 = nullptr;
  bool showAngleInputPopup = false;
  float angleInputDegrees = 60.0f;

  // --- Preview Overlay (lightweight, no CGAL) ---
  sf::VertexArray previewLineOverlay{sf::Lines, 2};
  bool hasPreviewLineOverlay = false;
  std::shared_ptr<sf::CircleShape> m_ghostCursor; // Visual feedback for tools (e.g., Smart Point)
  bool m_isHoveringIntersection = false;
  Point_2 m_hoveredIntersectionPos = Point_2(FT(0), FT(0));
  std::shared_ptr<Line> m_hoveredIntersectionLine1;
  std::shared_ptr<Line> m_hoveredIntersectionLine2;
  bool m_isHoveringLine = false;
  std::shared_ptr<Line> m_hoveredLine;
  Point_2 m_hoveredLineSnapPos = Point_2(FT(0), FT(0));
  PointUtils::SnapState m_snapState;
  bool m_wasSnapped = false;
  std::shared_ptr<Point> m_snapTargetPoint = nullptr;
  bool m_universalSnappingEnabled = true;  // Toggle for universal smart snapping

  // --- Panning State (primarily used by event handling logic) ---
  bool isPanning = false;
  sf::Vector2f panningStartMousePos_view;     // Mouse position in SFML view
                                              // coordinates when panning started
  sf::Vector2f panningStartViewCenter_world;  // Drawing view's center in SFML world
                                              // coordinates when panning started

  // --- Hover Message State ---
  sf::Text hoverMessageText;
  bool showHoverMessage = false;
  std::string hoverMessage;
  std::string currentTooltip; // For transformation info on hover
  sf::Text hoverText;
  sf::Font defaultFont;
  sf::Vector2f currentMousePos_sfml;

  unsigned int objectIdCounter = 0;
  GeometricObject *fillTarget = nullptr;  // Right-click fill target
  
  // --- Renaming State ---
  bool isRenaming = false;
  std::shared_ptr<Point> pointToRename = nullptr;
  std::string renameBuffer;
  
  bool objectExistsInAnyList(GeometricObject *obj);
  void sanitizeReferences(const GeometricObject* objToDelete);
  // --- Utility Methods ---
  sf::Vector2f toSFMLVector(const Point_2 &cgal_point) const;  // Convert CGAL point to SFML vector
  Point_2 toCGALPoint(const sf::Vector2f &sfml) const {
    // Add some debugging when converting points during dragging
    if (isDragging && dragMode == DragMode::MoveFreePoint) {
      std::cout << "Converting SFML position to CGAL: (" << sfml.x << ", " << sfml.y << ")"
                << std::endl;
    }
    return Point_2(FT(sfml.x), FT(sfml.y));
  }
  Vector_2 toCGALVector(const sf::Vector2f &sfmlVector) const;
  float getScaledTolerance(const sf::View &currentView) const;  // Tolerance adjusted for zoom level
  float length(const sf::Vector2f &vec) const;                  // Utility for SFML vector length

  // Operation functions
  void cancelOperation();
  void deleteSelected();
  void updateAllGeometry();
  void updateScaleFactor();
  void updateConstraintsOnly();
  
  // --- Project Save/Load/Export ---
  void saveProject(const std::string& filepath = "project.json");
  void loadProject(const std::string& filepath = "project.json");
  void exportSVG(const std::string& filepath = "export.svg");
  void clearScene();  // Clear all objects from the scene
  // Zoom limits
  const float MAX_ZOOM_IN = 0.05f;
  const float MAX_ZOOM_OUT = 10.0f;

  // --- Main Event Dispatcher ---
  void processEvents();  // Main loop for processing SFML events

  void render();                    // Renders all objects, grid, and GUI
  void update(sf::Time deltaTime);  // Updates game state, animations, etc.

  // --- Internal Helper Methods & Event Handlers ---
  // These would be called by processEvents()
  void handleWindowEvents(
      const sf::Event &event);  // Handles window-specific events (close, resize)
  void handleMousePress(const sf::Event::MouseButtonEvent &mouseButtonEvent);
  void handleMouseRelease(const sf::Event::MouseButtonEvent &mouseButtonEvent);
  void handleMouseMove(const sf::Event::MouseMoveEvent &mouseMoveEvent);
  void handleMouseWheel(const sf::Event::MouseWheelScrollEvent &mouseWheelEvent);
  void handleKeyPress(const sf::Event::KeyEvent &keyEvent);
  // -----Getter -----
  ObjectType getCurrentTool() const { return m_currentToolType; }
  std::string getCurrentToolName() const;
  // Object creation helpers
  // Object creation helpers
  void createNewPoint(const Point_2 &cgalPos, bool isIntersection = false);
  
  // -- Centralized Point Factory --
  // Creates, labels, and registers a new Point.
  std::shared_ptr<Point> createPoint(const Point_2 &cgalPos); 
  std::shared_ptr<Point> createPoint(const sf::Vector2f &sfmlPos);

  /**
   * @brief Returns a combined vector of all geometric objects in the editor.
   * Useful for label collision detection, multi-selection, etc.
   */
  std::vector<std::shared_ptr<GeometricObject>> getAllObjects() const;
  void startLineCreation(Point *startPt);
  void updateLinePreview(const Point_2 &currentMouseCgalPos);
  void finishLineCreation(Point *endPt);
  void startCircleCreation(const Point_2 &centerCgalPos);
  void updateCirclePreview(const Point_2 &currentMouseCgalPos);
  void finishCircleCreation();
  void resetCreationStates();
  void resetParallelLineToolState();
  void resetPerpendicularLineToolState();
  void handlePointDragging(Point *draggedPoint, const Point_2 &newPosition);
  std::shared_ptr<Line> getLineSharedPtr(Line *rawPtr);
  std::shared_ptr<Circle> getCircleSharedPtr(Circle *rawPtr);
  // Selection and Interaction
  void selectObjectAt(const sf::Vector2f &worldPos);
  void clearSelection();
  void deleteSelectedObject();  // Or deleteSelectedObjects if multi-select is planned
  void safeDeleteLine(std::shared_ptr<Line> lineToDelete);
  void clearHoverReferences(GeometricObject* obj);
  
  // Helper to find shared_ptr from raw pointer
  std::shared_ptr<GeometricObject> findSharedPtr(GeometricObject* raw);
  
  // Snapping (could be moved to a SnapManager class later)
  std::optional<Point_2> findSnapPoint(const Point_2 &currentCgalPos, float snapRadiusWorld);

  // Intersections
  void calculateIntersectionsOnClick(const Point_2 &clickPos_cgal);  // Example internal action
  void findAllIntersections();  // Recalculate all relevant intersections

  // View manipulation
  void zoomView(float factor, const sf::Vector2i &mousePixelPos);
  void panView(const sf::Vector2f &delta_view);
  void resetView();

  // Corrected public block for setCurrentTool and other methods
 public:
  void toggleGrid();
  void setCurrentTool(ObjectType newTool);  // Switches tool and updates GUI buttons
  const sf::View &getDrawingView() const;
  // Point_2 sfmlToCGAL(const sf::Vector2f &sfmlPos); // This is a duplicate of
  // toCGALPoint, consider removing one
  void createCircle(const Point_2 &center, double radius, const sf::Color &color);
  void handleResize(unsigned int width, unsigned int height);
  void replacePoint(std::shared_ptr<Point> oldPt, std::shared_ptr<Point> newPt);

  // Alternative panning control methods that work with existing structure
  void startPanning(const sf::Vector2f &mousePos);
  void stopPanning();

  // Helper method to check if any object is selected
  bool hasSelectedObject() const;

  // Window accessor (using the actual variable name)
  sf::RenderWindow &getWindow() { return window; }

  // Add or update these methods in the GeometryEditor class to properly handle
  // intersection mode
  bool isInIntersectionMode() const { return m_isInIntersectionMode; }
  void activateIntersectionMode() {
    m_isInIntersectionMode = true;
    m_firstIntersectionObject = nullptr;
    m_currentToolType = ObjectType::Intersection;
    std::cout << "Intersection mode activated." << std::endl;
  }
  void cancelCurrentOperation();
  void exitIntersectionMode() {
    m_isInIntersectionMode = false;
    m_firstIntersectionObject = nullptr;
    std::cout << "Intersection mode deactivated." << std::endl;
  }

  void handleIntersectionSelection(GeometricObject *obj) {
    if (!m_isInIntersectionMode) return;

    if (m_firstIntersectionObject == nullptr) {
      // First object selected
      m_firstIntersectionObject = obj;
      std::cout << "First object selected for intersection. Now select a "
                   "second object."
                << std::endl;
    } else if (m_firstIntersectionObject == obj) {
      // Clicking the same object twice - do nothing
      std::cout << "Same object selected twice. Please select a different "
                   "object for intersection."
                << std::endl;
    } else {
      // Second object selected - calculate intersection
      calculateIntersectionBetween(m_firstIntersectionObject, obj);
      m_firstIntersectionObject = nullptr;
    }
  }

  // Replace safelyCalculateIntersection with a properly defined function
  void calculateIntersectionBetween(GeometricObject *obj1, GeometricObject *obj2);

  // Utility functions for creating intersection points
  void createIntersectionPoint(Line *line1, Line *line2);
  void createIntersectionPoint(Line *line, Circle *circle);
  void createIntersectionPoint(Circle *circle1, Circle *circle2);

  // Add this new public method declaration
  void resetApplicationState();

  bool debugMode = false;  // Set to false to reduce console spam
  void toggleDebugMode() {
    debugMode = !debugMode;
    std::cout << "Debug mode " << (debugMode ? "enabled" : "disabled") << std::endl;
  }

  void debugObjectDetection(const sf::Vector2f &position, float tolerance) {
    std::cout << "DEBUG Object Detection at (" << position.x << ", " << position.y
              << "):" << std::endl;

    // Check points
    std::cout << "  Points: " << points.size() << std::endl;
    for (size_t i = 0; i < points.size(); ++i) {
      if (points[i] && points[i]->contains(position, tolerance))
        std::cout << "    Point #" << i << " CONTAINS point" << std::endl;
    }

    // Check lines
    std::cout << "  Lines: " << lines.size() << std::endl;
    for (size_t i = 0; i < lines.size(); ++i) {
      if (lines[i] && lines[i]->contains(position, tolerance))
        std::cout << "    Line #" << i << " CONTAINS point" << std::endl;
    }

    // Check circles
    std::cout << "  Circles: " << circles.size() << std::endl;
    for (size_t i = 0; i < circles.size(); ++i) {
      if (circles[i] && circles[i]->contains(position, tolerance))
        std::cout << "    Circle #" << i << " CONTAINS point" << std::endl;
    }

    // Check ObjectPoints
    std::cout << "  ObjectPoints: " << ObjectPoints.size() << std::endl;
    for (size_t i = 0; i < ObjectPoints.size(); ++i) {
      if (ObjectPoints[i] && ObjectPoints[i]->contains(position, tolerance))
        std::cout << "    ObjectPoint #" << i << " CONTAINS point" << std::endl;
    }
  }

  bool m_isInIntersectionMode = false;
  GeometricObject *m_firstIntersectionObject = nullptr;

  // Helper struct for storing a reference to an object's edge (or the object itself)
  struct EdgeReference {
      std::weak_ptr<GeometricObject> object;
      int edgeIndex = -1; // -1 for the object itself (if it's a line), >= 0 for shape edges
      
      void reset() { object.reset(); edgeIndex = -1; }
      std::shared_ptr<GeometricObject> lock() const { return object.lock(); }
      bool isValid() const { return !object.expired(); }
  };

  // Add these for parallel/perpendicular tool state
  EdgeReference m_parallelReference;
  bool m_isPlacingParallel = false;
  std::shared_ptr<Line> m_parallelPreviewLine;  // For visual feedback during placement
  Vector_2 m_parallelReferenceDirection;
  EdgeReference m_perpendicularReference;
  bool m_isPlacingPerpendicular = false;
  std::shared_ptr<Line> m_perpendicularPreviewLine;  // For visual feedback
  Vector_2 m_perpendicularReferenceDirection;

 public:
  // Current visual properties state
  sf::Color currentColor = sf::Color::Black;
  sf::Color backgroundColor = Constants::BACKGROUND_COLOR;

  // Axes handling
  std::shared_ptr<Line> xAxis;
  std::shared_ptr<Line> yAxis;

 public:
  void toggleAxes();
  bool areAxesVisible() const;
  void setAxesVisible(bool visible);
  Line* getXAxis() const { return xAxis.get(); }
  Line* getYAxis() const { return yAxis.get(); }
  const std::shared_ptr<Line>& getXAxisShared() const { return xAxis; }
  const std::shared_ptr<Line>& getYAxisShared() const { return yAxis; }
  bool isGridVisible() const { return grid.isVisible(); }
  float getCurrentGridSpacing() const { return grid.getCurrentGridSpacing(); }
  float getCurrentPointSize() const { return currentPointSize; }
 private:

  sf::Vector2u m_lastWindowSize;

  // Methods that should remain private
  // The following are examples of members that were explicitly removed
  // or replaced, kept here as comments for historical context if useful
  // during development: std::vector<sf::Vector2f> linePoints; //
  // Replaced by Line objects sf::Vector2f dragOffset; // Replaced by
  // calculating offset dynamically or dragStart_sfml Point
  // *selectedPoint; // Replaced by selectedObject and type checking Line
  // *selectedLine; // Replaced by selectedObject bool
  // isDraggingEndpoint; // Covered by dragMode or m_selectedEndpoint
  // Circle *selectedCircle; // Replaced by selectedObject bool
  // isDraggingCircle; // Covered by dragMode bool isResizingCircle; //
  // Covered by dragMode ObjectPoint *selectedObjectPoint; // Replaced by
  // selectedObject
  void loadFont();
  void setupDefaultViews();
  // Error handling methods
  void attemptToRepairGeometricObjects();
  void emergencyRender();

  // Variables for intersection mode
};
