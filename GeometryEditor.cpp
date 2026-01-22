#pragma message("--- In GeometryEditor.cpp: Checking CGAL preprocessor flags ---")

#ifdef CGAL_HAS_THREADS
#pragma message("GeometryEditor.cpp: CGAL_HAS_THREADS is DEFINED before any action.")
#else
#pragma message("GeometryEditor.cpp: CGAL_HAS_THREADS is NOT DEFINED before any action.")
#endif

// CGAL_USE_SSE2 check removed

// Attempt to set the desired state
#define CGAL_HAS_THREADS 1
// #undef CGAL_USE_SSE2 removed
#pragma message("GeometryEditor.cpp: Action: Defined CGAL_HAS_THREADS. CGAL_USE_SSE2 not modified.")

// Verify after action
#ifdef CGAL_HAS_THREADS
#pragma message("GeometryEditor.cpp: CGAL_HAS_THREADS is DEFINED after action.")
#else
#pragma message("GeometryEditor.cpp: CGAL_HAS_THREADS is NOT DEFINED after action. (Problem!)")
#endif

// CGAL_USE_SSE2 check pragmas removed

#pragma message("--- End of preprocessor checks in GeometryEditor.cpp ---")

// Attempt to fix char_traits issues by including these first globally.
#include <string>  // Ensure standard string is included very early

#include "CharTraitsFix.h"  // For char_traits<unsigned int> specialization

// Include this if you have a custom char_traits
#include "DynamicIntersection.h"
#include "GeometryEditor.h"
#include "HandleEvents.h"

// Add this include to make findIntersection available
#include <CGAL/config.h>

#include <SFML/Graphics.hpp>
#include <algorithm>  // For std::find_if, std::remove_if
#include <cmath>      // For std::sqrt, std::abs, std::round
#include <iostream>   // For debugging
#include <memory>
#include <optional>
#include <variant>  // Required for std::get_if
#include <vector>
#include <type_traits>

#include "Intersection.h"
// CGAL includes (ensure these are appropriate for your Types.h)
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>  // If Kernel is EPECK
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>

#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/get.hpp>
#include <boost/variant/variant.hpp>
// Project-specific includes
#include "Circle.h"
#include "CommandManager.h"
#include "Constants.h"
#include "GUI.h"
#include "Grid.h"
#include "Line.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "Types.h"  // For Point_2, Line_2 etc.

// #include "Command.h" // If base Command class is needed directly
// #include "GenericDeleteCommand.h" // If used directly here
// #include "ObjectType.h" // If ObjectType enum is defined here and not
// ForwardDeclarations #include "ProjectionUtils.h" // If used
float Constants::CURRENT_ZOOM = 1.0f;
// Constructor
GeometryEditor::GeometryEditor()
    : settings(0, 0, 0),
      window(sf::VideoMode(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT), "Geometry Editor",
             sf::Style::Default, settings),
      drawingView(sf::FloatRect(0.f, 0.f, static_cast<float>(Constants::WINDOW_WIDTH),
                                static_cast<float>(Constants::WINDOW_HEIGHT))),
      guiView(sf::FloatRect(0.f, 0.f, static_cast<float>(Constants::WINDOW_WIDTH),
                            static_cast<float>(Constants::WINDOW_HEIGHT))),
      gui(),
      grid(Constants::GRID_SIZE, true),
      commandManager(),
      objectIdCounter(0),
      m_colorPicker(sf::Vector2f(10, 100))
// Other members like vectors, bools, pointers are default-initialized or
// initialized in-class (in .h)
{
  loadFont();
  setupDefaultViews();
  window.setVerticalSyncEnabled(true);
  window.setFramerateLimit(120);

  // Initialize hoverMessageText
  if (Button::getFontLoaded()) {
    hoverMessageText.setFont(Button::getFont());
    hoverMessageText.setCharacterSize(Constants::GRID_LABEL_FONT_SIZE);
    hoverMessageText.setFillColor(Constants::AXIS_LABEL_COLOR);
  } else {
    std::cerr << "Error: Font not loaded when initializing GeometryEditor's "
                 "hoverMessageText."
              << std::endl;
    // hoverMessageText will use SFML's default font, which might not be what
    // you want, or it might behave unexpectedly if drawn. Consider not setting
    // character size or color if font is not loaded, or using a very basic
    // fallback if SFML allows.
  }

  // gui.setView(guiView); // Or gui.updateView(guiView);

  std::cout << "GeometryEditor initialized." << std::endl;
}

GeometryEditor::~GeometryEditor() {
  try {
    // First, add a debug message
    std::cout << "GeometryEditor shutting down, cleaning up objects in safe order..." << std::endl;
    clearAllDeletionTracking();
    // 1. First clear selection/hover state to avoid any dangling pointers
    // during cleanup
    selectedObject = nullptr;
    hoveredObject = nullptr;
    lineCreationPoint1 = nullptr;

    // 2. Clear any other raw pointers or references
    previewCircle.reset();

    // 3. Clear ObjectPoints first since they depend on other objects
    std::cout << "Clearing ObjectPoints..." << std::endl;
    ObjectPoints.clear();

    // 4. Clear Lines next since they depend on Points
    std::cout << "Clearing Lines..." << std::endl;
    lines.clear();

    // 5. Clear Circles
    std::cout << "Clearing Circles..." << std::endl;
    circles.clear();

    // 6. Clear Triangles
    std::cout << "Clearing Triangles..." << std::endl;
    triangles.clear();
    
    // 7. Clear Rectangles, Polygons, RegularPolygons
    std::cout << "Clearing Shapes..." << std::endl;
    rectangles.clear();
    polygons.clear();
    regularPolygons.clear();

    // 8. Clear Points last
    std::cout << "Clearing Points..." << std::endl;
    points.clear();

    std::cout << "GeometryEditor shutdown complete." << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Exception in GeometryEditor destructor: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception in GeometryEditor destructor" << std::endl;
  }
}
std::shared_ptr<Circle> GeometryEditor::getCircleSharedPtr(Circle *rawPtr) {
  if (!rawPtr) {
    return nullptr;
  }

  // Search through the circles vector to find the matching pointer
  for (auto &circlePtr : circles) {
    if (circlePtr.get() == rawPtr) {
      // Create a shared_ptr from the raw pointer, but don't let it manage the
      // lifetime
      return circlePtr;
    }
  }

  std::cerr << "GeometryEditor::getCircleSharedPtr: Circle not found in "
               "circles vector"
            << std::endl;
  return nullptr;
}
std::shared_ptr<Line> GeometryEditor::getLineSharedPtr(Line *rawPtr) {
  if (!rawPtr) {
    return nullptr;
  }

  // Search through the lines vector to find the matching shared_ptr
  for (auto &linePtr : lines) {
    if (linePtr.get() == rawPtr) {
      // Return the actual shared_ptr, not a fake one
      return linePtr;
    }
  }

  std::cerr << "GeometryEditor::getLineSharedPtr: Line not found in lines vector" << std::endl;
  return nullptr;
}

void GeometryEditor::run() {
  sf::Clock dtClock;

  while (window.isOpen()) {
    float dt = dtClock.restart().asSeconds();

    // Process events
    try {
      handleEvents(*this);
    } catch (const CGAL::Uncertain_conversion_exception &e) {
      std::cerr << "GeometryEditor::run: Caught Uncertain_conversion_exception "
                   "during event handling: "
                << e.what() << std::endl;
      // Consider setting a flag to inform the user or take appropriate action
      // For now, we'll continue execution but log the error
    } catch (const std::exception &e) {
      std::cerr << "GeometryEditor::run: Caught exception during event handling: " << e.what()
                << std::endl;
    } catch (...) {
      std::cerr << "GeometryEditor::run: Caught unknown exception during event "
                   "handling"
                << std::endl;
    }

    // Update
    try {
      update(sf::seconds(dt));
    } catch (const CGAL::Uncertain_conversion_exception &e) {
      std::cerr << "GeometryEditor::run: Caught Uncertain_conversion_exception "
                   "during update: "
                << e.what() << std::endl;
      // Try to repair any problematic data
      attemptToRepairGeometricObjects();
    } catch (const std::exception &e) {
      std::cerr << "GeometryEditor::run: Caught exception during update: " << e.what() << std::endl;
      attemptToRepairGeometricObjects();
    } catch (...) {
      std::cerr << "GeometryEditor::run: Caught unknown exception during update" << std::endl;
      attemptToRepairGeometricObjects();
    }

    // Render
    try {
      render();
    } catch (const CGAL::Uncertain_conversion_exception &e) {
      std::cerr << "GeometryEditor::run: Caught Uncertain_conversion_exception "
                   "during rendering: "
                << e.what() << std::endl;
      // For render exceptions, we might want to do a simpler backup render
      emergencyRender();
    } catch (const std::exception &e) {
      std::cerr << "GeometryEditor::run: Caught exception during rendering: " << e.what()
                << std::endl;
      emergencyRender();
    } catch (...) {
      std::cerr << "GeometryEditor::run: Caught unknown exception during rendering" << std::endl;
      emergencyRender();
    }
  }
}

void GeometryEditor::attemptToRepairGeometricObjects() {
  // Try to fix data in all important geometric objects
  for (auto &line : lines) {
    if (line) {
      line->attemptDataRepair();
    }
  }
  // Similar for other collections (points, circles, etc.)
}

void GeometryEditor::emergencyRender() {
  try {
    // Reset views to default state
    sf::View defaultView(sf::FloatRect(0.f, 0.f, static_cast<float>(window.getSize().x),
                                       static_cast<float>(window.getSize().y)));

    // Clear with a recognizable color to indicate emergency mode
    window.clear(sf::Color(255, 240, 240));  // Light red background

    // Use default view for safety
    window.setView(defaultView);

    // Try to display some basic text
    sf::Text errorText;
    bool fontLoaded = false;

    // Try to use Button's font if available
    if (Button::getFontLoaded()) {
      errorText.setFont(Button::getFont());
      fontLoaded = true;
    } else {
      // Try to load a system font as backup
      sf::Font emergencyFont;
      if (emergencyFont.loadFromFile(Constants::DEFAULT_FONT_PATH)) {
        errorText.setFont(emergencyFont);
        fontLoaded = true;
      }
    }

    if (fontLoaded) {
      errorText.setString("Rendering error - Press ESC to reset views");
      errorText.setCharacterSize(24);
      errorText.setFillColor(sf::Color::Red);
      errorText.setPosition(20.f, 20.f);
      window.draw(errorText);

      // Try to draw some basic UI buttons as well
      sf::RectangleShape resetButton(sf::Vector2f(200.f, 50.f));
      resetButton.setFillColor(sf::Color(100, 100, 255));
      resetButton.setPosition(20.f, 60.f);
      window.draw(resetButton);

      sf::Text buttonText;
      buttonText.setFont(*errorText.getFont());
      buttonText.setString("Reset View");
      buttonText.setCharacterSize(18);
      buttonText.setFillColor(sf::Color::White);
      buttonText.setPosition(50.f, 75.f);
      window.draw(buttonText);
    }

    window.display();

  } catch (...) {
    // Last resort if even emergency rendering fails
    window.clear(sf::Color::White);
    window.display();
  }
}
void GeometryEditor::updateConstraintsOnly() {
  QUICK_PROFILE("GeometryEditor::updateConstraintsOnly");

  try {
    // Update constraints for all lines without full geometry rebuild
    for (auto &linePtr : lines) {
      if (linePtr && linePtr->isValid()) {
        QUICK_PROFILE("Line::maintainConstraints");

        // Use your existing method: getIsUnderDirectManipulation() instead of
        // isUnderDirectManipulation()
        if (!linePtr->getIsUnderDirectManipulation()) {
          linePtr->maintainConstraints();
        }
      }
    }

    // Update ObjectPoints to stay on their host objects
    for (auto &objPointPtr : ObjectPoints) {
      if (objPointPtr && objPointPtr->isValid()) {
        QUICK_PROFILE("ObjectPoint::updatePositionFromHost");

        // Use your existing method: updatePositionFromHost() instead of
        // updatePosition()
        objPointPtr->updatePositionFromHost();
      }
    }

    // Note: We don't call updateSFMLShape() here to avoid expensive visual
    // updates Visual updates happen automatically when positions change via
    // setCGALPosition

  } catch (const std::exception &e) {
    std::cerr << "Error in updateConstraintsOnly: " << e.what() << std::endl;
  }
}
// Implementation of the render method
void GeometryEditor::render() {
  try {
    // Clear the window with the background color
    window.clear(Constants::BACKGROUND_COLOR);

    // Only log view properties if debugging is enabled
    if (Constants::DEBUG_GEOMETRY_UPDATES) {
      std::cout << "Drawing View - Size: (" << drawingView.getSize().x << ", "
                << drawingView.getSize().y << "), Center: (" << drawingView.getCenter().x << ", "
                << drawingView.getCenter().y << ")" << std::endl;
      std::cout << "GUI View - Size: (" << guiView.getSize().x << ", " << guiView.getSize().y
                << "), Center: (" << guiView.getCenter().x << ", " << guiView.getCenter().y << ")"
                << std::endl;
    }

    // Get zoom level from view (for safety checks)
    float zoomLevel = drawingView.getSize().y / static_cast<float>(Constants::WINDOW_HEIGHT);

    // CRITICAL FIX: Reset the views to their default state if they seem
    // corrupted
    if (zoomLevel <= 1e-10f || zoomLevel >= 1e10f || std::isnan(zoomLevel) ||
        std::isinf(zoomLevel)) {
      std::cerr << "Error: View appears corrupted with zoom level " << zoomLevel
                << ". Resetting views to default." << std::endl;

      // Reset drawing view
      drawingView = sf::View(sf::FloatRect(0.f, 0.f, static_cast<float>(window.getSize().x),
                                           static_cast<float>(window.getSize().y)));

      // Reset GUI view
      guiView = sf::View(sf::FloatRect(0.f, 0.f, static_cast<float>(window.getSize().x),
                                       static_cast<float>(window.getSize().y)));

      // Update grid for new view
      grid.update(drawingView, window.getSize());

      // Recalculate zoom level
      zoomLevel = drawingView.getSize().y / static_cast<float>(Constants::WINDOW_HEIGHT);
      std::cout << "Views reset. New zoom level: " << zoomLevel << std::endl;
    }

    // --- ADD THIS ---
    Constants::CURRENT_ZOOM = zoomLevel;

    // Draw the grid with correct views
    if (grid.isVisible()) {
      window.setView(drawingView);
      grid.draw(window, drawingView, guiView);
    }

    // Draw geometric objects with the drawing view
    window.setView(drawingView);

    // Draw all geometric objects with safety checks
    window.setView(drawingView);

    // points
    for (auto &pt : points) {
      if (pt && pt->isValid()) pt->draw(window);
    }
    // lines
    for (auto &ln : lines) {
      if (ln && ln->isValid()) ln->draw(window);
    }
    // circles
    for (auto &ci : circles) {
      if (ci && ci->isValid()) ci->draw(window);
    }
    // rectangles
    for (auto &rc : rectangles) {
      if (rc && rc->isValid()) rc->draw(window);
    }
    // polygons
    for (auto &pg : polygons) {
      if (pg && pg->isValid()) pg->draw(window);
    }
    // regular polygons
    for (auto &rp : regularPolygons) {
      if (rp && rp->isValid()) rp->draw(window);
    }
    // triangles
    for (auto &tr : triangles) {
      if (tr && tr->isValid()) tr->draw(window);
    }
    // object‐points
    for (auto &op : ObjectPoints) {
      if (op && op->isValid()) op->draw(window);
    }

    // --- Preview lines ---
    // Draw existing preview Line objects without triggering heavy updates
    if (m_parallelPreviewLine) {
      m_parallelPreviewLine->draw(window);
    }
    if (m_perpendicularPreviewLine) {
      m_perpendicularPreviewLine->draw(window);
    }
    // Draw lightweight overlay that follows the mouse in real-time
    if (hasPreviewLineOverlay) {
      window.draw(previewLineOverlay);
    }

    // selection box
    if (isDrawingSelectionBox) window.draw(selectionBoxShape);

    // preview circle
    if (isCreatingCircle && previewCircle && previewCircle->isValid()) previewCircle->draw(window);
    
    // preview rectangle
    if (isCreatingRectangle && previewRectangle && previewRectangle->isValid())
      previewRectangle->draw(window);
    
    // preview rotatable rectangle
    if (isCreatingRotatableRectangle && previewRectangle && previewRectangle->isValid())
      previewRectangle->draw(window);
    
    // preview polygon
    if (isCreatingPolygon && previewPolygon && previewPolygon->isValid())
      previewPolygon->draw(window);
    
    // preview regular polygon
    if (isCreatingRegularPolygon && previewRegularPolygon && previewRegularPolygon->isValid())
      previewRegularPolygon->draw(window);

    // preview triangle
    if (isCreatingTriangle && previewTriangle && previewTriangle->isValid())
      previewTriangle->draw(window);

    // hover message
    if (showHoverMessage) {
      window.setView(guiView);
      window.draw(hoverMessageText);
    }

    // finally GUI
    window.setView(guiView);
    gui.draw(window, drawingView, *this);

    window.display();

  } catch (const std::exception &e) {
    std::cerr << "Critical error in render(): " << e.what() << std::endl;
    emergencyRender();
  } catch (...) {
    std::cerr << "Unknown critical error in render()" << std::endl;
    emergencyRender();
  }
}
GeometricObject *GeometryEditor::lookForObjectAt(const sf::Vector2f &worldPos_sfml, float tolerance,
                                                 const std::vector<ObjectType> &allowedTypes) {
  bool checkAll = allowedTypes.empty();
  auto typeAllowed = [&](ObjectType type) {
    return checkAll ||
           (std::find(allowedTypes.begin(), allowedTypes.end(), type) != allowedTypes.end());
  };

  // Priority: ObjectPoints, then free Points, then Line endpoints (handled by
  // line contains), then Lines, then Circles.

  // 1. Check ObjectPoints
  if (typeAllowed(ObjectType::ObjectPoint)) {
    for (auto it = ObjectPoints.rbegin(); it != ObjectPoints.rend();
         ++it) {  // Iterate in reverse for top-most
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  // 2. Check free Points
  if (typeAllowed(ObjectType::Point)) {
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  // 2.5 Check Shapes (Triangles, RegularPolygons, Polygons, Rectangles)
  // Checked in reverse render order (Top to Bottom)
  
  if (typeAllowed(ObjectType::Triangle)) {
    for (auto it = triangles.rbegin(); it != triangles.rend(); ++it) {
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  if (typeAllowed(ObjectType::RegularPolygon)) {
    for (auto it = regularPolygons.rbegin(); it != regularPolygons.rend(); ++it) {
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  if (typeAllowed(ObjectType::Polygon)) {
    for (auto it = polygons.rbegin(); it != polygons.rend(); ++it) {
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  if (typeAllowed(ObjectType::Rectangle) || typeAllowed(ObjectType::RectangleRotatable)) {
    for (auto it = rectangles.rbegin(); it != rectangles.rend(); ++it) {
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  // 3. Check Lines (their 'contains' method should also handle endpoints
  // implicitly if designed that way,
  //    or you might need specific endpoint checks if 'contains' only checks the
  //    line body) If Line::contains doesn't check endpoints well enough, you
  //    might need a separate loop here to check distance to line endpoints if
  //    ObjectType::Point is also allowed.
  if (typeAllowed(ObjectType::Line) || typeAllowed(ObjectType::LineSegment)) {
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        // Ensure the type matches if both Line and LineSegment are distinct and
        // specified
        if (typeAllowed((*it)->getType())) {
          return it->get();
        }
      }
    }
  }

  // 4. Check Circles
  if (typeAllowed(ObjectType::Circle)) {
    for (auto it = circles.rbegin(); it != circles.rend(); ++it) {
      if (*it && (*it)->isValid() && (*it)->contains(worldPos_sfml, tolerance)) {
        // Check for center point interaction first if that's a distinct
        // interaction
        if ((*it)->isCenterPointHovered(worldPos_sfml, tolerance)) {
          // You might want to return the circle or a special marker for its
          // center For now, just returning the circle.
          return it->get();
        }
        // Then check circumference
        if ((*it)->isCircumferenceHovered(worldPos_sfml, tolerance)) {
          return it->get();
        }
        // Fallback to general contains if the specific hovers aren't met but
        // general contains is
        if ((*it)->contains(worldPos_sfml, tolerance) &&
            !(*it)->isCenterPointHovered(worldPos_sfml, tolerance) &&
            !(*it)->isCircumferenceHovered(worldPos_sfml, tolerance)) {
          return it->get();  // e.g. if contains means filled circle
        }
      }
    }
  }

  return nullptr;  // No object found
}

// Implementation for the std::initializer_list version (delegates to the vector
// version)
GeometricObject *GeometryEditor::lookForObjectAt(
    const sf::Vector2f &worldPos_sfml, float tolerance,
    std::initializer_list<ObjectType> allowedTypes_il) {
  std::vector<ObjectType> allowedTypes_vec(allowedTypes_il.begin(), allowedTypes_il.end());
  return lookForObjectAt(worldPos_sfml, tolerance, allowedTypes_vec);
}

// --- Public Interface Methods ---

void GeometryEditor::toggleGrid() {
  grid.toggleVisibility();
  std::cout << "Grid visibility toggled to: " << (grid.isVisible() ? "ON" : "OFF") << std::endl;
}

void GeometryEditor::setCurrentTool(ObjectType newTool) {
  // Debug output to track tool changes
  std::cout << "setCurrentTool called: Changing from " << static_cast<int>(m_currentToolType)
            << " to " << static_cast<int>(newTool) << std::endl;

  // If the new tool is the same as the current active creation tool,
  // and it's not 'None' (which would mean trying to toggle off the 'Move' tool,
  // not desired), then clicking it again means "deactivate this tool" and
  // revert to None/Move.
  if (newTool != ObjectType::None && newTool == m_currentToolType) {
    m_currentToolType = ObjectType::None;  // Deactivate current tool, revert to Move/None
    resetCreationStates();                 // Also reset any pending creation state (e.g. first
                                           // line point)
    std::cout << "Deactivating tool, reverting to Move/None" << std::endl;
  } else {
    resetCreationStates();        // Reset any pending creation state from a previous
                                  // tool
    m_currentToolType = newTool;  // Activate the new tool (or switch to it)
    std::cout << "Activating new tool: " << static_cast<int>(newTool) << std::endl;
  }

  std::cout << "Current tool set to: " << static_cast<int>(m_currentToolType) << std::endl;

  // Update GUI button states based on the new m_currentToolType.
  gui.deactivateAllTools();  // Deactivate all general tool buttons first

  // Activate the button corresponding to the now-current tool.
  // If m_currentToolType is None, the "Move" button should be active.
  switch (m_currentToolType) {
    case ObjectType::Point:
      gui.toggleButton("Point", true);
      break;
    case ObjectType::Line:
      gui.toggleButton("Line", true);
      break;
    case ObjectType::LineSegment:
      gui.toggleButton("Segment", true);
      break;
    case ObjectType::Circle:
      gui.toggleButton("Circle", true);
      // If "dragCircle" is a separate button that should also reflect this state:
      // gui.toggleButton("dragCircle", true);
      break;
    case ObjectType::ObjectPoint:
      gui.toggleButton("ObjPoint", true);
      break;
    case ObjectType::Rectangle:
      gui.toggleButton("Rect", true);
      break;
    case ObjectType::RectangleRotatable:
      gui.toggleButton("RotRect", true);
      break;
    case ObjectType::Polygon:
      gui.toggleButton("Polygon", true);
      break;
    case ObjectType::RegularPolygon:
      gui.toggleButton("RegPoly", true);
      break;
    case ObjectType::Triangle:
      gui.toggleButton("Triangle", true);
      break;
    case ObjectType::None:  // This is the "Move" tool state
      gui.toggleButton("Move", true);
      break;
    case ObjectType::Intersection:
      // If "Intersect" is a mode with a button that stays active:
      // gui.toggleButton("Intersect", true);
      // For now, assume Intersection is a one-shot action or doesn't keep a
      // button active, so "Move" becomes the active tool button. If Intersection
      // tool is selected, m_currentToolType will be Intersection. If it's a
      // one-shot action, it should reset m_currentToolType to None itself after
      // execution. For now, if Intersection tool is selected, we assume it might
      // be a mode. If it has a button, it should be toggled here. If not, 'Move'
      // will be default. Let's assume "Intersect" button exists and should be
      // active if this tool is chosen.
      gui.toggleButton("Intersect", true);
      // If "Intersect" has no button or is one-shot, then after its action,
      // setCurrentTool(ObjectType::None) should be called by the action handler.
      break;
    case ObjectType::ParallelLine:
      gui.toggleButton("Parallel", true);
      break;
    case ObjectType::PerpendicularLine:
      gui.toggleButton("Perp", true);
      break;
    default:
      std::cout << "setCurrentTool: Unhandled tool type for GUI "
                << static_cast<int>(m_currentToolType) << ", defaulting to Move." << std::endl;
      gui.toggleButton("Move", true);  // Fallback to Move tool active
      break;
  }
}

const sf::View &GeometryEditor::getDrawingView() const { return drawingView; }

void GeometryEditor::createCircle(const Point_2 &center, double radius, const sf::Color &color) {
  sf::Color selectedColor = getCurrentColor();
  
  // Create center point
  auto centerPoint = std::make_unique<Point>(center, 5.0f, selectedColor);
  Point *centerPtr = centerPoint.get();
  points.push_back(std::move(centerPoint));
  
  // Create circle attached to the center point
  circles.push_back(std::make_shared<Circle>(centerPtr, radius, selectedColor));
  std::cout << "Circle created programmatically." << std::endl;
}

// Implementation of the update method
void GeometryEditor::update(sf::Time deltaTime) {
  // Update GUI
  gui.update(deltaTime);

  // Update preview circle if creating a circle
  if (isCreatingCircle && previewCircle) {
    if (Constants::DEBUG_GEOMETRY_UPDATES) {
      std::cout << "Updating preview circle" << std::endl;
    }
    previewCircle->update();
  }

  // Always update existing intersections to maintain correct positions
  // regardless of whether auto-intersections is enabled
  DynamicIntersection::updateAllIntersections();

  // Update hover message position if needed
  if (showHoverMessage) {
    sf::Vector2f textPos(10.f, Constants::WINDOW_HEIGHT - 30.f);
    hoverMessageText.setPosition(textPos);
  }
}

// Implementation of resetCreationStates
void GeometryEditor::resetCreationStates() {
  // Reset circle creation state
  isCreatingCircle = false;
  previewCircle.reset();

  // Reset line creation state
  if (lineCreationPoint1) {
    lineCreationPoint1->setSelected(false);
    lineCreationPoint1 = nullptr;
  }

  dragMode = DragMode::None;
  isDragging = false;
}

void GeometryEditor::resetParallelLineToolState() {
  m_parallelReferenceLine.reset();
  m_isPlacingParallel = false;
  
  // ✅ FIX: Properly cleanup preview line before reset to prevent crashes
  if (m_parallelPreviewLine) {
    try {
      m_parallelPreviewLine->prepareForDestruction();
    } catch (...) {
      // Ignore errors during preview cleanup
    }
  }
  m_parallelPreviewLine.reset();
  // Potentially update GUI message or clear selection highlights related to
  // this tool
}

void GeometryEditor::resetPerpendicularLineToolState() {
  m_perpendicularReferenceLine.reset();
  m_isPlacingPerpendicular = false;
  
  // ✅ FIX: Properly cleanup preview line before reset to prevent crashes
  if (m_perpendicularPreviewLine) {
    try {
      m_perpendicularPreviewLine->prepareForDestruction();
    } catch (...) {
      // Ignore errors during preview cleanup
    }
  }
  m_perpendicularPreviewLine.reset();
}

// Implementation of findAllIntersections - updated to not create ALL
// intersections
void GeometryEditor::findAllIntersections() {
  try {
    // This function will no longer automatically create all intersections
    // It's now just an informational function that will find and report
    // the number of potential intersections, but won't create them

    int lineLine = 0;
    int lineCircle = 0;
    int circleCircle = 0;

    // Count potential line-line intersections
    for (size_t i = 0; i < lines.size(); ++i) {
      for (size_t j = i + 1; j < lines.size(); ++j) {
        if (!lines[i] || !lines[j]) continue;

        auto optIntersect = findIntersection(lines[i]->getCGALLine(), lines[j]->getCGALLine());
        if (optIntersect) lineLine++;
      }
    }

    // Count potential line-circle intersections
    for (const auto &line : lines) {
      for (const auto &circle : circles) {
        if (!line || !circle) continue;

        auto intersections = findIntersection(line->getCGALLine(), circle->getCGALCircle());
        lineCircle += intersections.size();
      }
    }

    // Count potential circle-circle intersections
    for (size_t i = 0; i < circles.size(); ++i) {
      for (size_t j = i + 1; j < circles.size(); ++j) {
        if (!circles[i] || !circles[j]) continue;

        auto intersections =
            findIntersection(circles[i]->getCGALCircle(), circles[j]->getCGALCircle());
        circleCircle += intersections.size();
      }
    }

    std::cout << "Found potential intersections:\n"
              << "  Line-Line: " << lineLine << "\n"
              << "  Line-Circle: " << lineCircle << "\n"
              << "  Circle-Circle: " << circleCircle << "\n"
              << "Total: " << (lineLine + lineCircle + circleCircle) << std::endl;

    std::cout << "Use the Intersection tool to create specific intersections." << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Exception in findAllIntersections: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception in findAllIntersections" << std::endl;
  }
}

// Implementation of coordinate conversion methods

sf::Vector2f GeometryEditor::toSFMLVector(const Point_2 &cgal_point) const {
  return sf::Vector2f(static_cast<float>(CGAL::to_double(cgal_point.x())),
                      static_cast<float>(CGAL::to_double(cgal_point.y())));
}
Vector_2 GeometryEditor::toCGALVector(const sf::Vector2f &sfmlVector) const {
  return Vector_2(sfmlVector.x, sfmlVector.y);
}
// Implementation of utility methods
float GeometryEditor::getScaledTolerance(const sf::View &currentView) const {
  return Constants::MOUSE_OVER_TOLERANCE *
         (currentView.getSize().x / static_cast<float>(Constants::WINDOW_WIDTH));
}

float GeometryEditor::length(const sf::Vector2f &vec) const {
  return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

// Make sure this is called whenever the view changes
void GeometryEditor::handleResize(unsigned int width, unsigned int height) {
  if (width == 0 || height == 0) return;

  // Update the viewport of both views
  drawingView.setSize(static_cast<float>(width), static_cast<float>(height));
  guiView.setSize(static_cast<float>(width), static_cast<float>(height));

  // Center the drawing view on the new size
  drawingView.setCenter(static_cast<float>(width) / 2.f, static_cast<float>(height) / 2.f);

  // Update the grid when the view changes
  grid.update(drawingView, window.getSize());
}

void GeometryEditor::startPanning(const sf::Vector2f &mousePos) {
  // Set the panning state
  isPanning = true;
  // Store the mouse position
  lastMousePos_sfml = mousePos;

  // Clear any hover state when panning starts
  if (hoveredObject) {
    hoveredObject->setHovered(false);
    hoveredObject = nullptr;
  }
  showHoverMessage = false;
}

void GeometryEditor::panView(const sf::Vector2f &delta_view) {
  // Update the view position based on the delta
  drawingView.move(delta_view);
  window.setView(drawingView);  // Apply the updated view to the window
}

void GeometryEditor::stopPanning() { isPanning = false; }

bool GeometryEditor::hasSelectedObject() const { return selectedObject != nullptr; }

// Implement the intersection functions
void GeometryEditor::createIntersectionPoint(Line *line1, Line *line2) {
  try {
    Line_2 cline1(line1->getStartPoint(), line1->getEndPoint());
    Line_2 cline2(line2->getStartPoint(), line2->getEndPoint());

    auto intersectionExistsNearby = [&](const Point_2 &p) {
      const double dupTol2 = 1e-6;  // world units squared
      for (auto &pt : points) {
        if (pt && pt->isValid()) {
          if (CGAL::to_double(CGAL::squared_distance(pt->getCGALPosition(), p)) < dupTol2) {
            return true;
          }
        }
      }
      return false;
    };

    if (cline1.is_degenerate() || cline2.is_degenerate()) {
      std::cout << "One of the lines is degenerate (a point). No intersection." << std::endl;
      return;
    }

    auto isPointOnSegments = [&](const Point_2& pt) -> bool {
         auto check = [&](Line* l) {
            if (l->getType() == ObjectType::LineSegment) {
                double px = CGAL::to_double(pt.x());
                double py = CGAL::to_double(pt.y());
                Point_2 s = l->getStartPoint();
                Point_2 e = l->getEndPoint();
                double sx = CGAL::to_double(s.x());
                double ex = CGAL::to_double(e.x());
                double sy = CGAL::to_double(s.y());
                double ey = CGAL::to_double(e.y());
                
                double minX = std::min(sx, ex) - 0.001;
                double maxX = std::max(sx, ex) + 0.001;
                double minY = std::min(sy, ey) - 0.001;
                double maxY = std::max(sy, ey) + 0.001;
                
                if (px < minX || px > maxX || py < minY || py > maxY) return false;
            }
            return true;
         };
         return check(line1) && check(line2);
    };

    auto result = CGAL::intersection(cline1, cline2);

    if (result) {
      using ActualVariantType = std::decay_t<decltype(*result)>;

      if constexpr (std::is_same_v<ActualVariantType, std::variant<Point_2, Line_2>>) {
        if (const Point_2 *p = std::get_if<Point_2>(&(*result))) {
          if (isPointOnSegments(*p)) {
              if (!intersectionExistsNearby(*p)) {
                auto newPoint = std::make_shared<Point>(*p, Constants::CURRENT_ZOOM,
                                                        Constants::INTERSECTION_POINT_COLOR);
                newPoint->setIntersectionPoint(true);
                points.push_back(newPoint);
                std::cout << "Line-Line intersection point created." << std::endl;
              } else {
                std::cout << "Intersection point already exists nearby, skipping duplicate." << std::endl;
              }
          } else {
             std::cout << "Intersection outside segment bounds." << std::endl;
          }
        }
      } else if constexpr (std::is_same_v<ActualVariantType, boost::variant<Point_2, Line_2>>) {
        struct BoostIntersectionVisitor : public boost::static_visitor<void> {
          GeometryEditor *editor_ptr;
          std::function<bool(const Point_2&)> checker;
          std::function<bool(const Point_2&)> dupChecker;

          BoostIntersectionVisitor(GeometryEditor *ed, std::function<bool(const Point_2&)> c,
                                   std::function<bool(const Point_2&)> dc) 
            : editor_ptr(ed), checker(c), dupChecker(dc) {}

          void operator()(const Point_2 &p_val) {
            if (checker(p_val)) {
                if (!dupChecker(p_val)) {
                  auto newPoint = std::make_shared<Point>(p_val, Constants::CURRENT_ZOOM,
                                                          Constants::INTERSECTION_POINT_COLOR);
                  newPoint->setIntersectionPoint(true);
                  editor_ptr->points.push_back(newPoint);
                  std::cout << "Line-Line intersection point created." << std::endl;
                } else {
                  std::cout << "Intersection point already exists nearby, skipping duplicate." << std::endl;
                }
            } else {
                std::cout << "Intersection outside segment bounds." << std::endl;
            }
          }

          void operator()(const Line_2 &) {
            std::cout << "Lines are coincident, no unique intersection point." << std::endl;
          }
        };

        BoostIntersectionVisitor visitor(this, isPointOnSegments, intersectionExistsNearby);
        boost::apply_visitor(visitor, *result);
      }
    } else {
      std::cout << "Lines are parallel or do not intersect." << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error creating line-line intersection: " << e.what() << std::endl;
  }
}

void GeometryEditor::handlePointDragging(Point *draggedPoint, const Point_2 &newPosition) {
  // Move the point
  draggedPoint->setCGALPosition(newPosition);

  // Force immediate update of all connected lines and their constraints
  for (auto &line : lines) {
    if (line &&
        (line->getStartPointPtr() == draggedPoint || line->getEndPointPtr() == draggedPoint)) {
      line->setExternallyMovedEndpoint(draggedPoint);
      line->update();

      // Force immediate constraint propagation
      line->notifyConstraintObserversOfChange();
    }
  }
}

void GeometryEditor::createIntersectionPoint(Line *line, Circle *circle) {
  try {
    Line_2 cline(line->getStartPoint(), line->getEndPoint());
    Point_2 center = circle->getCenterPoint();
    double radius = circle->getRadius();

    Point_2 projectedCenter = cline.projection(center);
    double distCenterToLine = std::sqrt(CGAL::to_double(CGAL::squared_distance(center, projectedCenter)));

    if (distCenterToLine > radius) {
      std::cout << "No intersection between line and circle." << std::endl;
      return;
    }
    
    // Helper to check constraints for line segment
    auto isPointOnSegment = [&](const Point_2& pt) {
         if (line->getType() == ObjectType::LineSegment) {
             double px = CGAL::to_double(pt.x());
             double py = CGAL::to_double(pt.y());
             Point_2 s = line->getStartPoint();
             Point_2 e = line->getEndPoint();
             double sx = CGAL::to_double(s.x());
             double ex = CGAL::to_double(e.x());
             double sy = CGAL::to_double(s.y());
             double ey = CGAL::to_double(e.y());
             
             double minX = std::min(sx, ex) - 0.001;
             double maxX = std::max(sx, ex) + 0.001;
             double minY = std::min(sy, ey) - 0.001;
             double maxY = std::max(sy, ey) + 0.001;
             
             if (px < minX || px > maxX || py < minY || py > maxY) return false;
         }
         return true;
    };

    if (std::abs(distCenterToLine - radius) < 0.000001) {
      // Tangent case
      if (isPointOnSegment(projectedCenter)) {
          auto newPoint = std::make_unique<Point>(projectedCenter, 1.0f, Constants::INTERSECTION_POINT_COLOR);
          points.push_back(std::move(newPoint));
          std::cout << "Line tangent to circle. Point created." << std::endl;
      }
      return;
    }

    // Two intersection points
    double halfChordLength = std::sqrt(radius * radius - distCenterToLine * distCenterToLine);
    Vector_2 lineDir = cline.to_vector();
    double lineDirLength = std::sqrt(CGAL::to_double(lineDir.squared_length()));
    Vector_2 unitLineDir = Vector_2(lineDir.x() / lineDirLength, lineDir.y() / lineDirLength);

    Point_2 intersection1(projectedCenter.x() + unitLineDir.x() * halfChordLength,
                          projectedCenter.y() + unitLineDir.y() * halfChordLength);

    Point_2 intersection2(projectedCenter.x() - unitLineDir.x() * halfChordLength,
                          projectedCenter.y() - unitLineDir.y() * halfChordLength);

    if (isPointOnSegment(intersection1)) {
        points.push_back(std::make_unique<Point>(intersection1, 1.0f, Constants::INTERSECTION_POINT_COLOR));
    }
    if (isPointOnSegment(intersection2)) {
        points.push_back(std::make_unique<Point>(intersection2, 1.0f, Constants::INTERSECTION_POINT_COLOR));
    }
    std::cout << "Line-Circle intersection check complete." << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error creating line-circle intersection: " << e.what() << std::endl;
  }
}

void GeometryEditor::calculateIntersectionBetween(GeometricObject *obj1, GeometricObject *obj2) {
  if (!obj1 || !obj2) {
    std::cout << "Cannot calculate intersection: One or both objects are invalid." << std::endl;
    return;
  }

  ObjectType type1 = obj1->getType();
  ObjectType type2 = obj2->getType();

  try {
    // Line-line intersection
    if ((type1 == ObjectType::Line || type1 == ObjectType::LineSegment) &&
        (type2 == ObjectType::Line || type2 == ObjectType::LineSegment)) {
      Line *line1 = static_cast<Line *>(obj1);
      Line *line2 = static_cast<Line *>(obj2);
      createIntersectionPoint(line1, line2);
    }
    // Line-circle intersection
    else if ((type1 == ObjectType::Line || type1 == ObjectType::LineSegment) &&
             type2 == ObjectType::Circle) {
      Line *line = static_cast<Line *>(obj1);
      Circle *circle = static_cast<Circle *>(obj2);
      createIntersectionPoint(line, circle);
    }
    // Circle-line intersection (reverse order of parameters)
    else if (type1 == ObjectType::Circle &&
             (type2 == ObjectType::Line || type2 == ObjectType::LineSegment)) {
      Circle *circle = static_cast<Circle *>(obj1);
      Line *line = static_cast<Line *>(obj2);
      createIntersectionPoint(line, circle);
    }
    // Circle-circle intersection
    else if (type1 == ObjectType::Circle && type2 == ObjectType::Circle) {
      Circle *circle1 = static_cast<Circle *>(obj1);
      Circle *circle2 = static_cast<Circle *>(obj2);
      createIntersectionPoint(circle1, circle2);
    } else {
      std::cout << "Intersection not supported between these object types." << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error calculating intersection: " << e.what() << std::endl;
  }

  // Exit intersection mode after creating intersection
  exitIntersectionMode();
  setCurrentTool(ObjectType::None);
}

void GeometryEditor::createIntersectionPoint(Circle *circle1, Circle *circle2) {
  try {
    // Get the CGAL representation of the circles
    Point_2 center1 = circle1->getCenterPoint();
    double radius1 = circle1->getRadius();

    Point_2 center2 = circle2->getCenterPoint();
    double radius2 = circle2->getRadius();

    // Calculate the distance between centers
    double centerDistance = std::sqrt(CGAL::to_double(CGAL::squared_distance(center1, center2)));

    // Check intersection conditions
    if (centerDistance > radius1 + radius2) {
      // Circles are separate
      std::cout << "No intersection: circles are separate." << std::endl;
      return;
    }

    if (centerDistance < std::abs(radius1 - radius2)) {
      // One circle is inside the other
      std::cout << "No intersection: one circle is inside the other." << std::endl;
      return;
    }

    if (centerDistance < 0.000001) {
      // Concentric circles
      if (std::abs(radius1 - radius2) < 0.000001) {
        std::cout << "Circles are coincident: infinite intersection points." << std::endl;
      } else {
        std::cout << "No intersection: circles are concentric." << std::endl;
      }
      return;
    }

    // At this point we have 1 or 2 intersection points
    // We use the radical line method to find intersection points

    // Calculate the parameters for the radical line equation
    double x1 = CGAL::to_double(center1.x());
    double y1 = CGAL::to_double(center1.y());
    double x2 = CGAL::to_double(center2.x());
    double y2 = CGAL::to_double(center2.y());

    // Calculate direction from center1 to center2
    double dx = x2 - x1;
    double dy = y2 - y1;

    // Normalize the direction vector
    double dirLength = std::sqrt(dx * dx + dy * dy);
    dx /= dirLength;
    dy /= dirLength;

    // Calculate how far from center1 the radical line crosses the line between
    // centers
    double a = (centerDistance * centerDistance + radius1 * radius1 - radius2 * radius2) /
               (2.0 * centerDistance);

    // Calculate the radical center (point on the line between centers)
    double radicalX = x1 + a * dx;
    double radicalY = y1 + a * dy;

    // If this is a tangent case (circles touch at one point)
    if (std::abs(centerDistance - (radius1 + radius2)) < 0.000001 ||
        std::abs(centerDistance - std::abs(radius1 - radius2)) < 0.000001) {
      // Tangent case - one intersection point
      Point_2 intersectionPoint(radicalX, radicalY);
      auto newPoint =
          std::make_unique<Point>(intersectionPoint, 1.0f, Constants::INTERSECTION_POINT_COLOR);
      points.push_back(std::move(newPoint));
      std::cout << "Circles are tangent. One intersection point created." << std::endl;
      return;
    }

    // Two intersection points case
    // Calculate half chord length (distance from radical line to either
    // intersection point)
    double halfChordLength = std::sqrt(radius1 * radius1 - a * a);

    // Calculate perpendicular direction to the line between centers
    double perpDx = -dy;
    double perpDy = dx;

    // Calculate the two intersection points
    Point_2 intersection1(radicalX + perpDx * halfChordLength, radicalY + perpDy * halfChordLength);
    Point_2 intersection2(radicalX - perpDx * halfChordLength, radicalY - perpDy * halfChordLength);

    // Create intersection points
    auto newPoint1 =
        std::make_unique<Point>(intersection1, 1.0f, Constants::INTERSECTION_POINT_COLOR);
    auto newPoint2 =
        std::make_unique<Point>(intersection2, 1.0f, Constants::INTERSECTION_POINT_COLOR);

    points.push_back(std::move(newPoint1));
    points.push_back(std::move(newPoint2));

    std::cout << "Two circle-circle intersection points created." << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error creating circle-circle intersection: " << e.what() << std::endl;
  }
}

// a method to completely reset the application state
//  Add a method to completely reset the application state
void GeometryEditor::resetApplicationState() {
  try {
    std::cout << "Resetting application state..." << std::endl;

    // Clear all selection and interaction states
    selectedObject = nullptr;
    hoveredObject = nullptr;
    lineCreationPoint1 = nullptr;
    isCreatingCircle = false;
    previewCircle.reset();
    isDragging = false;
    isDrawingSelectionBox = false;
    isPanning = false;
    dragMode = DragMode::None;
    m_selectedEndpoint = EndpointSelection::None;
    showHoverMessage = false;

    // Reset tool type
    m_currentToolType = ObjectType::None;
    gui.deactivateAllTools();
    gui.toggleButton("Move", true);

    // Reset views to default
    drawingView = sf::View(sf::FloatRect(0.f, 0.f, static_cast<float>(window.getSize().x),
                                         static_cast<float>(window.getSize().y)));

    guiView = sf::View(sf::FloatRect(0.f, 0.f, static_cast<float>(window.getSize().x),
                                     static_cast<float>(window.getSize().y)));

    // Update grid
    grid.update(drawingView, window.getSize());

    std::cout << "Application state reset complete." << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error during application state reset: " << e.what() << std::endl;
  }
}
void GeometryEditor::setGUIMessage(const std::string &message) {
  if (gui.isInitialized()) {  // Check if GUI is ready
    gui.setMessage(message);
  } else {
    // Log an error or handle gracefully if GUI isn't ready
    std::cerr << "Error: GUI not initialized when trying to set message: " << message << std::endl;
  }
}
std::string GeometryEditor::getCurrentToolName() const {
  switch (m_currentToolType) {
    case ObjectType::None:
      return "Move/Select";
    case ObjectType::Point:
      return "Point";
    case ObjectType::Line:
      return "Line";
    case ObjectType::LineSegment:
      return "Segment";
    case ObjectType::Circle:
      return "Circle";
    case ObjectType::ObjectPoint:
      return "ObjectPoint";
    case ObjectType::Intersection:
      return "Intersection";
    case ObjectType::ParallelLine:
      return "Parallel Line";
    case ObjectType::PerpendicularLine:
      return "Perpendicular Line";
    // Add other cases as needed
    default:
      return "Unknown";
  }
}

void GeometryEditor::cancelCurrentOperation() {
  // Reset any ongoing drawing or creation operations
  if (isCreatingCircle) {
    isCreatingCircle = false;
    previewCircle.reset();
  }

  if (lineCreationPoint1) {
    lineCreationPoint1->setSelected(false);
    lineCreationPoint1 = nullptr;
  }

  // Reset drag state
  dragMode = DragMode::None;
  isDragging = false;
  m_selectedEndpoint = EndpointSelection::None;

  // Reset selection box state
  isDrawingSelectionBox = false;
  selectionBoxShape.setSize(sf::Vector2f(0, 0));

  // Reset tool type if needed
  if (m_currentToolType == ObjectType::Circle || m_currentToolType == ObjectType::Line ||
      m_currentToolType == ObjectType::LineSegment ||
      m_currentToolType == ObjectType::ParallelLine ||
      m_currentToolType == ObjectType::PerpendicularLine) {
    setCurrentTool(ObjectType::None);
  }

  // Exit intersection mode if active
  if (m_isInIntersectionMode) {
    exitIntersectionMode();
  }

  // Reset hover message
  showHoverMessage = false;

  std::cout << "Current operation canceled." << std::endl;
}

// Add these member function implementations

void GeometryEditor::cancelOperation() {
  // Reset all operation-specific state
  isCreatingCircle = false;
  previewCircle.reset();
  lineCreationPoint1 = nullptr;
  dragMode = DragMode::None;
  isDragging = false;
  isDrawingSelectionBox = false;
  m_selectedEndpoint = EndpointSelection::None;

  // Log the cancellation
  std::cout << "Operation canceled" << std::endl;
}

// ...existing code...
void GeometryEditor::deleteSelected() {
  // === MARK-SWEEP DELETION PATTERN ===
  // This prevents use-after-free by:
  // 1. Collecting shared_ptrs (not raw pointers) - keeps objects alive
  // 2. Clearing all references first
  // 3. Removing from containers (but shared_ptrs in our temp collection keep objects alive)
  // 4. Letting temp collections go out of scope to deallocate safely

  std::cout << "=== deleteSelected() STARTING ===" << std::endl;

  // === PHASE 1: MARK - Collect shared_ptrs to selected objects ===
  // Using shared_ptrs keeps objects alive during deletion process
  std::vector<std::shared_ptr<Point>> pointsToDelete;
  std::vector<std::shared_ptr<Line>> linesToDelete;
  std::vector<std::shared_ptr<Circle>> circlesToDelete;
  std::vector<std::shared_ptr<ObjectPoint>> objPointsToDelete;
  std::vector<std::shared_ptr<Rectangle>> rectanglesToDelete;
  std::vector<std::shared_ptr<Polygon>> polygonsToDelete;
  std::vector<std::shared_ptr<RegularPolygon>> regularPolygonsToDelete;
  std::vector<std::shared_ptr<Triangle>> trianglesToDelete;

  // Collect selected objects by their actual shared_ptr (keeps them alive)
  for (auto &ptr : points) {
    if (ptr && ptr->isSelected()) {
      pointsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : lines) {
    if (ptr && ptr->isSelected()) {
      linesToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : circles) {
    if (ptr && ptr->isSelected()) {
      circlesToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : ObjectPoints) {
    if (ptr && ptr->isSelected()) {
      objPointsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : rectangles) {
    if (ptr && ptr->isSelected()) {
      rectanglesToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : polygons) {
    if (ptr && ptr->isSelected()) {
      polygonsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : regularPolygons) {
    if (ptr && ptr->isSelected()) {
      regularPolygonsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : triangles) {
    if (ptr && ptr->isSelected()) {
      trianglesToDelete.push_back(ptr);
    }
  }

  size_t totalToDelete = pointsToDelete.size() + linesToDelete.size() + 
                         circlesToDelete.size() + objPointsToDelete.size() +
                         rectanglesToDelete.size() + polygonsToDelete.size() +
                         regularPolygonsToDelete.size() + trianglesToDelete.size();

  if (totalToDelete == 0) {
    std::cout << "No objects currently selected for deletion." << std::endl;
    return;
  }

  std::cout << "Deleting " << totalToDelete << " selected object(s)." << std::endl;


  // === PHASE 2: CLEAR REFERENCES - Prevent UI from accessing deleted objects ===
  selectedObject = nullptr;
  hoveredObject = nullptr;
  lineCreationPoint1 = nullptr;
  activeVertexShape = nullptr;
  hoveredVertexShape = nullptr;
  activeVertexIndex = -1;
  hoveredVertexIndex = -1;

  // === PHASE 3: COLLECT DEPENDENTS - Find ObjectPoints attached to shapes being deleted ===
  // Shapes own their ObjectPoints, so we must delete them too
  auto collectDependentObjPoints = [this](GeometricObject* host) {
    std::vector<std::shared_ptr<ObjectPoint>> dependents;
    for (auto &objPtr : ObjectPoints) {
      if (objPtr && objPtr->getHostObject() == host) {
        dependents.push_back(objPtr);
      }
    }
    return dependents;
  };

  // Collect all dependent ObjectPoints from shapes being deleted
  for (auto &circlePtr : circlesToDelete) {
    auto deps = collectDependentObjPoints(circlePtr.get());
    for (auto &dep : deps) {
      if (std::find(objPointsToDelete.begin(), objPointsToDelete.end(), dep) == objPointsToDelete.end()) {
        objPointsToDelete.push_back(dep);
        std::cout << "  + Adding dependent ObjectPoint from Circle" << std::endl;
      }
    }
  }
  for (auto &rectPtr : rectanglesToDelete) {
    auto deps = collectDependentObjPoints(rectPtr.get());
    for (auto &dep : deps) {
      if (std::find(objPointsToDelete.begin(), objPointsToDelete.end(), dep) == objPointsToDelete.end()) {
        objPointsToDelete.push_back(dep);
        std::cout << "  + Adding dependent ObjectPoint from Rectangle" << std::endl;
      }
    }
  }
  for (auto &polyPtr : polygonsToDelete) {
    auto deps = collectDependentObjPoints(polyPtr.get());
    for (auto &dep : deps) {
      if (std::find(objPointsToDelete.begin(), objPointsToDelete.end(), dep) == objPointsToDelete.end()) {
        objPointsToDelete.push_back(dep);
        std::cout << "  + Adding dependent ObjectPoint from Polygon" << std::endl;
      }
    }
  }
  for (auto &regPolyPtr : regularPolygonsToDelete) {
    auto deps = collectDependentObjPoints(regPolyPtr.get());
    for (auto &dep : deps) {
      if (std::find(objPointsToDelete.begin(), objPointsToDelete.end(), dep) == objPointsToDelete.end()) {
        objPointsToDelete.push_back(dep);
        std::cout << "  + Adding dependent ObjectPoint from RegularPolygon" << std::endl;
      }
    }
  }
  for (auto &triPtr : trianglesToDelete) {
    auto deps = collectDependentObjPoints(triPtr.get());
    for (auto &dep : deps) {
      if (std::find(objPointsToDelete.begin(), objPointsToDelete.end(), dep) == objPointsToDelete.end()) {
        objPointsToDelete.push_back(dep);
        std::cout << "  + Adding dependent ObjectPoint from Triangle" << std::endl;
      }
    }
  }

  // Collect dependent Lines from Points being deleted
  for (auto &pointPtr : pointsToDelete) {
    for (auto &linePtr : lines) {
      if (linePtr && (linePtr->getStartPointObject() == pointPtr.get() ||
                      linePtr->getEndPointObject() == pointPtr.get())) {
        if (std::find(linesToDelete.begin(), linesToDelete.end(), linePtr) == linesToDelete.end()) {
          linesToDelete.push_back(linePtr);
          std::cout << "  + Adding dependent Line from Point" << std::endl;
        }
      }
    }
  }

  // === PHASE 4: SWEEP - Remove from containers (shared_ptrs in our lists keep objects alive) ===
  
  // Helper to remove shared_ptr from vector
  auto removeFromVector = [](auto& vec, const auto& ptrToRemove) {
    vec.erase(std::remove(vec.begin(), vec.end(), ptrToRemove), vec.end());
  };

  // Remove ObjectPoints first (dependents before masters)
  for (auto &objPtr : objPointsToDelete) {
    std::cout << "Removing ObjectPoint from list..." << std::endl;
    removeFromVector(ObjectPoints, objPtr);
  }

  // Remove Lines
  for (auto &linePtr : linesToDelete) {
    // Prepare line for destruction (clears internal references)
    try {
      linePtr->prepareForDestruction();
    } catch (...) {}
    std::cout << "Removing Line " << linePtr->getID() << " from list..." << std::endl;
    removeFromVector(lines, linePtr);
  }

  // Remove Circles
  for (auto &circlePtr : circlesToDelete) {
    std::cout << "Removing Circle " << circlePtr->getID() << " from list..." << std::endl;
    removeFromVector(circles, circlePtr);
  }

  // Remove Rectangles
  for (auto &rectPtr : rectanglesToDelete) {
    std::cout << "Removing Rectangle from list..." << std::endl;
    removeFromVector(rectangles, rectPtr);
  }

  // Remove Polygons
  for (auto &polyPtr : polygonsToDelete) {
    std::cout << "Removing Polygon from list..." << std::endl;
    removeFromVector(polygons, polyPtr);
  }

  // Remove RegularPolygons
  for (auto &regPolyPtr : regularPolygonsToDelete) {
    std::cout << "Removing RegularPolygon from list..." << std::endl;
    removeFromVector(regularPolygons, regPolyPtr);
  }

  // Remove Triangles
  for (auto &triPtr : trianglesToDelete) {
    std::cout << "Removing Triangle from list..." << std::endl;
    removeFromVector(triangles, triPtr);
  }

  // Remove Points last (may have had dependent lines)
  for (auto &pointPtr : pointsToDelete) {
    std::cout << "Removing Point " << pointPtr->getID() << " from list..." << std::endl;
    removeFromVector(points, pointPtr);
  }

  // === PHASE 5: DEALLOCATE ===
  // All temp vectors go out of scope here, releasing the shared_ptrs.
  // If these were the last references, objects are safely deallocated.
  // No dangling pointers possible - we're not accessing any raw pointers after this.

  std::cout << "=== deleteSelected() COMPLETE ===" << std::endl;
}



// Add getIDIfAvailable() to GeometricObject.h (virtual method) and implement in derived classes
// Example for GeometricObject.h:
// virtual int getIDIfAvailable() const { return -1; } // Default implementation
// Example for Point.h/cpp (if you have an m_id member):
// int Point::getIDIfAvailable() const { return m_id; }
// ...existing code...

// Helper function (you might need to add this to GeometryEditor or a utility
// class)
bool GeometryEditor::objectExistsInAnyList(GeometricObject *obj) {
  if (!obj) return false;
  for (const auto &p : points)
    if (p.get() == obj) return true;
  for (const auto &l : lines)
    if (l.get() == obj) return true;
  for (const auto &c : circles)
    if (c.get() == obj) return true;
  for (const auto &op : ObjectPoints)
    if (op.get() == obj) return true;
  for (const auto &r : rectangles)
    if (r.get() == obj) return true;
  for (const auto &p : polygons)
    if (p.get() == obj) return true;
  for (const auto &rp : regularPolygons)
    if (rp.get() == obj) return true;
  for (const auto &t : triangles)
    if (t.get() == obj) return true;
  return false;
}
void GeometryEditor::updateAllGeometry() {
  // Implementation already present in your code
  // Update all objects to ensure consistency after manipulations
  try {
    // Update points
    for (auto &point : points) {
      if (point && point->isValid()) {
        point->update();
      }
    }

    // Update lines and their constraints
    for (auto &line : lines) {
      if (line && line->isValid()) {
        line->update();
        line->maintainConstraints();
      }
    }

    // Update circles
    for (auto &circle : circles) {
      if (circle && circle->isValid()) {
        circle->update();
      }
    }

    // Update object points
    for (auto &objPoint : ObjectPoints) {
      if (objPoint && objPoint->isValid()) {
        objPoint->update();
      }
    }

    std::cout << "All geometry updated" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error updating geometry: " << e.what() << std::endl;
  }
}

void GeometryEditor::updateScaleFactor() {
  // Implementation already present in your code
  // Update scale factor based on current view
  float viewHeight = drawingView.getSize().y;
  if (viewHeight <= 0) viewHeight = 1.0f;  // Prevent division by zero

  // Calculate scale factor as ratio of current view height to original window
  // height
  float scaleFactor = viewHeight / static_cast<float>(Constants::WINDOW_HEIGHT);

  // Apply scale factor to any size-dependent elements
  // For example, update grid spacing, point sizes, etc.
}
void GeometryEditor::loadFont() {
  if (!defaultFont.loadFromFile(Constants::DEFAULT_FONT_PATH)) {
    std::cerr << "Error: Could not load default font from " << Constants::DEFAULT_FONT_PATH
              << std::endl;
    // Handle error: perhaps use a fallback or throw an exception
    // For now, we'll just print an error. The program might crash if text is
    // used without a loaded font.
  } else {
    std::cout << "Default font loaded successfully from " << Constants::DEFAULT_FONT_PATH
              << std::endl;
  }
  // Initialize any text objects that use this font AFTER it's loaded
  // gui.setFont(defaultFont); // Removed: GUI doesn't support setFont
  hoverText.setFont(defaultFont);
  // ... any other text objects
}

void GeometryEditor::setupDefaultViews() {
  // Setup drawingView
  drawingView.setSize(static_cast<float>(window.getSize().x),
                      static_cast<float>(window.getSize().y));
  drawingView.setCenter(0.f, 0.f);  // Or your preferred default center
  // drawingView.zoom(1.0f); // Set initial zoom if needed

  // Setup guiView (usually maps directly to window coordinates)
  guiView.setSize(static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y));
  guiView.setCenter(static_cast<float>(window.getSize().x) / 2.f,
                    static_cast<float>(window.getSize().y) / 2.f);

  std::cout << "Default views set up." << std::endl;
}
void GeometryEditor::changeSelectedObjectColor(sf::Color newColor) {
  if (!selectedObject) {
    std::cout << "No object selected to change color" << std::endl;
    return;
  }

  switch (selectedObject->getType()) {
    case ObjectType::Point:
      static_cast<Point *>(selectedObject)->setColor(newColor);
      break;
    case ObjectType::Line:
    case ObjectType::LineSegment:
      static_cast<Line *>(selectedObject)->setColor(newColor);
      break;
    case ObjectType::Circle:
      static_cast<Circle *>(selectedObject)->setColor(newColor);
      break;
    default:
      break;
  }

  std::cout << "Changed selected object color to RGB(" << (int)newColor.r << ", " << (int)newColor.g
            << ", " << (int)newColor.b << ")" << std::endl;
}

void GeometryEditor::safeDeleteLine(std::shared_ptr<Line> lineToDelete) {
  if (!lineToDelete) return;
  Line *rawLinePtr = lineToDelete.get();

  if (isObjectBeingDeleted(rawLinePtr)) {
    std::cout << "safeDeleteLine: Line " << lineToDelete->getID()
              << " is already marked for/being deleted. Skipping." << std::endl;
    return;
  }
  markObjectForDeletion(rawLinePtr);

  try {
    std::cout << "safeDeleteLine: Starting deletion of Line " << lineToDelete->getID() << std::endl;

    lineToDelete->setDeferSFMLUpdates(true);

    if (selectedObject == rawLinePtr) {
      selectedObject = nullptr;
    }
    // Use the member 'hoveredObject'
    if (hoveredObject == rawLinePtr) {
      hoveredObject->setHovered(false);  // Ensure its state is false before we nullify our pointer
      hoveredObject = nullptr;
    }

    try {
      lineToDelete->setHovered(false);  // Ensure the object itself knows it's not hovered
    } catch (const std::exception &e) {
      std::cerr << "Error clearing hover during line deletion: " << e.what() << std::endl;
    }

    // Clear tool references
    if (auto refLine = m_parallelReferenceLine.lock()) {
      if (refLine == lineToDelete) {
        resetParallelLineToolState();
      }
    }
    if (auto refLine = m_perpendicularReferenceLine.lock()) {
      if (refLine == lineToDelete) {
        resetPerpendicularLineToolState();
      }
    }
    if (m_parallelPreviewLine == lineToDelete) {
      m_parallelPreviewLine.reset();
    }
    if (m_perpendicularPreviewLine == lineToDelete) {
      m_perpendicularPreviewLine.reset();
    }

    lineToDelete->prepareForDestruction();

    auto it = std::find(lines.begin(), lines.end(), lineToDelete);
    if (it != lines.end()) {
      lines.erase(it);
      std::cout << "Line " << lineToDelete->getID() << " removed from lines vector." << std::endl;
    } else {
      std::cout << "safeDeleteLine: Line " << lineToDelete->getID()
                << " not found in lines vector (possibly already removed)." << std::endl;
    }

    unmarkObjectForDeletion(rawLinePtr);  // Successfully processed and removed
    std::cout << "safeDeleteLine: Successfully deleted Line " << lineToDelete->getID() << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Error during safe line deletion of " << lineToDelete->getID() << ": " << e.what()
              << std::endl;
    unmarkObjectForDeletion(
        rawLinePtr);  // Unmark on error, as processing for this line (by this call) is over.
  }
}
void GeometryEditor::clearHoverReferences(GeometricObject *obj) {
  // This function is called when an object `obj` might be deleted or its state reset.
  // We need to ensure that if `editor.hoveredObject` was pointing to `obj`,
  // it's properly cleared.
  if (hoveredObject == obj) {
    // The object itself should have its m_isHovered flag set to false
    // by its own logic or by the caller of clearHoverReferences.
    // Here, we just clear the editor's pointer to it.
    hoveredObject = nullptr;
    std::cout << "GeometryEditor::clearHoverReferences: Cleared editor.hoveredObject for " << obj
              << std::endl;
  }
  // If HandleEvents.cpp had its own static g_lastHoveredObject, this is where you'd
  // need a way to tell HandleEvents.cpp to clear its static variable.
  // But the better solution is to remove static hover variables from HandleEvents.cpp.
}