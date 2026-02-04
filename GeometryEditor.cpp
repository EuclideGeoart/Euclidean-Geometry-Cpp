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
// This comment is added to force a recompile of this file to resolve a linker error
// caused by removing drawLabel overrides in Triangle and Rectangle.

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
#include "IntersectionSystem.h"
// CGAL includes (ensure these are appropriate for your Types.h)
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>  // If Kernel is EPECK
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>

#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/get.hpp>
#include <boost/variant/variant.hpp>
// Project-specific includes
#include "Circle.h"
#include "CommandSystem.h"
#include "Constants.h"
#include "GUI.h"
#include "Grid.h"
#include "Line.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "Types.h"  // For Point_2, Line_2 etc.
#include "PointUtils.h"
#include "VariantUtils.h"  // For safe_get_point


// #include "Command.h" // If base Command class is needed directly
// #include "GenericDeleteCommand.h" // If used directly here
// #include "ObjectType.h" // If ObjectType enum is defined here and not
// ForwardDeclarations #include "ProjectionUtils.h" // If used
float Constants::CURRENT_ZOOM = 1.0f;
// Constructor
GeometryEditor::GeometryEditor()
    : settings(0, 0, 2),
      window(sf::VideoMode(sf::VideoMode::getDesktopMode().width * 0.7f, sf::VideoMode::getDesktopMode().height * 0.7f), "FluxGeo",
             sf::Style::Default, settings),
      // Initialize drawing view: Width = 30 units (approx), centered at (0,0)
      drawingView(sf::Vector2f(0.f, 0.f), sf::Vector2f(20.0f, 20.0f * (static_cast<float>(sf::VideoMode::getDesktopMode().height) / static_cast<float>(sf::VideoMode::getDesktopMode().width)))),
      // Initialize GUI view: matches window pixels exactly (top-left 0,0)
      guiView(sf::FloatRect(0.f, 0.f, static_cast<float>(sf::VideoMode::getDesktopMode().width * 0.8f),
                            static_cast<float>(sf::VideoMode::getDesktopMode().height * 0.8f))),
      gui(),
      grid(Constants::GRID_SIZE, true),
      commandManager(),
      objectIdCounter(0),
      backgroundColor(Constants::BACKGROUND_COLOR)
// Other members like vectors, bools, pointers are default-initialized or
// initialized in-class (in .h)
{
  loadFont();
  m_lastWindowSize = window.getSize();
  // Initialize Shared Font for Points
  if (Button::getFontLoaded()) {
      Point::commonFont = &Button::getFont();
  }
  
  // Use resetView() to establish the desired default zoom
  window.setVerticalSyncEnabled(true);
  window.setFramerateLimit(120);
  
  // Center the window on the screen
  auto desktop = sf::VideoMode::getDesktopMode();
  auto windowSize = window.getSize();
  window.setPosition(sf::Vector2i(
      (desktop.width - windowSize.x) / 2,
      (desktop.height - windowSize.y) / 2
  ));

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


  // Initialize Axes (as Class Members)
  auto xAxisStart = std::make_shared<Point>(Point_2(-1e8, 0), Constants::CURRENT_ZOOM, Constants::POINT_DEFAULT_COLOR, objectIdCounter++);
  auto xAxisEnd = std::make_shared<Point>(Point_2(1e8, 0), Constants::CURRENT_ZOOM, Constants::POINT_DEFAULT_COLOR, objectIdCounter++);
  xAxis = std::make_shared<Line>(xAxisStart, xAxisEnd, false, Constants::GRID_AXIS_COLOR, objectIdCounter++);
  xAxis->setLocked(true);
  xAxis->setVisible(true);
  xAxis->setThickness(1.0f);
  lines.push_back(xAxis);

  auto yAxisStart = std::make_shared<Point>(Point_2(0, -1e8), Constants::CURRENT_ZOOM, Constants::POINT_DEFAULT_COLOR, objectIdCounter++);
  auto yAxisEnd = std::make_shared<Point>(Point_2(0, 1e8), Constants::CURRENT_ZOOM, Constants::POINT_DEFAULT_COLOR, objectIdCounter++);
  yAxis = std::make_shared<Line>(yAxisStart, yAxisEnd, false, Constants::GRID_AXIS_COLOR, objectIdCounter++);
  yAxis->setLocked(true);
  yAxis->setVisible(true);
  yAxis->setThickness(1.0f);
  lines.push_back(yAxis);

  // Apply default view state (40 world units wide, Y+ up)
  resetView();

  std::cout << "GeometryEditor initialized." << std::endl;
}

std::vector<std::shared_ptr<Point>> GeometryEditor::getAllPoints() const {
  std::vector<std::shared_ptr<Point>> allPoints;
  allPoints.insert(allPoints.end(), points.begin(), points.end());
  allPoints.insert(allPoints.end(), ObjectPoints.begin(), ObjectPoints.end());
  return allPoints;
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

    // 3. Normalize Line shared_ptrs to avoid mixed control blocks
    auto normalizeLinePtr = [&](std::shared_ptr<Line>& sp) {
      if (!sp) return;
      for (auto& ln : lines) {
        if (ln && ln.get() == sp.get()) {
          sp = ln;
          return;
        }
      }
    };

    normalizeLinePtr(xAxis);
    normalizeLinePtr(yAxis);
    normalizeLinePtr(m_parallelPreviewLine);
    normalizeLinePtr(m_perpendicularPreviewLine);
    normalizeLinePtr(perpBisectorLineRef);
    normalizeLinePtr(angleBisectorLine1);
    normalizeLinePtr(angleBisectorLine2);
    normalizeLinePtr(angleLine1);
    normalizeLinePtr(angleLine2);
    normalizeLinePtr(m_hoveredIntersectionLine1);
    normalizeLinePtr(m_hoveredIntersectionLine2);
    normalizeLinePtr(m_hoveredLine);

    // 4. Clear ObjectPoints first since they depend on other objects
    std::cout << "Clearing ObjectPoints..." << std::endl;
    ObjectPoints.clear();

    // 5. Clear Lines next since they depend on Points
    std::cout << "Clearing Lines..." << std::endl;
    for (auto& ln : lines) {
      if (ln) {
        ln->prepareForDestruction();
      }
    }
    lines.clear();

    // 6. Clear Circles
    std::cout << "Clearing Circles..." << std::endl;
    circles.clear();

    // 7. Clear Triangles
    std::cout << "Clearing Triangles..." << std::endl;
    triangles.clear();
    
    // 8. Clear Rectangles, Polygons, RegularPolygons
    std::cout << "Clearing Shapes..." << std::endl;
    rectangles.clear();
    polygons.clear();
    regularPolygons.clear();

    // 9. Clear Points last
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

// Helper to find shared_ptr from raw pointer across all object containers
std::shared_ptr<GeometricObject> GeometryEditor::findSharedPtr(GeometricObject* raw) {
  if (!raw) return nullptr;
  
  // Lambda to check a container
  auto check = [&](auto& container) -> std::shared_ptr<GeometricObject> {
    for (auto& ptr : container) {
      if (ptr.get() == raw) return ptr;
    }
    return nullptr;
  };
  
  // Check all containers
  if (auto p = check(rectangles)) return p;
  if (auto p = check(polygons)) return p;
  if (auto p = check(regularPolygons)) return p;
  if (auto p = check(triangles)) return p;
  if (auto p = check(circles)) return p;
  if (auto p = check(lines)) return p;
  if (auto p = check(points)) return p;
  
  // Check ObjectPoints separately (they're shared_ptr<ObjectPoint> not shared_ptr<GeometricObject>)
  for (auto& ptr : ObjectPoints) {
    if (ptr.get() == raw) return std::static_pointer_cast<GeometricObject>(ptr);
  }
  
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

    // Update dependent points (transformations, intersections, etc.)
    for (auto &pt : points) {
      if (!pt || !pt->isValid()) continue;
      if (pt->isDependent() || pt->isIntersectionPoint() || pt->getType() == ObjectType::IntersectionPoint) {
        pt->update();
      }
    }

    // Keep intersections synced during drag
    DynamicIntersection::updateAllIntersections(*this);

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
    window.clear(backgroundColor);

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
    float viewHeightAbs = std::abs(drawingView.getSize().y);
    float zoomLevel = viewHeightAbs / static_cast<float>(Constants::WINDOW_HEIGHT);

    // CRITICAL FIX: Reset the views to their default state if they seem
    // corrupted
    if (zoomLevel <= 1e-10f || zoomLevel >= 1e10f || std::isnan(zoomLevel) ||
        std::isinf(zoomLevel)) {
      std::cerr << "Error: View appears corrupted with zoom level " << zoomLevel
                << ". Resetting views to default." << std::endl;

      // Reset drawing view to default state
      resetView();

      // Recalculate zoom level
      viewHeightAbs = std::abs(drawingView.getSize().y);
      zoomLevel = viewHeightAbs / static_cast<float>(Constants::WINDOW_HEIGHT);
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

    // Calculate scale factor for invariant rendering (world units per screen pixel)
    float scale = viewHeightAbs / static_cast<float>(window.getSize().y);

    // Sync axis visibility to the actual axis state (UI reflects this elsewhere)
    const bool axesVisible = areAxesVisible();
    if (xAxis) xAxis->setVisible(axesVisible);
    if (yAxis) yAxis->setVisible(axesVisible);

    // Helper lambda for ghost rendering
    auto drawObject = [&](auto& obj) {
        if (!obj || !obj->isValid()) return;

        // Check if we are in Ghost Mode (Hide Tool active)
        bool ghostMode = (m_currentToolType == ObjectType::Hide);

        float objectScale = scale;
        if (obj->getType() == ObjectType::Line || obj->getType() == ObjectType::LineSegment) {
          float thickness = obj->getThickness();
          if (thickness > 0.0f) {
            objectScale = scale * (thickness / Constants::LINE_THICKNESS_DEFAULT);
          }
        }

        // Pass forceVisible=true if in ghost mode
        // The object's draw method handles the alpha/transparency logic if valid but hidden
        obj->draw(window, objectScale, ghostMode);
    };

    // points
    for (auto &pt : points) drawObject(pt);
    // lines
    for (auto &ln : lines) drawObject(ln);
    // circles
    for (auto &ci : circles) drawObject(ci);
    // rectangles
    for (auto &rc : rectangles) drawObject(rc);
    // polygons
    for (auto &pg : polygons) drawObject(pg);
    // regular polygons
    for (auto &rp : regularPolygons) drawObject(rp);
    // triangles
    for (auto &tr : triangles) drawObject(tr);
    // object‐points
    for (auto &op : ObjectPoints) drawObject(op);
    // angles
    for (auto &ag : angles) drawObject(ag);

    // --- Preview lines ---
    // Draw existing preview Line objects without triggering heavy updates
    if (m_parallelPreviewLine) {
      m_parallelPreviewLine->draw(window, scale);
    }
    if (m_perpendicularPreviewLine) {
      m_perpendicularPreviewLine->draw(window, scale);
    }

    // Draw lightweight overlay that follows the mouse in real-time
    if (hasPreviewLineOverlay) {
      window.draw(previewLineOverlay);
    }
    
    // Draw snapping visuals for tool feedback
    PointUtils::drawSnappingVisuals(window, m_snapState, scale);

    // selection box
    if (isDrawingSelectionBox) {
        // dynamic thickness to keep it sharp (approx 2 pixels)
        selectionBoxShape.setOutlineThickness(2.0f * scale);
        window.draw(selectionBoxShape);
    }

    // preview circle
    if (isCreatingCircle && previewCircle && previewCircle->isValid()) previewCircle->draw(window, scale);
    
    // preview rectangle
    if (isCreatingRectangle && previewRectangle && previewRectangle->isValid())
      previewRectangle->draw(window, scale);
    
    // preview rotatable rectangle
    if (isCreatingRotatableRectangle && previewRectangle && previewRectangle->isValid())
      previewRectangle->draw(window, scale);
    
    // preview polygon
    if (isCreatingPolygon && previewPolygon && previewPolygon->isValid())
      previewPolygon->draw(window, scale);
    
    // preview regular polygon
    if (isCreatingRegularPolygon && previewRegularPolygon && previewRegularPolygon->isValid())
      previewRegularPolygon->draw(window, scale);

    // preview triangle
    if (isCreatingTriangle && previewTriangle && previewTriangle->isValid())
      previewTriangle->draw(window, scale);

    // --- LABEL PASS (Screen Space) ---
    // Switch to GUI view (1:1 pixels) for sharp text
    window.setView(guiView);
    
    auto drawLabel = [&](const auto& obj) {
      if (!obj) return;
      if (!showGlobalLabels) return;
      if (!obj->getShowLabel()) return;
      obj->drawLabel(window, drawingView); // Pass original World View for mapping
    };
    
    for (const auto &pt : points) drawLabel(pt);
    for (const auto &op : ObjectPoints) drawLabel(op);
    for (const auto &rect : rectangles) drawLabel(rect);
    for (const auto &reg : regularPolygons) drawLabel(reg);
    for (const auto &tri : triangles) drawLabel(tri);
    for (const auto &poly : polygons) drawLabel(poly);
    
    // hover message
    if (showHoverMessage) {
      window.setView(guiView);
      window.draw(hoverMessageText);
    }

    // finally GUI
    window.setView(guiView);
    gui.draw(window, drawingView, *this);

    // --- RENAME OVERLAY ---
    if (isRenaming && pointToRename) {
         // Map point position to GUI space (Screen Space)
         sf::Vector2f worldPos = pointToRename->getSFMLPosition();
         sf::Vector2i screenPos = window.mapCoordsToPixel(worldPos, drawingView);
         sf::Vector2f uiPos = window.mapPixelToCoords(screenPos, guiView);

         sf::Text text;
         if (Button::getFontLoaded()) {
             text.setFont(Button::getFont());
         } 
         text.setString(renameBuffer + "|"); // Cursor
         text.setCharacterSize(16);
         text.setFillColor(sf::Color::Black);
         text.setPosition(uiPos + sf::Vector2f(15.f, -15.f)); // Offset
         
         sf::FloatRect bounds = text.getGlobalBounds();
         sf::RectangleShape bg(sf::Vector2f(bounds.width + 10.f, bounds.height + 6.f));
         bg.setPosition(bounds.left - 5.f, bounds.top - 3.f);
         bg.setFillColor(sf::Color(255, 255, 255, 220)); // Semi-transparent white
         bg.setOutlineColor(sf::Color(0, 100, 255));
         bg.setOutlineThickness(1.0f);
         
         window.draw(bg);
         window.draw(text);
         
         // Helper instruction
         sf::Text hintText;
         if (Button::getFontLoaded()) hintText.setFont(Button::getFont());
         hintText.setString("Enter to Apply, Esc to Cancel");
         hintText.setCharacterSize(12);
         hintText.setFillColor(sf::Color(50, 50, 50));
         hintText.setPosition(bg.getPosition() + sf::Vector2f(0.f, bg.getSize().y + 2.f));
         window.draw(hintText);
    }

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

  // Allow detection of invisible objects ONLY in Hide Tool mode
  bool canSelectInvisible = (m_currentToolType == ObjectType::Hide);

  // Helper for checking visibility
  auto isValidCandidate = [&](GeometricObject* obj) {
      if (!obj || !obj->isValid()) return false;
      return obj->isVisible() || canSelectInvisible;
  };

  // Priority: ObjectPoints, then free Points, then Line endpoints (handled by
  // line contains), then Lines, then Circles.

  // 1. Check ObjectPoints
  if (typeAllowed(ObjectType::ObjectPoint)) {
    for (auto it = ObjectPoints.rbegin(); it != ObjectPoints.rend();
         ++it) {  // Iterate in reverse for top-most
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  // 2. Check free Points
  if (typeAllowed(ObjectType::Point)) {
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  // 2.5 Check Shapes (Triangles, RegularPolygons, Polygons, Rectangles)
  // Checked in reverse render order (Top to Bottom)
  
  if (typeAllowed(ObjectType::Triangle)) {
    for (auto it = triangles.rbegin(); it != triangles.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  if (typeAllowed(ObjectType::RegularPolygon)) {
    for (auto it = regularPolygons.rbegin(); it != regularPolygons.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  if (typeAllowed(ObjectType::Polygon)) {
    for (auto it = polygons.rbegin(); it != polygons.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  if (typeAllowed(ObjectType::Rectangle) || typeAllowed(ObjectType::RectangleRotatable)) {
    for (auto it = rectangles.rbegin(); it != rectangles.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  // 2.75 Check Angles
  if (typeAllowed(ObjectType::Angle)) {
    for (auto it = angles.rbegin(); it != angles.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        return it->get();
      }
    }
  }

  // 3. Check Lines (including Ray/Vector)
  if (typeAllowed(ObjectType::Line) || typeAllowed(ObjectType::LineSegment) ||
      typeAllowed(ObjectType::Ray) || typeAllowed(ObjectType::Vector)) {
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        if (typeAllowed((*it)->getType())) {
          return it->get();
        }
      }
    }
  }

  // 4. Check Circles
  if (typeAllowed(ObjectType::Circle)) {
    for (auto it = circles.rbegin(); it != circles.rend(); ++it) {
      if (isValidCandidate(it->get()) &&
          (*it)->contains(worldPos_sfml, tolerance)) {
        // Check for center point interaction first
        if ((*it)->isCenterPointHovered(worldPos_sfml, tolerance)) {
          return it->get();
        }
        // Then check circumference
        if ((*it)->isCircumferenceHovered(worldPos_sfml, tolerance)) {
          return it->get();
        }
        // Fallback
        return it->get();
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
    resetCreationStates();  // Reset any pending creation state from a previous tool
    m_currentToolType = newTool;
    gui.deactivateAllTools();

    switch (m_currentToolType) {
      case ObjectType::Point:
        gui.toggleButton("Point", true);
        break;
      case ObjectType::Line:
        gui.toggleButton("Line", true);
        setToolHint("Click Start -> Click End. (Hold Ctrl to Snap)");
        break;
      case ObjectType::LineSegment:
        gui.toggleButton("Segment", true);
        setToolHint("Click Start -> Click End. (Hold Ctrl to Snap)");
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
        setToolHint("Drag to move. (Hold Ctrl to Snap to vertices)");
        break;
      case ObjectType::Intersection:
        gui.toggleButton("Intersect", true);
        break;
      case ObjectType::ParallelLine:
        gui.toggleButton("Parallel", true);
        break;
      case ObjectType::PerpendicularLine:
        gui.toggleButton("Perp", true);
        break;
      case ObjectType::Angle:
        gui.toggleButton("Angle", true);
        break;
      case ObjectType::Hide:
        gui.toggleButton("Hide", true);
        break;
      case ObjectType::Detach:
        gui.toggleButton("Detach", true);
        break;
      default:
        std::cout << "setCurrentTool: Unhandled tool type for GUI "
                  << static_cast<int>(m_currentToolType) << ", defaulting to Move." << std::endl;
        gui.toggleButton("Move", true);  // Fallback to Move tool active
        break;
    }
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
  circles.push_back(std::make_shared<Circle>(centerPtr, nullptr, radius, selectedColor));
  std::cout << "Circle created programmatically." << std::endl;
}

// --- Centralized Point Factory ---
std::shared_ptr<Point> GeometryEditor::createPoint(const Point_2 &cgalPos) {
  unsigned int id = objectIdCounter++;
  auto newPoint = std::make_shared<Point>(cgalPos, Constants::CURRENT_ZOOM, currentColor, id);
  newPoint->setRadius(currentPointSize); // Apply current point size
  if (showGlobalLabels) {
    std::string label = LabelManager::instance().getNextLabel(getAllPoints());
    newPoint->setLabel(label);
  }
  points.push_back(newPoint);
  return newPoint;
}

std::shared_ptr<Point> GeometryEditor::createPoint(const sf::Vector2f &sfmlPos) {
    return createPoint(toCGALPoint(sfmlPos));
}

void GeometryEditor::replacePoint(std::shared_ptr<Point> oldPt, std::shared_ptr<Point> newPt) {
  if (!oldPt || !newPt || oldPt == newPt) return;

  // Update lines to reference the new point
  for (auto &linePtr : lines) {
    if (!linePtr || !linePtr->isValid()) continue;
    auto start = linePtr->getStartPointObjectShared();
    auto end = linePtr->getEndPointObjectShared();
    bool changed = false;
    if (start == oldPt) {
      start = newPt;
      changed = true;
    }
    if (end == oldPt) {
      end = newPt;
      changed = true;
    }
    if (changed && start && end) {
      linePtr->setPoints(start, end);
    }
  }

  // Update circle centers/radii if they reference the old point
  for (auto &circlePtr : circles) {
    if (!circlePtr || !circlePtr->isValid()) continue;
    
    // Check Center
    if (circlePtr->getCenterPointObject() == oldPt.get()) {
      circlePtr->setCenterPointObject(newPt.get());
    }
    
    // Check Radius Point
    if (circlePtr->getRadiusPointObject() == oldPt.get()) {
        circlePtr->setRadiusPoint(newPt);
    }
    
    circlePtr->update(); // Update shape after changes
  }
  
  // SANITIZE / UPDATE EDITOR REFERENCES
  if (selectedObject == oldPt.get()) {
      selectedObject = newPt.get();
      newPt->setSelected(true);
  }
  if (hoveredObject == oldPt.get()) {
      hoveredObject = newPt.get();
      newPt->setHovered(true);
  }
  if (lineCreationPoint1 == oldPt) lineCreationPoint1 = newPt;
  // If we are dragging the old point, we should probably update drag target?
  // But replacePoint is called ON RELEASE, so dragging is done.
  
  // Remove the old point from the editor to prevent ghost points
  auto it = std::remove(points.begin(), points.end(), oldPt);
  if (it != points.end()) {
      points.erase(it, points.end());
      std::cout << "Replaced point removed from master list." << std::endl;
  }
  
  // Also check ObjectPoints
  for(size_t i=0; i<ObjectPoints.size(); ) {
      if (ObjectPoints[i] == oldPt) {
          ObjectPoints.erase(ObjectPoints.begin() + i);
          std::cout << "Replaced point removed from ObjectPoints list." << std::endl;
      } else {
          ++i;
      }
  }

  // Update polygon vertices (match by proximity)
  const double tol2 = 1e-6;
  Point_2 oldPos = oldPt->getCGALPosition();
  Point_2 newPos = newPt->getCGALPosition();

  for (auto &poly : polygons) {
    if (!poly || !poly->isValid()) continue;
    auto verts = poly->getVertices();
    for (size_t i = 0; i < verts.size(); ++i) {
      double d = CGAL::to_double(CGAL::squared_distance(verts[i], oldPos));
      if (d <= tol2) {
        poly->setVertexPosition(i, newPos);
      }
    }
  }

  for (auto &tri : triangles) {
    if (!tri || !tri->isValid()) continue;
    auto verts = tri->getVertices();
    for (size_t i = 0; i < verts.size(); ++i) {
      double d = CGAL::to_double(CGAL::squared_distance(verts[i], oldPos));
      if (d <= tol2) {
        tri->setVertexPosition(i, newPos);
      }
    }
  }

  // Preserve selection if old point was selected
  if (selectedObject == oldPt.get()) {
    oldPt->setSelected(false);
    selectedObject = newPt.get();
    if (selectedObject) {
      selectedObject->setSelected(true);
    }
  }

  // Remove old point from free points list
  points.erase(std::remove(points.begin(), points.end(), oldPt), points.end());
}

// Implementation of the update method
void GeometryEditor::update(sf::Time deltaTime) {
  // Update GUI
  gui.update(deltaTime);

  if (selectedObject && selectedObject->getType() == ObjectType::Angle) {
    setToolHint("Angle Selected. Click Palette to change Color. Drag Perimeter to Resize.");
  }

  // Update preview circle if creating a circle
  if (isCreatingCircle && previewCircle) {
    if (Constants::DEBUG_GEOMETRY_UPDATES) {
      std::cout << "Updating preview circle" << std::endl;
    }
    previewCircle->update();
  }

  // Update all geometric objects to ensure constraints and states are maintained
  for (const auto &obj : lines) {
    if (obj) obj->update();
  }
  for (const auto &obj : circles) {
     if (obj) obj->update();
  }
  for (const auto &obj : rectangles) {
     if (obj) obj->update();
  }
  for (const auto &obj : polygons) {
     if (obj) obj->update();
  }
  for (const auto &obj : regularPolygons) {
     if (obj) obj->update();
  }
  for (const auto &obj : triangles) {
     if (obj) obj->update();
  }
  for (const auto &obj : points) {
     if (obj) obj->update();
  }
  for (const auto &obj : ObjectPoints) {
     if (obj) obj->update();
  }
    for (const auto &obj : angles) {
      if (obj) obj->update();
    }

  // Always update existing intersections to maintain correct positions
  // regardless of whether auto-intersections is enabled
  DynamicIntersection::updateAllIntersections(*this);

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

  anglePointA = nullptr;
  angleVertex = nullptr;
  anglePointB = nullptr;
  if(angleLine1) angleLine1->setSelected(false);
  angleLine1 = nullptr;
  angleLine2 = nullptr;

  // Reset Compass Tool State
  for (auto* obj : m_compassSelection) {
    if(obj) obj->setSelected(false);
  }
  m_compassSelection.clear();
  m_previewCompassCenter.reset();

  dragMode = DragMode::None;
  isDragging = false;
}

void GeometryEditor::resetParallelLineToolState() {
  m_parallelReference.reset();
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
  m_perpendicularReference.reset();
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

  sf::Vector2u previousSize = m_lastWindowSize;
  if (previousSize.x == 0 || previousSize.y == 0) {
    previousSize = window.getSize();
  }

  // Calculate current zoom factor (world units per pixel) from previous size
  float currentZoom = drawingView.getSize().x / static_cast<float>(previousSize.x);

  // Update GUI view to match window pixels
  guiView.reset(sf::FloatRect(0.f, 0.f, static_cast<float>(width), static_cast<float>(height)));
  gui.updateLayout(static_cast<float>(width));

  float toolbarHeight = gui.getToolbarHeight();
  float clampedToolbar = std::min(toolbarHeight, static_cast<float>(height));
  float contentHeight = static_cast<float>(height) - clampedToolbar;
  if (contentHeight < 1.0f) {
    contentHeight = 1.0f;
  }

  sf::Vector2f oldCenter = drawingView.getCenter();

  // Preserve zoom and center, only update view size based on new window size
  sf::Vector2f newViewSize(static_cast<float>(width) * currentZoom,
                           contentHeight * currentZoom);
  drawingView.setSize(newViewSize.x, -newViewSize.y);  // Keep Y+ up
  drawingView.setCenter(oldCenter);

  float topNorm = clampedToolbar / static_cast<float>(height);
  drawingView.setViewport(sf::FloatRect(0.f, topNorm, 1.f, 1.f - topNorm));

  // Update the grid when the view changes
  grid.update(drawingView, sf::Vector2u(width, height));

  m_lastWindowSize = sf::Vector2u(width, height);
}

void GeometryEditor::resetView() {
    // 1. Define the "Perfect Grid" Density
    // 48.0f means 1 world unit = 48 screen pixels.
    // This density is spacious enough for the grid to show 1, 2, 3...
    const float pixelsPerUnit = 48.0f; 

    // 2. Get current window dimensions
    const sf::Vector2u winSize = window.getSize();

    // 3. Calculate View Size in World Units
    // ViewWidth = ScreenWidth / PixelsPerUnit
    float viewW = static_cast<float>(winSize.x) / pixelsPerUnit;
    float viewH = static_cast<float>(winSize.y) / pixelsPerUnit;

    // 4. Apply to Drawing View
    // Note: We use negative height (-viewH) to enforce Y-Up (Cartesian) coordinates.
    drawingView.setSize(viewW, -viewH);
    drawingView.setCenter(0.0f, 0.0f);
    window.setView(drawingView);

    // 5. Reset GUI View (Standard 1:1 pixel mapping for UI overlays)
    guiView.setSize(static_cast<float>(winSize.x), static_cast<float>(winSize.y));
    guiView.setCenter(static_cast<float>(winSize.x) / 2.0f, static_cast<float>(winSize.y) / 2.0f);

    // 6. Sync Global Zoom Variable
    // Update the zoom tracker so mouse wheel operations don't "jump"
    if (Constants::WINDOW_HEIGHT > 0) {
        Constants::CURRENT_ZOOM = std::abs(drawingView.getSize().y) / static_cast<float>(Constants::WINDOW_HEIGHT);
    }

    // 7. Force Grid Update to match the new view
    grid.update(drawingView, winSize);
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

    // --- Helper: Business Logic (Reuse) ---
    auto processPoint = [&](const Point_2& p) {
        // 1. Check if point is actually ON the segments (Bounding Box Check)
        auto isPointOnSegments = [&](const Point_2& pt) {
             auto check = [&](Line* l) {
                if (l->getType() == ObjectType::LineSegment) {
                    double px = CGAL::to_double(pt.x());
                    double py = CGAL::to_double(pt.y());
                    Point_2 s = l->getStartPoint();
                    Point_2 e = l->getEndPoint();
                    
                    double minX = std::min(CGAL::to_double(s.x()), CGAL::to_double(e.x())) - 0.001;
                    double maxX = std::max(CGAL::to_double(s.x()), CGAL::to_double(e.x())) + 0.001;
                    double minY = std::min(CGAL::to_double(s.y()), CGAL::to_double(e.y())) - 0.001;
                    double maxY = std::max(CGAL::to_double(s.y()), CGAL::to_double(e.y())) + 0.001;
                    
                    if (px < minX || px > maxX || py < minY || py > maxY) return false;
                }
                return true;
             };
             return check(line1) && check(line2);
        };

        if (isPointOnSegments(p)) {
            // 2. Check for Duplicates
            bool exists = false;
            double dupTol2 = 1e-6;
            for (auto &pt : points) {
                if (pt && pt->isValid()) {
                    if (CGAL::to_double(CGAL::squared_distance(pt->getCGALPosition(), p)) < dupTol2) {
                        exists = true; break;
                    }
                }
            }
            // 3. Create Point
            if (!exists) {
                auto newPoint = std::make_shared<Point>(p, Constants::CURRENT_ZOOM,
                                                        Constants::INTERSECTION_POINT_COLOR);
                newPoint->setIntersectionPoint(true);
              std::string label = LabelManager::instance().getNextLabel(getAllPoints());
              newPoint->setLabel(label);
              newPoint->setShowLabel(true);
                points.push_back(newPoint);
                std::cout << "Line-Line intersection point created." << std::endl;
            } else {
                std::cout << "Intersection point already exists nearby." << std::endl;
            }
        } else {
            std::cout << "Intersection outside segment bounds." << std::endl;
        }
    };

    // --- EXECUTION ---
    auto result = CGAL::intersection(cline1, cline2);

    if (result) {
        // MAGIC: The compiler picks the correct 'safe_get_point' based on the type of result!
        const Point_2* pPtr = safe_get_point<Point_2>(&(*result));

        if (pPtr) {
            processPoint(*pPtr);
        } else {
            std::cout << "Intersection is not a single point (Overlap or other)." << std::endl;
        }
    } else {
      std::cout << "Lines are parallel or do not intersect." << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "Error creating line-line intersection: " << e.what() << std::endl;
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
          newPoint->setIntersectionPoint(true);
          std::string label = LabelManager::instance().getNextLabel(getAllPoints());
          newPoint->setLabel(label);
          newPoint->setShowLabel(true);
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
      auto newPoint = std::make_unique<Point>(intersection1, 1.0f, Constants::INTERSECTION_POINT_COLOR);
      newPoint->setIntersectionPoint(true);
      std::string label = LabelManager::instance().getNextLabel(getAllPoints());
      newPoint->setLabel(label);
      newPoint->setShowLabel(true);
      points.push_back(std::move(newPoint));
    }
    if (isPointOnSegment(intersection2)) {
      auto newPoint = std::make_unique<Point>(intersection2, 1.0f, Constants::INTERSECTION_POINT_COLOR);
      newPoint->setIntersectionPoint(true);
      std::string label = LabelManager::instance().getNextLabel(getAllPoints());
      newPoint->setLabel(label);
      newPoint->setShowLabel(true);
      points.push_back(std::move(newPoint));
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
  try {
    auto sp1 = findSharedPtr(obj1);
    auto sp2 = findSharedPtr(obj2);
    if (!sp1 || !sp2) {
      std::cout << "Intersection not supported between these object types." << std::endl;
    } else {
      auto created = DynamicIntersection::createGenericIntersection(sp1, sp2, *this);
      if (created.empty()) {
        std::cout << "Intersection not supported between these object types." << std::endl;
      }
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
        newPoint->setIntersectionPoint(true);
        std::string label = LabelManager::instance().getNextLabel(getAllPoints());
        newPoint->setLabel(label);
        newPoint->setShowLabel(true);
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

    newPoint1->setIntersectionPoint(true);
    std::string label1 = LabelManager::instance().getNextLabel(getAllPoints());
    newPoint1->setLabel(label1);
    newPoint1->setShowLabel(true);

    newPoint2->setIntersectionPoint(true);
    std::string label2 = LabelManager::instance().getNextLabel(getAllPoints());
    newPoint2->setLabel(label2);
    newPoint2->setShowLabel(true);

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
    handleResize(window.getSize().x, window.getSize().y);

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
    case ObjectType::Ray:
      return "Ray";
    case ObjectType::Vector:
      return "Vector";
    case ObjectType::Circle:
      return "Circle";
    case ObjectType::Semicircle:
      return "Semicircle";
    case ObjectType::ObjectPoint:
      return "ObjectPoint";
    case ObjectType::Intersection:
      return "Intersection";
    case ObjectType::Circle3P:
      return "Circle (3P)";
    case ObjectType::ParallelLine:
      return "Parallel Line";
    case ObjectType::PerpendicularLine:
      return "Perpendicular Line";
    case ObjectType::AngleGiven:
      return "Angle (Given)";
    case ObjectType::Detach:
      return "Detach";
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
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
      pointsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : lines) {
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
      linesToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : circles) {
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
      circlesToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : ObjectPoints) {
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
      objPointsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : rectangles) {
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
      rectanglesToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : polygons) {
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
      polygonsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : regularPolygons) {
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
      regularPolygonsToDelete.push_back(ptr);
    }
  }
  for (auto &ptr : triangles) {
    if (ptr && ptr->isSelected() && (!ptr->isLocked() || ptr->isDependent())) {
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
  // Collect dependent ObjectPoints from shapes being deleted
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
  // NEW: Collect dependent ObjectPoints from Lines being deleted
  for (auto &linePtr : linesToDelete) {
    auto deps = collectDependentObjPoints(linePtr.get());
    for (auto &dep : deps) {
        if (std::find(objPointsToDelete.begin(), objPointsToDelete.end(), dep) == objPointsToDelete.end()) {
            objPointsToDelete.push_back(dep);
            std::cout << "  + Adding dependent ObjectPoint from Line" << std::endl;
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

  // === PHASE 3.5: DISPATCH COMMAND ===
  std::vector<std::shared_ptr<GeometricObject>> objectsToDelete;
  auto appendObjects = [&objectsToDelete](auto &vec) {
    for (auto &ptr : vec) {
      if (ptr) {
        objectsToDelete.push_back(std::static_pointer_cast<GeometricObject>(ptr));
      }
    }
  };
  appendObjects(objPointsToDelete);
  appendObjects(linesToDelete);
  appendObjects(circlesToDelete);
  appendObjects(rectanglesToDelete);
  appendObjects(polygonsToDelete);
  appendObjects(regularPolygonsToDelete);
  appendObjects(trianglesToDelete);
  appendObjects(pointsToDelete);

  commandManager.execute(std::make_shared<DeleteCommand>(*this, objectsToDelete));

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
  for (const auto &a : angles)
    if (a.get() == obj) return true;
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

    // Update angles
    for (auto &angle : angles) {
      if (angle) {
        angle->update();
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

  sanitizeReferences(rawLinePtr);

  try {
    std::cout << "safeDeleteLine: Starting deletion of Line " << lineToDelete->getID() << std::endl;

    try {
      DynamicIntersection::removeConstraintsInvolving(lineToDelete.get(), *this);
    } catch (...) {
    }

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
    if (auto refObj = m_parallelReference.lock()) {
      if (refObj == lineToDelete) {
        resetParallelLineToolState();
      }
    }
    if (auto refObj = m_perpendicularReference.lock()) {
      if (refObj == lineToDelete) {
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

void GeometryEditor::sanitizeReferences(const GeometricObject* objToDelete) {
  if (!objToDelete) return;

  if (hoveredObject == objToDelete) hoveredObject = nullptr;
  if (selectedObject == objToDelete) selectedObject = nullptr;

  if (activeVertexShape == objToDelete) {
    activeVertexShape = nullptr;
    activeVertexIndex = -1;
  }
  if (hoveredVertexShape == objToDelete) {
    hoveredVertexShape = nullptr;
    hoveredVertexIndex = -1;
  }

  if (fillTarget == objToDelete) fillTarget = nullptr;
  if (m_firstIntersectionObject == objToDelete) m_firstIntersectionObject = nullptr;

  if (m_hoveredLine && m_hoveredLine.get() == objToDelete) {
    m_hoveredLine.reset();
    m_isHoveringLine = false;
  }
  if (m_hoveredIntersectionLine1 && m_hoveredIntersectionLine1.get() == objToDelete) {
    m_hoveredIntersectionLine1.reset();
  }
  if (m_hoveredIntersectionLine2 && m_hoveredIntersectionLine2.get() == objToDelete) {
    m_hoveredIntersectionLine2.reset();
  }
  if (m_isHoveringIntersection &&
      ((!m_hoveredIntersectionLine1 && !m_hoveredIntersectionLine2) ||
       (m_hoveredIntersectionLine1 && m_hoveredIntersectionLine1.get() == objToDelete) ||
       (m_hoveredIntersectionLine2 && m_hoveredIntersectionLine2.get() == objToDelete))) {
    m_isHoveringIntersection = false;
  }

  if (m_parallelPreviewLine && m_parallelPreviewLine.get() == objToDelete) {
    m_parallelPreviewLine.reset();
  }
  if (m_perpendicularPreviewLine && m_perpendicularPreviewLine.get() == objToDelete) {
    m_perpendicularPreviewLine.reset();
  }

  if (previewCircle && previewCircle.get() == objToDelete) previewCircle.reset();
  if (previewRectangle && previewRectangle.get() == objToDelete) previewRectangle.reset();
  if (previewPolygon && previewPolygon.get() == objToDelete) previewPolygon.reset();
  if (previewRegularPolygon && previewRegularPolygon.get() == objToDelete)
    previewRegularPolygon.reset();
  if (previewTriangle && previewTriangle.get() == objToDelete) previewTriangle.reset();

  if (lineCreationPoint1 && lineCreationPoint1.get() == objToDelete) lineCreationPoint1.reset();

  if (auto refObj = m_parallelReference.lock()) {
    if (refObj.get() == objToDelete) resetParallelLineToolState();
  }
  if (auto refObj = m_perpendicularReference.lock()) {
    if (refObj.get() == objToDelete) resetPerpendicularLineToolState();
  }

  if (m_hoveredEdge && m_hoveredEdge->host == objToDelete) {
    m_hoveredEdge.reset();
  }

  if (objToDelete->getType() == ObjectType::Point) {
    auto deletedPoint = static_cast<const Point *>(objToDelete);
    for (auto &circle : circles) {
      if (circle && circle->getCenterPointObject() == deletedPoint) {
        circle->clearCenterPoint();
        circle->setVisible(false);
      }
    }
  }

  bool snapUsesDeleted = false;
  if (m_snapState.point && m_snapState.point.get() == objToDelete) snapUsesDeleted = true;
  if (m_snapState.line && m_snapState.line.get() == objToDelete) snapUsesDeleted = true;
  if (m_snapState.line1 && m_snapState.line1.get() == objToDelete) snapUsesDeleted = true;
  if (m_snapState.line2 && m_snapState.line2.get() == objToDelete) snapUsesDeleted = true;
  if (m_snapState.shape && m_snapState.shape.get() == objToDelete) snapUsesDeleted = true;
  if (snapUsesDeleted) {
    m_snapState = PointUtils::SnapState{};
  }

  try {
    DynamicIntersection::removeConstraintsInvolving(const_cast<GeometricObject*>(objToDelete),
                                                    *this);
  } catch (...) {
  }
}

// ============================================================================
// PROJECT SAVE/LOAD/EXPORT
// ============================================================================

#include "ProjectSerializer.h"
#include "Deserializer.h"

void GeometryEditor::clearScene() {
  std::cout << "GeometryEditor::clearScene: Clearing all objects..." << std::endl;
  
  // Cancel any ongoing operations
  cancelOperation();
  selectedObject = nullptr;
  hoveredObject = nullptr;
  
  // Clear all object containers
  ObjectPoints.clear();
  lines.clear();
  circles.clear();
  triangles.clear();
  rectangles.clear();
  polygons.clear();
  regularPolygons.clear();
  points.clear();
  
  std::cout << "GeometryEditor::clearScene: Scene cleared." << std::endl;
}

void GeometryEditor::saveProject(const std::string& filepath) {
  if (ProjectSerializer::saveProject(*this, filepath)) {
    setGUIMessage("Project saved: " + filepath);
  } else {
    setGUIMessage("Error saving project!");
  }
}

void GeometryEditor::loadProject(const std::string& filepath) {
  if (Deserializer::loadProject(*this, filepath)) {
    setGUIMessage("Project loaded: " + filepath);
  } else {
    setGUIMessage("Error loading project!");
  }
}

void GeometryEditor::exportSVG(const std::string& filepath) {
  if (ProjectSerializer::exportSVG(*this, filepath)) {
    setGUIMessage("SVG exported: " + filepath);
  } else {
    setGUIMessage("Error exporting SVG!");
  }
}

void GeometryEditor::clearSelection() {
    if (selectedObject) {
        selectedObject->setSelected(false);
        selectedObject = nullptr;
    }
    for (auto* obj : selectedObjects) {
        if (obj) obj->setSelected(false);
    }
    selectedObjects.clear();
}

void GeometryEditor::toggleAxes() {
  bool newState = false;
  if (xAxis) {
      newState = !xAxis->isVisible();
      xAxis->setVisible(newState);
  }
  if (yAxis) {
      // If xAxis exists, we sync to it. If not, we toggle yAxis based on its own state
      if (!xAxis) newState = !yAxis->isVisible();
      yAxis->setVisible(newState);
  }
  setGUIMessage(newState ? "Axes Visible" : "Axes Hidden");
}

bool GeometryEditor::areAxesVisible() const {
  return xAxis && xAxis->isVisible();
}