// Standard library includes first
// #include "CharTraitsFix.h"

#pragma message("--- In HandleEvents.cpp: Checking CGAL preprocessor flags ---")

#ifdef CGAL_HAS_THREADS
#pragma message("HandleEvents.cpp: CGAL_HAS_THREADS is DEFINED before any action.")
#else
#pragma message("HandleEvents.cpp: CGAL_HAS_THREADS is NOT DEFINED before any action.")
#endif

// Attempt to set the desired state
#define CGAL_HAS_THREADS 1
// CGAL_USE_SSE2 directive removed
#pragma message("HandleEvents.cpp: Action: Defined CGAL_HAS_THREADS. CGAL_USE_SSE2 not modified.")

// Verify after action
#ifdef CGAL_HAS_THREADS
#pragma message("HandleEvents.cpp: CGAL_HAS_THREADS is DEFINED after action.")
#else
#pragma message("HandleEvents.cpp: CGAL_HAS_THREADS is NOT DEFINED after action. (Problem!)")
#endif

// CGAL_USE_SSE2 check pragmas removed

#pragma message("--- End of preprocessor checks in HandleEvents.cpp ---")

#include <algorithm>  // For std::remove_if, std::clamp
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>  // For std::unique_ptr
#include <unordered_set>

// Your compiler fixes
#include "CompilerFixes.h"  // Should be early

// Your custom forward declarations
#include "ForwardDeclarations.h"

// Include LineToolMode.h before any file that might use its declarations
#include "LineToolMode.h"  // Add this include to access the LineToolMode enum and global variables

// Add the CGALSafeUtils include
#include "CGALSafeUtils.h"

// Then your project headers
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/number_utils.h>
#include <math.h>

#include <cmath>

#include "Angle.h"
#include "CGALSafeUtils.h"  // For CGALSafeUtils functions
#include "Circle.h"
#include "Constants.h"
#include "ConstructionObjects.h"
#include "GUI.h"  // For editor.gui
#include "GeometricObject.h"
#include "GeometryEditor.h"  // Needs full definition for editor members
#include "Intersection.h"    // Use Master Implementation
#include "Line.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "PointUtils.h"
#include "PointUtils.h"  // For findShapeVertex and findNearestEdge
#include "Polygon.h"
#include "ProjectionUtils.h"  // Add this include
#include "QuickProfiler.h"
#include "Rectangle.h"
#include "RegularPolygon.h"
#include "Triangle.h"
#include "Types.h"
#include "VertexLabelManager.h"
#include "VariantUtils.h"

using namespace CGALSafeUtils;
// Helper functions for checking if CGAL types contain finite values
// MODIFIED: Replace with robust versions
float currentZoomFactor = 1.0f;
static std::set<GeometricObject*> objectsBeingDeleted;
static GeometricObject* g_lastHoveredObject = nullptr;
bool is_cgal_point_finite(const Point_2& point) {
  try {
    return std::isfinite(CGAL::to_double(point.x())) && std::isfinite(CGAL::to_double(point.y()));
  } catch (const CGAL::Uncertain_conversion_exception& e) {
    std::cerr << "Warning (is_cgal_point_finite): Uncertain conversion checking "
                 "point finiteness: "
              << e.what() << std::endl;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "Warning (is_cgal_point_finite): Exception checking point "
                 "finiteness: "
              << e.what() << std::endl;
    return false;
  }
}

// ============================================================================
// TOLERANCE CONVENTION
// ============================================================================
// For USER INTERACTION (clicking, hovering, selection):
//   Use getDynamicSelectionTolerance(), getDynamicSnapTolerance(), or getDynamicHoverTolerance()
//   These convert screen pixels -> world units based on current zoom
//
// For GEOMETRIC CALCULATIONS (intersections, precision):
//   Use Constants::EPSILON or similar fixed world-unit values
//   These are view-independent mathematical tolerances
// ============================================================================

// Helper: Convert screen pixels to world units based on current zoom
static float screenPixelsToWorldUnits(const GeometryEditor& editor, float screenPixels) {
    if (editor.window.getSize().x > 0) {
        float scale = editor.drawingView.getSize().x / static_cast<float>(editor.window.getSize().x);
        return screenPixels * scale;
    }
    // Fallback: microscopic value (only triggers if window size is 0)
    return 1e-12f;
}

// Dynamic tolerance for selection/clicking (7 screen pixels)
static float getDynamicSelectionTolerance(const GeometryEditor& editor) {
    return screenPixelsToWorldUnits(editor, Constants::SELECTION_SCREEN_PIXELS);
}

// Dynamic tolerance for snapping (12 screen pixels)
static float getDynamicSnapTolerance(const GeometryEditor& editor) {
    return screenPixelsToWorldUnits(editor, Constants::SNAP_SCREEN_PIXELS);
}

// Dynamic tolerance for hover detection (10 screen pixels)
static float getDynamicHoverTolerance(const GeometryEditor& editor) {
    return screenPixelsToWorldUnits(editor, Constants::HOVER_SCREEN_PIXELS);
}

bool is_cgal_ft_finite(const Kernel::FT& value) {
  try {
    return std::isfinite(CGAL::to_double(value));
  } catch (const CGAL::Uncertain_conversion_exception& e) {
    std::cerr << "Warning (is_cgal_ft_finite): Uncertain conversion checking "
                 "Kernel::FT finiteness: "
              << e.what() << std::endl;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "Warning (is_cgal_ft_finite): Exception checking Kernel::FT "
                 "finiteness: "
              << e.what() << std::endl;
    return false;
  }
}
// Helper function to remove an object from a vector of unique_ptrs
// Returns true if an object was found and removed, false otherwise.
template <typename T>
bool removeObjectFromVector(std::vector<std::unique_ptr<T>>& vec, GeometricObject* objToRemove) {
  if (!objToRemove) return false;
  auto it = std::remove_if(vec.begin(), vec.end(), [&](const std::unique_ptr<T>& ptr) { return ptr.get() == objToRemove; });
  if (it != vec.end()) {
    vec.erase(it, vec.end());
    return true;
  }
  return false;
}
void clearAllDeletionTracking() {
  objectsBeingDeleted.clear();
  g_lastHoveredObject = nullptr;
  std::cout << "Cleared all deletion tracking on program exit" << std::endl;
}
void markObjectForDeletion(GeometricObject* obj) {
  if (obj) {
    objectsBeingDeleted.insert(obj);

    // CRITICAL: Clear the GLOBAL hover reference immediately
    if (g_lastHoveredObject == obj) {
      g_lastHoveredObject = nullptr;
      std::cout << "Cleared GLOBAL hover reference to object being deleted: " << obj << std::endl;
    }

    std::cout << "Marked object " << obj << " for deletion" << std::endl;
  }
}

void unmarkObjectForDeletion(GeometricObject* obj) {
  if (obj) {
    objectsBeingDeleted.erase(obj);
    std::cout << "Unmarked object " << obj << " from deletion" << std::endl;
  }
}

bool isObjectBeingDeleted(GeometricObject* obj) { return objectsBeingDeleted.find(obj) != objectsBeingDeleted.end(); }

void deselectAllAndClearInteractionState(GeometryEditor& editor) {
  // Safely deselect all objects with more robust error handling

  try {
    // Deselect all individual points
    for (auto& pointPtr : editor.points) {
      if (pointPtr && pointPtr->isValid()) {
        try {
          pointPtr->setSelected(false);
        } catch (const std::exception& e) {
          std::cerr << "Error deselecting point (ID: " << pointPtr->getID() << "): " << e.what() << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing points in deselectAll: " << e.what() << std::endl;
  }

  try {
    // Deselect all lines
    for (auto& linePtr : editor.lines) {
      if (linePtr && linePtr->isValid()) {
        try {
          linePtr->setSelected(false);
        } catch (const std::exception& e) {
          std::cerr << "Error deselecting line (ID: " << linePtr->getID() << "): " << e.what() << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing lines in deselectAll: " << e.what() << std::endl;
  }

  try {
    // Deselect all circles
    for (auto& circlePtr : editor.circles) {
      if (circlePtr && circlePtr->isValid()) {
        try {
          circlePtr->setSelected(false);
        } catch (const std::exception& e) {
          std::cerr << "Error deselecting circle (ID: " << circlePtr->getID() << "): " << e.what() << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing circles in deselectAll: " << e.what() << std::endl;
  }

  try {
    // Deselect all object points
    for (auto& objPointPtr : editor.ObjectPoints) {
      if (objPointPtr && objPointPtr->isValid()) {
        try {
          objPointPtr->setSelected(false);
        } catch (const std::exception& e) {
          std::cerr << "Error deselecting object point (ID: " << objPointPtr->getID() << "): " << e.what() << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing object points in deselectAll: " << e.what() << std::endl;
  }

  editor.selectedObjects.clear();

  // Deselect all rectangles and reset vertex states
  try {
    for (auto& rectPtr : editor.rectangles) {
      if (rectPtr && rectPtr->isValid()) {
        rectPtr->setSelected(false);
        rectPtr->setHoveredVertex(-1);
        rectPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing rectangles in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all polygons and reset vertex states
  try {
    for (auto& polyPtr : editor.polygons) {
      if (polyPtr && polyPtr->isValid()) {
        polyPtr->setSelected(false);
        polyPtr->setHoveredVertex(-1);
        polyPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing polygons in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all regular polygons and reset vertex states
  try {
    for (auto& regPtr : editor.regularPolygons) {
      if (regPtr && regPtr->isValid()) {
        regPtr->setSelected(false);
        regPtr->setHoveredVertex(-1);
        regPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing regular polygons in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all triangles and reset vertex states
  try {
    for (auto& triPtr : editor.triangles) {
      if (triPtr && triPtr->isValid()) {
        triPtr->setSelected(false);
        triPtr->setHoveredVertex(-1);
        triPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing triangles in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all angles
  try {
    for (auto& anglePtr : editor.angles) {
      if (anglePtr && anglePtr->isValid()) {
        anglePtr->setSelected(false);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception processing angles in deselectAll: " << e.what() << std::endl;
  }
  // Reset editor vertex state
  editor.activeVertexIndex = -1;
  editor.activeVertexShape = nullptr;
  editor.hoveredVertexIndex = -1;
  editor.hoveredVertexShape = nullptr;

  // Safely handle the primary selected object
  if (editor.selectedObject) {
    // First, check if the object pointed to by selectedObject still exists in the main lists.
    if (editor.objectExistsInAnyList(editor.selectedObject)) {
      try {
        // Now that we know it exists in a list, calling isValid() and setSelected() is safer.
        if (editor.selectedObject->isValid()) {
          editor.selectedObject->setSelected(false);  // This was line 217
        }
      } catch (const std::exception& e) {
        std::cerr << "Error deselecting selected object (ID: " << editor.selectedObject->getID() << ", which exists in lists): " << e.what()
                  << std::endl;
      }
    } else {
      // The object editor.selectedObject was pointing to is no longer in the main lists.
      // It's a dangling pointer or was an ephemeral object not in the main lists.
      // This is expected in some cases, just silently clear the reference.
    }
    editor.selectedObject = nullptr;  // Always nullify, whether it was valid/existed or not.
  }

  // Reset all drag state
  editor.dragMode = DragMode::None;
  editor.isDragging = false;
  editor.m_selectedEndpoint = EndpointSelection::None;
  editor.fillTarget = nullptr;
}
void createObjectPointOnLine(GeometryEditor& editor, Line* lineHost, const Point_2& cgalWorldPos) {
  std::cout << "Creating ObjectPoint on line" << std::endl;

  // Basic validation
  if (!lineHost->getStartPointObject() || !lineHost->getEndPointObject()) {
    std::cerr << "Cannot create ObjectPoint: Line has invalid endpoints" << std::endl;
    return;
  }

  // Get line endpoints
  Point_2 startPos, endPos;
  try {
    startPos = lineHost->getStartPoint();
    endPos = lineHost->getEndPoint();

    if (startPos == endPos) {
      std::cerr << "Cannot create ObjectPoint: Line endpoints are coincident" << std::endl;
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error getting line endpoints: " << e.what() << std::endl;
    return;
  }

  // ✅ Use ProjectionUtils for cleaner calculation
  double relativePos;
  try {
    // Add this function to ProjectionUtils.h first (see below)
    relativePos = ProjectionUtils::getRelativePositionOnLine(cgalWorldPos, startPos, endPos, lineHost->isSegment());

    std::cout << "Calculated relative position: " << relativePos << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error calculating relative position: " << e.what() << std::endl;
    relativePos = 0.5;  // Fallback to middle
  }

  // Get shared_ptr to host line
  auto hostLineShared = editor.getLineSharedPtr(lineHost);
  if (!hostLineShared) {
    std::cerr << "Error: Could not get shared_ptr for line host" << std::endl;
    return;
  }

  // Create ObjectPoint
  try {
    auto newObjPoint = ObjectPoint::create(hostLineShared, relativePos, Constants::OBJECT_POINT_DEFAULT_COLOR);

    if (newObjPoint && newObjPoint->isValid()) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      std::string label = LabelManager::getNextLabel(labelPool);
      newObjPoint->setLabel(label);
      newObjPoint->setShowLabel(true);
      editor.ObjectPoints.push_back(newObjPoint);
      std::cout << "ObjectPoint created successfully at position " << relativePos << std::endl;
      std::cout << "Total ObjectPoints: " << editor.ObjectPoints.size() << std::endl;
      editor.setGUIMessage("ObjectPoint created on line");
    } else {
      std::cerr << "Failed to create valid ObjectPoint" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error creating ObjectPoint: " << e.what() << std::endl;
  }
}

void createObjectPointOnCircle(GeometryEditor& editor, std::shared_ptr<Circle> circlePtr, const Point_2& clickPos) {
  try {
    if (!circlePtr) {
      std::cerr << "ERROR: circlePtr is null" << std::endl;
      return;
    }

    // Calculate parameter on circle
    Point_2 center = circlePtr->getCenterPoint();
    Vector_2 vectorToClick = clickPos - center;
    double angleRad = std::atan2(CGAL::to_double(vectorToClick.y()), CGAL::to_double(vectorToClick.x()));

    // Create ObjectPoint
    auto objectPoint = ObjectPoint::create(circlePtr, angleRad, sf::Color::Red);

    if (objectPoint) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      std::string label = LabelManager::getNextLabel(labelPool);
      objectPoint->setLabel(label);
      objectPoint->setShowLabel(true);
      editor.ObjectPoints.push_back(objectPoint);
      std::cout << "ObjectPoint created successfully on circle" << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "ERROR creating ObjectPoint: " << e.what() << std::endl;
  }
}
// UNIVERSAL SNAP FUNCTION: Try to snap to any nearby object (line, circle, etc)
// Returns a valid Point (either ObjectPoint on object or free point at position)
// The caller is responsible for adding it to appropriate container
std::shared_ptr<Point> trySnapToNearbyObject(GeometryEditor& editor,
                                             const sf::Vector2f& worldPos_sfml,
                                             const Point_2& cgalWorldPos,
                                             float snapTolerance,
                                             bool pointSnapEnabled) {
  if (!pointSnapEnabled) return nullptr;  // Snap only when modifier is active
  // Try to snap to circles first (both center and circumference)

  // 1. Check circle centers with 4x tolerance
  for (auto& circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid()) continue;

    Point_2 center = circlePtr->getCenterPoint();
    float distToCenter = std::sqrt(CGAL::to_double(CGAL::squared_distance(cgalWorldPos, center)));

    if (distToCenter < snapTolerance) {
      // Check if ObjectPoint already exists at circle center
      for (auto& objPt : editor.ObjectPoints) {
        if (objPt && objPt->isValid() && objPt->getHostType() == ObjectType::Circle) {
          GeometricObject* hostRaw = objPt->getHostObject();
          if (hostRaw == circlePtr.get()) {
            std::cout << "[SNAP] Snapped to existing ObjectPoint at circle center" << std::endl;
            return std::static_pointer_cast<Point>(objPt);
          }
        }
      }

      // Create new ObjectPoint at circle center (use angle-based, angle = 0 for center representation)
      // Actually for center, we'll create a FreePoint that's hosted by the circle
      auto centerObjPt = ObjectPoint::create(circlePtr, 0.0, Constants::OBJECT_POINT_DEFAULT_COLOR);
      if (centerObjPt && centerObjPt->isValid()) {
        editor.ObjectPoints.push_back(centerObjPt);
        std::cout << "[SNAP] Created ObjectPoint at circle center" << std::endl;
        return std::static_pointer_cast<Point>(centerObjPt);
      }
    }
  }

  // 2. Check circle circumferences with 3x tolerance
  for (auto& circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid()) continue;

    float circumferenceTolerance = snapTolerance;
    if (circlePtr->isCircumferenceHovered(worldPos_sfml, circumferenceTolerance)) {
      Point_2 projectedPoint = circlePtr->projectOntoCircumference(cgalWorldPos);

      // Check for existing ObjectPoint at this location
      for (auto& objPt : editor.ObjectPoints) {
        if (objPt && objPt->isValid() && objPt->getHostType() == ObjectType::Circle) {
          GeometricObject* hostRaw = objPt->getHostObject();
          if (hostRaw == circlePtr.get()) {
            Point_2 objPtPos = objPt->getCGALPosition();
            if (CGAL::to_double(CGAL::squared_distance(objPtPos, projectedPoint)) < 4.0f) {
              std::cout << "[SNAP] Snapped to existing ObjectPoint on circumference" << std::endl;
              return std::static_pointer_cast<Point>(objPt);
            }
          }
        }
      }

      // Create ObjectPoint on circumference
      Point_2 center = circlePtr->getCenterPoint();
      double dx = CGAL::to_double(projectedPoint.x() - center.x());
      double dy = CGAL::to_double(projectedPoint.y() - center.y());
      double angleRad = std::atan2(dy, dx);

      auto circumferenceObjPt = ObjectPoint::create(circlePtr, angleRad, Constants::OBJECT_POINT_DEFAULT_COLOR);
      if (circumferenceObjPt && circumferenceObjPt->isValid()) {
        editor.ObjectPoints.push_back(circumferenceObjPt);
        std::cout << "[SNAP] Created ObjectPoint on circumference at angle " << angleRad << std::endl;
        return std::static_pointer_cast<Point>(circumferenceObjPt);
      }
    }
  }

  // 3. Check lines and line segments
  for (auto& linePtr : editor.lines) {
    if (!linePtr || !linePtr->isValid()) continue;

    // Check if mouse is near the line
    if (linePtr->contains(worldPos_sfml, snapTolerance)) {
      // Project point onto line to get CGAL position
      Point_2 projPos = cgalWorldPos;

      // Try to get the projection using line endpoints
      try {
        Point_2 startPos = linePtr->getStartPoint();
        Point_2 endPos = linePtr->getEndPoint();

        // Calculate relative position on line
        double relativePos = ProjectionUtils::getRelativePositionOnLine(cgalWorldPos, startPos, endPos, linePtr->isSegment());

        // Check for existing ObjectPoint at this location
        for (auto& objPt : editor.ObjectPoints) {
          if (objPt && objPt->isValid() && objPt->getHostType() == ObjectType::Line) {
            GeometricObject* hostRaw = objPt->getHostObject();
            if (hostRaw == linePtr.get()) {
              // Check if ObjectPoint is at roughly the same position
              ObjectPoint* objPointPtr = dynamic_cast<ObjectPoint*>(objPt.get());
              if (objPointPtr) {
                double existingRelPos = objPointPtr->getRelativePositionOnLine();
                if (std::abs(existingRelPos - relativePos) < 0.01) {
                  std::cout << "[SNAP] Snapped to existing ObjectPoint on line" << std::endl;
                  return std::static_pointer_cast<Point>(objPt);
                }
              }
            }
          }
        }

        // Create new ObjectPoint on line
        auto lineObjPt = ObjectPoint::create(linePtr, relativePos, Constants::OBJECT_POINT_DEFAULT_COLOR);
        if (lineObjPt && lineObjPt->isValid()) {
          editor.ObjectPoints.push_back(lineObjPt);
          std::cout << "[SNAP] Created ObjectPoint on line at relative position " << relativePos << std::endl;
          return std::static_pointer_cast<Point>(lineObjPt);
        }
      } catch (const std::exception& e) {
        std::cerr << "[SNAP] Error projecting onto line: " << e.what() << std::endl;
      }
    }
  }

  // If no snap occurred, return nullptr
  return nullptr;
}

// Move these function implementations upwards so they're defined before they're
// used Add this new function to handle point creation
void handlePointCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
  
  // Check if ALT is pressed (for merging mode)
  bool isAltPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) || 
                      sf::Keyboard::isKeyPressed(sf::Keyboard::RAlt);
  
  // --- ALT MODE: Snap/Merge to existing point ---
  // --- SELECTION (Raw mouse position, nearest neighbor) ---
  // Use centralized screen-pixel selection tolerance
  float selectionTolerance = getDynamicSelectionTolerance(editor);
  float bestDist = selectionTolerance;
  std::shared_ptr<Point> bestPoint = nullptr;

  // ObjectPoints first
  for (const auto& op : editor.ObjectPoints) {
    if (!op || !op->isValid() || !op->isVisible() || op->isLocked()) continue;
    sf::Vector2f pos = op->getSFMLPosition();
    float dx = pos.x - worldPos_sfml.x;
    float dy = pos.y - worldPos_sfml.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist <= bestDist) {
      bestDist = dist;
      bestPoint = std::static_pointer_cast<Point>(op);
    }
  }

  // Free points
  for (const auto& pt : editor.points) {
    if (!pt || !pt->isValid() || !pt->isVisible() || pt->isLocked()) continue;
    sf::Vector2f pos = pt->getSFMLPosition();
    float dx = pos.x - worldPos_sfml.x;
    float dy = pos.y - worldPos_sfml.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist <= bestDist) {
      bestDist = dist;
      bestPoint = pt;
    }
  }

  if (bestPoint) {
    bestPoint->setSelected(true);
    std::cout << "Selected nearest point within hitbox." << std::endl;
    return; // Do not create a new point
  }

  // --- ALT MODE: Snap/Merge to existing point ---
  if (isAltPressed) {
    // Use larger tolerance (10px) for ALT merge mode
    float selectionToleranceAlt = screenPixelsToWorldUnits(editor, 10.0f);
    auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, selectionToleranceAlt);
    
    // Check if smartPoint is an EXISTING point (already in our lists)
    bool isExistingPoint = false;
    auto itPoints = std::find(editor.points.begin(), editor.points.end(), smartPoint);
    if (itPoints != editor.points.end()) {
      isExistingPoint = true;
    }
    for (const auto& op : editor.ObjectPoints) {
      if (op == smartPoint) {
        isExistingPoint = true;
        break;
      }
    }
    
    if (isExistingPoint && smartPoint) {
      // Merge: Just select the existing point, don't create a new one
      smartPoint->setSelected(true);
      std::cout << "ALT+Click: Merged/selected existing point: " << smartPoint->getLabel() << std::endl;
      return; // EXIT EARLY: No new point created
    }
  }
  
  // --- STANDARD MODE (No Alt, or Alt didn't find existing point) ---
  
  // Create smart point - this may snap to lines/circles for ObjectPoint creation
  size_t prePointsCount = editor.points.size();
  size_t preObjPointsCount = editor.ObjectPoints.size();
  auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, selectionTolerance);
  bool createdNew = (editor.points.size() > prePointsCount) || (editor.ObjectPoints.size() > preObjPointsCount);
  bool isExistingPoint = false;
  auto itExisting = std::find(editor.points.begin(), editor.points.end(), smartPoint);
  if (itExisting != editor.points.end()) {
    isExistingPoint = true;
  }
  for (const auto& op : editor.ObjectPoints) {
    if (op == smartPoint) {
      isExistingPoint = true;
      break;
    }
  }

  // If we snapped to an existing point without ALT, create a new point instead
  if (!isAltPressed && smartPoint && isExistingPoint && !createdNew) {
    // Try to create an ObjectPoint on nearest edge/line
    if (auto edgeHit = PointUtils::findNearestEdge(editor, worldPos_sfml, selectionTolerance)) {
      if (edgeHit->host) {
        if (auto circle = dynamic_cast<Circle*>(edgeHit->host)) {
          auto circlePtr = std::dynamic_pointer_cast<Circle>(editor.findSharedPtr(circle));
          if (circlePtr) {
            auto objPoint = ObjectPoint::create(circlePtr, edgeHit->relativePosition * 2.0 * M_PI, Constants::OBJECT_POINT_DEFAULT_COLOR);
            if (objPoint && objPoint->isValid()) {
              editor.ObjectPoints.push_back(objPoint);
              smartPoint = objPoint;
            }
          }
        } else {
          auto hostPtr = editor.findSharedPtr(edgeHit->host);
          if (hostPtr) {
            auto objPoint = ObjectPoint::createOnShapeEdge(hostPtr, edgeHit->edgeIndex, edgeHit->relativePosition);
            if (objPoint && objPoint->isValid()) {
              objPoint->setIsVertexAnchor(false);
              editor.ObjectPoints.push_back(objPoint);
              smartPoint = objPoint;
            }
          }
        }
      }
    }

    if (smartPoint && isExistingPoint && smartPoint == *itExisting) {
      // Try line object snapping (as ObjectPoint)
      Point_2 cursor = editor.toCGALPoint(worldPos_sfml);
      double bestDist = static_cast<double>(selectionTolerance);
      std::shared_ptr<Line> bestLine = nullptr;
      double bestRel = 0.0;
      for (auto &linePtr : editor.lines) {
        if (!linePtr || !linePtr->isValid()) continue;
        if (!linePtr->contains(worldPos_sfml, selectionTolerance)) continue;
        try {
          Point_2 startP = linePtr->getStartPoint();
          Point_2 endP = linePtr->getEndPoint();
          Line_2 hostLine = linePtr->getCGALLine();
          Point_2 proj = hostLine.projection(cursor);
          double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(proj, cursor)));
          if (dist < bestDist) {
            bestDist = dist;
            bestLine = linePtr;
            Vector_2 lineVec = endP - startP;
            double lineLenSq = CGAL::to_double(lineVec.squared_length());
            double t = 0.5;
            if (lineLenSq > 0.0) {
              Vector_2 toProj = proj - startP;
              t = CGAL::to_double(toProj * lineVec) / lineLenSq;
              if (linePtr->isSegment()) {
                t = std::max(0.0, std::min(1.0, t));
              }
            }
            bestRel = t;
          }
        } catch (...) {
        }
      }
      if (bestLine) {
        auto objPoint = ObjectPoint::create(bestLine, bestRel, Constants::OBJECT_POINT_DEFAULT_COLOR);
        if (objPoint && objPoint->isValid()) {
          editor.ObjectPoints.push_back(objPoint);
          smartPoint = objPoint;
        }
      }
    }

    // Fallback: create a free point at raw mouse position
    if (smartPoint && isExistingPoint && smartPoint == *itExisting) {
      smartPoint = editor.createPoint(editor.toCGALPoint(worldPos_sfml));
    }
  }
  if (smartPoint && smartPoint->isValid()) {
    bool addedPoint = false;
    // Check if this shared_ptr is already in our points list
    auto it = std::find(editor.points.begin(), editor.points.end(), smartPoint);

    // Check if it's an ObjectPoint (to avoid duplicating it into the free points list)
    bool isObjectPoint = false;
    for (const auto& op : editor.ObjectPoints) {
      if (op == smartPoint) {
        isObjectPoint = true;
        break;
      }
    }

    if (it == editor.points.end() && !isObjectPoint) {
      // It's a truly new point (Free or Intersection), add it
      editor.points.push_back(smartPoint);
      addedPoint = true;
    }

    // --- Auto-Labeling (Fixed: include both points AND ObjectPoints in pool) ---
    if (addedPoint || smartPoint->getLabel().empty()) {
      std::vector<std::shared_ptr<Point>> labelPool = editor.points;
      for (const auto& op : editor.ObjectPoints) {
        labelPool.push_back(std::static_pointer_cast<Point>(op));
      }
      std::string label = LabelManager::getNextLabel(labelPool);
      smartPoint->setLabel(label);
      smartPoint->setShowLabel(true);
    }

    smartPoint->setSelected(true);
    if (addedPoint) {
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(smartPoint)));
      // Undo Hook: CommandManager captures state (User Request Validation)
    }
  }

  std::cout << "Point created at (" << cgalWorldPos.x() << ", " << cgalWorldPos.y() << ")" << std::endl;
}

// Create a simplified line creation handler that will directly manipulate
// objects
void handleLineCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

    // Add coordinate validation FIRST
    const float MAX_COORD = 1e15f;
    if (!std::isfinite(worldPos_sfml.x) || !std::isfinite(worldPos_sfml.y) || std::abs(worldPos_sfml.x) > MAX_COORD ||
        std::abs(worldPos_sfml.y) > MAX_COORD) {
      std::cerr << "handleLineCreation: Mouse coordinates out of bounds" << std::endl;
      return;
    }

    Point_2 cgalWorldPos;
    try {
      cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

      // Validate CGAL point before using it
      if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
        std::cerr << "handleLineCreation: CGAL point conversion failed" << std::endl;
        return;
      }

      // Check for reasonable coordinate ranges
      double x_double = CGAL::to_double(cgalWorldPos.x());
      double y_double = CGAL::to_double(cgalWorldPos.y());

      if (!std::isfinite(x_double) || !std::isfinite(y_double) || std::abs(x_double) > MAX_COORD || std::abs(y_double) > MAX_COORD) {
        std::cerr << "handleLineCreation: CGAL coordinates out of reasonable range" << std::endl;
        return;
      }
    } catch (const std::exception& e) {
      std::cerr << "Error converting position to CGAL: " << e.what() << std::endl;
      return;
    }

    float tolerance = getDynamicSnapTolerance(editor); // Use 12px snap tolerance

    std::cout << "Line creation: Click at (" << CGAL::to_double(cgalWorldPos.x()) << ", " << CGAL::to_double(cgalWorldPos.y()) << ")" << std::endl;

    std::shared_ptr<Point> clickedPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    if (!editor.lineCreationPoint1) {
      // --- Starting line creation - first point ---
      deselectAllAndClearInteractionState(editor);

      editor.lineCreationPoint1 = clickedPoint;

      if (!editor.lineCreationPoint1 || !editor.lineCreationPoint1->isValid()) {
        std::cerr << "Error: First point for line is invalid after "
                     "creation/selection."
                  << std::endl;
        editor.lineCreationPoint1 = nullptr;
        return;
      }

      editor.lineCreationPoint1->setSelected(true);
      editor.dragMode = DragMode::CreateLineP1;
      std::cout << "First point selected for line creation." << std::endl;

    } else {
      // --- Completing line creation - second point ---
      if (!editor.lineCreationPoint1->isValid()) {
        std::cerr << "Error: First point for line creation is no longer valid." << std::endl;
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        return;
      }

      std::shared_ptr<Point> secondPoint = clickedPoint;

      if (secondPoint == editor.lineCreationPoint1) {
        std::cout << "Line creation canceled: same point selected for start "
                     "and end."
                  << std::endl;
        editor.lineCreationPoint1->setSelected(false);
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        return;
      }

      // Ensure both points are valid before proceeding
      if (!editor.lineCreationPoint1 || !editor.lineCreationPoint1->isValid() || !secondPoint || !secondPoint->isValid()) {
        std::cerr << "Error: One or both points for line creation are invalid" << std::endl;
        if (editor.lineCreationPoint1) {
          editor.lineCreationPoint1->setSelected(false);
        }
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        return;
      }

      // VALIDATION BEFORE LINE CREATION
      try {
        Point_2 p1_pos_final = editor.lineCreationPoint1->getCGALPosition();
        Point_2 p2_pos_final = secondPoint->getCGALPosition();

        // Validate coordinates are finite
        if (!CGAL::is_finite(p1_pos_final.x()) || !CGAL::is_finite(p1_pos_final.y()) || !CGAL::is_finite(p2_pos_final.x()) ||
            !CGAL::is_finite(p2_pos_final.y())) {
          std::cerr << "Error: Line endpoint coordinates are not finite before "
                       "creation"
                    << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // Convert to double for validation WITHOUT triggering exact arithmetic
        double p1_x_final, p1_y_final, p2_x_final, p2_y_final;
        try {
          p1_x_final = CGAL::to_double(p1_pos_final.x());
          p1_y_final = CGAL::to_double(p1_pos_final.y());
          p2_x_final = CGAL::to_double(p2_pos_final.x());
          p2_y_final = CGAL::to_double(p2_pos_final.y());
        } catch (const std::exception& e) {
          std::cerr << "Error: Cannot convert line endpoint coordinates to double: " << e.what() << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // Validate converted coordinates
        if (!std::isfinite(p1_x_final) || !std::isfinite(p1_y_final) || !std::isfinite(p2_x_final) || !std::isfinite(p2_y_final)) {
          std::cerr << "Error: Line endpoint coordinates are not finite doubles" << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // Check coordinate ranges
        const double MAX_COORD_SAFE = 1e8;
        if (std::abs(p1_x_final) > MAX_COORD_SAFE || std::abs(p1_y_final) > MAX_COORD_SAFE || std::abs(p2_x_final) > MAX_COORD_SAFE ||
            std::abs(p2_y_final) > MAX_COORD_SAFE) {
          std::cerr << "Error: Line coordinates exceed safe range" << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // Final distance check using double arithmetic
        double dx_final = p2_x_final - p1_x_final;
        double dy_final = p2_y_final - p1_y_final;
        double distSquared_final = dx_final * dx_final + dy_final * dy_final;

        const double minDistSquared = Constants::MIN_DISTANCE_SQUARED_LINE_CREATION;

        if (!std::isfinite(distSquared_final) || distSquared_final < minDistSquared) {
          std::cerr << "Error: Final line endpoints are too close (distance² = " << distSquared_final << ", min required = " << minDistSquared << ")"
                    << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // If we get here, coordinates are safe - proceed with line creation
        bool isSegment = (editor.m_currentToolType == ObjectType::LineSegment);
        std::cout << "Creating " << (isSegment ? "segment" : "line") << " between validated points." << std::endl;

        // Now safe to create the line
        auto newLine = std::make_shared<Line>(editor.lineCreationPoint1, secondPoint, isSegment, editor.getCurrentColor(), editor.objectIdCounter++);

        if (!newLine || !newLine->isValid()) {
          std::cerr << "Error: Failed to create valid " << (isSegment ? "segment" : "line") << std::endl;
        } else {
          newLine->registerWithEndpoints();
          editor.lines.push_back(newLine);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));
          std::cout << (isSegment ? "Segment" : "Line") << " created successfully." << std::endl;
        }

      } catch (const std::exception& e) {
        std::cerr << "Error during final line validation/creation: " << e.what() << std::endl;
        editor.lineCreationPoint1->setSelected(false);
        editor.lineCreationPoint1 = nullptr;
        editor.dragMode = DragMode::None;
        return;
      }

      // Cleanup
      editor.lineCreationPoint1->setSelected(false);
      editor.lineCreationPoint1 = nullptr;
      editor.dragMode = DragMode::None;
    }

  } catch (const std::exception& e) {
    std::cerr << "Critical error in handleLineCreation: " << e.what() << std::endl;
    // Cleanup on error
    if (editor.lineCreationPoint1) {
      try {
        editor.lineCreationPoint1->setSelected(false);
      } catch (...) {
      }
    }
    editor.lineCreationPoint1 = nullptr;
    editor.dragMode = DragMode::None;
  }
}
// Implementation for handleParallelLineCreation
void handleParallelLineCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos;
  try {
    cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
    if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
      std::cerr << "ParallelLineCreation: Initial cgalWorldPos from click is "
                   "not finite."
                << std::endl;
      editor.resetParallelLineToolState();
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error converting click to CGAL point in ParallelLineCreation: " << e.what() << std::endl;
    editor.resetParallelLineToolState();
    return;
  }
  float tolerance = getDynamicSelectionTolerance(editor);

  try {
    if (!editor.m_isPlacingParallel) {  // First click: Select reference
      editor.resetParallelLineToolState();

      // Look for ANY object that might support edge alignment
      GeometricObject* refObj = editor.lookForObjectAt(worldPos_sfml, tolerance);  // Empty list = all types

      if (refObj) {
        // Prepare to store reference
        std::shared_ptr<GeometricObject> refObjSP;
        int edgeIndex = -1;
        Vector_2 refDirection;
        bool validReferenceFound = false;

        // Case 1: Lines (keep existing behavior/optimization)
        if (refObj->getType() == ObjectType::Line || refObj->getType() == ObjectType::LineSegment) {
          Line* rawLinePtr = static_cast<Line*>(refObj);
          refObjSP = editor.getLineSharedPtr(rawLinePtr);
          if (refObjSP) {
            if (auto linePtr = std::dynamic_pointer_cast<Line>(refObjSP)) {
              if (linePtr->isValid()) {
                Point_2 p1 = linePtr->getStartPoint();
                Point_2 p2 = linePtr->getEndPoint();
                if (p1 != p2) {
                  refDirection = Vector_2(p2.x() - p1.x(), p2.y() - p1.y());
                  validReferenceFound = true;
                }
              }
            }
          }
        }
        // Case 2: Complex Shapes (Iterate edges)
        else {
          std::vector<Segment_2> edges = refObj->getEdges();
          double minDistance = std::numeric_limits<double>::max();
          int bestEdgeIndex = -1;

          for (size_t i = 0; i < edges.size(); ++i) {
            double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(edges[i], cgalWorldPos)));
            if (dist < minDistance) {
              minDistance = dist;
              bestEdgeIndex = static_cast<int>(i);
            }
          }

          // Only accept if within tolerance (though lookForObjectAt already checked bounds, explicit edge check is safer)
          // Using a slightly loose tolerance for "closest edge" selection within the object
          if (bestEdgeIndex != -1) {
            // Retrieve shared_ptr for the generic object. Iterate lists?
            // Since lookForObjectAt works on raw pointers, we need the shared_ptr.
            // We don't have a generic "getSharedPtr" helper... we must find it.
            // Or we can rely on the fact it's in a list.
            // TODO: Efficiently get shared_ptr.
            // For now, iterate all vectors. Safe given object count.

            // Try to resolve shared_ptr
            if (refObj->getType() == ObjectType::Rectangle) {
              for (auto& r : editor.rectangles)
                if (r.get() == refObj) refObjSP = r;
            } else if (refObj->getType() == ObjectType::Triangle) {
              for (auto& t : editor.triangles)
                if (t.get() == refObj) refObjSP = t;
            } else if (refObj->getType() == ObjectType::Polygon) {
              for (auto& p : editor.polygons)
                if (p.get() == refObj) refObjSP = p;
            } else if (refObj->getType() == ObjectType::RegularPolygon) {
              for (auto& rp : editor.regularPolygons)
                if (rp.get() == refObj) refObjSP = rp;
            }

            if (refObjSP) {
              refDirection = edges[bestEdgeIndex].to_vector();
              edgeIndex = bestEdgeIndex;
              validReferenceFound = true;
            }
          }
        }

        if (validReferenceFound) {
          if (refDirection.squared_length() < Kernel::FT(Constants::CGAL_EPSILON_SQUARED)) {
            editor.setGUIMessage("Parallel: Edge too short.");
            editor.resetParallelLineToolState();
            return;
          }

          editor.m_parallelReference.object = refObjSP;
          editor.m_parallelReference.edgeIndex = edgeIndex;
          editor.m_parallelReferenceDirection = refDirection;

          editor.m_isPlacingParallel = true;
          editor.setGUIMessage("Parallel: Ref selected. Click to place line.");
          return;
        }
      }

      // Fallback: Axis Check
      bool isHorizontalAxis = false;
      bool isVerticalAxis = false;
      // ... Axis logic ...
      if (std::abs(worldPos_sfml.y) <= tolerance)
        isHorizontalAxis = true;
      else if (std::abs(worldPos_sfml.x) <= tolerance)
        isVerticalAxis = true;

      if (isHorizontalAxis) {
        editor.m_parallelReference.reset();
        editor.m_parallelReferenceDirection = Vector_2(1, 0);
        editor.m_isPlacingParallel = true;
        editor.setGUIMessage("Parallel: X-Axis selected. Click to place line.");
      } else if (isVerticalAxis) {
        editor.m_parallelReference.reset();
        editor.m_parallelReferenceDirection = Vector_2(0, 1);
        editor.m_isPlacingParallel = true;
        editor.setGUIMessage("Parallel: Y-Axis selected. Click to place line.");
      } else {
        editor.setGUIMessage("Parallel: Click on a line, shape edge, or axis to select reference.");
      }

    } else {  // Second click: Place the parallel line
      if (editor.m_parallelReferenceDirection == Vector_2(0, 0)) {
        std::cerr << "CRITICAL_ERROR (Parallel): No reference direction set "
                     "for placement."
                  << std::endl;
        editor.resetParallelLineToolState();
        return;
      }

      // Find if clicking on an existing anchor point (vertex/free/object point)
      Point_2 anchorPos_cgal = cgalWorldPos;
      std::shared_ptr<Point> clickedExistingPt = nullptr;

      // Unified anchor search (same as Line tool)
      std::shared_ptr<Point> anchor = PointUtils::findAnchorPoint(editor, worldPos_sfml, tolerance * 1.5f);
      if (anchor) {
        clickedExistingPt = anchor;
        anchorPos_cgal = anchor->getCGALPosition();

        // If this is a newly created vertex ObjectPoint, register it in the editor
        bool exists = false;
        for (const auto& p : editor.points) {
          if (p == clickedExistingPt) {
            exists = true;
            break;
          }
        }
        if (!exists) {
          for (const auto& p : editor.ObjectPoints) {
            if (p == clickedExistingPt) {
              exists = true;
              break;
            }
          }
        }
        if (!exists) {
          if (auto objPt = std::dynamic_pointer_cast<ObjectPoint>(clickedExistingPt)) {
            editor.ObjectPoints.push_back(objPt);
          }
        }
      }

      // Check for shape edges or circle circumference (snap to edge)
      if (!clickedExistingPt) {
        auto edgeHit = PointUtils::findNearestEdge(editor, worldPos_sfml, tolerance * 2.0f);
        if (edgeHit.has_value()) {
          const auto& hit = edgeHit.value();

          if (Circle* circle = dynamic_cast<Circle*>(hit.host)) {
            for (auto& circlePtr : editor.circles) {
              if (circlePtr.get() == circle) {
                auto objPoint = ObjectPoint::create(circlePtr, hit.relativePosition * 2.0 * M_PI, Constants::OBJECT_POINT_DEFAULT_COLOR);
                if (objPoint && objPoint->isValid()) {
                  editor.ObjectPoints.push_back(objPoint);
                  clickedExistingPt = std::static_pointer_cast<Point>(objPoint);
                  anchorPos_cgal = clickedExistingPt->getCGALPosition();
                }
                break;
              }
            }
          } else {
            std::shared_ptr<GeometricObject> hostPtr = nullptr;
            auto findHost = [&](auto& container) {
              for (auto& shape : container) {
                if (shape.get() == hit.host) {
                  hostPtr = std::static_pointer_cast<GeometricObject>(shape);
                  return true;
                }
              }
              return false;
            };

            if (findHost(editor.rectangles) || findHost(editor.polygons) || findHost(editor.regularPolygons) || findHost(editor.triangles)) {
              if (hostPtr) {
                auto objPoint = ObjectPoint::createOnShapeEdge(hostPtr, hit.edgeIndex, hit.relativePosition);
                if (objPoint && objPoint->isValid()) {
                  editor.ObjectPoints.push_back(objPoint);
                  clickedExistingPt = objPoint;
                  anchorPos_cgal = clickedExistingPt->getCGALPosition();
                }
              }
            }
          }
        }
      }

      // Check for line intersection to snap to
      if (!clickedExistingPt) {
        for (auto& linePtr : editor.lines) {
          if (auto refObj = editor.m_parallelReference.lock()) {
            if (linePtr && linePtr.get() != refObj.get() && linePtr->contains(worldPos_sfml, tolerance)) {
              // Project the click point onto this line for better snapping
              try {
                Line_2 hostLine = linePtr->getCGALLine();
                Point_2 projectedPoint = hostLine.projection(cgalWorldPos);
                anchorPos_cgal = projectedPoint;
                break;
              } catch (const std::exception& e) {
                std::cerr << "Error projecting to line: " << e.what() << std::endl;
              }
            }
          } else {
            // No reference line (axis mode), just check all lines
            if (linePtr && linePtr->contains(worldPos_sfml, tolerance)) {
              try {
                Line_2 hostLine = linePtr->getCGALLine();
                Point_2 projectedPoint = hostLine.projection(cgalWorldPos);
                anchorPos_cgal = projectedPoint;
                break;
              } catch (const std::exception& e) {
                std::cerr << "Error projecting to line: " << e.what() << std::endl;
              }
            }
          }
        }
      }

      // Create the parallel line
      Vector_2 unit_construction_dir;
      try {
        unit_construction_dir = CGALSafeUtils::normalize_vector_robust(editor.m_parallelReferenceDirection, "ParallelCreate_normalize_ref_dir");
      } catch (const std::runtime_error& e) {
        std::cerr << "CRITICAL_ERROR (Parallel): Normalizing construction "
                     "direction: "
                  << e.what() << std::endl;
        editor.resetParallelLineToolState();
        return;
      }

      Kernel::FT constructionLength_ft = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);

      Point_2 p1_final_pos = anchorPos_cgal;
      Point_2 p2_final_pos = anchorPos_cgal + (unit_construction_dir * constructionLength_ft * 0.5);

      // Create points and line
      std::shared_ptr<Point> finalStartPoint = nullptr;
      std::shared_ptr<Point> finalEndPoint = nullptr;

      if (clickedExistingPt) {
        finalStartPoint = clickedExistingPt;
        // Adjust p2 to maintain parallel direction through the existing point
        p2_final_pos = finalStartPoint->getCGALPosition() + (unit_construction_dir * constructionLength_ft);

        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);
        if (!newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Parallel): Failed to create valid newP2." << std::endl;
          editor.resetParallelLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP2);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP2)));
        finalEndPoint = newP2;
      } else {
        auto newP1 = std::make_shared<Point>(p1_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);
        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);

        if (!newP1 || !newP1->isValid() || !newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Parallel): Failed to create valid points." << std::endl;
          editor.resetParallelLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP1);
        editor.points.push_back(newP2);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP1)));
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP2)));
        finalStartPoint = newP1;
        finalEndPoint = newP2;
      }

      if (finalStartPoint && finalEndPoint && finalStartPoint->isValid() && finalEndPoint->isValid()) {
        auto newLine = std::make_shared<Line>(finalStartPoint, finalEndPoint, false, Constants::CONSTRUCTION_LINE_COLOR, editor.objectIdCounter++);
        if (newLine && newLine->isValid()) {
          newLine->registerWithEndpoints();  // Fix propagation lag
          editor.lines.push_back(newLine);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));

          // Use weak_ptr safely for setting parallel constraint
          if (auto refObj = editor.m_parallelReference.lock()) {
            newLine->setAsParallelLine(refObj, editor.m_parallelReference.edgeIndex, editor.m_parallelReferenceDirection);
          } else {
            // Axis-based parallel line
            newLine->setAsParallelLine(nullptr, -1, editor.m_parallelReferenceDirection);
            std::cout << "Parallel line created relative to axis" << std::endl;
          }

          editor.setGUIMessage("Parallel: Line placed. Select new reference.");
        } else {
          std::cerr << "Failed to create valid parallel line object." << std::endl;
        }
      }
      editor.resetParallelLineToolState();
    }
  } catch (const std::exception& e) {
    std::cerr << "CRITICAL EXCEPTION in handleParallelLineCreation: " << e.what() << std::endl;
    editor.resetParallelLineToolState();
  }
}

void handlePerpendicularLineCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos;
  try {
    cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
    if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
      std::cerr << "PerpLineCreation: Initial cgalWorldPos from click is not "
                   "finite."
                << std::endl;
      editor.resetPerpendicularLineToolState();
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error converting click to CGAL point in "
                 "PerpendicularLineCreation: "
              << e.what() << std::endl;
    editor.resetPerpendicularLineToolState();
    return;
  }
  float tolerance = getDynamicSelectionTolerance(editor);

  try {
    if (!editor.m_isPlacingPerpendicular) {  // First click: Select reference
      editor.resetPerpendicularLineToolState();

      // Look for ANY object that might support edge alignment
      GeometricObject* refObj = editor.lookForObjectAt(worldPos_sfml, tolerance);  // Empty list = all types

      if (refObj) {
        // Prepare to store reference
        std::shared_ptr<GeometricObject> refObjSP;
        int edgeIndex = -1;
        Vector_2 refDirection;
        bool validReferenceFound = false;

        // Case 1: Lines (keep existing behavior/optimization)
        if (refObj->getType() == ObjectType::Line || refObj->getType() == ObjectType::LineSegment) {
          Line* rawLinePtr = static_cast<Line*>(refObj);
          refObjSP = editor.getLineSharedPtr(rawLinePtr);
          if (refObjSP) {
            if (auto linePtr = std::dynamic_pointer_cast<Line>(refObjSP)) {
              if (linePtr->isValid()) {
                Point_2 p1 = linePtr->getStartPoint();
                Point_2 p2 = linePtr->getEndPoint();
                if (p1 != p2) {
                  refDirection = Vector_2(p2.x() - p1.x(), p2.y() - p1.y());
                  validReferenceFound = true;
                }
              }
            }
          }
        }
        // Case 2: Complex Shapes (Iterate edges)
        else {
          std::vector<Segment_2> edges = refObj->getEdges();
          double minDistance = std::numeric_limits<double>::max();
          int bestEdgeIndex = -1;

          for (size_t i = 0; i < edges.size(); ++i) {
            double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(edges[i], cgalWorldPos)));
            if (dist < minDistance) {
              minDistance = dist;
              bestEdgeIndex = static_cast<int>(i);
            }
          }

          if (bestEdgeIndex != -1) {
            // Try to resolve shared_ptr (Generic iteration would be better but explicit is safe)
            if (refObj->getType() == ObjectType::Rectangle) {
              for (auto& r : editor.rectangles)
                if (r.get() == refObj) refObjSP = r;
            } else if (refObj->getType() == ObjectType::Triangle) {
              for (auto& t : editor.triangles)
                if (t.get() == refObj) refObjSP = t;
            } else if (refObj->getType() == ObjectType::Polygon) {
              for (auto& p : editor.polygons)
                if (p.get() == refObj) refObjSP = p;
            } else if (refObj->getType() == ObjectType::RegularPolygon) {
              for (auto& rp : editor.regularPolygons)
                if (rp.get() == refObj) refObjSP = rp;
            }

            if (refObjSP) {
              refDirection = edges[bestEdgeIndex].to_vector();
              edgeIndex = bestEdgeIndex;
              validReferenceFound = true;
            }
          }
        }

        if (validReferenceFound) {
          if (refDirection.squared_length() < Kernel::FT(Constants::CGAL_EPSILON_SQUARED)) {
            editor.setGUIMessage("Perp: Edge too short.");
            editor.resetPerpendicularLineToolState();
            return;
          }

          editor.m_perpendicularReference.object = refObjSP;
          editor.m_perpendicularReference.edgeIndex = edgeIndex;
          editor.m_perpendicularReferenceDirection = refDirection;

          editor.m_isPlacingPerpendicular = true;
          editor.setGUIMessage("Perp: Ref selected. Click to place line.");
          return;
        }
      }

      // Fallback: Axis Check
      bool isHorizontalAxis = false;
      bool isVerticalAxis = false;
      // ... Axis logic ...
      if (std::abs(worldPos_sfml.y) <= tolerance)
        isHorizontalAxis = true;
      else if (std::abs(worldPos_sfml.x) <= tolerance)
        isVerticalAxis = true;

      if (isHorizontalAxis) {
        editor.m_perpendicularReference.reset();
        editor.m_perpendicularReferenceDirection = Vector_2(1, 0);
        editor.m_isPlacingPerpendicular = true;
        editor.setGUIMessage("Perp: X-Axis selected. Click to place line.");
      } else if (isVerticalAxis) {
        editor.m_perpendicularReference.reset();
        editor.m_perpendicularReferenceDirection = Vector_2(0, 1);
        editor.m_isPlacingPerpendicular = true;
        editor.setGUIMessage("Perp: Y-Axis selected. Click to place line.");
      } else {
        editor.setGUIMessage("Select a line or edge to invoke Perpendicular Tool.");
      }
    } else {  // Second click: Place the perpendicular line
      if (editor.m_perpendicularReferenceDirection == Vector_2(0, 0)) {
        std::cerr << "CRITICAL_ERROR (Perp): No reference direction set for "
                     "placement."
                  << std::endl;
        editor.resetPerpendicularLineToolState();
        return;
      }

      // Use smart factory for anchor point
      Point_2 anchorPos_cgal = cgalWorldPos;
      std::shared_ptr<Point> clickedExistingPt = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
      if (clickedExistingPt) {
        anchorPos_cgal = clickedExistingPt->getCGALPosition();
      }

      // Create perpendicular direction
      Vector_2 perp_to_ref_dir(-editor.m_perpendicularReferenceDirection.y(), editor.m_perpendicularReferenceDirection.x());

      Vector_2 unit_construction_dir;
      try {
        unit_construction_dir = CGALSafeUtils::normalize_vector_robust(perp_to_ref_dir, "PerpCreate_normalize_perp_dir");
      } catch (const std::runtime_error& e) {
        std::cerr << "CRITICAL_ERROR (Perp): Normalizing construction direction: " << e.what() << std::endl;
        editor.resetPerpendicularLineToolState();
        return;
      }

      Kernel::FT constructionLength_ft = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);

      Point_2 p1_final_pos = anchorPos_cgal;
      Point_2 p2_final_pos = anchorPos_cgal + (unit_construction_dir * constructionLength_ft * 0.5);

      // Create points and line
      std::shared_ptr<Point> finalStartPoint = nullptr;
      std::shared_ptr<Point> finalEndPoint = nullptr;

      if (clickedExistingPt) {
        finalStartPoint = clickedExistingPt;
        // Adjust p2 to maintain perpendicular direction through the existing
        // point
        p2_final_pos = finalStartPoint->getCGALPosition() + (unit_construction_dir * constructionLength_ft);

        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);
        if (!newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Perp): Failed to create valid newP2." << std::endl;
          editor.resetPerpendicularLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP2);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP2)));
        finalEndPoint = newP2;
      } else {
        auto newP1 = std::make_shared<Point>(p1_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);
        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR, editor.objectIdCounter++);

        if (!newP1 || !newP1->isValid() || !newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Perp): Failed to create valid points." << std::endl;
          editor.resetPerpendicularLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP1);
        editor.points.push_back(newP2);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP1)));
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newP2)));
        finalStartPoint = newP1;
        finalEndPoint = newP2;
      }

      if (finalStartPoint && finalEndPoint && finalStartPoint->isValid() && finalEndPoint->isValid()) {
        auto newLine = std::make_shared<Line>(finalStartPoint, finalEndPoint, false, Constants::CONSTRUCTION_LINE_COLOR, editor.objectIdCounter++);
        if (newLine && newLine->isValid()) {
          newLine->registerWithEndpoints();  // Fix propagation lag
          editor.lines.push_back(newLine);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newLine)));

          // Use weak_ptr safely for setting perpendicular constraint
          if (auto refObj = editor.m_perpendicularReference.lock()) {
            newLine->setAsPerpendicularLine(refObj, editor.m_perpendicularReference.edgeIndex, editor.m_perpendicularReferenceDirection);
          } else {
            // Axis-based perpendicular line
            Vector_2 perpDir(-editor.m_perpendicularReferenceDirection.y(), editor.m_perpendicularReferenceDirection.x());
            newLine->setAsPerpendicularLine(nullptr, -1, editor.m_perpendicularReferenceDirection);
            std::cout << "Perpendicular line created relative to axis" << std::endl;
          }

          editor.setGUIMessage("Perp: Line placed. Select new reference.");
        } else {
          std::cerr << "Failed to create valid perpendicular line object." << std::endl;
        }
      }
      editor.resetPerpendicularLineToolState();
    }
  } catch (const std::exception& e) {
    std::cerr << "CRITICAL EXCEPTION in handlePerpendicularLineCreation: " << e.what() << std::endl;
    editor.resetPerpendicularLineToolState();
  }
}

// Construct a perpendicular bisector from two points or a single segment selection
void handlePerpendicularBisectorCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), editor.drawingView);
  float tolerance = getDynamicSelectionTolerance(editor);

  auto resetState = [&]() {
    editor.isCreatingPerpendicularBisector = false;
    editor.perpBisectorP1 = nullptr;
    editor.perpBisectorP2 = nullptr;
    editor.perpBisectorLineRef = nullptr;
  };

  auto createBisectorFromPoints = [&](std::shared_ptr<Point> a, std::shared_ptr<Point> b) {
    if (!a || !b) {
      editor.setGUIMessage("Error: Missing points for bisector.");
      resetState();
      return;
    }

    Vector_2 v = b->getCGALPosition() - a->getCGALPosition();
    double lenSq = CGAL::to_double(v.squared_length());
    if (lenSq < 1e-12) {
      editor.setGUIMessage("Error: Cannot build bisector of zero-length segment.");
      resetState();
      return;
    }

    auto bisector = std::make_shared<PerpendicularBisector>(a, b, editor.objectIdCounter++);
    if (bisector && bisector->isValid()) {
      editor.lines.push_back(bisector);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(bisector)));
      editor.setGUIMessage("Perpendicular bisector created.");
    } else {
      editor.setGUIMessage("Error: Failed to create bisector line.");
    }
    resetState();
  };

  // If a segment is clicked first, build immediately from its endpoints
  std::vector<ObjectType> allowed = {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::Line, ObjectType::LineSegment};
  GeometricObject* obj = editor.lookForObjectAt(worldPos_sfml, tolerance, allowed);

  if (!editor.isCreatingPerpendicularBisector) {
    if (obj && (obj->getType() == ObjectType::Line || obj->getType() == ObjectType::LineSegment)) {
      auto* lineRaw = static_cast<Line*>(obj);
      createBisectorFromPoints(lineRaw->getStartPointObjectShared(), lineRaw->getEndPointObjectShared());
      return;
    }

    auto first = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    if (!first || !first->isValid()) {
      editor.setGUIMessage("Select a point or segment for bisector.");
      return;
    }
    editor.perpBisectorP1 = first;
    editor.isCreatingPerpendicularBisector = true;
    editor.setGUIMessage("Select second point for bisector.");
    return;
  }

  // Second selection (point)
  auto second = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
  if (!second || !second->isValid()) {
    editor.setGUIMessage("Select a valid second point.");
    resetState();
    return;
  }
  editor.perpBisectorP2 = second;
  createBisectorFromPoints(editor.perpBisectorP1, editor.perpBisectorP2);
}

// Angle bisector from three points (A,B,C with B as vertex) or two lines
void handleAngleBisectorCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), editor.drawingView);
  float tolerance = getDynamicSelectionTolerance(editor);

  auto resetState = [&]() {
    editor.isCreatingAngleBisector = false;
    editor.angleBisectorPoints.clear();
    editor.angleBisectorLine1 = nullptr;
    editor.angleBisectorLine2 = nullptr;
  };

  // If first selections are lines, use line mode
  std::vector<ObjectType> allowed = {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::Line, ObjectType::LineSegment};
  GeometricObject* obj = editor.lookForObjectAt(worldPos_sfml, tolerance, allowed);

  if (!editor.isCreatingAngleBisector && obj && (obj->getType() == ObjectType::Line || obj->getType() == ObjectType::LineSegment)) {
    editor.angleBisectorLine1 = editor.getLineSharedPtr(static_cast<Line*>(obj));
    editor.isCreatingAngleBisector = true;
    editor.setGUIMessage("AngleBis: Select second line.");
    return;
  }

  if (editor.angleBisectorLine1 && obj && (obj->getType() == ObjectType::Line || obj->getType() == ObjectType::LineSegment)) {
    editor.angleBisectorLine2 = editor.getLineSharedPtr(static_cast<Line*>(obj));
    if (!editor.angleBisectorLine2 || editor.angleBisectorLine1.get() == obj) {
      editor.setGUIMessage("AngleBis: Invalid second line.");
      resetState();
      return;
    }

    auto result = CGAL::intersection(editor.angleBisectorLine1->getCGALLine(), editor.angleBisectorLine2->getCGALLine());
    if (!result) {
      editor.setGUIMessage("AngleBis: Lines do not intersect.");
      resetState();
      return;
    }
    const Point_2* I = safe_get_point<Point_2>(&(*result));
    if (!I) {
      editor.setGUIMessage("AngleBis: Intersection not a point.");
      resetState();
      return;
    }
    Vector_2 d1 = editor.angleBisectorLine1->getCGALLine().direction().to_vector();
    Vector_2 d2 = editor.angleBisectorLine2->getCGALLine().direction().to_vector();
    double l1 = std::sqrt(CGAL::to_double(d1.squared_length()));
    double l2 = std::sqrt(CGAL::to_double(d2.squared_length()));
    if (l1 < 1e-9 || l2 < 1e-9) {
      editor.setGUIMessage("AngleBis: Invalid line directions.");
      resetState();
      return;
    }
    Vector_2 u = d1 / FT(l1);
    Vector_2 v = d2 / FT(l2);
    Vector_2 w = u + v;
    if (CGAL::to_double(w.squared_length()) < 1e-12) {
      editor.setGUIMessage("AngleBis: Lines are opposite; bisector undefined.");
      resetState();
      return;
    }
    auto bisector = std::make_shared<AngleBisector>(editor.angleBisectorLine1, editor.angleBisectorLine2, editor.objectIdCounter++);
    if (bisector && bisector->isValid()) {
      editor.lines.push_back(bisector);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(bisector)));
      editor.setGUIMessage("Angle bisector created.");
    } else {
      editor.setGUIMessage("AngleBis: Failed to create line.");
    }
    resetState();
    return;
  }

  // Point-based mode (A,B vertex,C)
  auto pt = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
  if (!pt || !pt->isValid()) {
    editor.setGUIMessage("AngleBis: Select valid points.");
    resetState();
    return;
  }
  editor.angleBisectorPoints.push_back(pt);
  editor.isCreatingAngleBisector = true;
  if (editor.angleBisectorPoints.size() < 3) {
    editor.setGUIMessage("AngleBis: Select more points (need 3).");
    return;
  }

  auto A = editor.angleBisectorPoints[0]->getCGALPosition();
  auto B = editor.angleBisectorPoints[1]->getCGALPosition();
  auto C = editor.angleBisectorPoints[2]->getCGALPosition();
  Vector_2 u = A - B;
  Vector_2 v = C - B;
  double lu = std::sqrt(CGAL::to_double(u.squared_length()));
  double lv = std::sqrt(CGAL::to_double(v.squared_length()));
  if (lu < 1e-9 || lv < 1e-9) {
    editor.setGUIMessage("AngleBis: Points too close.");
    resetState();
    return;
  }
  Vector_2 uN = u / FT(lu);
  Vector_2 vN = v / FT(lv);
  Vector_2 w = uN + vN;
  if (CGAL::to_double(w.squared_length()) < 1e-12) {
    editor.setGUIMessage("AngleBis: 180-degree angle; bisector undefined.");
    resetState();
    return;
  }
  auto bisector = std::make_shared<AngleBisector>(editor.angleBisectorPoints[1], editor.angleBisectorPoints[0], editor.angleBisectorPoints[2],
                                                  editor.objectIdCounter++);
  if (bisector && bisector->isValid()) {
    editor.lines.push_back(bisector);
    editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(bisector)));
    editor.setGUIMessage("Angle bisector created.");
  } else {
    editor.setGUIMessage("AngleBis: Failed to create line.");
  }
  resetState();
}

// Tangent tool: handle point+circle selection and build tangents (outside and on-circle cases)
void handleTangentCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), editor.drawingView);
  float tolerance = getDynamicSelectionTolerance(editor);

  auto resetState = [&]() {
    editor.isCreatingTangent = false;
    editor.tangentAnchorPoint = nullptr;
    editor.tangentCircle = nullptr;
  };

  auto addTangentLine = [&](int solutionIndex, const std::string& msg) {
    auto tangent = std::make_shared<TangentLine>(editor.tangentAnchorPoint, editor.tangentCircle, solutionIndex, editor.objectIdCounter++);
    if (tangent && tangent->isValid()) {
      editor.lines.push_back(tangent);
      editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(tangent)));
      editor.setGUIMessage(msg);
    } else {
      editor.setGUIMessage("Error: Failed to create tangent line.");
    }
  };

  // Selection ordering: point then circle (preferred), but allow reversed order
  std::vector<ObjectType> allowed = {ObjectType::Point, ObjectType::ObjectPoint, ObjectType::Circle};
  GeometricObject* obj = editor.lookForObjectAt(worldPos_sfml, tolerance, allowed);

  auto ensurePoint = [&](GeometricObject* o) -> std::shared_ptr<Point> {
    if (!o) return nullptr;
    if (o->getType() == ObjectType::Point || o->getType() == ObjectType::ObjectPoint) {
      return PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    }
    return nullptr;
  };

  if (!editor.tangentAnchorPoint && obj && obj->getType() != ObjectType::Circle) {
    editor.tangentAnchorPoint = ensurePoint(obj);
    editor.setGUIMessage("Tangent: select a circle.");
    return;
  }

  if (!editor.tangentCircle && obj && obj->getType() == ObjectType::Circle) {
    for (auto& c : editor.circles) {
      if (c.get() == obj) {
        editor.tangentCircle = c;
        break;
      }
    }
    if (!editor.tangentCircle) {
      editor.setGUIMessage("Tangent: circle selection failed.");
      resetState();
      return;
    }
    if (!editor.tangentAnchorPoint) {
      editor.setGUIMessage("Tangent: now pick a point.");
      return;
    }
  }

  if (!editor.tangentAnchorPoint && !editor.tangentCircle) {
    // Try resolving selection order automatically
    if (obj && obj->getType() == ObjectType::Circle) {
      for (auto& c : editor.circles) {
        if (c.get() == obj) editor.tangentCircle = c;
      }
      editor.setGUIMessage("Tangent: now pick a point.");
      return;
    }
    editor.tangentAnchorPoint = ensurePoint(obj);
    editor.setGUIMessage("Tangent: select a circle.");
    return;
  }

  if (!editor.tangentAnchorPoint || !editor.tangentCircle) {
    resetState();
    return;
  }

  Point_2 P = editor.tangentAnchorPoint->getCGALPosition();
  Point_2 O = editor.tangentCircle->getCenterPoint();
  double r = editor.tangentCircle->getRadius();
  Vector_2 OP = P - O;
  double d = std::sqrt(CGAL::to_double(OP.squared_length()));
  if (d < 1e-9) {
    editor.setGUIMessage("Tangent: point coincides with center.");
    resetState();
    return;
  }

  if (d < r - 1e-6) {
    editor.setGUIMessage("Tangent: Point inside circle.");
    resetState();
    return;
  }

  if (std::abs(d - r) < 1e-6) {
    addTangentLine(0, "Tangent created (point on circle).");
    resetState();
    return;
  }

  // Outside case: two tangents
  addTangentLine(0, "Tangent created.");
  addTangentLine(1, "Tangent created.");
  resetState();
}
// Implementation for handleObjectPointCreation
void handleObjectPointCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  std::cout << "handleObjectPointCreation: ENTERED" << std::endl;

  // Convert mouse position
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos = editor.toCGALPoint(worldPos);
  float tolerance = getDynamicSelectionTolerance(editor);

  std::cout << "Looking for host at (" << worldPos.x << ", " << worldPos.y << ") with tolerance " << tolerance << std::endl;

  // Find host object (prioritize lines over circles)
  std::shared_ptr<Line> hostLine = nullptr;
  std::shared_ptr<Circle> hostCircle = nullptr;

  // First try lines (typically thinner and harder to hit)
  for (auto& line : editor.lines) {
    if (line && line->contains(worldPos, tolerance)) {
      hostLine = line;
      std::cout << "Found line host: " << line->getID() << std::endl;
      break;
    }
  }

  // If no line found, try circles
  if (!hostLine) {
    std::cout << "getCircleAtPosition: Checking " << editor.circles.size() << " circles" << std::endl;

    for (size_t i = 0; i < editor.circles.size(); ++i) {
      auto& circle_sp_in_list = editor.circles[i];

      std::cout << "  Circle[" << i << "]: ptr=" << circle_sp_in_list.get() << ", use_count=" << circle_sp_in_list.use_count() << std::endl;

      if (circle_sp_in_list && circle_sp_in_list->isValid() && circle_sp_in_list->contains(worldPos, tolerance)) {
        std::cout << "  Found matching circle at index " << i << std::endl;

        // CRITICAL: Test shared_from_this() on the circle BEFORE using it
        try {
          auto shared_test = circle_sp_in_list->shared_from_this();
          std::cout << "  Circle shared_from_this() test: use_count=" << shared_test.use_count() << std::endl;

          if (!shared_test) {
            std::cout << "  ERROR: Circle's shared_from_this() returned null!" << std::endl;
            std::cout << "  This indicates enable_shared_from_this was never initialized" << std::endl;
            std::cout << "  Skipping this circle to avoid crash" << std::endl;
            continue;  // Skip this circle and try the next one
          }
        } catch (const std::exception& e) {
          std::cout << "  ERROR: Exception testing shared_from_this(): " << e.what() << std::endl;
          std::cout << "  Skipping this circle to avoid crash" << std::endl;
          continue;  // Skip this circle and try the next one
        }

        hostCircle = circle_sp_in_list;  // COPY the shared_ptr, don't move.
        if (hostCircle) {
          std::cout << "Found circle host: " << hostCircle->getID() << ", current use_count: " << hostCircle.use_count() << std::endl;
        }
        break;
      }
    }

    if (!hostCircle) {
      std::cout << "No circle found at position" << std::endl;
    }
  }

  if (!hostLine && !hostCircle) {
    std::cout << "No suitable host object found for ObjectPoint creation" << std::endl;
    editor.setGUIMessage("ObjectPoint: Click on a line or circle");
    return;
  }

  // Create ObjectPoint based on host type
  try {
    if (hostLine) {
      createObjectPointOnLine(editor, hostLine.get(), cgalWorldPos);
    } else if (hostCircle) {
      // Add a check before passing to ensure hostCircle is valid and has a positive use_count
      if (!hostCircle || hostCircle.use_count() == 0) {
        std::cerr << "ERROR: hostCircle for ObjectPoint creation is null or has zero use_count "
                     "before calling createObjectPointOnCircle."
                  << std::endl;
        editor.setGUIMessage("Error: Invalid circle host for ObjectPoint");
        return;  // Prevent calling with an invalid shared_ptr
      }
      std::cout << "Passing hostCircle to createObjectPointOnCircle, use_count: " << hostCircle.use_count() << std::endl;
      // Pass shared_ptr directly!
      createObjectPointOnCircle(editor, hostCircle, cgalWorldPos);
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception creating ObjectPoint: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to create ObjectPoint");
  }
}

static void applyRectangleVertexLabels(GeometryEditor& editor, const std::shared_ptr<Rectangle>& rect) {
  if (!rect) return;
  auto c1 = rect->getCorner1Point();
  auto c2 = rect->getCorner2Point();

  auto isSamePoint = [](const Point_2& a, const Point_2& b) {
    return CGAL::to_double(CGAL::squared_distance(a, b)) < 1e-8;
  };

  if (rect->isRotatable()) {
    if (c1) {
      c1->setShowLabel(false);
    }
    if (c2) {
      c2->setShowLabel(false);
    }
    rect->setDependentCornerPoints(nullptr, nullptr);
    return;
  }

  if (!rect->isRotatable()) {
    auto verts = rect->getVertices();
    if (verts.size() != 4) return;

    if (c1) {
      c1->setLabel("A");
      c1->setShowLabel(true);
    }
    if (c2) {
      c2->setLabel("C");
      c2->setShowLabel(true);
    }

    auto bPoint = editor.createPoint(verts[1]);
    auto dPoint = editor.createPoint(verts[3]);
    if (bPoint) {
      bPoint->setLabel("B");
      bPoint->setShowLabel(true);
      bPoint->setDependent(true);
      bPoint->setCreatedWithShape(true);
    }
    if (dPoint) {
      dPoint->setLabel("D");
      dPoint->setShowLabel(true);
      dPoint->setDependent(true);
      dPoint->setCreatedWithShape(true);
    }
    rect->setDependentCornerPoints(bPoint, dPoint);
    return;
  }

  if (c1) {
    c1->setLabel("A");
    c1->setShowLabel(true);
  }
  if (c2) {
    c2->setLabel("B");
    c2->setShowLabel(true);
  }

  Point_2 a = rect->getCorner1();
  Point_2 b = rect->getCorner2();
  double dx = CGAL::to_double(b.x()) - CGAL::to_double(a.x());
  double dy = CGAL::to_double(b.y()) - CGAL::to_double(a.y());
  double height = std::sqrt(dx * dx + dy * dy);
  if (height < 1e-9) return;
  double ux = -dy / height;
  double uy = dx / height;

  double width = rect->getWidth();

  Point_2 c(FT(CGAL::to_double(b.x()) + ux * width), FT(CGAL::to_double(b.y()) + uy * width));
  Point_2 d(FT(CGAL::to_double(a.x()) + ux * width), FT(CGAL::to_double(a.y()) + uy * width));

  auto cPoint = editor.createPoint(c);
  auto dPoint = editor.createPoint(d);
  if (cPoint) {
    cPoint->setLabel("C");
    cPoint->setShowLabel(true);
    cPoint->setCreatedWithShape(true);
  }
  if (dPoint) {
    dPoint->setLabel("D");
    dPoint->setShowLabel(true);
    dPoint->setDependent(true);
    dPoint->setCreatedWithShape(true);
  }
  rect->setDependentCornerPoints(cPoint, dPoint);
}

// 1. handleRectangleCreation
void handleRectangleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);

    if (!editor.isCreatingRectangle) {
      editor.rectangleCorner1 = cgalWorldPos;
      editor.rectangleCorner1Point = smartPoint;
      editor.isCreatingRectangle = true;
      try {
        editor.previewRectangle =
            std::make_shared<Rectangle>(editor.rectangleCorner1, editor.rectangleCorner1, false, editor.getCurrentColor(), editor.objectIdCounter);
      } catch (...) {
        editor.previewRectangle.reset();
      }
      editor.setGUIMessage("Rectangle: Click 2nd corner (Snapping Enabled)");
    } else {
      editor.rectangleCorner2 = cgalWorldPos;
      editor.rectangleCorner2Point = smartPoint;
      if (editor.rectangleCorner1 != editor.rectangleCorner2) {
        auto corner1 = editor.rectangleCorner1Point ? editor.rectangleCorner1Point : editor.createPoint(editor.rectangleCorner1);
        auto corner2 = editor.rectangleCorner2Point ? editor.rectangleCorner2Point : editor.createPoint(editor.rectangleCorner2);
        if (corner1 && !editor.rectangleCorner1Point) {
          corner1->setCreatedWithShape(true);
        }
        if (corner2 && !editor.rectangleCorner2Point) {
          corner2->setCreatedWithShape(true);
        }
        auto newRectangle =
            std::make_shared<Rectangle>(corner1, corner2, false, editor.getCurrentColor(), editor.objectIdCounter++);
        applyRectangleVertexLabels(editor, newRectangle);
        editor.rectangles.push_back(newRectangle);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newRectangle)));
        editor.setGUIMessage("Rectangle created");
      }
      editor.isCreatingRectangle = false;
      editor.rectangleCorner1Point.reset();
      editor.rectangleCorner2Point.reset();
      editor.previewRectangle.reset();
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleRectangleCreation: " << e.what() << std::endl;
    editor.isCreatingRectangle = false;
  }
}

// 2. handleRotatableRectangleCreation
void handleRotatableRectangleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    // --- TOPOLOGICAL SMART SNAP LOGIC ---
    auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);
    // ------------------------------------

    if (!editor.isCreatingRotatableRectangle) {
      editor.rectangleCorner1 = cgalWorldPos;
      editor.rectangleCorner1Point = smartPoint;
      editor.isCreatingRotatableRectangle = true;
      try {
        editor.previewRectangle =
            std::make_shared<Rectangle>(editor.rectangleCorner1, editor.rectangleCorner1, 0.0, editor.getCurrentColor(), editor.objectIdCounter);
      } catch (...) {
        editor.previewRectangle.reset();
      }
      editor.setGUIMessage("RotRect: Click adjacent point (Snapping Enabled)");
    } else {
      editor.rectangleCorner2 = cgalWorldPos;
      editor.rectangleCorner2Point = smartPoint;

      double dx = CGAL::to_double(editor.rectangleCorner2.x() - editor.rectangleCorner1.x());
      double dy = CGAL::to_double(editor.rectangleCorner2.y() - editor.rectangleCorner1.y());
      double sideLength = std::sqrt(dx * dx + dy * dy);

      if (sideLength > Constants::MIN_CIRCLE_RADIUS) {
        auto corner1 = editor.rectangleCorner1Point ? editor.rectangleCorner1Point : editor.createPoint(editor.rectangleCorner1);
        auto corner2 = editor.rectangleCorner2Point ? editor.rectangleCorner2Point : editor.createPoint(editor.rectangleCorner2);
        if (corner1 && !editor.rectangleCorner1Point) {
          corner1->setCreatedWithShape(true);
        }
        if (corner2 && !editor.rectangleCorner2Point) {
          corner2->setCreatedWithShape(true);
        }
        auto newRectangle =
            std::make_shared<Rectangle>(corner1, corner2, sideLength, editor.getCurrentColor(), editor.objectIdCounter++);
        applyRectangleVertexLabels(editor, newRectangle);
        editor.rectangles.push_back(newRectangle);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newRectangle)));
        editor.setGUIMessage("Rotatable rectangle created");
      }
      editor.isCreatingRotatableRectangle = false;
      editor.rectangleCorner1Point.reset();
      editor.rectangleCorner2Point.reset();
      editor.previewRectangle.reset();
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleRotatableRectangleCreation: " << e.what() << std::endl;
    editor.isCreatingRotatableRectangle = false;
  }
}

// 3. handlePolygonCreation
void handlePolygonCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    // --- TOPOLOGICAL SMART SNAP LOGIC ---
    auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);
    // ------------------------------------

    if (!editor.isCreatingPolygon) {
      editor.isCreatingPolygon = true;
      editor.polygonVertices.clear();
      editor.polygonVertexPoints.clear();
      editor.polygonVertices.push_back(cgalWorldPos);
      editor.polygonVertexPoints.push_back(smartPoint);  // Can be null, resolved at creation
    } else {
      if (editor.polygonVertices.size() >= 3 && cgalWorldPos == editor.polygonVertices[0]) {
        // Convert null ptrs to new Points
        std::vector<std::shared_ptr<Point>> finalPoints;
        for (size_t i = 0; i < editor.polygonVertexPoints.size(); ++i) {
          if (editor.polygonVertexPoints[i])
            finalPoints.push_back(editor.polygonVertexPoints[i]);
          else
            finalPoints.push_back(editor.createPoint(editor.polygonVertices[i]));
        }

        for (size_t i = 0; i < finalPoints.size(); ++i) {
          if (finalPoints[i]) {
            finalPoints[i]->setLabel("P" + std::to_string(i + 1));
          }
        }
        auto newPoly = std::make_shared<Polygon>(finalPoints, editor.getCurrentColor(), editor.objectIdCounter++);
        editor.polygons.push_back(newPoly);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoly)));
        editor.isCreatingPolygon = false;
        editor.polygonVertices.clear();
        editor.polygonVertexPoints.clear();
        editor.previewPolygon.reset();
        editor.setGUIMessage("Polygon closed.");
        return;
      }
      editor.polygonVertices.push_back(cgalWorldPos);
      editor.polygonVertexPoints.push_back(smartPoint);
    }

    if (!editor.polygonVertices.empty()) {
      editor.previewPolygon = std::make_shared<Polygon>(editor.polygonVertices, editor.getCurrentColor(), editor.objectIdCounter);
    }
    editor.setGUIMessage("Polygon: Add vertex or click start to close");
  } catch (const std::exception& e) {
    std::cerr << "Exception in handlePolygonCreation: " << e.what() << std::endl;
  }
}

// 4. handleRegularPolygonCreation
void handleRegularPolygonCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    // --- TOPOLOGICAL SMART SNAP LOGIC ---
    auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);
    // ------------------------------------

    if (editor.regularPolygonPhase == 0) {
      editor.regularPolygonCenter = cgalWorldPos;
      editor.regularPolygonCenterPoint = smartPoint;
      editor.regularPolygonPhase = 1;
      editor.setGUIMessage("RegPoly: Define radius (Snapping Enabled)");
    } else if (editor.regularPolygonPhase == 1) {
      editor.regularPolygonFirstVertex = cgalWorldPos;
      editor.regularPolygonFirstVertexPoint = smartPoint;

      double dx = CGAL::to_double(editor.regularPolygonFirstVertex.x() - editor.regularPolygonCenter.x());
      double dy = CGAL::to_double(editor.regularPolygonFirstVertex.y() - editor.regularPolygonCenter.y());
      if (std::sqrt(dx * dx + dy * dy) > Constants::MIN_CIRCLE_RADIUS) {
        auto newRegPoly = std::make_shared<RegularPolygon>(
            editor.regularPolygonCenterPoint ? editor.regularPolygonCenterPoint : editor.createPoint(editor.regularPolygonCenter),
            editor.regularPolygonFirstVertexPoint ? editor.regularPolygonFirstVertexPoint : editor.createPoint(editor.regularPolygonFirstVertex),
            editor.regularPolygonNumSides, editor.getCurrentColor(), editor.objectIdCounter++);
        editor.regularPolygons.push_back(newRegPoly);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newRegPoly)));
        editor.setGUIMessage("Regular polygon created");
        editor.regularPolygonPhase = 0;
        editor.regularPolygonCenterPoint.reset();
        editor.regularPolygonFirstVertexPoint.reset();
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleRegularPolygonCreation: " << e.what() << std::endl;
    editor.regularPolygonPhase = 0;
  }
}

// 5. handleTriangleCreation
void handleTriangleCreation(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) return;

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    float tolerance = getDynamicSelectionTolerance(editor);

    // --- TOPOLOGICAL SMART SNAP LOGIC ---
    auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
    Point_2 cgalWorldPos = smartPoint ? smartPoint->getCGALPosition() : editor.toCGALPoint(worldPos_sfml);
    // ------------------------------------

    editor.triangleVertices.push_back(cgalWorldPos);
    editor.triangleVertexPoints.push_back(smartPoint);

    if (editor.triangleVertices.size() == 1) {
      editor.setGUIMessage("Triangle: Click 2nd vertex");
      editor.isCreatingTriangle = true;
    } else if (editor.triangleVertices.size() == 2) {
      editor.setGUIMessage("Triangle: Click 3rd vertex");
      editor.previewTriangle = nullptr;
    } else if (editor.triangleVertices.size() == 3) {
      if (!CGAL::collinear(editor.triangleVertices[0], editor.triangleVertices[1], editor.triangleVertices[2])) {
        // Resolve Points
        auto p1 = editor.triangleVertexPoints[0] ? editor.triangleVertexPoints[0] : editor.createPoint(editor.triangleVertices[0]);
        auto p2 = editor.triangleVertexPoints[1] ? editor.triangleVertexPoints[1] : editor.createPoint(editor.triangleVertices[1]);
        auto p3 = editor.triangleVertexPoints[2] ? editor.triangleVertexPoints[2] : editor.createPoint(editor.triangleVertices[2]);

        auto newTriangle = std::make_shared<Triangle>(p1, p2, p3, editor.getCurrentColor(), editor.objectIdCounter++);
        editor.triangles.push_back(newTriangle);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newTriangle)));
        editor.setGUIMessage("Triangle created");
      } else {
        editor.setGUIMessage("Error: Points are collinear");
      }
      editor.triangleVertices.clear();
      editor.triangleVertexPoints.clear();
      editor.isCreatingTriangle = false;
      editor.previewTriangle = nullptr;
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception in handleTriangleCreation: " << e.what() << std::endl;
    editor.triangleVertices.clear();
    editor.triangleVertexPoints.clear();
    editor.isCreatingTriangle = false;
  }
}

void handleMousePress(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  // Get the mouse position in world coordinates
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);

  sf::Vector2f guiPos = editor.window.mapPixelToCoords(pixelPos, editor.guiView);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

  // If the color palette is open and the mouse is over it, let GUI handle it
  if (editor.gui.isMouseOverPalette(guiPos)) {
    sf::Event eventToForward;
    eventToForward.type = sf::Event::MouseButtonPressed;
    eventToForward.mouseButton = mouseEvent;
    editor.gui.handleEvent(editor.window, eventToForward, editor);
    return;
  }

  // Grid snapping (Shift): snap world position to nearest grid point
  bool gridSnapActive = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
  if (gridSnapActive) {
    float g = Constants::GRID_SIZE;
    worldPos_sfml.x = std::round(worldPos_sfml.x / g) * g;
    worldPos_sfml.y = std::round(worldPos_sfml.y / g) * g;
  }

  // Debug the mouse position
  std::cout << "Mouse press at pixel: (" << pixelPos.x << ", " << pixelPos.y << "), world: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")"
            << std::endl;

  Point_2 worldPos_cgal;
  try {
    worldPos_cgal = editor.toCGALPoint(worldPos_sfml);
    std::cout << "Converted to CGAL: (" << CGAL::to_double(worldPos_cgal.x()) << ", " << CGAL::to_double(worldPos_cgal.y()) << ")" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error converting to CGAL point: " << e.what() << std::endl;
  }

  float tolerance = getDynamicSelectionTolerance(editor);
  std::cout << "Dynamic tolerance: " << tolerance << std::endl;
  std::cout << "Current tolerance: " << tolerance << std::endl;

  editor.showHoverMessage = false;  // Hide hover message on press
  if (editor.debugMode) {
    editor.debugObjectDetection(worldPos_sfml, tolerance);
  }
  // --- GUI Interaction Check ---
  // If interacting with GUI, let GUI handle it and return.
  // We also ensure that if a left click is on the canvas, panning is
  // disabled.
  bool eventHandledByGui = false;
  std::cout << "GUI check: y=" << guiPos.y << ", threshold=" << (Constants::BUTTON_SIZE.y + 0.7f) << std::endl;
  if (guiPos.y <= Constants::BUTTON_SIZE.y + 0.7f) {  // Approximate GUI area
    std::cout << "In GUI button area, forwarding to GUI handler..." << std::endl;
    sf::Event eventToForward;
    eventToForward.type = sf::Event::MouseButtonPressed;
    eventToForward.mouseButton = mouseEvent;
    if (editor.gui.handleEvent(editor.window, eventToForward, editor)) {
      std::cout << "GUI handled the event!" << std::endl;
      eventHandledByGui = true;
    } else {
      std::cout << "GUI did not handle the event" << std::endl;
    }
  }
  if (eventHandledByGui) {
    return;  // GUI handled the event
  }

  if (mouseEvent.button == sf::Mouse::Left && editor.showGlobalLabels) {
    sf::Vector2f mouseScreenPos(static_cast<float>(pixelPos.x), static_cast<float>(pixelPos.y));

    auto hitLabel = [&](GeometricObject* obj, sf::Vector2f& outLabelPos) -> bool {
      if (!obj) return false;
      if (!obj->getShowLabel()) return false;

      auto* pt = dynamic_cast<Point*>(obj);
      if (!pt || !pt->isVisible()) return false;

      std::string label = pt->getLabel();
      if (label.empty()) return false;

      const sf::Font* labelFont = Point::commonFont ? Point::commonFont : (Button::getFontLoaded() ? &Button::getFont() : nullptr);
      if (!labelFont) return false;

      sf::Vector2f worldPos = pt->getSFMLPosition();
      sf::Vector2i screenPos = editor.window.mapCoordsToPixel(worldPos, editor.drawingView);
      sf::Vector2f labelPos(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
      labelPos += pt->getLabelOffset();
      labelPos.x = std::round(labelPos.x);
      labelPos.y = std::round(labelPos.y);

      sf::Text text;
      text.setFont(*labelFont);
      text.setString(label);
      text.setCharacterSize(Constants::GRID_LABEL_FONT_SIZE);
      text.setPosition(labelPos);

      sf::FloatRect bounds = text.getGlobalBounds();
      if (bounds.contains(mouseScreenPos)) {
        outLabelPos = labelPos;
        return true;
      }
      return false;
    };

    auto tryLabelDrag = [&](const auto& container) -> bool {
      for (const auto& obj : container) {
        if (!obj) continue;
        sf::Vector2f labelPos;
        if (hitLabel(obj.get(), labelPos)) {
          deselectAllAndClearInteractionState(editor);
          editor.selectedObject = obj.get();
          editor.selectedObjects.clear();
          editor.selectedObjects.push_back(obj.get());
          obj->setSelected(true);

          editor.isDraggingLabel = true;
          editor.labelDragObject = obj.get();
          editor.labelDragGrabOffset = mouseScreenPos - labelPos;
          editor.isDragging = false;
          editor.dragMode = DragMode::None;
          editor.m_selectedEndpoint = EndpointSelection::None;
          return true;
        }
      }
      return false;
    };

    if (tryLabelDrag(editor.points)) return;
    if (tryLabelDrag(editor.ObjectPoints)) return;
  }

  // Removed Legacy Right-Click logic block that blocked Context Menu

  if (mouseEvent.button == sf::Mouse::Right && sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Right+Ctrl at: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Space+Left at: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Alt+Left at: (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Right) {
    // Double Right-Click Detection for Context Menu
    static sf::Clock lastRightClickClock;
    static int rightClickCount = 0;

    float timeSinceLast = lastRightClickClock.getElapsedTime().asSeconds();
    if (timeSinceLast < 0.4f) {  // Slightly generous limit
      rightClickCount++;
    } else {
      rightClickCount = 1;
    }
    lastRightClickClock.restart();

    if (rightClickCount == 2) {
      // Use tolerance consistent with other tools
      float tolerance = getDynamicSelectionTolerance(editor);
      GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance);

      if (hitObj) {
        editor.selectedObject = hitObj;
        hitObj->setShowLabel(!hitObj->getShowLabel());
        std::cout << "Toggle Label Visibility" << std::endl;
        // Map to GUI coordinates (using default view to match GUI overlay)
        sf::View DefaultView = editor.window.getDefaultView();
        sf::Vector2f guiPos = editor.window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), DefaultView);

        editor.getGUI().getContextMenu().open(guiPos, editor);

        editor.isPanning = false;
        rightClickCount = 0;
        return;
      }
    }

    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Right click" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && !eventHandledByGui) {
    editor.isPanning = false;  // Stop any ongoing pan if left-clicking on canvas
  }

  if (eventHandledByGui) {
    return;  // GUI handled the event
  }

  // --- Special Tool Handling ---
  // Hide tool handling
  if (editor.m_currentToolType == ObjectType::Hide) {
    if (mouseEvent.button == sf::Mouse::Left) {
      float tolerance = getDynamicSelectionTolerance(editor);
      GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance);
      if (hitObj) {
        hitObj->setVisible(!hitObj->isVisible());  // Toggle visibility
        hitObj->setSelected(false);
        hitObj->setHovered(false);
        if (editor.selectedObject == hitObj) {
          editor.selectedObject = nullptr;
        }
      }
      return;
    }
  }

  // Detach tool handling
  if (editor.m_currentToolType == ObjectType::Detach) {
    if (mouseEvent.button == sf::Mouse::Left) {
      float tolerance = getDynamicSelectionTolerance(editor);
      double bestDistSq = static_cast<double>(tolerance) * static_cast<double>(tolerance);

      GeometricObject* bestObj = nullptr;
      std::shared_ptr<Point> pointToDetach = nullptr;
      enum class PointRole { None, LineStart, LineEnd, CircleCenter, CircleRadius };
      PointRole role = PointRole::None;

      Point_2 clickPos = editor.toCGALPoint(worldPos_sfml);

      // Check Lines
      for (auto& linePtr : editor.lines) {
        if (!linePtr || !linePtr->isValid() || !linePtr->isVisible()) continue;
        auto start = linePtr->getStartPointObjectShared();
        auto end = linePtr->getEndPointObjectShared();

        if (start) {
          double d1 = CGAL::to_double(CGAL::squared_distance(clickPos, start->getCGALPosition()));
          if (d1 <= bestDistSq) {
            bestDistSq = d1;
            bestObj = linePtr.get();
            pointToDetach = start;
            role = PointRole::LineStart;
          }
        }

        if (end) {
          double d2 = CGAL::to_double(CGAL::squared_distance(clickPos, end->getCGALPosition()));
          if (d2 <= bestDistSq) {
            bestDistSq = d2;
            bestObj = linePtr.get();
            pointToDetach = end;
            role = PointRole::LineEnd;
          }
        }
      }

      // Check Circles
      for (auto& circlePtr : editor.circles) {
        if (!circlePtr || !circlePtr->isValid() || !circlePtr->isVisible()) continue;

        // Check Center
        // Note: Circle stores center as raw pointer typically, but we need shared_ptr to detach/manage it.
        // GeometricObjects usually don't expose shared_ptr to components easily if stored as raw.
        // BUT PointUtils checks editor.points.
        // We need the shared_ptr of the center point.
        Point* centerRaw = circlePtr->getCenterPointObject();
        std::shared_ptr<Point> centerShared;

        // Find shared ptr for center
        if (centerRaw) {
          for (auto& p : editor.points) {
            if (p.get() == centerRaw) {
              centerShared = p;
              break;
            }
          }
          if (!centerShared) {
            for (auto& p : editor.ObjectPoints) {
              if (p.get() == centerRaw) {
                centerShared = std::static_pointer_cast<Point>(p);
                break;
              }
            }
          }
        }

        if (centerShared) {
          double dCenter = CGAL::to_double(CGAL::squared_distance(clickPos, centerShared->getCGALPosition()));
          if (dCenter <= bestDistSq) {
            bestDistSq = dCenter;
            bestObj = circlePtr.get();
            pointToDetach = centerShared;
            role = PointRole::CircleCenter;
          }
        }

        // Check Radius Point (if exists)
        // Access radius point if available (we added getter? No, only setter/raw getter)
        Point* radiusRaw = circlePtr->getRadiusPointObject();  // We have this raw getter
        std::shared_ptr<Point> radiusShared;
        if (radiusRaw) {
          for (auto& p : editor.points) {
            if (p.get() == radiusRaw) {
              radiusShared = p;
              break;
            }
          }
          if (!radiusShared) {
            for (auto& p : editor.ObjectPoints) {
              if (p.get() == radiusRaw) {
                radiusShared = std::static_pointer_cast<Point>(p);
                break;
              }
            }
          }
        }

        if (radiusShared) {
          double dRadius = CGAL::to_double(CGAL::squared_distance(clickPos, radiusShared->getCGALPosition()));
          if (dRadius <= bestDistSq) {
            bestDistSq = dRadius;
            bestObj = circlePtr.get();
            pointToDetach = radiusShared;
            role = PointRole::CircleRadius;
          }
        }
      }

      if (bestObj && pointToDetach) {
        // Calculate usage count
        int usageCount = 0;

        // Count lines using this point
        for (auto& linePtr : editor.lines) {
          if (!linePtr || !linePtr->isValid()) continue;
          if (linePtr->getStartPointObjectShared() == pointToDetach || linePtr->getEndPointObjectShared() == pointToDetach) {
            ++usageCount;
          }
        }

        // Count circles using this point
        for (auto& circlePtr : editor.circles) {
          if (!circlePtr || !circlePtr->isValid()) continue;
          Point* c = circlePtr->getCenterPointObject();
          Point* r = circlePtr->getRadiusPointObject();
          if (c == pointToDetach.get()) ++usageCount;
          if (r == pointToDetach.get()) ++usageCount;
        }

        if (usageCount > 1) {
          // Clone
          auto newPoint =
              std::make_shared<Point>(pointToDetach->getCGALPosition(), 1.0f, pointToDetach->getFillColor(), pointToDetach->getOutlineColor());
          newPoint->setVisible(true);
          newPoint->update();
          editor.points.push_back(newPoint);

          // Update reference
          if (role == PointRole::LineStart) {
            static_cast<Line*>(bestObj)->setPoints(newPoint, static_cast<Line*>(bestObj)->getEndPointObjectShared());
          } else if (role == PointRole::LineEnd) {
            static_cast<Line*>(bestObj)->setPoints(static_cast<Line*>(bestObj)->getStartPointObjectShared(), newPoint);
          } else if (role == PointRole::CircleCenter) {
            static_cast<Circle*>(bestObj)->setCenterPointObject(newPoint.get());
          } else if (role == PointRole::CircleRadius) {
            static_cast<Circle*>(bestObj)->setRadiusPoint(newPoint);
          }

          editor.selectedObject = newPoint.get();
          editor.selectedObject->setSelected(true);
          editor.setGUIMessage("Endpoint/Center detached.");
        } else {
          editor.setGUIMessage("Point is not shared.");
        }
      }
      return;
    }
  }

  // Angle tool handling
  if (editor.m_currentToolType == ObjectType::Angle) {
    if (mouseEvent.button == sf::Mouse::Left) {
      float tolerance = getDynamicSelectionTolerance(editor);

      // 1. Try to find a Point first (standard 3-point method)
      auto anchor = PointUtils::findAnchorPoint(editor, worldPos_sfml, tolerance * 1.5f);

      if (anchor && anchor->isValid()) {
        auto ensurePointStored = [&](const std::shared_ptr<Point>& pt) {
          if (!pt) return;
          for (auto& p : editor.points)
            if (p == pt) return;
          for (auto& op : editor.ObjectPoints)
            if (op == pt) return;
          // Store if new (auto-created helper points?)
          // Actually findAnchorPoint usually returns existing points.
          // If it returns a temporary smart point, we might need to store it.
          if (std::find(editor.points.begin(), editor.points.end(), pt) == editor.points.end() &&
              std::find(editor.ObjectPoints.begin(), editor.ObjectPoints.end(), pt) == editor.ObjectPoints.end()) {
            pt->setVisible(true);
            pt->update();
            editor.points.push_back(pt);
          }
        };

        ensurePointStored(anchor);

        // Reset line selection if switching to point mode
        if (editor.angleLine1) {
          editor.angleLine1->setSelected(false);
          editor.angleLine1 = nullptr;
          editor.setGUIMessage("Angle: Switched to Point Selection");
        }

        if (!editor.anglePointA) {
          editor.anglePointA = anchor;
          editor.setGUIMessage("Angle: Vertex Point Selected? (Click Vertex next)");
          // Actually standard logic is A -> V -> B so:
          editor.setGUIMessage("Angle: Point A selected. Click Vertex.");
          return;
        }
        if (!editor.angleVertex) {
          editor.angleVertex = anchor;
          editor.setGUIMessage("Angle: Vertex selected. Click Point B.");
          return;
        }
        if (!editor.anglePointB) {
          editor.anglePointB = anchor;
        }

        if (editor.anglePointA && editor.angleVertex && editor.anglePointB) {
          auto angle = std::make_shared<Angle>(editor.anglePointA, editor.angleVertex, editor.anglePointB, false, editor.getCurrentColor());
          angle->setVisible(true);
          angle->update();
          editor.angles.push_back(angle);
          editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(angle)));
          std::cout << "Angle created via 3 points." << std::endl;
          editor.setGUIMessage("Angle created.");

          editor.anglePointA = nullptr;
          editor.angleVertex = nullptr;
          editor.anglePointB = nullptr;
        }
        return;
      }

      // 2. If no point clicked, check for Line (2-Line method)
      GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance, {ObjectType::Line, ObjectType::LineSegment});
      if (hitObj) {
        // Reset point selection if switching to line mode
        if (editor.anglePointA) editor.anglePointA = nullptr;
        if (editor.angleVertex) editor.angleVertex = nullptr;

        Line* lineRaw = static_cast<Line*>(hitObj);
        auto linePtr = editor.getLineSharedPtr(lineRaw);

        if (linePtr) {
          if (!editor.angleLine1) {
            editor.angleLine1 = linePtr;
            editor.angleLine1->setSelected(true);
            editor.setGUIMessage("Angle: Line 1 selected. Click Line 2.");
          } else if (!editor.angleLine2 && linePtr != editor.angleLine1) {
            editor.angleLine2 = linePtr;

            // --- USER REQUEST: STRICT DOT PRODUCT CHECK ---
            Vector_2 v1 = editor.angleLine1->getCGALLine().direction().to_vector();
            Vector_2 v2 = editor.angleLine2->getCGALLine().direction().to_vector();
            // Normalize
            double len1 = std::sqrt(CGAL::to_double(v1.squared_length()));
            double len2 = std::sqrt(CGAL::to_double(v2.squared_length()));

            // Guard: one or both lines are degenerate (zero-length), which can happen when
            // a perpendicular/parallel helper was created from a single point. In that case
            // we cannot compute a valid angle.
            constexpr double kEpsLen = 1e-9;
            if (len1 < kEpsLen || len2 < kEpsLen) {
              editor.setGUIMessage("Error: Cannot measure angle of zero-length segment.");
              editor.angleLine1->setSelected(false);
              editor.angleLine1 = nullptr;
              editor.angleLine2 = nullptr;
              return;
            }

            double dot = CGAL::to_double(v1 * v2) / (len1 * len2);

            // Parallel Check: dot is 1.0 or -1.0
            // Perpendicular is 0.0 (Allowed)
            if (std::abs(std::abs(dot) - 1.0) < 1e-9) {
              editor.setGUIMessage("Error: Lines are parallel, cannot form angle.");
              editor.angleLine1->setSelected(false);
              editor.angleLine1 = nullptr;
              editor.angleLine2 = nullptr;
              return;
            }

            // Calculate Intersection
            auto result = CGAL::intersection(editor.angleLine1->getCGALLine(), editor.angleLine2->getCGALLine());
            if (result) {
              const Point_2* intersectionPt = nullptr;
              Point_2 tempPt;

              if (const Point_2* p = safe_get_point<Point_2>(&(*result))) {
                intersectionPt = p;
              } else if (const auto* p = safe_get_point<Point_2>(&(*result))) {
                intersectionPt = p;
              }

              if (intersectionPt) {
                // 1. Create/Find Vertex Point
                auto vertex = PointUtils::createSmartPoint(editor, editor.toSFMLVector(*intersectionPt), tolerance);

                // Ensure vertex is stored
                if (std::find(editor.points.begin(), editor.points.end(), vertex) == editor.points.end() &&
                    std::find(editor.ObjectPoints.begin(), editor.ObjectPoints.end(), vertex) == editor.ObjectPoints.end()) {
                  vertex->setVisible(true);
                  editor.points.push_back(vertex);
                }

                Point_2 vPos = vertex->getCGALPosition();

                // 2. Safely Pick Arm Points (User Fix for Zero-Length Vector)
                // 2. Safely Pick Arm Points (User Fix for Zero-Length Vector)
                // Line 1
                Point_2 p1_cgal;
                double d1_start = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine1->getStartPoint()));
                double d1_end = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine1->getEndPoint()));

                if (d1_start > d1_end) {
                  p1_cgal = editor.angleLine1->getStartPoint();
                } else {
                  p1_cgal = editor.angleLine1->getEndPoint();
                }

                // Line 2
                Point_2 p2_cgal;
                double d2_start = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine2->getStartPoint()));
                double d2_end = CGAL::to_double(CGAL::squared_distance(vPos, editor.angleLine2->getEndPoint()));

                if (d2_start > d2_end) {
                  p2_cgal = editor.angleLine2->getStartPoint();
                } else {
                  p2_cgal = editor.angleLine2->getEndPoint();
                }

                // Validation: Ensure we didn't pick the vertex itself (handling single-point lines)
                if (std::max(d1_start, d1_end) < 1e-9 || std::max(d2_start, d2_end) < 1e-9) {
                  editor.setGUIMessage("Error: Cannot measure angle of zero-length segment.");
                  // Cleanup
                  editor.angleLine1->setSelected(false);
                  editor.angleLine1 = nullptr;
                  editor.angleLine2 = nullptr;
                  return;
                }

                // Helper to find/create arm points
                auto ensureArmPoint = [&](const Point_2& target) -> std::shared_ptr<Point> {
                  for (auto& p : editor.points) {
                    if (p && p->isValid() && p->getCGALPosition() == target) return p;
                  }
                  for (auto& p : editor.ObjectPoints) {
                    if (p && p->isValid() && p->getCGALPosition() == target) return p;
                  }
                  auto newP = std::make_shared<Point>(target, 1.0f, sf::Color::Transparent);
                  newP->setVisible(false);
                  editor.points.push_back(newP);
                  return newP;
                };

                auto p1 = ensureArmPoint(p1_cgal);
                auto p2 = ensureArmPoint(p2_cgal);

                // 3. Create Angle (No dot product check filters here, we accept all angles)
                if (p1 && vertex && p2) {
                  auto angle = std::make_shared<Angle>(p1, vertex, p2, false, editor.getCurrentColor());
                  angle->setVisible(true);
                  angle->update();
                  editor.angles.push_back(angle);
                  editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(angle)));
                  std::cout << "Angle created via 2 lines." << std::endl;
                  editor.setGUIMessage("Angle created.");
                } else {
                  editor.setGUIMessage("Error: Failed to determine angle vertices.");
                }
              } else {
                // This happens if lines are parallel (intersect at infinity or empty)
                // But we checked result bool.
                editor.setGUIMessage("Error: Intersection is not a point (Collinear/Overlapping?).");
              }
            } else {
              editor.setGUIMessage("Error: Lines are parallel.");
            }

            // Cleanup
            editor.angleLine1->setSelected(false);
            editor.angleLine1 = nullptr;
            editor.angleLine2 = nullptr;
          }
        }
      }
      return;
    }
  }

  // Add point creation tool handling
  if (editor.m_currentToolType == ObjectType::Point) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handlePointCreation(editor, mouseEvent);
      return;
    }
  }

  // Intersection tool handling (mirror Point tool intersection creation)
  if (editor.m_currentToolType == ObjectType::Intersection) {
    if (mouseEvent.button == sf::Mouse::Left) {
      float tolerance = getDynamicSelectionTolerance(editor);
      auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
      if (smartPoint && smartPoint->isValid()) {
        smartPoint->setSelected(true);
      }
      return;
    }
  }

  // Add parallel line tool handling
  if (editor.m_currentToolType == ObjectType::ParallelLine) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleParallelLineCreation(editor, mouseEvent);
      return;
    }
  }

  // Add perpendicular line tool handling
  if (editor.m_currentToolType == ObjectType::PerpendicularLine) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handlePerpendicularLineCreation(editor, mouseEvent);
      return;
    }
  }

  // Perpendicular bisector tool handling
  if (editor.m_currentToolType == ObjectType::PerpendicularBisector) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handlePerpendicularBisectorCreation(editor, mouseEvent);
      return;
    }
  }

  // Angle bisector tool handling (points or lines)
  if (editor.m_currentToolType == ObjectType::AngleBisector) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleAngleBisectorCreation(editor, mouseEvent);
      return;
    }
  }

  // Tangent tool handling (point + circle)
  if (editor.m_currentToolType == ObjectType::TangentLine) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleTangentCreation(editor, mouseEvent);
      return;
    }
  }

  // A. Circle Creation Tool
  if (editor.m_currentToolType == ObjectType::Circle) {
    if (mouseEvent.button == sf::Mouse::Left) {
      if (!editor.isCreatingCircle) {
        // Start circle creation - create the center point first
        sf::Vector2i mousePos = sf::Mouse::getPosition(editor.window);
        sf::Vector2f worldPos = editor.window.mapPixelToCoords(mousePos, editor.drawingView);
        float tolerance = getDynamicSelectionTolerance(editor);
        auto centerPoint = PointUtils::createSmartPoint(editor, worldPos, tolerance);

        if (!centerPoint || !centerPoint->isValid()) {
          return;
        }

        Point_2 center = centerPoint->getCGALPosition();
        editor.isCreatingCircle = true;
        editor.createStart_cgal = center;

        // Create preview circle using the center point with selected color
        sf::Color selectedColor = editor.getCurrentColor();
        editor.previewCircle = Circle::create(centerPoint.get(), nullptr, 0.0, selectedColor);
        std::cout << "Preview circle created at address: " << editor.previewCircle.get() << std::endl;
        std::cout << "Creating circle with center at (" << CGAL::to_double(center.x()) << ", " << CGAL::to_double(center.y()) << ") and radius 0"
                  << std::endl;
      } else {
        // Complete circle creation
        try {
          float tolerance = getDynamicSelectionTolerance(editor);
          auto radiusPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
          if (!radiusPoint || !radiusPoint->isValid()) {
            return;
          }

          Point* centerPoint = editor.previewCircle ? editor.previewCircle->getCenterPointObject() : nullptr;
          if (!centerPoint) {
            return;
          }

          Point_2 center = centerPoint->getCGALPosition();
          double radius_sq = CGAL::to_double(CGAL::squared_distance(center, radiusPoint->getCGALPosition()));
          double radius = std::sqrt(radius_sq);

          if (radius >= Constants::MIN_CIRCLE_RADIUS) {
            // The center point already exists from preview creation
            // Create the final circle with a persistent radius point
            sf::Color selectedColor = editor.previewCircle ? editor.previewCircle->getColor() : editor.getCurrentColor();
            auto finalCircle = Circle::create(centerPoint, radiusPoint, radius, selectedColor);
            if (finalCircle && finalCircle->isValid()) {
              editor.circles.push_back(finalCircle);
              editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(finalCircle)));
              std::cout << "Circle Created with radius: " << radius << std::endl;
            } else {
              std::cerr << "Preview circle is invalid" << std::endl;
            }
          } else {
            std::cout << "Circle not created: radius too small." << std::endl;
          }
        } catch (const std::exception& e) {
          std::cerr << "Error creating circle: " << e.what() << std::endl;
        }

        editor.isCreatingCircle = false;
        editor.previewCircle.reset();  // ✅ Use reset() for shared_ptr
        editor.dragMode = DragMode::None;
      }
      return;
    }
  }
  // B. Line/Segment Creation Tool
  if (editor.m_currentToolType == ObjectType::Line || editor.m_currentToolType == ObjectType::LineSegment) {
    if (mouseEvent.button == sf::Mouse::Left) {
      try {
        handleLineCreation(editor, mouseEvent);
      } catch (const std::exception& e) {
        std::cerr << "handleLineCreation threw exception: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "handleLineCreation threw unknown exception" << std::endl;
      }
      return;
    }
  }
  // C. ObjectPoint Creation Tool
  if (editor.m_currentToolType == ObjectType::ObjectPoint) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleObjectPointCreation(editor, mouseEvent);
      return;
    }
  }

  // D. Rectangle Creation Tool
  if (editor.m_currentToolType == ObjectType::Rectangle) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleRectangleCreation(editor, mouseEvent);
      return;
    }
  }

  // E. Rotatable Rectangle Creation Tool
  if (editor.m_currentToolType == ObjectType::RectangleRotatable) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleRotatableRectangleCreation(editor, mouseEvent);
      return;
    }
  }

  // F. Polygon Creation Tool
  if (editor.m_currentToolType == ObjectType::Polygon) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handlePolygonCreation(editor, mouseEvent);
      return;
    }
  }

  // G. Regular Polygon Creation Tool
  if (editor.m_currentToolType == ObjectType::RegularPolygon) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleRegularPolygonCreation(editor, mouseEvent);
      return;
    }
  }

  // H. Triangle Creation Tool (3-sided regular polygon)
  if (editor.m_currentToolType == ObjectType::Triangle) {
    if (mouseEvent.button == sf::Mouse::Left) {
      handleTriangleCreation(editor, mouseEvent);
      return;
    }
  }

  // --- Default Interaction / "Move" Tool Active ---
  if (mouseEvent.button == sf::Mouse::Left && (editor.m_currentToolType == ObjectType::None)) {
    // First, attempt to select a single object directly under the mouse
    // Clear previous selection only AFTER we determine if we're clicking on
    // an object

    // Save state of previously selected object before potentially
    // deselecting
    GeometricObject* previousSelection = editor.selectedObject;

    // Attempt to find object under mouse cursor
    GeometricObject* potentialSelection = nullptr;
    DragMode potentialDragMode = DragMode::None;
    EndpointSelection potentialEndpointSelection = EndpointSelection::None;
    int potentialVertexIndex = -1;

    // bool objectFound = false; // Removed unused variable

    // Reset vertex drag state
    editor.activeVertexIndex = -1;
    editor.activeVertexShape = nullptr;

    // Debug what we're looking for
    std::cout << "Looking for object at (" << worldPos_sfml.x << ", " << worldPos_sfml.y << ") with tolerance " << tolerance << std::endl;

    // --- PASS 1: HIGHEST PRIORITY - POINTS & VERTICES ---
    // Tolerance is unchanged for these "precision" targets

    // 1. ObjectPoints (Snap points) - Highest Priority
    // for (auto& objPointPtr : editor.ObjectPoints) {
    //   if (objPointPtr && objPointPtr->isValid() && objPointPtr->isVisible() && !objPointPtr->isLocked() && objPointPtr->contains(worldPos_sfml, tolerance)) {
    //     std::cout << "Found ObjectPoint (Priority 1)" << std::endl;
    //     potentialSelection = objPointPtr.get();
    //     potentialDragMode = DragMode::DragObjectPoint;
    //     goto objectFoundLabel;
    //   }
    // }

    // // 2. Free Points (Entities) - Includes Line Endpoints if they are Points
    // // This fixes the "Free Point Mobility" and "Line Endpoint" conflict by treating all Points as first-class draggables.
    // for (auto& pointPtr : editor.points) {
    //   if (pointPtr && pointPtr->isValid() && pointPtr->isVisible() && !pointPtr->isLocked() && pointPtr->contains(worldPos_sfml, tolerance)) {
    //     std::cout << "Found Point (Priority 2)" << std::endl;
    //     potentialSelection = pointPtr.get();
    //     potentialDragMode = DragMode::MoveFreePoint;
    //     goto objectFoundLabel;
    //   }
    // }
// --- PASS 1: HIGHEST PRIORITY - POINTS & VERTICES (UNIFIED NEAREST NEIGHBOR) ---
    // Instead of checking ObjectPoints then FreePoints sequentially, we check BOTH
    // and pick the one with the absolute smallest distance to the cursor.
    {
        GeometricObject* bestPointCandidate = nullptr;
        // Use squared distance to avoid expensive sqrt() during search
        double minDistanceSq = static_cast<double>(tolerance) * static_cast<double>(tolerance);
        
        // Helper to check a candidate point
        auto checkPointCandidate = [&](GeometricObject* obj) {
            if (!obj || !obj->isValid() || !obj->isVisible() || obj->isLocked()) return;
            
            // Calculate exact distance
            sf::Vector2f objPos;
            if (auto pt = dynamic_cast<Point*>(obj)) {
                objPos = pt->getSFMLPosition();
            } else {
                 return; 
            }

            double dx = worldPos_sfml.x - objPos.x;
            double dy = worldPos_sfml.y - objPos.y;
            double distSq = dx*dx + dy*dy;

            // Strictly closer?
            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                bestPointCandidate = obj;
            }
        };

        // 1. Check ALL ObjectPoints
        for (auto& objPointPtr : editor.ObjectPoints) {
            checkPointCandidate(objPointPtr.get());
        }

        // 2. Check ALL Free Points
        for (auto& pointPtr : editor.points) {
            checkPointCandidate(pointPtr.get());
        }

        // If we found a winner (the true closest point), select it
        if (bestPointCandidate) {
            potentialSelection = bestPointCandidate;
            
            // Set the correct Drag Mode based on what we actually found
            if (potentialSelection->getType() == ObjectType::ObjectPoint) {
                std::cout << "Found Closest ObjectPoint" << std::endl;
                potentialDragMode = DragMode::DragObjectPoint;
            } else {
                std::cout << "Found Closest Free Point" << std::endl;
                potentialDragMode = DragMode::MoveFreePoint;
            }
            
            goto objectFoundLabel;
        }
    }
    // 3. Shape Vertices (Proxies)
    {
      auto checkVertexHit = [&](auto& container) -> bool {
        for (auto& ptr : container) {
          if (!ptr || !ptr->isValid() || !ptr->isVisible() || ptr->isLocked()) continue;
          auto verts = ptr->getInteractableVertices();
          for (size_t i = 0; i < verts.size(); ++i) {
            float vx = static_cast<float>(CGAL::to_double(verts[i].x()));
            float vy = static_cast<float>(CGAL::to_double(verts[i].y()));
            float dx = vx - worldPos_sfml.x;
            float dy = vy - worldPos_sfml.y;
            float dist = std::sqrt(dx * dx + dy * dy);

            // Slightly looser tolerance for vertices to ensure they are picked over edges
            if (dist <= tolerance * 1.5f) {
              std::cout << "Found Shape Vertex (Priority 2.5)" << std::endl;
              potentialSelection = ptr.get();
              potentialDragMode = DragMode::MoveShapeVertex;
              potentialVertexIndex = static_cast<int>(i);
              return true;
            }
          }
        }
        return false;
      };

      // Special handling for RegularPolygon creation points
      auto checkRegularPolygonCreationPoints = [&]() -> bool {
        for (auto& ptr : editor.regularPolygons) {
          if (!ptr || !ptr->isValid() || !ptr->isVisible() || ptr->isLocked()) continue;
          auto creationPts = ptr->getCreationPointsSFML();
          for (size_t i = 0; i < creationPts.size(); ++i) {
            float dx = creationPts[i].x - worldPos_sfml.x;
            float dy = creationPts[i].y - worldPos_sfml.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= tolerance * 1.5f) {
              potentialSelection = ptr.get();
              potentialDragMode = DragMode::MoveShapeVertex;
              potentialVertexIndex = static_cast<int>(i);
              return true;
            }
          }
        }
        return false;
      };

      if (checkVertexHit(editor.rectangles) || checkVertexHit(editor.polygons) || checkRegularPolygonCreationPoints() ||
          checkVertexHit(editor.triangles)) {
        goto objectFoundLabel;
      }
    }

    // 3. Angles (selection only)
    for (auto& anglePtr : editor.angles) {
      if (anglePtr && anglePtr->isValid() && anglePtr->isVisible() && !anglePtr->isLocked() && anglePtr->contains(worldPos_sfml, tolerance)) {
        std::cout << "Found Angle (Priority 2.8)" << std::endl;
        potentialSelection = anglePtr.get();

        // Check for resizing interaction (clicking the arc itself)
        if (anglePtr->isMouseOverArc(worldPos_sfml, tolerance)) {
          editor.isResizingAngle = true;
          std::cout << "Angle Resize Mode Activated" << std::endl;
        } else {
          editor.isResizingAngle = false;
        }

        potentialDragMode = DragMode::None;  // We handle resizing via the bool flag
        goto objectFoundLabel;
      }
    }

    // --- PASS 2: MEDIUM PRIORITY - LINEAR ELEMENTS (EDGES) ---
    // If we missed vertices, check edges.

    // 4. Lines (Bodies)
    for (auto& linePtr : editor.lines) {
      if (linePtr && linePtr->isValid() && linePtr->isVisible() && !linePtr->isLocked() && linePtr->contains(worldPos_sfml, tolerance)) {
        std::cout << "Found Line Body (Priority 3)" << std::endl;
        potentialSelection = linePtr.get();
        
        // Check if endpoints are ObjectPoints or otherwise constrained
        // If so, the line should NOT be translatable - only its endpoints can be moved
        Point* startPt = linePtr->getStartPointObject();
        Point* endPt = linePtr->getEndPointObject();
        bool startConstrained = (dynamic_cast<ObjectPoint*>(startPt) != nullptr) || (startPt && startPt->isLocked());
        bool endConstrained = (dynamic_cast<ObjectPoint*>(endPt) != nullptr) || (endPt && endPt->isLocked());
        
        if (startConstrained || endConstrained) {
          // Line has constrained endpoints - don't allow translation
          potentialDragMode = DragMode::None;
          std::cout << "  Line has constrained endpoint(s) - translation disabled" << std::endl;
        } else {
          // Free line - allow translation
          potentialDragMode = DragMode::TranslateLine;
        }
        editor.dragStart_sfml = worldPos_sfml;
        goto objectFoundLabel;
      }
    }

    // 5. Shape Edges (Explicit Edge Check)
    // This allows selecting a shape by its edge even if the fill is transparent or hit logic is strictly boundary-based
    {
      auto checkShapeEdges = [&](auto& container) -> bool {
        for (auto& shape : container) {
          if (!shape || !shape->isValid() || !shape->isVisible() || shape->isLocked()) continue;
          auto edges = shape->getEdges();
          for (const auto& edge : edges) {
            Point_2 proj;
            double relPos;
            // Use standard tolerance
            double dist = PointUtils::projectPointOntoSegment(editor.toCGALPoint(worldPos_sfml), edge, proj, relPos);

            // Compare squared distance for efficiency? PointUtils returns double distance.
            if (dist < tolerance) {
              std::cout << "Found Shape Edge (Priority 3.5)" << std::endl;
              potentialSelection = shape.get();
              potentialDragMode = DragMode::TranslateShape;
              return true;
            }
          }
        }
        return false;
      };

      if (checkShapeEdges(editor.rectangles) || checkShapeEdges(editor.polygons) || checkShapeEdges(editor.regularPolygons) ||
          checkShapeEdges(editor.triangles)) {
        goto objectFoundLabel;
      }
    }

    // --- PASS 3: LOWEST PRIORITY - INTERIORS / BODIES ---
    // Only if nothing else was hit.
    {
      // 6. Circles (Center then Body)
      for (auto& circlePtr : editor.circles) {
        if (!circlePtr || !circlePtr->isValid() || !circlePtr->isVisible() || circlePtr->isLocked()) continue;
        // Center check (already covered by Point check if center is a Point obj, but Point might be hidden)
        float centerTolerance = tolerance * 4.0f;
        if (circlePtr->isCenterPointHovered(worldPos_sfml, centerTolerance)) {
          potentialSelection = circlePtr.get();
          potentialDragMode = DragMode::InteractWithCircle;  // Needs specific mode?
          editor.dragStart_sfml = worldPos_sfml;
          // Special case: if we want to move center, maybe drag mode TranslateShape?
          // But logic elsewhere handles InteractWithCircle
          goto objectFoundLabel;
        } else if (circlePtr->contains(worldPos_sfml, tolerance)) {
          potentialSelection = circlePtr.get();
          potentialDragMode = DragMode::InteractWithCircle;
          editor.dragStart_sfml = worldPos_sfml;
          goto objectFoundLabel;
        }
      }

      // 7. Shape Interiors (Global Contains)
      auto checkShapeContains = [&](auto& container) -> bool {
        for (auto& ptr : container) {
          if (!ptr || !ptr->isValid() || !ptr->isVisible() || ptr->isLocked()) continue;
          if (ptr->contains(worldPos_sfml, tolerance)) {
            std::cout << "Found Shape Interior (Priority 4)" << std::endl;
            potentialSelection = ptr.get();
            potentialDragMode = DragMode::TranslateShape;
            return true;
          }
        }
        return false;
      };

      if (checkShapeContains(editor.rectangles) || checkShapeContains(editor.polygons) || checkShapeContains(editor.regularPolygons) ||
          checkShapeContains(editor.triangles)) {
        goto objectFoundLabel;
      }
    }

    if (potentialSelection) {
      // We found an object under the cursor
      std::cout << "Object found! Type: " << static_cast<int>(potentialSelection->getType()) << std::endl;

      // Additive Selection (Ctrl)
      bool isCtrlHeld = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl);

      if (potentialSelection) {
        if (isCtrlHeld) {
          auto it = std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), potentialSelection);
          if (it != editor.selectedObjects.end()) {
            potentialSelection->setSelected(false);
            editor.selectedObjects.erase(it);
            if (editor.selectedObject == potentialSelection) {
              editor.selectedObject = nullptr;
            }
          } else {
            potentialSelection->setSelected(true);
            editor.selectedObjects.push_back(potentialSelection);
            editor.selectedObject = potentialSelection;
          }
        } else {
          auto it = std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), potentialSelection);
          if (it == editor.selectedObjects.end()) {
            deselectAllAndClearInteractionState(editor);
            editor.selectedObject = potentialSelection;
            editor.selectedObjects.push_back(potentialSelection);
            potentialSelection->setSelected(true);
          } else {
            editor.selectedObject = potentialSelection;
          }
        }
      }

      if (editor.selectedObject) {
        try {
          std::cout << "Setting object as selected" << std::endl;
          editor.selectedObject->setSelected(true);
          sf::Color selectedColor = editor.selectedObject->getColor();
          editor.setCurrentColor(selectedColor);
          editor.gui.setCurrentColor(selectedColor);
          if (auto& picker = editor.gui.getColorPicker()) {
            picker->setCurrentColor(selectedColor);
          }
        } catch (const std::exception& e) {
          std::cerr << "Error setting selection state: " << e.what() << std::endl;
        }
      }

      if (editor.selectedObject && editor.selectedObject->isLocked()) {
        editor.dragMode = DragMode::None;
        editor.m_selectedEndpoint = EndpointSelection::None;
        editor.isDragging = false;
        editor.lastMousePos_sfml = worldPos_sfml;
        editor.dragStart_sfml = worldPos_sfml;
        return;
      }

      // Configure drag mode
      editor.dragMode = potentialDragMode;
      editor.m_selectedEndpoint = potentialEndpointSelection;
      editor.isDragging = true;
      editor.lastMousePos_sfml = worldPos_sfml;
      editor.dragStart_sfml = worldPos_sfml;

      std::cout << "DEBUG: Before MoveShapeVertex check - potentialDragMode=" << static_cast<int>(potentialDragMode) << " (6=MoveShapeVertex)"
                << std::endl;
      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        std::cout << "DEBUG handleMousePress: Set activeVertexShape=" << editor.activeVertexShape << " activeVertexIndex=" << editor.activeVertexIndex
                  << std::endl;
        if (auto* rect = dynamic_cast<Rectangle*>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto* poly = dynamic_cast<Polygon*>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto* reg = dynamic_cast<RegularPolygon*>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        } else if (auto* tri = dynamic_cast<Triangle*>(potentialSelection)) {
          tri->setActiveVertex(potentialVertexIndex);
        }
      }

      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        if (auto* rect = dynamic_cast<Rectangle*>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto* poly = dynamic_cast<Polygon*>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto* reg = dynamic_cast<RegularPolygon*>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        } else if (auto* tri = dynamic_cast<Triangle*>(potentialSelection)) {
          tri->setActiveVertex(potentialVertexIndex);
        }
      }

      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        if (auto* rect = dynamic_cast<Rectangle*>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto* poly = dynamic_cast<Polygon*>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto* reg = dynamic_cast<RegularPolygon*>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        }
      }

      std::cout << "Object selected with drag mode: " << static_cast<int>(editor.dragMode) << std::endl;
    } else {
      // No object found under cursor -> deselect all
      deselectAllAndClearInteractionState(editor);
      editor.selectedObject = nullptr;
      editor.potentialSelectionBoxStart_sfml = worldPos_sfml;
      editor.isDrawingSelectionBox = false;
      return;
    }
  objectFoundLabel:  // Label for goto
    if (potentialSelection) {
      // We found an object under the cursor
      std::cout << "Object found! Type: " << static_cast<int>(potentialSelection->getType()) << std::endl;

      bool isCtrlHeld = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl);

      if (potentialSelection) {
        if (isCtrlHeld) {
          auto it = std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), potentialSelection);
          if (it != editor.selectedObjects.end()) {
            potentialSelection->setSelected(false);
            editor.selectedObjects.erase(it);
            if (editor.selectedObject == potentialSelection) {
              editor.selectedObject = nullptr;
            }
          } else {
            potentialSelection->setSelected(true);
            editor.selectedObjects.push_back(potentialSelection);
            editor.selectedObject = potentialSelection;
          }
        } else {
          auto it = std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), potentialSelection);
          if (it == editor.selectedObjects.end()) {
            deselectAllAndClearInteractionState(editor);
            editor.selectedObject = potentialSelection;
            editor.selectedObjects.push_back(potentialSelection);
            potentialSelection->setSelected(true);
          } else {
            editor.selectedObject = potentialSelection;
          }
        }
      }

      editor.dragMode = potentialDragMode;
      editor.m_selectedEndpoint = potentialEndpointSelection;
      editor.isDragging = true;
      editor.lastMousePos_sfml = worldPos_sfml;
      editor.dragStart_sfml = worldPos_sfml;

      if (editor.selectedObject && editor.selectedObject->isLocked()) {
        editor.dragMode = DragMode::None;
        editor.m_selectedEndpoint = EndpointSelection::None;
        editor.isDragging = false;
        editor.lastMousePos_sfml = worldPos_sfml;
        editor.dragStart_sfml = worldPos_sfml;
        return;
      }

      // Set vertex state for MoveShapeVertex mode (CRITICAL FIX)
      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        std::cout << "DEBUG objectFoundLabel: Set activeVertexShape=" << editor.activeVertexShape << " activeVertexIndex=" << editor.activeVertexIndex
                  << std::endl;
        // Set the active vertex visual on the shape
        if (auto* rect = dynamic_cast<Rectangle*>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto* poly = dynamic_cast<Polygon*>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto* reg = dynamic_cast<RegularPolygon*>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        } else if (auto* tri = dynamic_cast<Triangle*>(potentialSelection)) {
          tri->setActiveVertex(potentialVertexIndex);
        }
      }

      std::cout << "Object selected with drag mode: " << static_cast<int>(editor.dragMode) << std::endl;
      if (editor.dragMode == DragMode::MoveLineEndpointStart || editor.dragMode == DragMode::MoveLineEndpointEnd) {
        std::cout << "Dragging Line Endpoint. Selected Line: " << editor.selectedObject << std::endl;
      } else if (editor.dragMode == DragMode::MoveFreePoint) {
        std::cout << "Dragging Free Point. Selected Point: " << editor.selectedObject << std::endl;
      }

    } else {
      // No object found under cursor -> deselect all
      deselectAllAndClearInteractionState(editor);
      editor.selectedObject = nullptr;
      editor.potentialSelectionBoxStart_sfml = worldPos_sfml;
      editor.isDrawingSelectionBox = false;  // Ensure this is reset
      return;
    }
    return;  // Crucial: exit after handling the "Move" tool click
  }

  // Handle intersection mode
  // ...existing code for intersection mode...

  editor.lastMousePos_sfml = worldPos_sfml;  // Update last mouse pos
}
void handleKeyPress(GeometryEditor& editor, const sf::Event::KeyEvent& keyEvent) {
  // Handle key press events
  if (keyEvent.code == sf::Keyboard::Home) {
    editor.resetView();
    editor.setGUIMessage("View Reset");
    return;
  }
  if (keyEvent.code == sf::Keyboard::R) {
    if (!editor.isRenaming && editor.selectedObject &&
        (editor.selectedObject->getType() == ObjectType::Point || editor.selectedObject->getType() == ObjectType::ObjectPoint ||
         editor.selectedObject->getType() == ObjectType::IntersectionPoint)) {
      auto sharedObj = editor.findSharedPtr(editor.selectedObject);
      if (sharedObj) {
        auto pt = std::dynamic_pointer_cast<Point>(sharedObj);
        if (pt) {
          editor.isRenaming = true;
          editor.pointToRename = pt;
          editor.renameBuffer = pt->getLabel();
          // If label is empty (rare), set default? No, empty buffer is fine.
          editor.setGUIMessage("Renaming: " + editor.renameBuffer);
          return;
        }
      }
    }
  }

  // Font Size Adjustment (hold F and press +/-)
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::F)) {
    bool sizeChanged = false;
    if (keyEvent.code == sf::Keyboard::Equal || keyEvent.code == sf::Keyboard::Add) {
      Constants::BUTTON_TEXT_SIZE += 1;
      Constants::GUI_MESSAGE_TEXT_SIZE += 1;
      sizeChanged = true;
    } else if (keyEvent.code == sf::Keyboard::Dash || keyEvent.code == sf::Keyboard::Subtract) {
      if (Constants::BUTTON_TEXT_SIZE > 8) {
        Constants::BUTTON_TEXT_SIZE -= 1;
        Constants::GUI_MESSAGE_TEXT_SIZE -= 1;
        sizeChanged = true;
      }
    }

    if (sizeChanged) {
      editor.getGUI().updateFontSizes();
      editor.setGUIMessage("Font Size updated: " + std::to_string(Constants::BUTTON_TEXT_SIZE));
      return;
    }
  }

  if (keyEvent.control && keyEvent.code == sf::Keyboard::Z) {
    if (keyEvent.shift) {
      editor.commandManager.redo();
    } else {
      editor.commandManager.undo();
    }
    return;
  }
  if (keyEvent.control && keyEvent.code == sf::Keyboard::Y) {
    editor.commandManager.redo();
    return;
  }
  if (keyEvent.code == sf::Keyboard::Escape) {
    // Close color picker if it's open
    auto& colorPickerPtr = editor.gui.getColorPicker();
    if (colorPickerPtr && colorPickerPtr->isOpen()) {
      colorPickerPtr->setOpen(false);
      editor.setGUIMessage("Color selection canceled.");
      return;
    }

    editor.cancelOperation();  // This should internally call the specific
    // tool resets if needed or call them
    // explicitly here.
    editor.isCreatingCircle = false;
    editor.previewCircle.reset();
    if (editor.lineCreationPoint1) {
      try {
        editor.lineCreationPoint1->setSelected(false);
      } catch (const std::exception& e) {
        std::cerr << "Error deselecting lineCreationPoint1 on Esc: " << e.what() << std::endl;
      }
      editor.lineCreationPoint1 = nullptr;
    }
    editor.dragMode = DragMode::None;

    // Use GeometryEditor's state reset methods
    editor.resetParallelLineToolState();
    editor.resetPerpendicularLineToolState();

    editor.setGUIMessage("Operation Canceled. Current tool: " + editor.getCurrentToolName());
    std::cout << "Operation canceled with Escape key" << std::endl;
  } else if (keyEvent.code == sf::Keyboard::Enter) {
    if (editor.isCreatingPolygon) {
      if (editor.polygonVertices.size() >= 3) {
        std::vector<std::shared_ptr<Point>> finalPoints;
        finalPoints.reserve(editor.polygonVertexPoints.size());
        for (size_t i = 0; i < editor.polygonVertexPoints.size(); ++i) {
          if (editor.polygonVertexPoints[i]) {
            finalPoints.push_back(editor.polygonVertexPoints[i]);
          } else {
            finalPoints.push_back(editor.createPoint(editor.polygonVertices[i]));
          }
        }
        for (size_t i = 0; i < finalPoints.size(); ++i) {
          if (finalPoints[i]) {
            finalPoints[i]->setLabel("P" + std::to_string(i + 1));
          }
        }
        auto newPoly = std::make_shared<Polygon>(finalPoints, editor.getCurrentColor(), editor.objectIdCounter++);
        editor.polygons.push_back(newPoly);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, std::static_pointer_cast<GeometricObject>(newPoly)));
        editor.isCreatingPolygon = false;
        editor.polygonVertices.clear();
        editor.polygonVertexPoints.clear();
        editor.previewPolygon.reset();
        editor.setGUIMessage("Polygon created via Enter key.");
        std::cout << "Polygon created via Enter key." << std::endl;
      } else {
        editor.setGUIMessage("Cannot create polygon: Need at least 3 vertices.");
      }
    }
  } else if (keyEvent.code == sf::Keyboard::Delete) {
    // Collect ALL selected objects for Undo-able deletion
    std::vector<std::shared_ptr<GeometricObject>> objectsToDelete;

    auto collectSelected = [&](const auto& container) {
      for (const auto& obj : container) {
        if (obj && obj->isSelected() && !obj->isLocked()) {
          objectsToDelete.push_back(obj);
        }
      }
    };

    collectSelected(editor.points);
    collectSelected(editor.lines);
    collectSelected(editor.circles);
    collectSelected(editor.ObjectPoints);
    collectSelected(editor.rectangles);
    collectSelected(editor.polygons);
    collectSelected(editor.regularPolygons);
    collectSelected(editor.triangles);
    collectSelected(editor.angles);

    auto addToDelete = [&](const std::shared_ptr<GeometricObject>& obj) {
      if (!obj) return;
      if (std::find(objectsToDelete.begin(), objectsToDelete.end(), obj) == objectsToDelete.end()) {
        objectsToDelete.push_back(obj);
      }
    };

    // Collect dependent ObjectPoints (e.g., from deleted Rectangles)
    // Shapes own their ObjectPoints, so we must delete them too if the shape is deleted
    std::vector<std::shared_ptr<ObjectPoint>> dependents;
    for (auto& obj : objectsToDelete) {
      for (auto& op : editor.ObjectPoints) {
        if (op && op->getHostObject() == obj.get()) {
          // Avoid duplicates
          if (std::find(objectsToDelete.begin(), objectsToDelete.end(), op) == objectsToDelete.end()) {
            dependents.push_back(op);
          }
        }
      }
      auto rect = std::dynamic_pointer_cast<Rectangle>(obj);
      if (rect) {
        auto a = rect->getCorner1Point();
        auto c = rect->getCorner2Point();
        auto b = rect->getCornerBPoint();
        auto d = rect->getCornerDPoint();

        if (b && b->isDependent()) addToDelete(std::static_pointer_cast<GeometricObject>(b));
        if (d && d->isDependent()) addToDelete(std::static_pointer_cast<GeometricObject>(d));
        if (a && a->isCreatedWithShape()) addToDelete(std::static_pointer_cast<GeometricObject>(a));
        if (c && c->isCreatedWithShape()) addToDelete(std::static_pointer_cast<GeometricObject>(c));
      }
    }
    // Append dependents
    objectsToDelete.insert(objectsToDelete.end(), dependents.begin(), dependents.end());

    if (!objectsToDelete.empty()) {
      auto cmd = std::make_shared<DeleteCommand>(editor, objectsToDelete);
      editor.commandManager.execute(cmd);
      std::cout << "Executed Undo-able Delete for " << objectsToDelete.size() << " objects." << std::endl;
      editor.setGUIMessage("Deleted " + std::to_string(objectsToDelete.size()) + " object(s).");
    } else {
      std::cout << "Delete key pressed but nothing selected." << std::endl;
    }
    // Note: We bypass editor.deleteSelected() entirely now to ensure purely Command-based deletion
  } else if (keyEvent.code == sf::Keyboard::Space) {
    // Toggle grid visibility or other useful feature
    std::cout << "Space key pressed" << std::endl;
  } else if (keyEvent.code == sf::Keyboard::D && keyEvent.control) {
    editor.toggleDebugMode();
  } else if (keyEvent.code == sf::Keyboard::L) {
    // Toggle vertex label visibility
    VertexLabelManager::instance().toggleVisible();
    bool visible = VertexLabelManager::instance().isVisible();
    editor.setGUIMessage(visible ? "Vertex labels: ON" : "Vertex labels: OFF");
    std::cout << "Vertex labels toggled: " << (visible ? "ON" : "OFF") << std::endl;
  } else if (keyEvent.code == sf::Keyboard::H) {
    editor.showGlobalLabels = !editor.showGlobalLabels;
    editor.setGUIMessage(editor.showGlobalLabels ? "Labels: ON" : "Labels: OFF");
    std::cout << "Global labels toggled: " << (editor.showGlobalLabels ? "ON" : "OFF") << std::endl;
  }
  // Add other key handlers as needed
}

void handleMouseMove(GeometryEditor& editor, const sf::Event::MouseMoveEvent& moveEvent) {
  QUICK_PROFILE("handleMouseMove");

  if (editor.hoveredObject && !editor.objectExistsInAnyList(editor.hoveredObject)) {
    editor.hoveredObject = nullptr;
  }
  if (g_lastHoveredObject && !editor.objectExistsInAnyList(g_lastHoveredObject)) {
    g_lastHoveredObject = nullptr;
  }
  if (editor.selectedObject && !editor.objectExistsInAnyList(editor.selectedObject)) {
    editor.selectedObject = nullptr;
    editor.isDragging = false;
    editor.dragMode = DragMode::None;
    editor.isResizingAngle = false;  // Ensure flag is cleared if object is lost
  }

  sf::Vector2i pixelPos(moveEvent.x, moveEvent.y);
  sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

  // Dynamic hover tolerance (match selection/preview logic)
  double dynamicWorldTolerance = getDynamicSelectionTolerance(editor);

  // Angle Resizing Logic
  if (editor.isDragging && editor.isResizingAngle && editor.selectedObject && editor.selectedObject->getType() == ObjectType::Angle) {
    auto angle = dynamic_cast<Angle*>(editor.selectedObject);
    if (angle) {
      Point_2 vertexPos = angle->getCGALPosition();
      double dx = worldPos.x - CGAL::to_double(vertexPos.x());
      double dy = worldPos.y - CGAL::to_double(vertexPos.y());
      double newRadius = std::sqrt(dx * dx + dy * dy);

      // Clamp radius to reasonable values
      newRadius = std::max(10.0, std::min(newRadius, 2000.0));

      angle->setRadius(newRadius);
    }
    return;  // Consume event
  }

  if (editor.isDraggingLabel && editor.labelDragObject) {
    sf::Vector2f mouseScreenPos(static_cast<float>(moveEvent.x), static_cast<float>(moveEvent.y));
    if (auto* pt = dynamic_cast<Point*>(editor.labelDragObject)) {
      sf::Vector2f worldPos = pt->getSFMLPosition();
      sf::Vector2i pointScreen = editor.window.mapCoordsToPixel(worldPos, editor.drawingView);
      sf::Vector2f pointScreenPos(static_cast<float>(pointScreen.x), static_cast<float>(pointScreen.y));
      sf::Vector2f labelTopLeft = mouseScreenPos - editor.labelDragGrabOffset;
      sf::Vector2f newOffset = labelTopLeft - pointScreenPos;
      pt->setLabelOffset(newOffset);
    }
    return;
  }
  sf::Vector2f guiPos = editor.window.mapPixelToCoords(pixelPos, editor.guiView);

  // Grid snapping (Shift): adjust world position before further processing
  bool gridSnapActive = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
  if (gridSnapActive) {
    float g = Constants::GRID_SIZE;
    worldPos.x = std::round(worldPos.x / g) * g;
    worldPos.y = std::round(worldPos.y / g) * g;
  }

  // Snapping Status Feedback (Alt)
  static bool wasSnapping = false;
  bool isSnapping = sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) || sf::Keyboard::isKeyPressed(sf::Keyboard::RAlt);
  if (isSnapping != wasSnapping) {
    if (isSnapping)
      editor.setGUIMessage("Vertex Snapping: Enabled");
    else
      editor.setGUIMessage("Vertex Snapping: Disabled");
    wasSnapping = isSnapping;
  }

  // Add bounds checking FIRST
  const float MAX_WORLD_COORD = 1e10f;
  if (!std::isfinite(worldPos.x) || !std::isfinite(worldPos.y) || std::abs(worldPos.x) > MAX_WORLD_COORD || std::abs(worldPos.y) > MAX_WORLD_COORD) {
    return;  // Skip processing invalid coordinates
  }

  Point_2 cgalWorldPos;
  try {
    cgalWorldPos = Point_2(worldPos.x, worldPos.y);
    if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error in CGAL point conversion: " << e.what() << std::endl;
    return;
  }

  static bool justFinishedDrag = false;
  static bool hasSetDeferredThisDrag = false;

  // Get the current zoom factor for point construction
  float currentActualZoomFactor = 1.0f;
  try {
    if (Constants::WINDOW_HEIGHT > 0) {
      currentActualZoomFactor = editor.drawingView.getSize().y / static_cast<float>(Constants::WINDOW_HEIGHT);
      if (currentActualZoomFactor <= 0) {
        currentActualZoomFactor = 1.0f;
      }
    }
  } catch (...) {
    currentActualZoomFactor = 1.0f;
  }

  // ✅ MINIMAL throttling for real-time preview responsiveness
  static sf::Clock previewUpdateClock;
  static sf::Vector2f lastPreviewPos(-999999, -999999);

  float moveDistance = std::sqrt(std::pow(worldPos.x - lastPreviewPos.x, 2) + std::pow(worldPos.y - lastPreviewPos.y, 2));
  sf::Int32 timeSinceLastPreview = previewUpdateClock.getElapsedTime().asMilliseconds();

  // Update preview every mouse move for overlay-based tools (GeoGebra-like)
  // Fallback to minimal gating for other modes
  bool shouldUpdatePreview = (moveDistance > 5.0f) || (timeSinceLastPreview > 16);
  if (editor.m_currentToolType == ObjectType::ParallelLine || editor.m_currentToolType == ObjectType::PerpendicularLine) {
    shouldUpdatePreview = true;
  }

  if (shouldUpdatePreview) {
    previewUpdateClock.restart();
    lastPreviewPos = worldPos;
  }

  // Safe cleanup of preview lines ONLY when the tool is not actively placing them
  try {
    const bool parallelPreviewActive = (editor.m_currentToolType == ObjectType::ParallelLine && editor.m_isPlacingParallel);
    const bool perpendicularPreviewActive = (editor.m_currentToolType == ObjectType::PerpendicularLine && editor.m_isPlacingPerpendicular);

    if (!parallelPreviewActive && editor.m_parallelPreviewLine) {
      try {
        editor.m_parallelPreviewLine->prepareForDestruction();
      } catch (...) {
        // Ignore errors during preview cleanup
      }
      editor.m_parallelPreviewLine.reset();
    }
    if (!perpendicularPreviewActive && editor.m_perpendicularPreviewLine) {
      try {
        editor.m_perpendicularPreviewLine->prepareForDestruction();
      } catch (...) {
        // Ignore errors during preview cleanup
      }
      editor.m_perpendicularPreviewLine.reset();
    }

    if (!parallelPreviewActive && !perpendicularPreviewActive) {
      editor.hasPreviewLineOverlay = false;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error cleaning preview lines: " << e.what() << std::endl;
    editor.m_parallelPreviewLine.reset();
    editor.m_perpendicularPreviewLine.reset();
  }

  editor.currentMousePos_sfml = worldPos;

  // === LINE TOOL VERTEX/EDGE SNAP PREVIEW ===
  // When Line Tool is active, highlight nearby vertices/edges to give visual feedback
  if (editor.m_currentToolType == ObjectType::Line || editor.m_currentToolType == ObjectType::LineSegment) {
    float tolerance = getDynamicSelectionTolerance(editor) * 2.0f;

    // Clear previous hover state
    if (editor.hoveredVertexShape) {
      if (auto* rect = dynamic_cast<Rectangle*>(editor.hoveredVertexShape)) rect->setHoveredVertex(-1);
      if (auto* poly = dynamic_cast<Polygon*>(editor.hoveredVertexShape)) poly->setHoveredVertex(-1);
      if (auto* reg = dynamic_cast<RegularPolygon*>(editor.hoveredVertexShape)) reg->setHoveredVertex(-1);
      if (auto* tri = dynamic_cast<Triangle*>(editor.hoveredVertexShape)) tri->setHoveredVertex(-1);
    }
    editor.hoveredVertexShape = nullptr;
    editor.hoveredVertexIndex = -1;

    // Try to find a vertex via PointUtils
    GeometricObject* outShape = nullptr;
    size_t outVertexIndex = 0;
    if (PointUtils::findShapeVertex(editor, worldPos, tolerance, outShape, outVertexIndex)) {
      // Found a shape vertex - highlight it
      editor.hoveredVertexShape = outShape;
      editor.hoveredVertexIndex = static_cast<int>(outVertexIndex);

      if (auto* rect = dynamic_cast<Rectangle*>(outShape))
        rect->setHoveredVertex(static_cast<int>(outVertexIndex));
      else if (auto* poly = dynamic_cast<Polygon*>(outShape))
        poly->setHoveredVertex(static_cast<int>(outVertexIndex));
      else if (auto* reg = dynamic_cast<RegularPolygon*>(outShape))
        reg->setHoveredVertex(static_cast<int>(outVertexIndex));
      else if (auto* tri = dynamic_cast<Triangle*>(outShape))
        tri->setHoveredVertex(static_cast<int>(outVertexIndex));

      std::cout << "[LINE TOOL] Hovering over shape vertex " << outVertexIndex << std::endl;
    } else {
      // Check for edge hover (for ObjPoint preview)
      auto edgeHit = PointUtils::findNearestEdge(editor, worldPos, tolerance);
      if (edgeHit.has_value() && edgeHit->host) {
        edgeHit->host->setHovered(true);
        std::cout << "[LINE TOOL] Hovering over shape edge " << edgeHit->edgeIndex << std::endl;
      }
    }
  }

  // MUCH SAFER preview creation with exception handling
  if (shouldUpdatePreview) {
    auto updatePreviewLine = [&](std::shared_ptr<Line>& previewLine, const Point_2& p1, const Point_2& p2) {
      if (!CGAL::is_finite(p1.x()) || !CGAL::is_finite(p1.y()) || !CGAL::is_finite(p2.x()) || !CGAL::is_finite(p2.y())) {
        return;
      }

      if (!previewLine || !previewLine->isValid()) {
        auto tempP1 = std::make_shared<Point>(p1, currentActualZoomFactor, Constants::PREVIEW_COLOR);
        auto tempP2 = std::make_shared<Point>(p2, currentActualZoomFactor, Constants::PREVIEW_COLOR);

        if (tempP1 && tempP1->isValid() && tempP2 && tempP2->isValid()) {
          previewLine = std::make_shared<Line>(tempP1, tempP2, false, Constants::PREVIEW_COLOR);
          if (previewLine && previewLine->isValid()) {
            previewLine->setTemporaryPreviewPoints(tempP1, tempP2);
          }
        }
      }

      if (previewLine && previewLine->isValid()) {
        if (auto* p1Obj = previewLine->getStartPointObject()) {
          p1Obj->setCGALPosition(p1);
        }
        if (auto* p2Obj = previewLine->getEndPointObject()) {
          p2Obj->setCGALPosition(p2);
        }
        previewLine->updateCGALLine();
        previewLine->updateSFMLShape();
      }
    };

    // Handle parallel line preview with persistent objects
    if (editor.m_currentToolType == ObjectType::ParallelLine && editor.m_isPlacingParallel) {
      try {
        editor.hasPreviewLineOverlay = false;

        Vector_2 currentRefVec = editor.m_parallelReferenceDirection;
        if (auto refObj = editor.m_parallelReference.lock()) {
          if (refObj->isValid()) {
            if (editor.m_parallelReference.edgeIndex == -1) {
              if (auto line = std::dynamic_pointer_cast<Line>(refObj)) {
                if (line->isValid()) {
                  currentRefVec = line->getDirection().vector();
                }
              }
            } else {
              auto edges = refObj->getEdges();
              if (editor.m_parallelReference.edgeIndex >= 0 && editor.m_parallelReference.edgeIndex < static_cast<int>(edges.size())) {
                currentRefVec = edges[editor.m_parallelReference.edgeIndex].to_vector();
              }
            }
          }
        }

        if (currentRefVec != Vector_2(0, 0)) {
          Vector_2 unitDir = CGALSafeUtils::normalize_vector_robust(currentRefVec, "ParallelPreview_normalize_ref_dir");
          Kernel::FT length = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
          Point_2 p1_preview = cgalWorldPos;
          Point_2 p2_preview = cgalWorldPos + (unitDir * length * 0.5);
          updatePreviewLine(editor.m_parallelPreviewLine, p1_preview, p2_preview);
        } else if (editor.m_parallelPreviewLine) {
          editor.m_parallelPreviewLine.reset();
        }
      } catch (const std::exception& e) {
        std::cerr << "Error in parallel preview update: " << e.what() << std::endl;
        editor.m_parallelPreviewLine.reset();
      }
    }

    // Handle perpendicular line preview with persistent objects
    if (editor.m_currentToolType == ObjectType::PerpendicularLine && editor.m_isPlacingPerpendicular) {
      try {
        editor.hasPreviewLineOverlay = false;

        Vector_2 currentRefVec = editor.m_perpendicularReferenceDirection;
        if (auto refObj = editor.m_perpendicularReference.lock()) {
          if (refObj->isValid()) {
            if (editor.m_perpendicularReference.edgeIndex == -1) {
              if (auto line = std::dynamic_pointer_cast<Line>(refObj)) {
                if (line->isValid()) {
                  currentRefVec = line->getDirection().vector();
                }
              }
            } else {
              auto edges = refObj->getEdges();
              if (editor.m_perpendicularReference.edgeIndex >= 0 && editor.m_perpendicularReference.edgeIndex < static_cast<int>(edges.size())) {
                currentRefVec = edges[editor.m_perpendicularReference.edgeIndex].to_vector();
              }
            }
          }
        }

        if (currentRefVec != Vector_2(0, 0)) {
          Vector_2 perpVec(-currentRefVec.y(), currentRefVec.x());
          Vector_2 unitDir = CGALSafeUtils::normalize_vector_robust(perpVec, "PerpendicularPreview_normalize_perp_dir");
          Kernel::FT length = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
          Point_2 p1_preview = cgalWorldPos;
          Point_2 p2_preview = cgalWorldPos + (unitDir * length * 0.5);
          updatePreviewLine(editor.m_perpendicularPreviewLine, p1_preview, p2_preview);
        } else if (editor.m_perpendicularPreviewLine) {
          editor.m_perpendicularPreviewLine.reset();
        }
      } catch (const std::exception& e) {
        std::cerr << "Error in perpendicular preview update: " << e.what() << std::endl;
        editor.m_perpendicularPreviewLine.reset();
      }
    }
  }

  // === UNIVERSAL SMART SNAPPING ===
  if (editor.m_currentToolType == ObjectType::Point || editor.m_currentToolType == ObjectType::Line ||
      editor.m_currentToolType == ObjectType::LineSegment || editor.m_currentToolType == ObjectType::Circle ||
      editor.m_currentToolType == ObjectType::Intersection || editor.m_currentToolType == ObjectType::Rectangle ||
      editor.m_currentToolType == ObjectType::RectangleRotatable || editor.m_currentToolType == ObjectType::Polygon ||
      editor.m_currentToolType == ObjectType::RegularPolygon || editor.m_currentToolType == ObjectType::Triangle) {
    float tolerance = getDynamicSelectionTolerance(editor);
    editor.m_snapState = PointUtils::checkSnapping(editor, worldPos, tolerance);

    // Apply snap to cursor position if snapping is active (Alt key)
    bool isSnapping = sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) || sf::Keyboard::isKeyPressed(sf::Keyboard::RAlt);

    if (isSnapping && editor.m_snapState.kind != PointUtils::SnapState::Kind::None) {
      cgalWorldPos = editor.m_snapState.position;
      // worldPos = editor.toSFMLVector(cgalWorldPos); // Optional: Sync SFML pos if needed
    }
  } else {
    editor.m_snapState = PointUtils::SnapState{};
  }

  // Update selection box if we're drawing one
  if (editor.isDrawingSelectionBox) {
    // NORMALIZED SIZE AND POSITION: handle drag in any direction
    float left = std::min(editor.selectionBoxStart_sfml.x, worldPos.x);
    float top = std::min(editor.selectionBoxStart_sfml.y, worldPos.y);
    float width = std::abs(worldPos.x - editor.selectionBoxStart_sfml.x);
    float height = std::abs(worldPos.y - editor.selectionBoxStart_sfml.y);
    editor.selectionBoxShape.setPosition(sf::Vector2f(left, top));
    editor.selectionBoxShape.setSize(sf::Vector2f(width, height));
    return;
  } else if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && !editor.isDragging &&  // Not dragging an existing object
             editor.m_currentToolType == ObjectType::None &&                       // No tool active
             !editor.isDrawingSelectionBox) {                                      // And not already drawing a
    // selection box

    // Check if the mouse has moved significantly from the initial press to
    // start selection box
    // Use centralized constant for drag threshold
    float minDragDistance = screenPixelsToWorldUnits(editor, Constants::DRAG_THRESHOLD_PIXELS);
    sf::Vector2f moveDelta = worldPos - editor.potentialSelectionBoxStart_sfml;

    if (std::abs(moveDelta.x) > minDragDistance || std::abs(moveDelta.y) > minDragDistance) {
      editor.isDrawingSelectionBox = true;
      editor.selectionBoxStart_sfml = editor.potentialSelectionBoxStart_sfml;  // Use the stored press
      // position
      editor.selectionBoxShape.setFillColor(Constants::SELECTION_BOX_FILL_COLOR);
      editor.selectionBoxShape.setOutlineColor(Constants::SELECTION_BOX_OUTLINE_COLOR);
      editor.selectionBoxShape.setOutlineThickness(Constants::SELECTION_BOX_OUTLINE_THICKNESS);
      std::cout << "Selection box drag started." << std::endl;

      // Update size immediately based on current mouse position (NORMALIZED)
      float left = std::min(editor.selectionBoxStart_sfml.x, worldPos.x);
      float top = std::min(editor.selectionBoxStart_sfml.y, worldPos.y);
      float width = std::abs(worldPos.x - editor.selectionBoxStart_sfml.x);
      float height = std::abs(worldPos.y - editor.selectionBoxStart_sfml.y);
      editor.selectionBoxShape.setPosition(sf::Vector2f(left, top));
      editor.selectionBoxShape.setSize(sf::Vector2f(width, height));
    }
  }

  // Handle panning with middle mouse button
  if (editor.isPanning) {
    sf::Vector2f delta = editor.lastMousePos_sfml - worldPos;
    editor.drawingView.move(delta);
    editor.window.setView(editor.drawingView);
    return;
  }

  // Handle preview circle during circle creation
  if (editor.isCreatingCircle && editor.previewCircle) {
    try {
      double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(editor.toCGALPoint(worldPos), editor.createStart_cgal)));
      editor.previewCircle->setRadius(dist);
    } catch (const std::exception& e) {
      std::cerr << "Error updating preview circle: " << e.what() << std::endl;
    }
  }
  // Handle preview triangle
  if (shouldUpdatePreview && editor.isCreatingTriangle && !editor.triangleVertices.empty()) {
    if (editor.triangleVertices.size() == 1) {
      editor.previewLineOverlay.setPrimitiveType(sf::Lines);
      editor.previewLineOverlay.resize(2);
      editor.previewLineOverlay[0].position = editor.toSFMLVector(editor.triangleVertices[0]);
      editor.previewLineOverlay[1].position = worldPos;
      editor.previewLineOverlay[0].color = editor.getCurrentColor();
      editor.previewLineOverlay[1].color = editor.getCurrentColor();
      editor.hasPreviewLineOverlay = true;
    } else if (editor.triangleVertices.size() == 2) {
      if (!editor.previewTriangle) {
        editor.previewTriangle =
            std::make_shared<Triangle>(editor.triangleVertices[0], editor.triangleVertices[1], cgalWorldPos, editor.getCurrentColor());
      } else {
        editor.previewTriangle->setVertexPosition(2, cgalWorldPos);
      }
    }
  }

  // Handle preview regular polygon
  if (shouldUpdatePreview && editor.isCreatingRegularPolygon && editor.regularPolygonPhase == 1) {
    if (!editor.previewRegularPolygon) {
      editor.previewRegularPolygon =
          std::make_shared<RegularPolygon>(editor.regularPolygonCenter, cgalWorldPos, editor.regularPolygonNumSides, editor.getCurrentColor());
    } else {
      editor.previewRegularPolygon->setVertexPosition(0, cgalWorldPos);
    }
  }

  // Handle preview polygon
  if (shouldUpdatePreview && editor.isCreatingPolygon && !editor.polygonVertices.empty()) {
    // Create a copy of existing vertices and add the current mouse position
    std::vector<Point_2> currentVerts = editor.polygonVertices;
    currentVerts.push_back(cgalWorldPos);

    // Recreate the preview polygon with the updated set of vertices
    editor.previewPolygon = std::make_shared<Polygon>(currentVerts, editor.getCurrentColor());
  }

  // Handle preview rectangle
  if (shouldUpdatePreview && editor.isCreatingRectangle && editor.previewRectangle) {
    // Update the rectangle so that it spans from the start corner (stored as corner1)
    // to the current mouse position
    editor.previewRectangle->setCorners(editor.rectangleCorner1, cgalWorldPos);
  }

  // Handle preview rotatable rectangle (recreate preview with current mouse as adjacent point)
  if (shouldUpdatePreview && editor.isCreatingRotatableRectangle) {
    try {
      double dx = CGAL::to_double(cgalWorldPos.x() - editor.rectangleCorner1.x());
      double dy = CGAL::to_double(cgalWorldPos.y() - editor.rectangleCorner1.y());
      double side = std::sqrt(dx * dx + dy * dy);
      double width = side;  // square-like preview for clarity
      editor.previewRectangle =
          std::make_shared<Rectangle>(editor.rectangleCorner1, cgalWorldPos, width, editor.getCurrentColor(), editor.objectIdCounter /* temp id */);
    } catch (const std::exception& e) {
      std::cerr << "Error updating rotatable rectangle preview: " << e.what() << std::endl;
      editor.previewRectangle.reset();
    }
  }

  // Handle dragging
  if (editor.isDragging) {
    if (editor.selectedObject && editor.selectedObject->isLocked()) {
      editor.dragMode = DragMode::None;
      editor.m_selectedEndpoint = EndpointSelection::None;
      editor.isDragging = false;
      return;
    }
    QUICK_PROFILE("DragOperations");
    // 1. Calculate Raw World Position from Mouse
    Point_2 targetPos = editor.toCGALPoint(worldPos);

    // 2a. ATOMIC TOPOLOGICAL TRANSLATION (Gather → Move → Update)
    if (editor.selectedObjects.size() > 1) {
      Point_2 currCGAL = editor.toCGALPoint(worldPos);
      Point_2 lastCGAL = editor.toCGALPoint(editor.lastMousePos_sfml);
      Vector_2 delta = currCGAL - lastCGAL;

      if (CGAL::to_double(delta.squared_length()) <= 1e-12) {
        return;
      }

      std::unordered_set<const Point*> pointsToMove;
      std::unordered_set<GeometricObject*> selectedObjectSet;
      std::vector<std::shared_ptr<Point>> uniquePoints;
      std::vector<Line*> constrainedLinesToRestore;

      for (auto* obj : editor.selectedObjects) {
        if (obj) selectedObjectSet.insert(obj);
      }

      auto addPoint = [&](const std::shared_ptr<Point>& p) {
        if (!p) return;
        if (auto op = std::dynamic_pointer_cast<ObjectPoint>(p)) {
          GeometricObject* host = op->getHostObject();
          if (host && selectedObjectSet.find(host) != selectedObjectSet.end()) {
            return;
          }
        }
        const Point* raw = p.get();
        if (pointsToMove.find(raw) == pointsToMove.end()) {
          pointsToMove.insert(raw);
          uniquePoints.push_back(p);
        }
      };

      for (auto* obj : editor.selectedObjects) {
        if (!obj) continue;

        switch (obj->getType()) {
          case ObjectType::Point:
          case ObjectType::ObjectPoint: {
            auto sharedObj = editor.findSharedPtr(obj);
            if (sharedObj) addPoint(std::static_pointer_cast<Point>(sharedObj));
            break;
          }
          case ObjectType::Line:
          case ObjectType::LineSegment: {
            auto line = static_cast<Line*>(obj);
            line->setIsUnderDirectManipulation(true);
            constrainedLinesToRestore.push_back(line);
            addPoint(line->getStartPointObjectShared());
            addPoint(line->getEndPointObjectShared());
            break;
          }
          case ObjectType::Rectangle:
          case ObjectType::RectangleRotatable: {
            auto rect = static_cast<Rectangle*>(obj);
            addPoint(rect->getCorner1Point());
            addPoint(rect->getCorner2Point());
            break;
          }
          case ObjectType::Triangle: {
            auto tri = static_cast<Triangle*>(obj);
            addPoint(tri->getVertexPoint(0));
            addPoint(tri->getVertexPoint(1));
            addPoint(tri->getVertexPoint(2));
            break;
          }
          case ObjectType::Polygon: {
            auto poly = static_cast<Polygon*>(obj);
            size_t count = poly->getVertexCount();
            for (size_t i = 0; i < count; ++i) {
              addPoint(poly->getVertexPoint(i));
            }
            break;
          }
          case ObjectType::RegularPolygon: {
            auto regPoly = static_cast<RegularPolygon*>(obj);
            addPoint(regPoly->getCenterPoint());
            addPoint(regPoly->getFirstVertexPoint());
            break;
          }
          case ObjectType::Circle: {
            auto circle = static_cast<Circle*>(obj);
            if (circle) {
              if (circle->getCenterPointObject()) {
                auto sharedCenter = editor.findSharedPtr(circle->getCenterPointObject());
                if (sharedCenter) addPoint(std::static_pointer_cast<Point>(sharedCenter));
              }
              if (circle->getRadiusPointObject()) {
                auto sharedRadius = editor.findSharedPtr(circle->getRadiusPointObject());
                if (sharedRadius) addPoint(std::static_pointer_cast<Point>(sharedRadius));
              }
            }
            break;
          }
          default:
            break;
        }
      }

      for (auto& pt : uniquePoints) {
        pt->translate(delta);
      }

      for (auto* obj : editor.selectedObjects) {
        if (obj) obj->update();
      }

      for (auto* line : constrainedLinesToRestore) {
        if (line) line->setIsUnderDirectManipulation(false);
      }

      editor.lastMousePos_sfml = worldPos;
      return;
    }

    // 2. SNAP LOGIC (Must happen BEFORE applying position)
    if ((sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) || sf::Keyboard::isKeyPressed(sf::Keyboard::RAlt)) &&
        (editor.dragMode == DragMode::MoveFreePoint || editor.dragMode == DragMode::MoveLineEndpointStart ||
         editor.dragMode == DragMode::MoveLineEndpointEnd || editor.dragMode == DragMode::DragObjectPoint)) {  // Added DragObjectPoint support
      // std::cout << "Ctrl held. Searching for snap..." << std::endl;

      double bestDistSq = 1000.0;
      bool foundSnap = false;
      Point_2 snapLocation;
      std::shared_ptr<Point> bestSnapPoint;

      Point* draggedPointRaw = nullptr;
      if (editor.dragMode == DragMode::MoveFreePoint && editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
        draggedPointRaw = static_cast<Point*>(editor.selectedObject);
      } else if ((editor.dragMode == DragMode::MoveLineEndpointStart || editor.dragMode == DragMode::MoveLineEndpointEnd) && editor.selectedObject &&
                 (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
        Line* selectedLine = static_cast<Line*>(editor.selectedObject);
        draggedPointRaw =
            (editor.dragMode == DragMode::MoveLineEndpointStart) ? selectedLine->getStartPointObject() : selectedLine->getEndPointObject();
      }

      // A. Check Independent Points
      for (const auto& pt : editor.points) {
        if (!pt || !pt->isValid() || !pt->isVisible()) continue;
        if (pt.get() == draggedPointRaw) continue;
        double d = CGAL::to_double(CGAL::squared_distance(targetPos, pt->getCGALPosition()));
        if (d < bestDistSq) {
          bestDistSq = d;
          snapLocation = pt->getCGALPosition();
          bestSnapPoint = pt;
          foundSnap = true;
        }
      }

      // B. Check Line Endpoints (CRITICAL)
      for (const auto& line : editor.lines) {
        if (!line || !line->isValid() || !line->isVisible()) continue;
        auto startPoint = line->getStartPointObjectShared();
        auto endPoint = line->getEndPointObjectShared();

        if (startPoint && startPoint.get() != draggedPointRaw) {
          double d1 = CGAL::to_double(CGAL::squared_distance(targetPos, startPoint->getCGALPosition()));
          if (d1 < bestDistSq) {
            bestDistSq = d1;
            snapLocation = startPoint->getCGALPosition();
            bestSnapPoint = startPoint;
            foundSnap = true;
          }
        }

        if (endPoint && endPoint.get() != draggedPointRaw) {
          double d2 = CGAL::to_double(CGAL::squared_distance(targetPos, endPoint->getCGALPosition()));
          if (d2 < bestDistSq) {
            bestDistSq = d2;
            snapLocation = endPoint->getCGALPosition();
            bestSnapPoint = endPoint;
            foundSnap = true;
          }
        }
      }

      // C. APPLY SNAP
      if (foundSnap) {
        // std::cout << "SNAP! Adjusted pos to: " << snapLocation << std::endl;
        targetPos = snapLocation;
        worldPos = editor.toSFMLVector(targetPos);
      }

      editor.m_wasSnapped = foundSnap;
      editor.m_snapTargetPoint = foundSnap ? bestSnapPoint : nullptr;
    } else {
      editor.m_wasSnapped = false;
      editor.m_snapTargetPoint = nullptr;
    }

    if (!hasSetDeferredThisDrag) {
      std::cout << "SETTING UP SFML DEFERRING for " << editor.lines.size() << " lines" << std::endl;

      // Get currently manipulated line to exclude from deferring
      Line* currentlyManipulatedLine = nullptr;
      if (editor.selectedObject &&
          (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
        currentlyManipulatedLine = static_cast<Line*>(editor.selectedObject);
      }

      // Defer SFML updates for all lines EXCEPT the one being manipulated
      for (auto& linePtr : editor.lines) {
        if (linePtr && linePtr->isValid() && linePtr.get() != currentlyManipulatedLine) {
          std::cout << "  NOT deferring SFML updates - keeping real-time updates enabled" << std::endl;
          // REMOVED: linePtr->setDeferSFMLUpdates(true); - keep updates real-time
        }
      }
      if (editor.dragMode == DragMode::MoveFreePoint) {
        for (auto& linePtr : editor.lines) {
          if (linePtr && linePtr->isValid()) {
            // REMOVED deferring - enable real-time updates during point drag
            linePtr->setDeferSFMLUpdates(false);  // Enable immediate visual updates
          }
        }
      }

      // DEFER CONSTRAINT UPDATES FOR POINTS DURING LINE TRANSLATION
      if (editor.dragMode == DragMode::TranslateLine && currentlyManipulatedLine) {
        Point* startPoint = currentlyManipulatedLine->getStartPointObject();
        Point* endPoint = currentlyManipulatedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
      }

      // DEFER CONSTRAINT UPDATES FOR ENDPOINT DRAGGING
      if (editor.dragMode == DragMode::MoveLineEndpointStart || editor.dragMode == DragMode::MoveLineEndpointEnd) {
        for (auto& pointPtr : editor.points) {
          if (pointPtr && pointPtr->isValid()) {
            pointPtr->setDeferConstraintUpdates(false);  // Real-time updates
          }
        }
      }

      // REAL-TIME UPDATES: Don't defer constraint updates for dragged points
      if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
        Point* selectedPoint = static_cast<Point*>(editor.selectedObject);
        selectedPoint->setDeferConstraintUpdates(false);  // Keep real-time updates
      }

      hasSetDeferredThisDrag = true;
      std::cout << "SFML DEFERRING SETUP COMPLETE" << std::endl;
    }
    try {
      if (editor.dragMode == DragMode::MoveFreePoint) {
        // Cast to Point and update position
        if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
          Point* selectedPoint = static_cast<Point*>(editor.selectedObject);
          if (selectedPoint->isIntersectionPoint()) {
            return;
          }
          // Route rectangle corner points through rectangle constraint logic
          for (auto& rectPtr : editor.rectangles) {
            if (!rectPtr || !rectPtr->isValid()) continue;
            if (rectPtr->isRotatable()) continue;
            auto a = rectPtr->getCorner1Point();
            auto b = rectPtr->getCorner2Point();
            auto c = rectPtr->getCornerBPoint();
            auto d = rectPtr->getCornerDPoint();

            if (a && a.get() == selectedPoint) {
              if (rectPtr->isRotatable()) {
                rectPtr->setCorner1Position(targetPos);
                rectPtr->update();
              } else {
                rectPtr->setVertexPosition(0, targetPos);
              }
              return;
            }
            if (b && b.get() == selectedPoint) {
              if (rectPtr->isRotatable()) {
                rectPtr->setCorner2Position(targetPos);
                rectPtr->update();
              } else {
                rectPtr->setVertexPosition(2, targetPos);
              }
              return;
            }
            if (c && c.get() == selectedPoint) {
              if (rectPtr->isRotatable()) {
                c->setCGALPosition(targetPos);
                rectPtr->update();
              } else {
                rectPtr->setVertexPosition(1, targetPos);
              }
              return;
            }
            if (d && d.get() == selectedPoint) {
              if (rectPtr->isRotatable()) {
                Point_2 aPos = rectPtr->getCorner1Position();
                Point_2 bPos = rectPtr->getCorner2Position();
                Vector_2 heightVec = targetPos - aPos;
                Point_2 newC = bPos + heightVec;
                if (c) c->setCGALPosition(newC);
                rectPtr->update();
              } else {
                rectPtr->setVertexPosition(3, targetPos);
              }
              return;
            }
          }
          if (selectedPoint->isDependent()) {
            return;
          }
          selectedPoint->setCGALPosition(targetPos);
          selectedPoint->update();
          selectedPoint->setDeferConstraintUpdates(false);  // Enable real-time updates
          std::cout << "Point position updated to: (" << worldPos.x << ", " << worldPos.y << ")" << std::endl;

          // Update any circles that use this point as their center or radius point
          for (auto& circlePtr : editor.circles) {
            if (circlePtr && circlePtr->isValid()) {
              // Check if this circle's center Point is the selected point (by pointer)
              if (circlePtr->getCenterPointObject() == selectedPoint || circlePtr->getRadiusPointObject() == selectedPoint) {
                circlePtr->update();  // Trigger circle to re-render in real-time
              }
            }
          }
        }
      } else if (editor.dragMode == DragMode::DragObjectPoint) {
        // Cast to ObjectPoint and handle movement
        if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::ObjectPoint) {
          ObjectPoint* selectedObjPoint = static_cast<ObjectPoint*>(editor.selectedObject);
          // Use the snapped 'targetPos' converted back to SFML world coords
          // 'worldPos' is already updated to snapped position in Step 2C if snap occurred
          selectedObjPoint->updateFromMousePos(worldPos);
        }
      } else if (editor.dragMode == DragMode::TranslateLine) {
        // Cast to Line and handle translation
        if (editor.selectedObject &&
            (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
          Line* selectedLine = static_cast<Line*>(editor.selectedObject);

          Point* startPoint = selectedLine->getStartPointObject();
          Point* endPoint = selectedLine->getEndPointObject();

          if (dynamic_cast<ObjectPoint*>(startPoint) || dynamic_cast<ObjectPoint*>(endPoint)) {
            return;
          }

          selectedLine->setIsUnderDirectManipulation(true);

          sf::Vector2f delta_sfml = worldPos - editor.lastMousePos_sfml;
          Vector_2 cgal_Delta = editor.toCGALVector(delta_sfml);

          // Keep constraint updates real-time during translation
          if (startPoint) {
            startPoint->setDeferConstraintUpdates(false);  // Real-time updates
          }
          if (endPoint) {
            endPoint->setDeferConstraintUpdates(false);  // Real-time updates
          }

          if (Constants::DEBUG_CGAL_POINT) {
            std::cout << "HandleEvents (Line Drag): sfml_delta: (" << delta_sfml.x << ", " << delta_sfml.y << ")" << std::endl;
            std::cout << "HandleEvents (Line Drag): cgal_delta before translate: (" << CGAL::to_double(cgal_Delta.x()) << ", "
                      << CGAL::to_double(cgal_Delta.y()) << ")" << std::endl;
            std::cout << "  cgal_delta.x finite: " << CGAL::is_finite(cgal_Delta.x()) << ", cgal_delta.y finite: " << CGAL::is_finite(cgal_Delta.y())
                      << std::endl;
            if (!CGAL::is_finite(cgal_Delta.x()) || !CGAL::is_finite(cgal_Delta.y())) {
              std::cerr << "  CRITICAL: cgal_delta is NON-FINITE before "
                           "calling translateWithDependents!"
                        << std::endl;
            }
          }
          selectedLine->translateWithDependents(cgal_Delta);

          // ✅ FORCE IMMEDIATE LINE VISUAL UPDATE ONLY
          selectedLine->setDeferSFMLUpdates(false);
          selectedLine->updateSFMLShape();
        }
      } else if (editor.dragMode == DragMode::MoveLineEndpointStart || editor.dragMode == DragMode::MoveLineEndpointEnd) {
        // Cast to Line and handle endpoint movement
        if (editor.selectedObject &&
            (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
          Line* selectedLine = static_cast<Line*>(editor.selectedObject);

          if (editor.dragMode == DragMode::MoveLineEndpointStart) {
            Point* startPoint = selectedLine->getStartPointObject();
            if (startPoint) {
              // ✅ ENABLE REAL-TIME UPDATES
              startPoint->setDeferConstraintUpdates(false);  // Real-time updates
              startPoint->setCGALPosition(targetPos);

              // ✅ FORCE IMMEDIATE LINE VISUAL UPDATE ONLY
              selectedLine->setDeferSFMLUpdates(false);
              // selectedLine->updateSFMLShape();
              selectedLine->update();
            }
          } else {  // MoveLineEndpointEnd
            Point* endPoint = selectedLine->getEndPointObject();
            if (endPoint) {
              // ✅ ENABLE REAL-TIME UPDATES
              endPoint->setDeferConstraintUpdates(false);  // Real-time updates
              endPoint->setCGALPosition(targetPos);

              // ✅ FORCE IMMEDIATE LINE VISUAL UPDATE ONLY
              selectedLine->setDeferSFMLUpdates(false);
              // selectedLine->updateSFMLShape();
              selectedLine->update();
            }
          }
        }
      } else if (editor.dragMode == DragMode::MoveShapeVertex) {
        std::cout << "DEBUG MoveShapeVertex: activeVertexShape=" << editor.activeVertexShape << " activeVertexIndex=" << editor.activeVertexIndex
                  << std::endl;
        if (editor.activeVertexShape && editor.activeVertexIndex >= 0) {
          Point_2 newPos = editor.toCGALPoint(worldPos);
          std::cout << "DEBUG MoveShapeVertex: Moving vertex " << editor.activeVertexIndex << " to (" << CGAL::to_double(newPos.x()) << ","
                    << CGAL::to_double(newPos.y()) << ")" << std::endl;
          switch (editor.activeVertexShape->getType()) {
            case ObjectType::Rectangle:
            case ObjectType::RectangleRotatable: {
              auto* rect = static_cast<Rectangle*>(editor.activeVertexShape);
              std::cout << "DEBUG MoveShapeVertex: Calling rect->setVertexPosition" << std::endl;
              rect->setVertexPosition(static_cast<size_t>(editor.activeVertexIndex), newPos);
              rect->setActiveVertex(editor.activeVertexIndex);
              break;
            }
            case ObjectType::Polygon: {
              auto* poly = static_cast<Polygon*>(editor.activeVertexShape);
              poly->setVertexPosition(static_cast<size_t>(editor.activeVertexIndex), newPos);
              poly->setActiveVertex(editor.activeVertexIndex);
              break;
            }
            case ObjectType::RegularPolygon: {
              auto* reg = static_cast<RegularPolygon*>(editor.activeVertexShape);
              // Use creation point method: 0=center (translate), 1=first vertex (scale)
              reg->setCreationPointPosition(static_cast<size_t>(editor.activeVertexIndex), newPos);
              reg->setActiveVertex(editor.activeVertexIndex);
              break;
            }
            case ObjectType::Triangle: {
              auto* tri = static_cast<Triangle*>(editor.activeVertexShape);
              tri->setVertexPosition(static_cast<size_t>(editor.activeVertexIndex), newPos);
              tri->setActiveVertex(editor.activeVertexIndex);
              break;
            }
            default:
              std::cout << "DEBUG MoveShapeVertex: Unknown object type!" << std::endl;
              break;
          }
        } else {
          std::cout << "DEBUG MoveShapeVertex: Condition FAILED (no activeVertexShape or invalid index)" << std::endl;
        }
      } else if (editor.dragMode == DragMode::TranslateShape) {
        // Translate rectangle/polygon/regular polygon
        if (editor.selectedObject) {
          sf::Vector2f delta_sfml = worldPos - editor.lastMousePos_sfml;
          Vector_2 delta_cgal = editor.toCGALVector(delta_sfml);

          switch (editor.selectedObject->getType()) {
            case ObjectType::Rectangle:
            case ObjectType::RectangleRotatable: {
              auto* rect = static_cast<Rectangle*>(editor.selectedObject);
              rect->translate(delta_cgal);
              break;
            }
            case ObjectType::Polygon: {
              auto* poly = static_cast<Polygon*>(editor.selectedObject);
              poly->translate(delta_cgal);
              break;
            }
            case ObjectType::RegularPolygon: {
              auto* reg = static_cast<RegularPolygon*>(editor.selectedObject);
              reg->translate(delta_cgal);
              break;
            }
            case ObjectType::Triangle: {
              auto* tri = static_cast<Triangle*>(editor.selectedObject);
              tri->translate(delta_cgal);
              break;
            }
            default:
              break;
          }
        }
      } else if (editor.dragMode == DragMode::InteractWithCircle) {
        // Handle circle dragging: move center or resize
        if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Circle) {
          Circle* selectedCircle = static_cast<Circle*>(editor.selectedObject);
          if (!selectedCircle || !selectedCircle->isValid()) return;
          // Translate continuously while dragging to avoid jitter from hover checks
          sf::Vector2f delta_sfml = worldPos - editor.lastMousePos_sfml;
          Vector_2 delta_cgal = editor.toCGALVector(delta_sfml);

          Point* centerPoint = selectedCircle->getCenterPointObject();
          Point* radiusPoint = selectedCircle->getRadiusPointObject();

          // Move BOTH to achieve rigid body translation
          if (centerPoint) {
            centerPoint->translate(delta_cgal);
          }
          if (radiusPoint) {
            radiusPoint->translate(delta_cgal);
          }

          selectedCircle->update();  // Refresh visuals
          std::cout << "[CIRCLE] Translating Rigid Body" << std::endl;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "Error during drag operation: " << e.what() << std::endl;
    }

    static int dragFrameCount = 0;
    dragFrameCount++;

    if (dragFrameCount % 3 == 0) {  // Update constraints every 3rd frame
      QUICK_PROFILE("PeriodicConstraintUpdate");
      editor.updateConstraintsOnly();  // New method - constraints without full
      // geometry rebuild
    }
  } else {
    hasSetDeferredThisDrag = false;
    static sf::Clock postDragCooldown;

    // Check if we just finished dragging
    if (justFinishedDrag) {
      if (postDragCooldown.getElapsedTime().asMilliseconds() > 100) {
        justFinishedDrag = false;  // Reset after cooldown
      } else {
        editor.lastMousePos_sfml = worldPos;
        return;  // Skip hover updates during cooldown
      }
    }

    // Set the flag when drag ends (add this to your mouse release handler)
    // In handleMouseRelease after "Drag ended, geometry updated":
    if (editor.dragMode != DragMode::None) {
      justFinishedDrag = true;
      postDragCooldown.restart();
    }
  }

  // Update hover states of all objects
  try {
    if (editor.isDragging) {
      editor.lastMousePos_sfml = worldPos;
      return;
    }
    if (justFinishedDrag) {
      editor.lastMousePos_sfml = worldPos;
      return;
    }
    // AGGRESSIVE THROTTLING FOR HOVER UPDATES
    static sf::Clock hoverUpdateClock;
    static sf::Vector2f lastHoverCheckPos(-999999, -999999);

    float moveDistance = std::sqrt(std::pow(worldPos.x - lastHoverCheckPos.x, 2) + std::pow(worldPos.y - lastHoverCheckPos.y, 2));
    sf::Int32 timeSinceLastUpdate = hoverUpdateClock.getElapsedTime().asMilliseconds();
    bool shouldSkipHover = false;
    if (timeSinceLastUpdate < 50) {
      shouldSkipHover = true;
    }
    // Skip if small movement and not enough time has passed
    else if (moveDistance < 10.0f && timeSinceLastUpdate < 100) {
      shouldSkipHover = true;
    }

    if (shouldSkipHover) {
      editor.lastMousePos_sfml = worldPos;
      return;
    }
    hoverUpdateClock.restart();
    lastHoverCheckPos = worldPos;

    QUICK_PROFILE("HoverDetection");
    float tolerance = static_cast<float>(dynamicWorldTolerance);
    // IMPROVED: Only reset hover if something was previously hovered

    // Reset only the previously hovered object if it's still valid and not being deleted
    if (g_lastHoveredObject && !isObjectBeingDeleted(g_lastHoveredObject)) {
      try {
        g_lastHoveredObject->setHovered(false);
      } catch (const std::exception& e) {
        std::cerr << "Error clearing hover (object may be deleted): " << e.what() << std::endl;
      }
    }
    g_lastHoveredObject = nullptr;

    // Clear vertex hover state
    if (editor.hoveredVertexShape) {
      if (auto* rect = dynamic_cast<Rectangle*>(editor.hoveredVertexShape)) rect->setHoveredVertex(-1);
      if (auto* poly = dynamic_cast<Polygon*>(editor.hoveredVertexShape)) poly->setHoveredVertex(-1);
      if (auto* reg = dynamic_cast<RegularPolygon*>(editor.hoveredVertexShape)) reg->setHoveredVertex(-1);
      if (auto* tri = dynamic_cast<Triangle*>(editor.hoveredVertexShape)) tri->setHoveredVertex(-1);
    }
    editor.hoveredVertexShape = nullptr;
    editor.hoveredVertexIndex = -1;

    // Check for hover, prioritizing smaller objects
    GeometricObject* newHoveredObject = nullptr;
    const float QUICK_REJECT_DISTANCE = tolerance * 3.0f;

    // --- SMART BEST-CANDIDATE HOVER (conservative override) ---
    bool useSmartHover = true;
    if (useSmartHover) {
      // Reset hover state on all objects
      auto clearHover = [&](auto& container) {
        for (auto& ptr : container) {
          if (!ptr || isObjectBeingDeleted(ptr.get())) continue;
          try { ptr->setHovered(false); } catch (...) {}
        }
      };
      clearHover(editor.ObjectPoints);
      clearHover(editor.points);
      clearHover(editor.lines);
      clearHover(editor.circles);
      clearHover(editor.rectangles);
      clearHover(editor.polygons);
      clearHover(editor.regularPolygons);
      clearHover(editor.triangles);
      clearHover(editor.angles);

      g_lastHoveredObject = nullptr;
      editor.hoveredObject = nullptr;

      GeometricObject* bestObj = nullptr;
      double bestDist = static_cast<double>(tolerance);
      int bestPriority = -1;  // 0 = Axis, 1 = Line/Shape, 2 = Point

      auto considerCandidate = [&](GeometricObject* obj, double dist, int priority) {
        if (!obj) return;
        if (dist > tolerance) return;
        
        // First candidate becomes best by default
        if (!bestObj) {
          bestObj = obj;
          bestDist = dist;
          bestPriority = priority;
          return;
        }
        
        // NEAREST NEIGHBOR: Higher priority always wins
        if (priority > bestPriority) {
          bestObj = obj;
          bestDist = dist;
          bestPriority = priority;
          return;
        }
        
        // Same priority: pick the closer one (true nearest neighbor)
        if (priority == bestPriority && dist < bestDist) {
          bestObj = obj;
          bestDist = dist;
          bestPriority = priority;
          return;
        }
        
        // Lower priority than current best: skip
      };

      auto distanceToSegment = [&](const sf::Vector2f& a, const sf::Vector2f& b, const sf::Vector2f& p, bool clamp) -> double {
        sf::Vector2f ab = b - a;
        float len2 = ab.x * ab.x + ab.y * ab.y;
        if (len2 < 1e-8f) {
          float dx = p.x - a.x;
          float dy = p.y - a.y;
          return std::sqrt(dx * dx + dy * dy);
        }
        float t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2;
        if (clamp) t = std::clamp(t, 0.0f, 1.0f);
        sf::Vector2f proj(a.x + ab.x * t, a.y + ab.y * t);
        float dx = p.x - proj.x;
        float dy = p.y - proj.y;
        return std::sqrt(dx * dx + dy * dy);
      };

      // Points and ObjectPoints (highest priority)
      for (auto& objPointPtr : editor.ObjectPoints) {
        if (!objPointPtr || !objPointPtr->isValid() || !objPointPtr->isVisible() || isObjectBeingDeleted(objPointPtr.get())) continue;
        sf::Vector2f pos = objPointPtr->getSFMLPosition();
        double dist = std::sqrt(std::pow(pos.x - worldPos.x, 2) + std::pow(pos.y - worldPos.y, 2));
        considerCandidate(objPointPtr.get(), dist, 2);
      }
      for (auto& pointPtr : editor.points) {
        if (!pointPtr || !pointPtr->isValid() || !pointPtr->isVisible() || isObjectBeingDeleted(pointPtr.get())) continue;
        sf::Vector2f pos = pointPtr->getSFMLPosition();
        double dist = std::sqrt(std::pow(pos.x - worldPos.x, 2) + std::pow(pos.y - worldPos.y, 2));
        considerCandidate(pointPtr.get(), dist, 2);
      }

      // Shapes (priority 1)
      auto considerEdges = [&](auto& container) {
        for (auto& shape : container) {
          if (!shape || !shape->isValid() || !shape->isVisible() || isObjectBeingDeleted(shape.get())) continue;
          auto edges = shape->getEdges();
          double minDist = static_cast<double>(tolerance) + 1.0;
          for (size_t i = 0; i < edges.size(); ++i) {
            Point_2 proj;
            double relPos;
            double dist = PointUtils::projectPointOntoSegment(cgalWorldPos, edges[i], proj, relPos);
            if (dist < minDist) minDist = dist;
          }
          considerCandidate(shape.get(), minDist, 1);
        }
      };
      considerEdges(editor.rectangles);
      considerEdges(editor.polygons);
      considerEdges(editor.regularPolygons);
      considerEdges(editor.triangles);

      // Lines (priority 1, axes lower)
      for (auto& linePtr : editor.lines) {
        if (!linePtr || !linePtr->isValid() || !linePtr->isVisible() || isObjectBeingDeleted(linePtr.get())) continue;
        
        double dist;
        // SPECIALIZED AXIS CHECK: Bypass floating-point precision issues at high zoom
        if (linePtr == editor.getXAxis()) {
          // X-axis is at Y=0, so distance is simply |mouseY|
          dist = std::abs(static_cast<double>(worldPos.y));
        } else if (linePtr == editor.getYAxis()) {
          // Y-axis is at X=0, so distance is simply |mouseX|
          dist = std::abs(static_cast<double>(worldPos.x));
        } else {
          sf::Vector2f p1 = editor.toSFMLVector(linePtr->getStartPoint());
          sf::Vector2f p2 = editor.toSFMLVector(linePtr->getEndPoint());
          dist = distanceToSegment(p1, p2, worldPos, linePtr->isSegment());
        }
        
        int priority = linePtr->isLocked() ? 0 : 1;
        considerCandidate(linePtr.get(), dist, priority);
      }

      // Circles (priority 1)
      for (auto& circlePtr : editor.circles) {
        if (!circlePtr || !circlePtr->isValid() || !circlePtr->isVisible() || isObjectBeingDeleted(circlePtr.get())) continue;
        sf::Vector2f center = editor.toSFMLVector(circlePtr->getCenterPoint());
        double dx = center.x - worldPos.x;
        double dy = center.y - worldPos.y;
        double distCenter = std::sqrt(dx * dx + dy * dy);
        double dist = std::abs(distCenter - circlePtr->getRadius());
        considerCandidate(circlePtr.get(), dist, 1);
      }

      // Angles (priority 1)
      for (auto& anglePtr : editor.angles) {
        if (!anglePtr || !anglePtr->isValid() || !anglePtr->isVisible() || isObjectBeingDeleted(anglePtr.get())) continue;
        if (anglePtr->contains(worldPos, tolerance)) {
          considerCandidate(anglePtr.get(), 0.0, 1);
        }
      }

      if (bestObj && !isObjectBeingDeleted(bestObj)) {
        try {
          bestObj->setHovered(true);
          g_lastHoveredObject = bestObj;
          editor.hoveredObject = bestObj;
        } catch (const std::exception& e) {
          std::cerr << "Error setting hover on object: " << e.what() << std::endl;
          bestObj = nullptr;
        }
      }
      newHoveredObject = bestObj;
    }

    if (!useSmartHover) {

    // Check vertex handles first
    auto checkVertexHover = [&](auto& container) -> bool {
      for (auto& ptr : container) {
        if (!ptr || !ptr->isValid() || !ptr->isVisible()) continue;
        auto verts = ptr->getVerticesSFML();
        for (size_t i = 0; i < verts.size(); ++i) {
          if (auto* regPtr = dynamic_cast<RegularPolygon*>(ptr.get())) {
            if (i != 0) continue;  // Only first vertex hover for regular polygons
            (void)regPtr;
          }
          float dx = verts[i].x - worldPos.x;
          float dy = verts[i].y - worldPos.y;
          float dist = std::sqrt(dx * dx + dy * dy);
          if (dist <= tolerance * 1.5f) {
            ptr->setHoveredVertex(static_cast<int>(i));
            editor.hoveredVertexShape = ptr.get();
            editor.hoveredVertexIndex = static_cast<int>(i);
            newHoveredObject = ptr.get();
            return true;
          }
        }
      }
      return false;
    };

    if (checkVertexHover(editor.rectangles) || checkVertexHover(editor.polygons) || checkVertexHover(editor.regularPolygons) ||
        checkVertexHover(editor.triangles)) {
      // continue to set hover state; still allow color set below
    }

    // Check ObjectPoints first (smallest, highest priority)
    if (!newHoveredObject) {
      for (auto& objPointPtr : editor.ObjectPoints) {
        if (objPointPtr && objPointPtr->isVisible() && !isObjectBeingDeleted(objPointPtr.get())) {
          // Quick distance check before expensive contains()
          sf::Vector2f objPos = objPointPtr->getSFMLPosition();
          float quickDist = std::abs(worldPos.x - objPos.x) + std::abs(worldPos.y - objPos.y);  // Manhattan distance (faster)

          if (quickDist <= QUICK_REJECT_DISTANCE && objPointPtr->contains(worldPos, tolerance)) {
            newHoveredObject = objPointPtr.get();
            break;
          }
        }
      }
    }

    // Check for Free Points explicitly if not found yet
    if (!newHoveredObject) {
      for (auto& pointPtr : editor.points) {
        if (pointPtr && pointPtr->isVisible() && !isObjectBeingDeleted(pointPtr.get())) {
          sf::Vector2f pointPos = pointPtr->getSFMLPosition();
          float quickDist = std::abs(worldPos.x - pointPos.x) + std::abs(worldPos.y - pointPos.y);

          // Use slightly larger tolerance for easier selection
          if (quickDist <= QUICK_REJECT_DISTANCE && pointPtr->contains(worldPos, tolerance)) {
            newHoveredObject = pointPtr.get();
            break;
          }
        }
      }
    }

    // Check for Shape Edges explicitly
    if (!newHoveredObject) {
      auto checkShapeEdges = [&](auto& container) -> GeometricObject* {
        for (auto& shape : container) {
          if (!shape || !shape->isValid() || !shape->isVisible()) continue;

          auto edges = shape->getEdges();
          for (size_t i = 0; i < edges.size(); ++i) {
            Point_2 proj;
            double relPos;
            // Use PointUtils helper for consistent math
            double dist = PointUtils::projectPointOntoSegment(editor.toCGALPoint(worldPos), edges[i], proj, relPos);

            // Convert distance back to float/pixels approx for check
            if (dist < tolerance) {  // tolerance is in view units? no, getScaledTolerance is in world units
              return shape.get();
            }
          }
        }
        return nullptr;
      };

      if (auto* hit = checkShapeEdges(editor.rectangles))
        newHoveredObject = hit;
      else if (auto* hit = checkShapeEdges(editor.polygons))
        newHoveredObject = hit;
      else if (auto* hit = checkShapeEdges(editor.regularPolygons))
        newHoveredObject = hit;
      else if (auto* hit = checkShapeEdges(editor.triangles))
        newHoveredObject = hit;
    }

    // Lines and Circles with safety checks
    if (!newHoveredObject) {
      for (auto& linePtr : editor.lines) {
        if (linePtr && linePtr->isVisible() && !isObjectBeingDeleted(linePtr.get()) && linePtr->contains(worldPos, tolerance)) {
          newHoveredObject = linePtr.get();
          break;
        }
      }
    }

    if (!newHoveredObject) {
      for (auto& anglePtr : editor.angles) {
        if (anglePtr && anglePtr->isVisible() && !isObjectBeingDeleted(anglePtr.get()) && anglePtr->contains(worldPos, tolerance)) {
          newHoveredObject = anglePtr.get();
          break;
        }
      }
    }

    if (!newHoveredObject) {
      for (auto& circlePtr : editor.circles) {
        if (circlePtr && circlePtr->isVisible() && !isObjectBeingDeleted(circlePtr.get())) {
          // Check circle center first (highest priority for circles)
          float centerTolerance = tolerance * 4.0f;
          if (circlePtr->isCenterPointHovered(worldPos, centerTolerance)) {
            newHoveredObject = circlePtr.get();
            std::cout << "[HOVER] Circle center hovered!" << std::endl;
            break;
          }
          // Then check normal circle body
          else if (circlePtr->contains(worldPos, tolerance)) {
            newHoveredObject = circlePtr.get();
            break;
          }
        }
      }
    }

    // ONLY UPDATE HOVER STATE IF DIFFERENT FROM LAST AND NOT BEING DELETED
    if (newHoveredObject != g_lastHoveredObject) {
      // Clear previous hover safely
      if (g_lastHoveredObject && !isObjectBeingDeleted(g_lastHoveredObject)) {
        try {
          g_lastHoveredObject->setHovered(false);
        } catch (const std::exception& e) {
          std::cerr << "Error clearing hover on object: " << e.what() << std::endl;
        }
      }

      // Set new hover safely
      if (newHoveredObject && !isObjectBeingDeleted(newHoveredObject)) {
        try {
          newHoveredObject->setHovered(true);
        } catch (const std::exception& e) {
          std::cerr << "Error setting hover on object: " << e.what() << std::endl;
          newHoveredObject = nullptr;  // Clear the reference if it failed
        }
      }

      g_lastHoveredObject = newHoveredObject;
    }

    }

  } catch (const std::exception& e) {
    std::cerr << "Error in hover detection: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown error in hover detection" << std::endl;
  }

  // === EDGE HOVER DETECTION (Universal Snapping) ===
  // Only check for edge hover if no object is currently hovered
  try {
    if (!g_lastHoveredObject && !editor.isDragging) {
      float tolerance = getDynamicSelectionTolerance(editor);
      auto edgeHit = PointUtils::findNearestEdge(editor, worldPos, tolerance);

      if (edgeHit) {
        // Store the edge hit for use during click
        editor.m_hoveredEdge = edgeHit;

        // Set hover on the parent shape to provide visual feedback
        if (edgeHit->host && !isObjectBeingDeleted(edgeHit->host)) {
          edgeHit->host->setHovered(true);
        }
      } else {
        // Clear hovered edge if no edge is near
        editor.m_hoveredEdge = std::nullopt;
      }
    } else {
      // Clear hovered edge when dragging or hovering over an object
      editor.m_hoveredEdge = std::nullopt;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error in edge hover detection: " << e.what() << std::endl;
  }

  editor.lastMousePos_sfml = worldPos;
}

// --- Precise Selection Helpers ---

// Helper: Check if line segment intersects AABB (FloatRect)
// Using Liang-Barsky algorithm or checking endpoints + intersection with rect edges
bool lineIntersectsRect(const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::FloatRect& rect) {
    // 1. AABB Trivial Reject
    float minX = std::min(p1.x, p2.x);
    float maxX = std::max(p1.x, p2.x);
    float minY = std::min(p1.y, p2.y);
    float maxY = std::max(p1.y, p2.y);
    
    if (maxX < rect.left || minX > rect.left + rect.width ||
        maxY < rect.top || minY > rect.top + rect.height) {
        return false;
    }
    
    // 2. Check if either endpoint is inside (Trivial Accept)
    if (rect.contains(p1) || rect.contains(p2)) return true;
    
    // 3. Line-Segment Intersects Rect Edges?
    // Function to check intersection of segment p1-p2 with segment q1-q2
    auto intersect = [](sf::Vector2f a, sf::Vector2f b, sf::Vector2f c, sf::Vector2f d) {
        auto cross = [](sf::Vector2f v, sf::Vector2f w) { return v.x*w.y - v.y*w.x; };
        sf::Vector2f r = b - a;
        sf::Vector2f s = d - c;
        float rxs = cross(r, s);
        if (std::abs(rxs) < 1e-5f) return false;
        sf::Vector2f cma = c - a;
        float t = cross(cma, s) / rxs;
        float u = cross(cma, r) / rxs;
        return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
    };
    
    sf::Vector2f tl(rect.left, rect.top);
    sf::Vector2f tr(rect.left + rect.width, rect.top);
    sf::Vector2f br(rect.left + rect.width, rect.top + rect.height);
    sf::Vector2f bl(rect.left, rect.top + rect.height);
    
    if (intersect(p1, p2, tl, tr)) return true; // Top
    if (intersect(p1, p2, tr, br)) return true; // Right
    if (intersect(p1, p2, br, bl)) return true; // Bottom
    if (intersect(p1, p2, bl, tl)) return true; // Left
    
    return false;
}

// Helper: Check if Circle intersects AABB
bool circleIntersectsRect(const sf::Vector2f& center, float radius, const sf::FloatRect& rect) {
    // Find closest point on Rect to Circle Center
    float closestX = std::clamp(center.x, rect.left, rect.left + rect.width);
    float closestY = std::clamp(center.y, rect.top, rect.top + rect.height);
    
    float dx = center.x - closestX;
    float dy = center.y - closestY;
    
    // If the distance is less than radius, they intersect
    return (dx * dx + dy * dy) < (radius * radius);
}

void handleMouseRelease(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

  if (mouseEvent.button == sf::Mouse::Left && editor.isDraggingLabel) {
    editor.isDraggingLabel = false;
    editor.labelDragObject = nullptr;
    return;
  }

  // Clear active vertex drag state
  if (editor.activeVertexShape) {
    if (auto* rect = dynamic_cast<Rectangle*>(editor.activeVertexShape)) rect->setActiveVertex(-1);
    if (auto* poly = dynamic_cast<Polygon*>(editor.activeVertexShape)) poly->setActiveVertex(-1);
    if (auto* reg = dynamic_cast<RegularPolygon*>(editor.activeVertexShape)) reg->setActiveVertex(-1);
    if (auto* tri = dynamic_cast<Triangle*>(editor.activeVertexShape)) tri->setActiveVertex(-1);
  }
  editor.activeVertexShape = nullptr;
  editor.activeVertexIndex = -1;

  // Check if GUI (especially color picker) should handle this release event

  sf::Event eventToForward;
  eventToForward.type = sf::Event::MouseButtonReleased;
  eventToForward.mouseButton = mouseEvent;
  if (editor.gui.handleEvent(editor.window, eventToForward, editor)) {
    return;  // GUI consumed the event
  }

  // Check if we're releasing a selection box drag
  if (editor.isDrawingSelectionBox && mouseEvent.button == sf::Mouse::Left) {
    editor.isDrawingSelectionBox = false;

    // Check for Ctrl Additive Selection
    bool isCtrlHeld = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl);

    if (!isCtrlHeld) {
      editor.clearSelection();
      deselectAllAndClearInteractionState(editor);
    }
    sf::FloatRect selectionBox(std::min(editor.selectionBoxStart_sfml.x, worldPos.x), std::min(editor.selectionBoxStart_sfml.y, worldPos.y),
                               std::abs(worldPos.x - editor.selectionBoxStart_sfml.x), std::abs(worldPos.y - editor.selectionBoxStart_sfml.y));

    try {
      std::vector<GeometricObject*> newlySelectedObjects;

      // Points: Check if point position is inside selection box
      // (Don't use getGlobalBounds() - it has fixed world-unit size that's huge at high zoom)
      for (auto& pointPtr : editor.points) {
        if (pointPtr && pointPtr->isValid() && pointPtr->isVisible()) {
          sf::Vector2f pos = pointPtr->getSFMLPosition();
          if (selectionBox.contains(pos)) {
            newlySelectedObjects.push_back(pointPtr.get());
          }
        }
      }
      
      // ObjectPoints: Same position-based check
      for (auto& objPointPtr : editor.ObjectPoints) {
        if (objPointPtr && objPointPtr->isValid() && objPointPtr->isVisible()) {
          sf::Vector2f pos = objPointPtr->getSFMLPosition();
          if (selectionBox.contains(pos)) {
            newlySelectedObjects.push_back(objPointPtr.get());
          }
        }
      }
      
      // Lines: Use precise Line-Rect intersection
      for (auto& linePtr : editor.lines) {
        if (linePtr && linePtr->isValid() && linePtr->isVisible() && !linePtr->isLocked()) {
             // Get standard points (start/end)
             // ...
             // Logic: Existing getGlobalBounds returns bounds for the *segment* representation in standard Line impl.
             // Let's assume Segment behavior for selection to be intuitive (select the observable part).
             
             Point_2 start = linePtr->getStartPoint();
             Point_2 end = linePtr->getEndPoint();
             // Convert to float for SFML check
             sf::Vector2f p1(static_cast<float>(CGAL::to_double(start.x())), static_cast<float>(CGAL::to_double(start.y())));
             sf::Vector2f p2(static_cast<float>(CGAL::to_double(end.x())), static_cast<float>(CGAL::to_double(end.y())));
             
             if (lineIntersectsRect(p1, p2, selectionBox)) {
                  newlySelectedObjects.push_back(linePtr.get());
             }
        }
      }
      
      // Circles: Use precise Circle-Rect intersection
      for (auto& circlePtr : editor.circles) {
        if (circlePtr && circlePtr->isValid() && circlePtr->isVisible() && !circlePtr->isLocked()) {
             Point_2 c = circlePtr->getCenterPoint();
             sf::Vector2f center(static_cast<float>(CGAL::to_double(c.x())), static_cast<float>(CGAL::to_double(c.y())));
             float radius = static_cast<float>(circlePtr->getRadius());
             
             if (circleIntersectsRect(center, radius, selectionBox)) {
                newlySelectedObjects.push_back(circlePtr.get());
             }
        }
      }
      
      // ObjectPoints
      for (auto& objPointPtr : editor.ObjectPoints) {
        if (objPointPtr && objPointPtr->isValid() && objPointPtr->isVisible() && !objPointPtr->isLocked() && selectionBox.intersects(objPointPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(objPointPtr.get());
        }
      }
      // Add Rectangle selection
      for (auto& rectPtr : editor.rectangles) {
        if (rectPtr && rectPtr->isValid() && rectPtr->isVisible() && !rectPtr->isLocked() && selectionBox.intersects(rectPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(rectPtr.get());
        }
      }
      // Add Polygon selection
      for (auto& polyPtr : editor.polygons) {
        if (polyPtr && polyPtr->isValid() && polyPtr->isVisible() && !polyPtr->isLocked() && selectionBox.intersects(polyPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(polyPtr.get());
        }
      }
      // Add RegularPolygon selection
      for (auto& regPolyPtr : editor.regularPolygons) {
        if (regPolyPtr && regPolyPtr->isValid() && regPolyPtr->isVisible() && !regPolyPtr->isLocked() && selectionBox.intersects(regPolyPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(regPolyPtr.get());
        }
      }
      // Add Triangle selection
      for (auto& triPtr : editor.triangles) {
        if (triPtr && triPtr->isValid() && triPtr->isVisible() && !triPtr->isLocked()) {
             if(selectionBox.intersects(triPtr->getGlobalBounds())) {
                 newlySelectedObjects.push_back(triPtr.get());
             }
        }
      }
      for (auto& anglePtr : editor.angles) {
        if (anglePtr && anglePtr->isValid() && anglePtr->isVisible() && !anglePtr->isLocked() && selectionBox.intersects(anglePtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(anglePtr.get());
        }
      }

      // Apply selection to all found objects
      for (GeometricObject* obj : newlySelectedObjects) {
        if (obj) {  // Basic safety check
          if (std::find(editor.selectedObjects.begin(), editor.selectedObjects.end(), obj) == editor.selectedObjects.end()) {
            obj->setSelected(true);
            editor.selectedObjects.push_back(obj);
          }
        }
      }

      // If exactly one object was selected by the box, set it as the
      // primary selected object
      if (newlySelectedObjects.size() == 1) {
        editor.selectedObject = newlySelectedObjects[0];
        if (editor.selectedObject) {
          sf::Color selectedColor = editor.selectedObject->getColor();
          editor.setCurrentColor(selectedColor);
          editor.gui.setCurrentColor(selectedColor);
          if (auto& picker = editor.gui.getColorPicker()) {
            picker->setCurrentColor(selectedColor);
          }
        }
      }
      // If multiple objects are selected, editor.selectedObject remains
      // nullptr (as set by deselectAllAndClearInteractionState)

    } catch (const std::exception& e) {
      std::cerr << "Error processing selection box: " << e.what() << std::endl;
    }

    std::cout << "Selection box completed" << std::endl;
    editor.lastMousePos_sfml = worldPos;
    return;  // Selection box handling is exclusive
  }

  // Handle end of dragging an object
  if (editor.isDragging && mouseEvent.button == sf::Mouse::Left) {
    // ✅ CRITICAL FIX: Defer constraint updates DURING the cleanup phase
    std::cout << "PREPARING FOR COORDINATED UPDATE" << std::endl;

    // Keep constraint updates deferred while we clean up visual deferring
    if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
      Point* draggedPoint = static_cast<Point*>(editor.selectedObject);
      // Keep constraint updates deferred for now
      draggedPoint->setDeferConstraintUpdates(false);  // This will trigger update
      for (auto& linePtr : editor.lines) {
        if (linePtr && linePtr->isValid()) {
          // REMOVED deferring - enable real-time updates during point drag
          linePtr->setDeferSFMLUpdates(false);  // Enable immediate visual updates
        }
      }
      editor.updateAllGeometry();
      for (auto& linePtr : editor.lines) {
        if (linePtr && linePtr->isValid()) {
          linePtr->setDeferSFMLUpdates(false);
          linePtr->updateSFMLShape();  // Single final update
        }
      }
    } else {
      // Handle line translation points
      if (editor.dragMode == DragMode::TranslateLine && editor.selectedObject &&
          (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
        Line* selectedLine = static_cast<Line*>(editor.selectedObject);
        Point* startPoint = selectedLine->getStartPointObject();
        Point* endPoint = selectedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
      }

      // Handle endpoint dragging
      if ((editor.dragMode == DragMode::MoveLineEndpointStart || editor.dragMode == DragMode::MoveLineEndpointEnd) && editor.selectedObject &&
          (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
        Line* selectedLine = static_cast<Line*>(editor.selectedObject);
        Point* startPoint = selectedLine->getStartPointObject();
        Point* endPoint = selectedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
      }

      std::cout << "CLEARING SFML DEFERRING" << std::endl;
      // Re-enable SFML updates for all lines (but keep constraint updates
      // deferred)
      for (auto& linePtr : editor.lines) {
        if (linePtr && linePtr->isValid()) {
          linePtr->setDeferSFMLUpdates(false);  // Enable visual updates
          // Don't call updateSFMLShape yet - let constraint update handle it
        }
      }

      GeometricObject* draggedObject = editor.selectedObject;
      DragMode previousDragMode = editor.dragMode;

      // Topological merge on snap release
      // Check Ctrl key state explicitly if m_wasSnapped is unreliable?
      // Actually, standard behavior relies on m_wasSnapped being effectively 'latched' until release.
      bool isCtrlHeld = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl);

      std::cout << "[DEBUG] handleMouseRelease: m_wasSnapped=" << editor.m_wasSnapped
                << ", m_snapTargetPoint=" << (editor.m_snapTargetPoint ? "Valid" : "Null") << ", CtrlHeld=" << isCtrlHeld
                << ", DragMode=" << (int)previousDragMode << std::endl;

      if (editor.m_wasSnapped && editor.m_snapTargetPoint &&
          (previousDragMode == DragMode::MoveFreePoint || previousDragMode == DragMode::MoveLineEndpointStart ||
           previousDragMode == DragMode::MoveLineEndpointEnd)) {
        std::shared_ptr<Point> draggedPointShared = nullptr;

        if (previousDragMode == DragMode::MoveFreePoint && draggedObject && draggedObject->getType() == ObjectType::Point) {
          Point* draggedPointRaw = static_cast<Point*>(draggedObject);
          for (auto& pt : editor.points) {
            if (pt && pt.get() == draggedPointRaw) {
              draggedPointShared = pt;
              break;
            }
          }
        } else if ((previousDragMode == DragMode::MoveLineEndpointStart || previousDragMode == DragMode::MoveLineEndpointEnd) && draggedObject &&
                   (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment)) {
          Line* selectedLine = static_cast<Line*>(draggedObject);
          draggedPointShared = (previousDragMode == DragMode::MoveLineEndpointStart) ? selectedLine->getStartPointObjectShared()
                                                                                     : selectedLine->getEndPointObjectShared();
        }

        // TOPOLOGICAL MERGE: Replace all references to the dragged point with the target point
        if (draggedPointShared && draggedPointShared != editor.m_snapTargetPoint) {
          std::cout << "[TOPOLOGY] Merging Point " << draggedPointShared->getID() << " into " << editor.m_snapTargetPoint->getID() << std::endl;

          // This function updates all Lines, Circles, and Polygons to use the target point
          editor.replacePoint(draggedPointShared, editor.m_snapTargetPoint);
        }
      }

      editor.m_wasSnapped = false;
      editor.m_snapTargetPoint = nullptr;
      // Clear snapping message
      if (!editor.isDrawingSelectionBox) {
        editor.setGUIMessage("");
      }

      editor.isDragging = false;  // Stop dragging state

      // Reset m_isUnderDirectManipulation flag for lines
      if (draggedObject && (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment)) {
        if (previousDragMode == DragMode::TranslateLine || previousDragMode == DragMode::MoveLineEndpointStart ||
            previousDragMode == DragMode::MoveLineEndpointEnd) {
          static_cast<Line*>(draggedObject)->setIsUnderDirectManipulation(false);
          if (Constants::DEBUG_DRAGGING) {
            std::cout << "Line " << draggedObject << ": m_isUnderDirectManipulation set to false (drag end)" << std::endl;
          }
        }
      }

      // ✅ NOW enable constraint updates - this will trigger coordinated
      // geometry update
      std::cout << "ENABLING CONSTRAINT UPDATES FOR FINAL UPDATE" << std::endl;

      if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
        Point* draggedPoint = static_cast<Point*>(editor.selectedObject);
        draggedPoint->setDeferConstraintUpdates(false);  // This will trigger update
      }

      // Handle line translation points
      if (previousDragMode == DragMode::TranslateLine && draggedObject &&
          (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment)) {
        Line* selectedLine = static_cast<Line*>(draggedObject);
        Point* startPoint = selectedLine->getStartPointObject();
        Point* endPoint = selectedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);
        }
      }

      // Handle endpoint dragging
      if ((previousDragMode == DragMode::MoveLineEndpointStart || previousDragMode == DragMode::MoveLineEndpointEnd) && draggedObject &&
          (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment)) {
        Line* selectedLine = static_cast<Line*>(draggedObject);
        Point* startPoint = selectedLine->getStartPointObject();
        Point* endPoint = selectedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);
        }
      }

      // Update geometry if a relevant drag operation occurred
      if (previousDragMode == DragMode::MoveFreePoint || previousDragMode == DragMode::DragObjectPoint ||
          previousDragMode == DragMode::TranslateLine || previousDragMode == DragMode::MoveLineEndpointStart ||
          previousDragMode == DragMode::MoveLineEndpointEnd) {
        editor.updateAllGeometry();  // Final coordinated update
        std::cout << "Drag ended, geometry updated" << std::endl;
        editor.updateAllGeometry();  // Final coordinated update
        std::cout << "Drag ended, geometry updated" << std::endl;
        // static bool justFinishedDrag = true; // Removed unused
        static sf::Clock postDragCooldown;
        // justFinishedDrag = true; // Removed unused
        postDragCooldown.restart();
      }

      // Record translation command for move operations
      if (previousDragMode == DragMode::MoveFreePoint || previousDragMode == DragMode::DragObjectPoint ||
          previousDragMode == DragMode::TranslateLine || previousDragMode == DragMode::TranslateShape) {
        sf::Vector2f totalDeltaSfml = worldPos - editor.dragStart_sfml;
        float threshold = std::max(0.001f, getDynamicSelectionTolerance(editor) * 0.1f);
        if (editor.length(totalDeltaSfml) > threshold) {
          std::vector<GeometricObject*> translationTargets = editor.selectedObjects;
          if (translationTargets.empty() && draggedObject) {
            translationTargets.push_back(draggedObject);
          }
          if (!translationTargets.empty()) {
            Vector_2 cgalDelta = editor.toCGALVector(totalDeltaSfml);
            editor.commandManager.pushHistoryOnly(std::make_shared<TranslateCommand>(translationTargets, cgalDelta));
          }
        }
      }
    }

    // Reset general drag interaction state
    editor.isDragging = false;
    editor.dragMode = DragMode::None;
    editor.m_selectedEndpoint = EndpointSelection::None;
    editor.isResizingAngle = false;
  }

  // Handle end of panning
  if (editor.isPanning &&
      (mouseEvent.button == sf::Mouse::Middle ||  // Original
       mouseEvent.button == sf::Mouse::Right ||   // New options
       (mouseEvent.button == sf::Mouse::Left && (sf::Keyboard::isKeyPressed(sf::Keyboard::Space) || sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) ||
                                                 sf::Keyboard::isKeyPressed(sf::Keyboard::LControl))))) {
    editor.isPanning = false;
    std::cout << "Panning ended" << std::endl;
  }
}

void handleZoom(GeometryEditor& editor, float scrollDelta, const sf::Vector2i& mousePixelPos) {
  sf::View& view = editor.drawingView;

  sf::Vector2f worldPosBeforeZoom = editor.window.mapPixelToCoords(mousePixelPos, view);

  float zoomDirection = (scrollDelta > 0) ? (1.0f / Constants::MOUSE_WHEEL_ZOOM_FACTOR)  // Zoom In
                                          : Constants::MOUSE_WHEEL_ZOOM_FACTOR;          // Zoom Out

  float currentViewHeight = view.getSize().y;
  float newViewHeight = currentViewHeight * zoomDirection;

  // Calculate zoom level based on window height
  // Calculate zoom level based on window height
  // float currentZoomLevel = currentViewHeight / static_cast<float>(Constants::WINDOW_HEIGHT); // Unused
  float projectedZoomLevel = newViewHeight / static_cast<float>(Constants::WINDOW_HEIGHT);

  // Zoom limits removed for infinite zoom
  // (Original limit logic removed)

  if (std::abs(zoomDirection - 1.0f) < Constants::EPSILON) {  // Avoid zooming if factor is effectively 1
    return;
  }

  view.zoom(zoomDirection);

  sf::Vector2f worldPosAfterZoom = editor.window.mapPixelToCoords(mousePixelPos, view);
  sf::Vector2f offset = worldPosBeforeZoom - worldPosAfterZoom;
  view.move(offset);

  // editor.window.setView(view); // This is usually done in the main render
  // loop
  editor.grid.update(view, editor.window.getSize());
  editor.updateScaleFactor();  // If you have such a function

  // Optional: Log zoom changes
  // std::cout << "Zoom: delta=" << scrollDelta << ", factorApplied=" <<
  // zoomDirection
  //           << ", newLevel=" << (view.getSize().y /
  //           static_cast<float>(Constants::WINDOW_HEIGHT))
  //           << std::endl;
}

// Ensure handleMouseWheelScroll calls this:
void handleMouseWheelScroll(GeometryEditor& editor, const sf::Event::MouseWheelScrollEvent& wheelEvent) {
  // wheelEvent.delta will be positive for scroll up (zoom in for many)
  // or negative for scroll down (zoom out for many)
  // SFML typically: scroll up is positive delta, scroll down is negative.
  // If your mouse is inverted, adjust the sign of wheelEvent.delta.
  // Get mouse position in pixel coordinates from the event
  sf::Vector2i mousePixelPos(wheelEvent.x, wheelEvent.y);
  handleZoom(editor, wheelEvent.delta, mousePixelPos);
}

// Add this implementation at the end of the file
void handleEvents(GeometryEditor& editor) {
  // Monitor for potential errors with views
  try {
    float zoomLevel = editor.drawingView.getSize().y / static_cast<float>(Constants::WINDOW_HEIGHT);
    if (std::isnan(zoomLevel) || std::isinf(zoomLevel)) {
      std::cerr << "WARNING: View appears corrupted with NaN/Inf zoom level" << std::endl;
      editor.resetApplicationState();
    }
  } catch (...) {
    // Silently recover if even checking the view causes an exception
    editor.resetApplicationState();
  }

  sf::Event event;
  while (editor.window.pollEvent(event)) {
    bool guiHandledThisEvent = false;  // Flag to track if GUI handled the event
    // First, let the GUI have a chance to handle the event
    if (editor.gui.isInitialized()) {
      guiHandledThisEvent = editor.gui.handleEvent(editor.window, event, editor);
    }

    if (guiHandledThisEvent) {
      continue;  // GUI consumed the event
    }

    switch (event.type) {
      case sf::Event::Closed:
        editor.window.close();
        break;
      case sf::Event::Resized:
        editor.handleResize(event.size.width, event.size.height);
        // Fix UI Layout: Update button positions dynamically
        editor.gui.updateLayout(static_cast<float>(event.size.width));
        break;
      case sf::Event::MouseButtonPressed:
        handleMousePress(editor, event.mouseButton);
        break;
      case sf::Event::MouseButtonReleased:
        handleMouseRelease(editor, event.mouseButton);
        break;
      case sf::Event::MouseMoved:
        handleMouseMove(editor, event.mouseMove);
        break;
      case sf::Event::MouseWheelScrolled:
        handleMouseWheelScroll(editor, event.mouseWheelScroll);
        break;
      case sf::Event::TextEntered:
        if (editor.isRenaming) {
          if (event.text.unicode == 8) {  // Backspace
            if (!editor.renameBuffer.empty()) editor.renameBuffer.pop_back();
          } else if (event.text.unicode == 13) {  // Enter
            if (editor.pointToRename) {
              // Commit Rename
              // Note: We might want to check LabelManager for uniqueness here or allow duplicates?
              // Prompt says: "LabelManager validates uniqueness... If conflict: Reject OR Auto-adjust"
              // For now, minimal working solution: Set it.
              editor.pointToRename->setLabel(editor.renameBuffer);
            }
            editor.isRenaming = false;
          } else if (event.text.unicode == 27) {  // Escape
            editor.isRenaming = false;
          } else if (event.text.unicode >= 32 && event.text.unicode < 128) {
            editor.renameBuffer += static_cast<char>(event.text.unicode);
          }
        }
        break;
      case sf::Event::KeyPressed:
        handleKeyPress(editor, event.key);
        if (event.key.code == sf::Keyboard::LAlt || event.key.code == sf::Keyboard::RAlt) {
          // Force update of snap state (magnet effect) immediately on key press
          sf::Vector2i mousePos = sf::Mouse::getPosition(editor.window);
          sf::Event::MouseMoveEvent moveEvent;
          moveEvent.x = mousePos.x;
          moveEvent.y = mousePos.y;
          handleMouseMove(editor, moveEvent);
        }
        break;
      case sf::Event::KeyReleased:
        if (event.key.code == sf::Keyboard::LAlt || event.key.code == sf::Keyboard::RAlt) {
          // Force update of snap state (unsnap) immediately on key release
          sf::Vector2i mousePos = sf::Mouse::getPosition(editor.window);
          sf::Event::MouseMoveEvent moveEvent;
          moveEvent.x = mousePos.x;
          moveEvent.y = mousePos.y;
          handleMouseMove(editor, moveEvent);
        }
        break;
      default:
        break;
    }
  }
}
