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

#include "CGALSafeUtils.h"  // For CGALSafeUtils functions
#include "Circle.h"
#include "Constants.h"
#include "Angle.h"
#include "GUI.h"  // For editor.gui
#include "GeometricObject.h"
#include "GeometryEditor.h"  // Needs full definition for editor members
#include "Intersection.h" // Use Master Implementation
#include "Line.h"
#include "PointUtils.h"
#include "Polygon.h"
#include "Rectangle.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "RegularPolygon.h"
#include "Triangle.h"
#include "VertexLabelManager.h"
#include "ProjectionUtils.h"  // Add this include
#include "PointUtils.h"       // For findShapeVertex and findNearestEdge
#include "QuickProfiler.h"
#include "Types.h"

using namespace CGALSafeUtils;
// Helper functions for checking if CGAL types contain finite values
// MODIFIED: Replace with robust versions
float currentZoomFactor = 1.0f;
static std::set<GeometricObject *> objectsBeingDeleted;
static GeometricObject *g_lastHoveredObject = nullptr;
bool is_cgal_point_finite(const Point_2 &point) {
  try {
    return std::isfinite(CGAL::to_double(point.x())) && std::isfinite(CGAL::to_double(point.y()));
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "Warning (is_cgal_point_finite): Uncertain conversion checking "
                 "point finiteness: "
              << e.what() << std::endl;
    return false;
  } catch (const std::exception &e) {
    std::cerr << "Warning (is_cgal_point_finite): Exception checking point "
                 "finiteness: "
              << e.what() << std::endl;
    return false;
  }
}

bool is_cgal_ft_finite(const Kernel::FT &value) {
  try {
    return std::isfinite(CGAL::to_double(value));
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "Warning (is_cgal_ft_finite): Uncertain conversion checking "
                 "Kernel::FT finiteness: "
              << e.what() << std::endl;
    return false;
  } catch (const std::exception &e) {
    std::cerr << "Warning (is_cgal_ft_finite): Exception checking Kernel::FT "
                 "finiteness: "
              << e.what() << std::endl;
    return false;
  }
}
// Helper function to remove an object from a vector of unique_ptrs
// Returns true if an object was found and removed, false otherwise.
template <typename T>
bool removeObjectFromVector(std::vector<std::unique_ptr<T>> &vec, GeometricObject *objToRemove) {
  if (!objToRemove) return false;
  auto it = std::remove_if(vec.begin(), vec.end(),
                           [&](const std::unique_ptr<T> &ptr) { return ptr.get() == objToRemove; });
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
void markObjectForDeletion(GeometricObject *obj) {
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

void unmarkObjectForDeletion(GeometricObject *obj) {
  if (obj) {
    objectsBeingDeleted.erase(obj);
    std::cout << "Unmarked object " << obj << " from deletion" << std::endl;
  }
}

bool isObjectBeingDeleted(GeometricObject *obj) {
  return objectsBeingDeleted.find(obj) != objectsBeingDeleted.end();
}

void deselectAllAndClearInteractionState(GeometryEditor &editor) {
  // Safely deselect all objects with more robust error handling

  try {
    // Deselect all individual points
    for (auto &pointPtr : editor.points) {
      if (pointPtr && pointPtr->isValid()) {
        try {
          pointPtr->setSelected(false);
        } catch (const std::exception &e) {
          std::cerr << "Error deselecting point (ID: " << pointPtr->getID() << "): " << e.what()
                    << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing points in deselectAll: " << e.what() << std::endl;
  }

  try {
    // Deselect all lines
    for (auto &linePtr : editor.lines) {
      if (linePtr && linePtr->isValid()) {
        try {
          linePtr->setSelected(false);
        } catch (const std::exception &e) {
          std::cerr << "Error deselecting line (ID: " << linePtr->getID() << "): " << e.what()
                    << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing lines in deselectAll: " << e.what() << std::endl;
  }

  try {
    // Deselect all circles
    for (auto &circlePtr : editor.circles) {
      if (circlePtr && circlePtr->isValid()) {
        try {
          circlePtr->setSelected(false);
        } catch (const std::exception &e) {
          std::cerr << "Error deselecting circle (ID: " << circlePtr->getID() << "): " << e.what()
                    << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing circles in deselectAll: " << e.what() << std::endl;
  }

  try {
    // Deselect all object points
    for (auto &objPointPtr : editor.ObjectPoints) {
      if (objPointPtr && objPointPtr->isValid()) {
        try {
          objPointPtr->setSelected(false);
        } catch (const std::exception &e) {
          std::cerr << "Error deselecting object point (ID: " << objPointPtr->getID()
                    << "): " << e.what() << std::endl;
          // Continue with next object
        }
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing object points in deselectAll: " << e.what() << std::endl;
  }

  editor.selectedObjects.clear();

  // Deselect all rectangles and reset vertex states
  try {
    for (auto &rectPtr : editor.rectangles) {
      if (rectPtr && rectPtr->isValid()) {
        rectPtr->setSelected(false);
        rectPtr->setHoveredVertex(-1);
        rectPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing rectangles in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all polygons and reset vertex states
  try {
    for (auto &polyPtr : editor.polygons) {
      if (polyPtr && polyPtr->isValid()) {
        polyPtr->setSelected(false);
        polyPtr->setHoveredVertex(-1);
        polyPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing polygons in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all regular polygons and reset vertex states
  try {
    for (auto &regPtr : editor.regularPolygons) {
      if (regPtr && regPtr->isValid()) {
        regPtr->setSelected(false);
        regPtr->setHoveredVertex(-1);
        regPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing regular polygons in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all triangles and reset vertex states
  try {
    for (auto &triPtr : editor.triangles) {
      if (triPtr && triPtr->isValid()) {
        triPtr->setSelected(false);
        triPtr->setHoveredVertex(-1);
        triPtr->setActiveVertex(-1);
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception processing triangles in deselectAll: " << e.what() << std::endl;
  }

  // Deselect all angles
  try {
    for (auto &anglePtr : editor.angles) {
      if (anglePtr && anglePtr->isValid()) {
        anglePtr->setSelected(false);
      }
    }
  } catch (const std::exception &e) {
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
      } catch (const std::exception &e) {
        std::cerr << "Error deselecting selected object (ID: " << editor.selectedObject->getID()
                  << ", which exists in lists): " << e.what() << std::endl;
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
void createObjectPointOnLine(GeometryEditor &editor, Line *lineHost, const Point_2 &cgalWorldPos) {
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
  } catch (const std::exception &e) {
    std::cerr << "Error getting line endpoints: " << e.what() << std::endl;
    return;
  }

  // ✅ Use ProjectionUtils for cleaner calculation
  double relativePos;
  try {
    // Add this function to ProjectionUtils.h first (see below)
    relativePos = ProjectionUtils::getRelativePositionOnLine(cgalWorldPos, startPos, endPos,
                                                             lineHost->isSegment());

    std::cout << "Calculated relative position: " << relativePos << std::endl;
  } catch (const std::exception &e) {
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
    auto newObjPoint =
        ObjectPoint::create(hostLineShared, relativePos, Constants::OBJECT_POINT_DEFAULT_COLOR);

    if (newObjPoint && newObjPoint->isValid()) {
      editor.ObjectPoints.push_back(newObjPoint);
      std::cout << "ObjectPoint created successfully at position " << relativePos << std::endl;
      std::cout << "Total ObjectPoints: " << editor.ObjectPoints.size() << std::endl;
      editor.setGUIMessage("ObjectPoint created on line");
    } else {
      std::cerr << "Failed to create valid ObjectPoint" << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error creating ObjectPoint: " << e.what() << std::endl;
  }
}

void createObjectPointOnCircle(GeometryEditor &editor, std::shared_ptr<Circle> circlePtr,
                               const Point_2 &clickPos) {
  try {
    if (!circlePtr) {
      std::cerr << "ERROR: circlePtr is null" << std::endl;
      return;
    }

    // Calculate parameter on circle
    Point_2 center = circlePtr->getCenterPoint();
    Vector_2 vectorToClick = clickPos - center;
    double angleRad =
        std::atan2(CGAL::to_double(vectorToClick.y()), CGAL::to_double(vectorToClick.x()));

    // Create ObjectPoint
    auto objectPoint = ObjectPoint::create(circlePtr, angleRad, sf::Color::Red);

    if (objectPoint) {
      editor.ObjectPoints.push_back(objectPoint);
      std::cout << "ObjectPoint created successfully on circle" << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "ERROR creating ObjectPoint: " << e.what() << std::endl;
  }
}
// UNIVERSAL SNAP FUNCTION: Try to snap to any nearby object (line, circle, etc)
// Returns a valid Point (either ObjectPoint on object or free point at position)
// The caller is responsible for adding it to appropriate container
std::shared_ptr<Point> trySnapToNearbyObject(GeometryEditor &editor, 
                                             const sf::Vector2f &worldPos_sfml,
                                             const Point_2 &cgalWorldPos,
                                             float snapTolerance,
                                             bool pointSnapEnabled) {
  if (!pointSnapEnabled) return nullptr;  // Snap only when modifier is active
  // Try to snap to circles first (both center and circumference)
  
  // 1. Check circle centers with 4x tolerance
  for (auto &circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid()) continue;
    
    Point_2 center = circlePtr->getCenterPoint();
    float distToCenter = std::sqrt(CGAL::to_double(CGAL::squared_distance(cgalWorldPos, center)));
    
    if (distToCenter < snapTolerance * 4.0f) {
      // Check if ObjectPoint already exists at circle center
      for (auto &objPt : editor.ObjectPoints) {
        if (objPt && objPt->isValid() && objPt->getHostType() == ObjectType::Circle) {
          GeometricObject *hostRaw = objPt->getHostObject();
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
  for (auto &circlePtr : editor.circles) {
    if (!circlePtr || !circlePtr->isValid()) continue;
    
    float circumferenceTolerance = snapTolerance * 3.0f;
    if (circlePtr->isCircumferenceHovered(worldPos_sfml, circumferenceTolerance)) {
      Point_2 projectedPoint = circlePtr->projectOntoCircumference(cgalWorldPos);
      
      // Check for existing ObjectPoint at this location
      for (auto &objPt : editor.ObjectPoints) {
        if (objPt && objPt->isValid() && objPt->getHostType() == ObjectType::Circle) {
          GeometricObject *hostRaw = objPt->getHostObject();
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
  for (auto &linePtr : editor.lines) {
    if (!linePtr || !linePtr->isValid()) continue;
    
    // Check if mouse is near the line
    if (linePtr->contains(worldPos_sfml, snapTolerance * 3.0f)) {
      // Project point onto line to get CGAL position
      Point_2 projPos = cgalWorldPos;
      
      // Try to get the projection using line endpoints
      try {
        Point_2 startPos = linePtr->getStartPoint();
        Point_2 endPos = linePtr->getEndPoint();
        
        // Calculate relative position on line
        double relativePos = ProjectionUtils::getRelativePositionOnLine(
            cgalWorldPos, startPos, endPos, linePtr->isSegment());
        
        // Check for existing ObjectPoint at this location
        for (auto &objPt : editor.ObjectPoints) {
          if (objPt && objPt->isValid() && objPt->getHostType() == ObjectType::Line) {
            GeometricObject *hostRaw = objPt->getHostObject();
            if (hostRaw == linePtr.get()) {
              // Check if ObjectPoint is at roughly the same position
              ObjectPoint *objPointPtr = dynamic_cast<ObjectPoint*>(objPt.get());
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
      } catch (const std::exception &e) {
        std::cerr << "[SNAP] Error projecting onto line: " << e.what() << std::endl;
      }
    }
  }
  
  // If no snap occurred, return nullptr
  return nullptr;
}

// Move these function implementations upwards so they're defined before they're
// used Add this new function to handle point creation
void handlePointCreation(GeometryEditor &editor, const sf::Event::MouseButtonEvent &mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);
  float tolerance = editor.getScaledTolerance(editor.drawingView);
  auto smartPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
  if (smartPoint && smartPoint->isValid()) {
    smartPoint->setSelected(true);

    bool exists = false;
    for (auto &p : editor.points) {
      if (p == smartPoint) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      for (auto &p : editor.ObjectPoints) {
        if (p == smartPoint) {
          exists = true;
          break;
        }
      }
    }

    if (!exists) {
      editor.points.push_back(smartPoint);
      std::cout << "Persisted new intersection point." << std::endl;
    }
  }

  std::cout << "Point created at (" << cgalWorldPos.x() << ", " << cgalWorldPos.y() << ")"
            << std::endl;
}

// Create a simplified line creation handler that will directly manipulate
// objects
void handleLineCreation(GeometryEditor &editor, const sf::Event::MouseButtonEvent &mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

    // Add coordinate validation FIRST
    const float MAX_COORD = 1e15f;
    if (!std::isfinite(worldPos_sfml.x) || !std::isfinite(worldPos_sfml.y) ||
        std::abs(worldPos_sfml.x) > MAX_COORD || std::abs(worldPos_sfml.y) > MAX_COORD) {
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

      if (!std::isfinite(x_double) || !std::isfinite(y_double) || std::abs(x_double) > MAX_COORD ||
          std::abs(y_double) > MAX_COORD) {
        std::cerr << "handleLineCreation: CGAL coordinates out of reasonable range" << std::endl;
        return;
      }
    } catch (const std::exception &e) {
      std::cerr << "Error converting position to CGAL: " << e.what() << std::endl;
      return;
    }

    float tolerance = editor.getScaledTolerance(editor.drawingView);

    std::cout << "Line creation: Click at (" << CGAL::to_double(cgalWorldPos.x()) << ", "
              << CGAL::to_double(cgalWorldPos.y()) << ")" << std::endl;

    std::shared_ptr<Point> clickedPoint =
        PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
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
      if (!editor.lineCreationPoint1 || !editor.lineCreationPoint1->isValid() || !secondPoint ||
          !secondPoint->isValid()) {
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
        if (!CGAL::is_finite(p1_pos_final.x()) || !CGAL::is_finite(p1_pos_final.y()) ||
            !CGAL::is_finite(p2_pos_final.x()) || !CGAL::is_finite(p2_pos_final.y())) {
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
        } catch (const std::exception &e) {
          std::cerr << "Error: Cannot convert line endpoint coordinates to double: " << e.what()
                    << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // Validate converted coordinates
        if (!std::isfinite(p1_x_final) || !std::isfinite(p1_y_final) ||
            !std::isfinite(p2_x_final) || !std::isfinite(p2_y_final)) {
          std::cerr << "Error: Line endpoint coordinates are not finite doubles" << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // Check coordinate ranges
        const double MAX_COORD_SAFE = 1e8;
        if (std::abs(p1_x_final) > MAX_COORD_SAFE || std::abs(p1_y_final) > MAX_COORD_SAFE ||
            std::abs(p2_x_final) > MAX_COORD_SAFE || std::abs(p2_y_final) > MAX_COORD_SAFE) {
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
          std::cerr << "Error: Final line endpoints are too close (distance² = "
                    << distSquared_final << ", min required = " << minDistSquared << ")"
                    << std::endl;
          editor.lineCreationPoint1->setSelected(false);
          editor.lineCreationPoint1 = nullptr;
          editor.dragMode = DragMode::None;
          return;
        }

        // If we get here, coordinates are safe - proceed with line creation
        bool isSegment = (editor.m_currentToolType == ObjectType::LineSegment);
        std::cout << "Creating " << (isSegment ? "segment" : "line") << " between validated points."
                  << std::endl;

        // Now safe to create the line
        auto newLine = std::make_shared<Line>(editor.lineCreationPoint1, secondPoint, isSegment,
                                              editor.getCurrentColor(), editor.objectIdCounter++);

        if (!newLine || !newLine->isValid()) {
          std::cerr << "Error: Failed to create valid " << (isSegment ? "segment" : "line")
                    << std::endl;
        } else {
          newLine->registerWithEndpoints();
          editor.lines.push_back(newLine);
          std::cout << (isSegment ? "Segment" : "Line") << " created successfully." << std::endl;
        }

      } catch (const std::exception &e) {
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

  } catch (const std::exception &e) {
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
void handleParallelLineCreation(GeometryEditor &editor,
                                const sf::Event::MouseButtonEvent &mouseEvent) {
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
  } catch (const std::exception &e) {
    std::cerr << "Error converting click to CGAL point in ParallelLineCreation: " << e.what()
              << std::endl;
    editor.resetParallelLineToolState();
    return;
  }
  float tolerance = editor.getScaledTolerance(editor.drawingView);

  try {
    if (!editor.m_isPlacingParallel) {  // First click: Select reference
      editor.resetParallelLineToolState();
      
      // Look for ANY object that might support edge alignment
      GeometricObject *refObj = editor.lookForObjectAt(worldPos_sfml, tolerance); // Empty list = all types

      if (refObj) {
        // Prepare to store reference
        std::shared_ptr<GeometricObject> refObjSP;
        int edgeIndex = -1;
        Vector_2 refDirection;
        bool validReferenceFound = false;

        // Case 1: Lines (keep existing behavior/optimization)
        if (refObj->getType() == ObjectType::Line || refObj->getType() == ObjectType::LineSegment) {
             Line *rawLinePtr = static_cast<Line *>(refObj);
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
                    for(auto& r : editor.rectangles) if(r.get() == refObj) refObjSP = r;
                 } else if (refObj->getType() == ObjectType::Triangle) {
                    for(auto& t : editor.triangles) if(t.get() == refObj) refObjSP = t;
                 } else if (refObj->getType() == ObjectType::Polygon) {
                    for(auto& p : editor.polygons) if(p.get() == refObj) refObjSP = p;
                 } else if (refObj->getType() == ObjectType::RegularPolygon) {
                    for(auto& rp : editor.regularPolygons) if(rp.get() == refObj) refObjSP = rp;
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
      if (std::abs(worldPos_sfml.y) <= tolerance) isHorizontalAxis = true;
      else if (std::abs(worldPos_sfml.x) <= tolerance) isVerticalAxis = true;

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
      std::shared_ptr<Point> anchor =
          PointUtils::findAnchorPoint(editor, worldPos_sfml, tolerance * 1.5f);
      if (anchor) {
        clickedExistingPt = anchor;
        anchorPos_cgal = anchor->getCGALPosition();

        // If this is a newly created vertex ObjectPoint, register it in the editor
        bool exists = false;
        for (const auto &p : editor.points) {
          if (p == clickedExistingPt) {
            exists = true;
            break;
          }
        }
        if (!exists) {
          for (const auto &p : editor.ObjectPoints) {
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
          const auto &hit = edgeHit.value();

          if (Circle *circle = dynamic_cast<Circle *>(hit.host)) {
            for (auto &circlePtr : editor.circles) {
              if (circlePtr.get() == circle) {
                auto objPoint = ObjectPoint::create(circlePtr,
                                                    hit.relativePosition * 2.0 * M_PI,
                                                    Constants::OBJECT_POINT_DEFAULT_COLOR);
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
            auto findHost = [&](auto &container) {
              for (auto &shape : container) {
                if (shape.get() == hit.host) {
                  hostPtr = std::static_pointer_cast<GeometricObject>(shape);
                  return true;
                }
              }
              return false;
            };

            if (findHost(editor.rectangles) || findHost(editor.polygons) ||
                findHost(editor.regularPolygons) || findHost(editor.triangles)) {
              if (hostPtr) {
                auto objPoint =
                    ObjectPoint::createOnShapeEdge(hostPtr, hit.edgeIndex, hit.relativePosition);
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
        for (auto &linePtr : editor.lines) {
          if (auto refObj = editor.m_parallelReference.lock()) {
            if (linePtr && linePtr.get() != refObj.get() &&
                linePtr->contains(worldPos_sfml, tolerance)) {
              // Project the click point onto this line for better snapping
              try {
                Line_2 hostLine = linePtr->getCGALLine();
                Point_2 projectedPoint = hostLine.projection(cgalWorldPos);
                anchorPos_cgal = projectedPoint;
                break;
              } catch (const std::exception &e) {
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
              } catch (const std::exception &e) {
                std::cerr << "Error projecting to line: " << e.what() << std::endl;
              }
            }
          }
        }
      }

      // Create the parallel line
      Vector_2 unit_construction_dir;
      try {
        unit_construction_dir = CGALSafeUtils::normalize_vector_robust(
            editor.m_parallelReferenceDirection, "ParallelCreate_normalize_ref_dir");
      } catch (const std::runtime_error &e) {
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
        p2_final_pos =
            finalStartPoint->getCGALPosition() + (unit_construction_dir * constructionLength_ft);

        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR,
                                             editor.objectIdCounter++);
        if (!newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Parallel): Failed to create valid newP2." << std::endl;
          editor.resetParallelLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP2);
        finalEndPoint = newP2;
      } else {
        auto newP1 = std::make_shared<Point>(p1_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR,
                                             editor.objectIdCounter++);
        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR,
                                             editor.objectIdCounter++);

        if (!newP1 || !newP1->isValid() || !newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Parallel): Failed to create valid points." << std::endl;
          editor.resetParallelLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP1);
        editor.points.push_back(newP2);
        finalStartPoint = newP1;
        finalEndPoint = newP2;
      }

      if (finalStartPoint && finalEndPoint && finalStartPoint->isValid() &&
          finalEndPoint->isValid()) {
        auto newLine =
            std::make_shared<Line>(finalStartPoint, finalEndPoint, false,
                                   Constants::CONSTRUCTION_LINE_COLOR, editor.objectIdCounter++);
        if (newLine && newLine->isValid()) {
          newLine->registerWithEndpoints(); // Fix propagation lag
          editor.lines.push_back(newLine);

          // Use weak_ptr safely for setting parallel constraint
          if (auto refObj = editor.m_parallelReference.lock()) {
             newLine->setAsParallelLine(refObj, editor.m_parallelReference.edgeIndex,
                                        editor.m_parallelReferenceDirection);
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
  } catch (const std::exception &e) {
    std::cerr << "CRITICAL EXCEPTION in handleParallelLineCreation: " << e.what() << std::endl;
    editor.resetParallelLineToolState();
  }
}

void handlePerpendicularLineCreation(GeometryEditor &editor,
                                     const sf::Event::MouseButtonEvent &mouseEvent) {
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
  } catch (const std::exception &e) {
    std::cerr << "Error converting click to CGAL point in "
                 "PerpendicularLineCreation: "
              << e.what() << std::endl;
    editor.resetPerpendicularLineToolState();
    return;
  }
  float tolerance = editor.getScaledTolerance(editor.drawingView);

  try {
    if (!editor.m_isPlacingPerpendicular) {  // First click: Select reference
      editor.resetPerpendicularLineToolState();

      // Look for ANY object that might support edge alignment
      GeometricObject *refObj = editor.lookForObjectAt(worldPos_sfml, tolerance); // Empty list = all types

      if (refObj) {
        // Prepare to store reference
        std::shared_ptr<GeometricObject> refObjSP;
        int edgeIndex = -1;
        Vector_2 refDirection;
        bool validReferenceFound = false;

        // Case 1: Lines (keep existing behavior/optimization)
        if (refObj->getType() == ObjectType::Line || refObj->getType() == ObjectType::LineSegment) {
             Line *rawLinePtr = static_cast<Line *>(refObj);
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
                    for(auto& r : editor.rectangles) if(r.get() == refObj) refObjSP = r;
                 } else if (refObj->getType() == ObjectType::Triangle) {
                    for(auto& t : editor.triangles) if(t.get() == refObj) refObjSP = t;
                 } else if (refObj->getType() == ObjectType::Polygon) {
                    for(auto& p : editor.polygons) if(p.get() == refObj) refObjSP = p;
                 } else if (refObj->getType() == ObjectType::RegularPolygon) {
                    for(auto& rp : editor.regularPolygons) if(rp.get() == refObj) refObjSP = rp;
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
      if (std::abs(worldPos_sfml.y) <= tolerance) isHorizontalAxis = true;
      else if (std::abs(worldPos_sfml.x) <= tolerance) isVerticalAxis = true;

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
      std::shared_ptr<Point> clickedExistingPt =
          PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
      if (clickedExistingPt) {
        anchorPos_cgal = clickedExistingPt->getCGALPosition();
      }

      // Create perpendicular direction
      Vector_2 perp_to_ref_dir(-editor.m_perpendicularReferenceDirection.y(),
                               editor.m_perpendicularReferenceDirection.x());

      Vector_2 unit_construction_dir;
      try {
        unit_construction_dir = CGALSafeUtils::normalize_vector_robust(
            perp_to_ref_dir, "PerpCreate_normalize_perp_dir");
      } catch (const std::runtime_error &e) {
        std::cerr << "CRITICAL_ERROR (Perp): Normalizing construction direction: " << e.what()
                  << std::endl;
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
        p2_final_pos =
            finalStartPoint->getCGALPosition() + (unit_construction_dir * constructionLength_ft);

        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR,
                                             editor.objectIdCounter++);
        if (!newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Perp): Failed to create valid newP2." << std::endl;
          editor.resetPerpendicularLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP2);
        finalEndPoint = newP2;
      } else {
        auto newP1 = std::make_shared<Point>(p1_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR,
                                             editor.objectIdCounter++);
        auto newP2 = std::make_shared<Point>(p2_final_pos, 1.0f, Constants::POINT_DEFAULT_COLOR,
                                             editor.objectIdCounter++);

        if (!newP1 || !newP1->isValid() || !newP2 || !newP2->isValid()) {
          std::cerr << "CRITICAL_ERROR (Perp): Failed to create valid points." << std::endl;
          editor.resetPerpendicularLineToolState();
          return;
        }
        newP2->setVisible(false);
        editor.points.push_back(newP1);
        editor.points.push_back(newP2);
        finalStartPoint = newP1;
        finalEndPoint = newP2;
      }

      if (finalStartPoint && finalEndPoint && finalStartPoint->isValid() &&
          finalEndPoint->isValid()) {
        auto newLine =
            std::make_shared<Line>(finalStartPoint, finalEndPoint, false,
                                   Constants::CONSTRUCTION_LINE_COLOR, editor.objectIdCounter++);
        if (newLine && newLine->isValid()) {
          newLine->registerWithEndpoints(); // Fix propagation lag
          editor.lines.push_back(newLine);

          // Use weak_ptr safely for setting perpendicular constraint
          if (auto refObj = editor.m_perpendicularReference.lock()) {
             newLine->setAsPerpendicularLine(refObj, editor.m_perpendicularReference.edgeIndex,
                                             editor.m_perpendicularReferenceDirection);
          } else {
            // Axis-based perpendicular line
             Vector_2 perpDir(-editor.m_perpendicularReferenceDirection.y(),
                              editor.m_perpendicularReferenceDirection.x());
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
  } catch (const std::exception &e) {
    std::cerr << "CRITICAL EXCEPTION in handlePerpendicularLineCreation: " << e.what() << std::endl;
    editor.resetPerpendicularLineToolState();
  }
}
// Implementation for handleObjectPointCreation
void handleObjectPointCreation(GeometryEditor &editor,
                               const sf::Event::MouseButtonEvent &mouseEvent) {
  std::cout << "handleObjectPointCreation: ENTERED" << std::endl;

  // Convert mouse position
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  Point_2 cgalWorldPos = editor.toCGALPoint(worldPos);
  float tolerance = editor.getScaledTolerance(editor.drawingView);

  std::cout << "Looking for host at (" << worldPos.x << ", " << worldPos.y << ") with tolerance "
            << tolerance << std::endl;

  // Find host object (prioritize lines over circles)
  std::shared_ptr<Line> hostLine = nullptr;
  std::shared_ptr<Circle> hostCircle = nullptr;

  // First try lines (typically thinner and harder to hit)
  for (auto &line : editor.lines) {
    if (line && line->contains(worldPos, tolerance)) {
      hostLine = line;
      std::cout << "Found line host: " << line->getID() << std::endl;
      break;
    }
  }

  // If no line found, try circles
  if (!hostLine) {
    std::cout << "getCircleAtPosition: Checking " << editor.circles.size() << " circles"
              << std::endl;

    for (size_t i = 0; i < editor.circles.size(); ++i) {
      auto &circle_sp_in_list = editor.circles[i];

      std::cout << "  Circle[" << i << "]: ptr=" << circle_sp_in_list.get()
                << ", use_count=" << circle_sp_in_list.use_count() << std::endl;

      if (circle_sp_in_list && circle_sp_in_list->isValid() &&
          circle_sp_in_list->contains(worldPos, tolerance)) {
        std::cout << "  Found matching circle at index " << i << std::endl;

        // CRITICAL: Test shared_from_this() on the circle BEFORE using it
        try {
          auto shared_test = circle_sp_in_list->shared_from_this();
          std::cout << "  Circle shared_from_this() test: use_count=" << shared_test.use_count()
                    << std::endl;

          if (!shared_test) {
            std::cout << "  ERROR: Circle's shared_from_this() returned null!" << std::endl;
            std::cout << "  This indicates enable_shared_from_this was never initialized"
                      << std::endl;
            std::cout << "  Skipping this circle to avoid crash" << std::endl;
            continue;  // Skip this circle and try the next one
          }
        } catch (const std::exception &e) {
          std::cout << "  ERROR: Exception testing shared_from_this(): " << e.what() << std::endl;
          std::cout << "  Skipping this circle to avoid crash" << std::endl;
          continue;  // Skip this circle and try the next one
        }

        hostCircle = circle_sp_in_list;  // COPY the shared_ptr, don't move.
        if (hostCircle) {
          std::cout << "Found circle host: " << hostCircle->getID()
                    << ", current use_count: " << hostCircle.use_count() << std::endl;
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
      std::cout << "Passing hostCircle to createObjectPointOnCircle, use_count: "
                << hostCircle.use_count() << std::endl;
      // Pass shared_ptr directly!
      createObjectPointOnCircle(editor, hostCircle, cgalWorldPos);
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception creating ObjectPoint: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to create ObjectPoint");
  }
}

// Implementation for handleRectangleCreation
void handleRectangleCreation(GeometryEditor &editor,
                             const sf::Event::MouseButtonEvent &mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

    if (!editor.isCreatingRectangle) {
      // First click - set first corner
      editor.rectangleCorner1 = cgalWorldPos;
      editor.isCreatingRectangle = true;
      // Create a preview rectangle starting at the first corner
      try {
        editor.previewRectangle = std::make_shared<Rectangle>(
            editor.rectangleCorner1, editor.rectangleCorner1, false,
            editor.getCurrentColor(), editor.objectIdCounter /* temp id */);
      } catch (...) {
        editor.previewRectangle.reset();
      }
      editor.setGUIMessage("Rectangle: Click second corner");
      std::cout << "Rectangle first corner set at (" << CGAL::to_double(cgalWorldPos.x())
                << ", " << CGAL::to_double(cgalWorldPos.y()) << ")" << std::endl;
    } else {
      // Second click - create rectangle
      editor.rectangleCorner2 = cgalWorldPos;
      
      if (editor.rectangleCorner1.x() != editor.rectangleCorner2.x() &&
          editor.rectangleCorner1.y() != editor.rectangleCorner2.y()) {
        auto newRectangle = std::make_shared<Rectangle>(
            editor.rectangleCorner1, editor.rectangleCorner2, false, editor.getCurrentColor(),
            editor.objectIdCounter++);
        editor.rectangles.push_back(newRectangle);
        editor.setGUIMessage("Rectangle created");
        std::cout << "Rectangle created" << std::endl;
      } else {
        editor.setGUIMessage("Invalid rectangle: corners must differ");
      }

      editor.isCreatingRectangle = false;
      editor.previewRectangle.reset();
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception in handleRectangleCreation: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to create rectangle");
    editor.isCreatingRectangle = false;
  }
}

// Implementation for handleRotatableRectangleCreation
void handleRotatableRectangleCreation(GeometryEditor &editor,
                                      const sf::Event::MouseButtonEvent &mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

    if (!editor.isCreatingRotatableRectangle) {
      // First click - set first corner
      editor.rectangleCorner1 = cgalWorldPos;
      editor.isCreatingRotatableRectangle = true;
      // Initialize a preview (may be invisible until mouse moves)
      try {
        editor.previewRectangle = std::make_shared<Rectangle>(
            editor.rectangleCorner1, editor.rectangleCorner1, /*width*/ 0.0,
            editor.getCurrentColor(), editor.objectIdCounter /* temp id */);
      } catch (...) {
        editor.previewRectangle.reset();
      }
      editor.setGUIMessage("RotRect: Click adjacent point to define side");
      std::cout << "RotRect first corner set at (" << CGAL::to_double(cgalWorldPos.x())
                << ", " << CGAL::to_double(cgalWorldPos.y()) << ")" << std::endl;
    } else {
      // Second click - set adjacent point and ask for width
      editor.rectangleCorner2 = cgalWorldPos;
      
      double dx = CGAL::to_double(editor.rectangleCorner2.x() - editor.rectangleCorner1.x());
      double dy = CGAL::to_double(editor.rectangleCorner2.y() - editor.rectangleCorner1.y());
      double sideLength = std::sqrt(dx * dx + dy * dy);

      if (sideLength > Constants::MIN_CIRCLE_RADIUS) {
        // For simplicity, create rectangle with width = sideLength and default height
        double defaultHeight = sideLength;  // Make it square for now
        auto newRectangle = std::make_shared<Rectangle>(
            editor.rectangleCorner1, editor.rectangleCorner2, defaultHeight,
            editor.getCurrentColor(), editor.objectIdCounter++);
        editor.rectangles.push_back(newRectangle);
        editor.setGUIMessage("Rotatable rectangle created");
        std::cout << "Rotatable rectangle created" << std::endl;
      } else {
        editor.setGUIMessage("Invalid rectangle: side too small");
      }

      editor.isCreatingRotatableRectangle = false;
      editor.previewRectangle.reset();
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception in handleRotatableRectangleCreation: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to create rotatable rectangle");
    editor.isCreatingRotatableRectangle = false;
  }
}

// Implementation for handlePolygonCreation
void handlePolygonCreation(GeometryEditor &editor,
                           const sf::Event::MouseButtonEvent &mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

    if (!editor.isCreatingPolygon) {
      // Start polygon creation
      editor.isCreatingPolygon = true;
      editor.polygonVertices.clear();
      // Add first vertex
      editor.polygonVertices.push_back(cgalWorldPos);
      std::cout << "Polygon creation started. Vertex 1 added." << std::endl;
    } else {
        // Check if clicking near start point to close loop
        if (editor.polygonVertices.size() >= 3) {
             Point_2 startP = editor.polygonVertices[0];
             sf::Vector2f startSfml = editor.toSFMLVector(startP);
             float dist = editor.length(startSfml - worldPos_sfml);
             if (dist < editor.getScaledTolerance(editor.drawingView) * 1.5f) {
                 // Close and Create
                 auto newPoly = std::make_shared<Polygon>(editor.polygonVertices, editor.getCurrentColor(),
                                            editor.objectIdCounter++);
                 editor.polygons.push_back(newPoly);
                 editor.isCreatingPolygon = false;
                 editor.polygonVertices.clear();
                 editor.previewPolygon.reset();
                 editor.setGUIMessage("Polygon created successfully.");
                 std::cout << "Polygon creation finished (closed loop)." << std::endl;
                 return;
             }
        }
        
        // Add vertex
        editor.polygonVertices.push_back(cgalWorldPos);
        std::cout << "Polygon vertex " << editor.polygonVertices.size() << " added." << std::endl;
    }

    if (editor.polygonVertices.size() >= 1) {
      // Update preview polygon
      editor.previewPolygon =
          std::make_shared<Polygon>(editor.polygonVertices, editor.getCurrentColor(),
                                    editor.objectIdCounter); // Temporary ID
    }

    editor.setGUIMessage("Polygon: Click to add vertices (click start to finish), Enter to finish, Esc to cancel");
  } catch (const std::exception &e) {
    std::cerr << "Exception in handlePolygonCreation: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to add polygon vertex");
  }
}

// Implementation for handleRegularPolygonCreation
void handleRegularPolygonCreation(GeometryEditor &editor,
                                  const sf::Event::MouseButtonEvent &mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

    if (editor.regularPolygonPhase == 0) {
      // First click - set center
      editor.regularPolygonCenter = cgalWorldPos;
      editor.regularPolygonPhase = 1;
      editor.setGUIMessage("RegPoly: Click to define radius");
      std::cout << "RegPoly center set at (" << CGAL::to_double(cgalWorldPos.x()) << ", "
                << CGAL::to_double(cgalWorldPos.y()) << ")" << std::endl;
    } else if (editor.regularPolygonPhase == 1) {
      // Second click - set first vertex (defines radius)
      editor.regularPolygonFirstVertex = cgalWorldPos;
      
      double dx = CGAL::to_double(editor.regularPolygonFirstVertex.x() - editor.regularPolygonCenter.x());
      double dy = CGAL::to_double(editor.regularPolygonFirstVertex.y() - editor.regularPolygonCenter.y());
      double radius = std::sqrt(dx * dx + dy * dy);

      if (radius > Constants::MIN_CIRCLE_RADIUS) {
        auto newRegPoly = std::make_shared<RegularPolygon>(
            editor.regularPolygonCenter, editor.regularPolygonFirstVertex,
            editor.regularPolygonNumSides, editor.getCurrentColor(), editor.objectIdCounter++);
        editor.regularPolygons.push_back(newRegPoly);
        editor.setGUIMessage("Regular polygon created (6 sides)");
        std::cout << "Regular polygon created with " << editor.regularPolygonNumSides
                  << " sides and radius " << radius << std::endl;

        editor.regularPolygonPhase = 0;
      } else {
        editor.setGUIMessage("Invalid polygon: radius too small");
        editor.regularPolygonPhase = 0;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception in handleRegularPolygonCreation: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to create regular polygon");
    editor.regularPolygonPhase = 0;
  }
}

// Implementation for handleTriangleCreation (general triangle with 3 arbitrary vertices)
void handleTriangleCreation(GeometryEditor &editor,
                            const sf::Event::MouseButtonEvent &mouseEvent) {
  if (mouseEvent.button != sf::Mouse::Left) {
    return;
  }

  try {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    sf::Vector2f worldPos_sfml = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
    Point_2 cgalWorldPos = editor.toCGALPoint(worldPos_sfml);

    // Add vertex to triangle
    editor.triangleVertices.push_back(cgalWorldPos);
    std::cout << "Triangle vertex " << editor.triangleVertices.size() << " added at ("
              << CGAL::to_double(cgalWorldPos.x()) << ", " 
              << CGAL::to_double(cgalWorldPos.y()) << ")" << std::endl;

    if (editor.triangleVertices.size() == 1) {
      editor.setGUIMessage("Triangle: Click second vertex");
      editor.isCreatingTriangle = true;
    } else if (editor.triangleVertices.size() == 2) {
      editor.setGUIMessage("Triangle: Click third vertex");
      
      // Create preview line showing first two vertices
      editor.previewTriangle = nullptr;  // Clear any existing preview
    } else if (editor.triangleVertices.size() == 3) {
      // Validate non-collinearity using CGAL
      if (!CGAL::collinear(editor.triangleVertices[0], 
                           editor.triangleVertices[1], 
                           editor.triangleVertices[2])) {
        auto newTriangle = std::make_shared<Triangle>(
            editor.triangleVertices[0],
            editor.triangleVertices[1],
            editor.triangleVertices[2],
            editor.getCurrentColor(),
            editor.objectIdCounter++
        );
        editor.triangles.push_back(newTriangle);
        editor.setGUIMessage("Triangle created");
        std::cout << "Triangle created successfully" << std::endl;
      } else {
        editor.setGUIMessage("Error: Points are collinear - cannot form a triangle");
        std::cout << "Triangle creation failed: vertices are collinear" << std::endl;
      }
      
      // Reset state
      editor.triangleVertices.clear();
      editor.isCreatingTriangle = false;
      editor.previewTriangle = nullptr;
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception in handleTriangleCreation: " << e.what() << std::endl;
    editor.setGUIMessage("Error: Failed to create triangle");
    editor.triangleVertices.clear();
    editor.isCreatingTriangle = false;
    editor.previewTriangle = nullptr;
  }
}


void handleMousePress(GeometryEditor &editor, const sf::Event::MouseButtonEvent &mouseEvent) {
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
  bool gridSnapActive = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) ||
                        sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
  if (gridSnapActive) {
    float g = Constants::GRID_SIZE;
    worldPos_sfml.x = std::round(worldPos_sfml.x / g) * g;
    worldPos_sfml.y = std::round(worldPos_sfml.y / g) * g;
  }

  // Debug the mouse position
  std::cout << "Mouse press at pixel: (" << pixelPos.x << ", " << pixelPos.y << "), world: ("
            << worldPos_sfml.x << ", " << worldPos_sfml.y << ")" << std::endl;

  Point_2 worldPos_cgal;
  try {
    worldPos_cgal = editor.toCGALPoint(worldPos_sfml);
    std::cout << "Converted to CGAL: (" << CGAL::to_double(worldPos_cgal.x()) << ", "
              << CGAL::to_double(worldPos_cgal.y()) << ")" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error converting to CGAL point: " << e.what() << std::endl;
  }

  float tolerance = editor.getScaledTolerance(editor.drawingView);
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

  // Removed Legacy Right-Click logic block that blocked Context Menu


  if (mouseEvent.button == sf::Mouse::Right && sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Right+Ctrl at: (" << worldPos_sfml.x << ", "
              << worldPos_sfml.y << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Space+Left at: (" << worldPos_sfml.x << ", "
              << worldPos_sfml.y << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Left && sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt)) {
    editor.isPanning = true;
    editor.lastMousePos_sfml = worldPos_sfml;
    std::cout << "Panning started with Alt+Left at: (" << worldPos_sfml.x << ", " << worldPos_sfml.y
              << ")" << std::endl;
    return;
  }
  if (mouseEvent.button == sf::Mouse::Right) {
    // Double Right-Click Detection for Context Menu
    static sf::Clock lastRightClickClock;
    static int rightClickCount = 0;
    
    float timeSinceLast = lastRightClickClock.getElapsedTime().asSeconds();
    if (timeSinceLast < 0.4f) { // Slightly generous limit
        rightClickCount++;
    } else {
        rightClickCount = 1;
    }
    lastRightClickClock.restart();

    if (rightClickCount == 2) {
        // Use tolerance consistent with other tools
        float tolerance = editor.getScaledTolerance(editor.drawingView);
        GeometricObject* hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance); 
        
        if (hitObj) {
            editor.selectedObject = hitObj;
            // Map to GUI coordinates (using default view to match GUI overlay)
            sf::View DefaultView = editor.window.getDefaultView(); 
            sf::Vector2f guiPos = editor.window.mapPixelToCoords(
                sf::Vector2i(mouseEvent.x, mouseEvent.y), DefaultView);
                
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
      float tolerance = editor.getScaledTolerance(editor.drawingView);
      GeometricObject *hitObj = editor.lookForObjectAt(worldPos_sfml, tolerance);
      if (hitObj) {
        hitObj->setVisible(!hitObj->isVisible()); // Toggle visibility
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
      float tolerance = editor.getScaledTolerance(editor.drawingView);
      double bestDistSq = static_cast<double>(tolerance) * static_cast<double>(tolerance);
      Line *bestLine = nullptr;
      EndpointSelection bestEndpoint = EndpointSelection::None;

      Point_2 clickPos = editor.toCGALPoint(worldPos_sfml);

      for (auto &linePtr : editor.lines) {
        if (!linePtr || !linePtr->isValid() || !linePtr->isVisible()) continue;
        auto start = linePtr->getStartPointObjectShared();
        auto end = linePtr->getEndPointObjectShared();

        if (start) {
          double d1 = CGAL::to_double(CGAL::squared_distance(clickPos, start->getCGALPosition()));
          if (d1 <= bestDistSq) {
            bestDistSq = d1;
            bestLine = linePtr.get();
            bestEndpoint = EndpointSelection::Start;
          }
        }

        if (end) {
          double d2 = CGAL::to_double(CGAL::squared_distance(clickPos, end->getCGALPosition()));
          if (d2 <= bestDistSq) {
            bestDistSq = d2;
            bestLine = linePtr.get();
            bestEndpoint = EndpointSelection::End;
          }
        }
      }

      if (bestLine && bestEndpoint != EndpointSelection::None) {
        auto endpointPoint = (bestEndpoint == EndpointSelection::Start)
                                 ? bestLine->getStartPointObjectShared()
                                 : bestLine->getEndPointObjectShared();
        if (!endpointPoint || !endpointPoint->isValid()) {
          return;
        }

        int usageCount = 0;
        for (auto &linePtr : editor.lines) {
          if (!linePtr || !linePtr->isValid()) continue;
          if (linePtr->getStartPointObjectShared() == endpointPoint ||
              linePtr->getEndPointObjectShared() == endpointPoint) {
            ++usageCount;
          }
        }

        if (usageCount > 1) {
          auto newPoint = std::make_shared<Point>(endpointPoint->getCGALPosition(), 1.0f,
                                                   endpointPoint->getFillColor(),
                                                   endpointPoint->getOutlineColor());
          newPoint->setVisible(true);
          newPoint->update();
          editor.points.push_back(newPoint);

          if (bestEndpoint == EndpointSelection::Start) {
            bestLine->setPoints(newPoint, bestLine->getEndPointObjectShared());
          } else {
            bestLine->setPoints(bestLine->getStartPointObjectShared(), newPoint);
          }

          editor.selectedObject = newPoint.get();
          editor.selectedObject->setSelected(true);
          editor.setGUIMessage("Endpoint detached.");
        } else {
          editor.setGUIMessage("Endpoint is not shared.");
        }
      }
      return;
    }
  }

  // Angle tool handling
  if (editor.m_currentToolType == ObjectType::Angle) {
    if (mouseEvent.button == sf::Mouse::Left) {
      float tolerance = editor.getScaledTolerance(editor.drawingView);
      
      // 1. Try to find a Point first (standard 3-point method)
      auto anchor = PointUtils::findAnchorPoint(editor, worldPos_sfml, tolerance * 1.5f);
      
      if (anchor && anchor->isValid()) {
        auto ensurePointStored = [&](const std::shared_ptr<Point> &pt) {
            if (!pt) return;
            for (auto &p : editor.points) if (p == pt) return;
            for (auto &op : editor.ObjectPoints) if (op == pt) return;
            // Store if new (auto-created helper points?)
            // Actually findAnchorPoint usually returns existing points.
            // If it returns a temporary smart point, we might need to store it.
            if(std::find(editor.points.begin(), editor.points.end(), pt) == editor.points.end() && 
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
          auto angle = std::make_shared<Angle>(editor.anglePointA, editor.angleVertex,
                                               editor.anglePointB, false, editor.getCurrentColor());
          angle->setVisible(true);
          angle->update();
          editor.angles.push_back(angle);
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
          if(editor.anglePointA) editor.anglePointA = nullptr;
          if(editor.angleVertex) editor.angleVertex = nullptr;

          Line* lineRaw = static_cast<Line*>(hitObj);
          auto linePtr = editor.getLineSharedPtr(lineRaw);
          
          if (linePtr) {
              if (!editor.angleLine1) {
                  editor.angleLine1 = linePtr;
                  editor.angleLine1->setSelected(true);
                  editor.setGUIMessage("Angle: Line 1 selected. Click Line 2.");
              } else if (!editor.angleLine2 && linePtr != editor.angleLine1) {
                  editor.angleLine2 = linePtr;
                  // editor.angleLine2->setSelected(true); // Will be cleared soon
                  
                  // Calculate Intersection
                  auto result = CGAL::intersection(editor.angleLine1->getCGALLine(), editor.angleLine2->getCGALLine());
                  if (result) {
                      using ActualVariantType = std::decay_t<decltype(*result)>;
                      const Point_2* intersectionPt = nullptr;
                      Point_2 tempPt;
                      
                      if(const Point_2* p = std::get_if<Point_2>(&(*result))) {
                          intersectionPt = p;
                      } else if (const auto* p = std::get_if<Point_2>(&(*result))) { 
                          // Handle variant access safely if different variant type
                          intersectionPt = p;
                      }
                      // Handle boost variant if needed (not standard here but covered in other parts)
                      
                       // Simplified variant handling for this context:
                       // We assume typical CGAL variants. If lines are parallel, result is empty or Line_2.
                       
                       if (!intersectionPt) {
                           // If std::get_if failed, it might be parallel (no point).
                           // Safe to assume no point if get_if failed for Point_2.
                       }

                       if (intersectionPt) {
                           // Create Smart Vertex
                           // Check if there's already a point there
                           auto vertex = PointUtils::createSmartPoint(editor, editor.toSFMLVector(*intersectionPt), tolerance);
                           // Force add if not in list
                            if(std::find(editor.points.begin(), editor.points.end(), vertex) == editor.points.end() && 
                               std::find(editor.ObjectPoints.begin(), editor.ObjectPoints.end(), vertex) == editor.ObjectPoints.end()) {
                                 vertex->setVisible(false); // Helper point can be hidden? Or visible? 
                                 // Let's make it visible as it's an intersection
                                 vertex->setVisible(true);
                                 editor.points.push_back(vertex);
                            }
                           
                           // Select arms: Use endpoints furthest from vertex?
                           // Or endpoints that are NOT the vertex.
                           Point_2 vPos = vertex->getCGALPosition();
                           
                           auto getArmPoint = [&](const std::shared_ptr<Line>& l) -> std::shared_ptr<Point> {
                               Point_2 s = l->getStartPoint();
                               Point_2 e = l->getEndPoint();
                               double d1 = CGAL::to_double(CGAL::squared_distance(s, vPos));
                               double d2 = CGAL::to_double(CGAL::squared_distance(e, vPos));
                               Point_2 target = (d1 > d2) ? s : e;
                               
                               // Search for existing point object
                               for(auto& p : editor.points) {
                                   if (p && p->isValid() && p->getCGALPosition() == target) return p;
                               }
                               // Also check ObjectPoints
                               for(auto& p : editor.ObjectPoints) {
                                   if (p && p->isValid() && p->getCGALPosition() == target) return p; 
                               }
                               
                               // If not found, create new invisible point
                               auto newP = std::make_shared<Point>(target, 1.0f, sf::Color::Transparent);
                               newP->setVisible(false);
                               editor.points.push_back(newP);
                               return newP;
                           };
                           
                           auto p1 = getArmPoint(editor.angleLine1);
                           auto p2 = getArmPoint(editor.angleLine2);
                           
                           if (p1 && vertex && p2) {
                               auto angle = std::make_shared<Angle>(p1, vertex, p2, false, editor.getCurrentColor());
                                angle->setVisible(true);
                                angle->update();
                                editor.angles.push_back(angle);
                                std::cout << "Angle created via 2 lines." << std::endl;
                                editor.setGUIMessage("Angle created.");
                           }
                           
                       } else {
                           editor.setGUIMessage("Error: Lines do not intersect at a unique point.");
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
      float tolerance = editor.getScaledTolerance(editor.drawingView);
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

  // A. Circle Creation Tool
  if (editor.m_currentToolType == ObjectType::Circle) {
    if (mouseEvent.button == sf::Mouse::Left) {
      if (!editor.isCreatingCircle) {
        // Start circle creation - create the center point first
        sf::Vector2i mousePos = sf::Mouse::getPosition(editor.window);
        sf::Vector2f worldPos = editor.window.mapPixelToCoords(mousePos, editor.drawingView);
        float tolerance = editor.getScaledTolerance(editor.drawingView);
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
        std::cout << "Preview circle created at address: " << editor.previewCircle.get()
                  << std::endl;
        std::cout << "Creating circle with center at (" << CGAL::to_double(center.x()) << ", "
                  << CGAL::to_double(center.y()) << ") and radius 0" << std::endl;
      } else {
        // Complete circle creation
        try {
          float tolerance = editor.getScaledTolerance(editor.drawingView);
          auto radiusPoint = PointUtils::createSmartPoint(editor, worldPos_sfml, tolerance);
          if (!radiusPoint || !radiusPoint->isValid()) {
            return;
          }

          Point *centerPoint = editor.previewCircle ? editor.previewCircle->getCenterPointObject()
                                                    : nullptr;
          if (!centerPoint) {
            return;
          }

          Point_2 center = centerPoint->getCGALPosition();
          double radius_sq = CGAL::to_double(
              CGAL::squared_distance(center, radiusPoint->getCGALPosition()));
          double radius = std::sqrt(radius_sq);

          if (radius >= Constants::MIN_CIRCLE_RADIUS) {
            // The center point already exists from preview creation
            // Create the final circle with a persistent radius point
            sf::Color selectedColor = editor.previewCircle ? editor.previewCircle->getColor()
                                                           : editor.getCurrentColor();
            auto finalCircle = Circle::create(centerPoint, radiusPoint, radius, selectedColor);
            if (finalCircle && finalCircle->isValid()) {
              editor.circles.push_back(finalCircle);
              std::cout << "Circle Created with radius: " << radius << std::endl;
            } else {
              std::cerr << "Preview circle is invalid" << std::endl;
            }
          } else {
            std::cout << "Circle not created: radius too small." << std::endl;
          }
        } catch (const std::exception &e) {
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
  if (editor.m_currentToolType == ObjectType::Line ||
      editor.m_currentToolType == ObjectType::LineSegment) {
    if (mouseEvent.button == sf::Mouse::Left) {
      try {
        handleLineCreation(editor, mouseEvent);
      } catch (const std::exception &e) {
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
    GeometricObject *previousSelection = editor.selectedObject;

    // Attempt to find object under mouse cursor
    GeometricObject *potentialSelection = nullptr;
    DragMode potentialDragMode = DragMode::None;
    EndpointSelection potentialEndpointSelection = EndpointSelection::None;
    int potentialVertexIndex = -1;

    // bool objectFound = false; // Removed unused variable

    // Reset vertex drag state
    editor.activeVertexIndex = -1;
    editor.activeVertexShape = nullptr;

    // Debug what we're looking for
    std::cout << "Looking for object at (" << worldPos_sfml.x << ", " << worldPos_sfml.y
              << ") with tolerance " << tolerance << std::endl;

    // --- PASS 1: HIGHEST PRIORITY - POINTS & VERTICES ---
    // Tolerance is unchanged for these "precision" targets
    
    // 1. ObjectPoints (Snap points) - Highest Priority
    for (auto &objPointPtr : editor.ObjectPoints) {
      if (objPointPtr && objPointPtr->isValid() && objPointPtr->isVisible() &&
          objPointPtr->contains(worldPos_sfml, tolerance)) {
        std::cout << "Found ObjectPoint (Priority 1)" << std::endl;
        potentialSelection = objPointPtr.get();
        potentialDragMode = DragMode::DragObjectPoint;
        goto objectFoundLabel;
      }
    }

    // 2. Free Points (Entities) - Includes Line Endpoints if they are Points
    // This fixes the "Free Point Mobility" and "Line Endpoint" conflict by treating all Points as first-class draggables.
    for (auto &pointPtr : editor.points) {
      if (pointPtr && pointPtr->isValid() && pointPtr->isVisible() &&
          pointPtr->contains(worldPos_sfml, tolerance)) {
        std::cout << "Found Point (Priority 2)" << std::endl;
        potentialSelection = pointPtr.get();
        potentialDragMode = DragMode::MoveFreePoint;
        goto objectFoundLabel;
      }
    }

    // 3. Shape Vertices (Proxies)
    {
      auto checkVertexHit = [&](auto &container) -> bool {
        for (auto &ptr : container) {
          if (!ptr || !ptr->isValid() || !ptr->isVisible()) continue;
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
        for (auto &ptr : editor.regularPolygons) {
          if (!ptr || !ptr->isValid() || !ptr->isVisible()) continue;
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

      if (checkVertexHit(editor.rectangles) || checkVertexHit(editor.polygons) ||
          checkRegularPolygonCreationPoints() || checkVertexHit(editor.triangles)) {
        goto objectFoundLabel;
      }
    }

      // 3. Angles (selection only)
      for (auto &anglePtr : editor.angles) {
        if (anglePtr && anglePtr->isValid() && anglePtr->isVisible() &&
            anglePtr->contains(worldPos_sfml, tolerance)) {
          std::cout << "Found Angle (Priority 2.8)" << std::endl;
          potentialSelection = anglePtr.get();
          
          // Check for resizing interaction (clicking the arc itself)
          if (anglePtr->isMouseOverArc(worldPos_sfml, tolerance)) {
              editor.isResizingAngle = true;
              std::cout << "Angle Resize Mode Activated" << std::endl;
          } else {
              editor.isResizingAngle = false;
          }
          
          potentialDragMode = DragMode::None; // We handle resizing via the bool flag
          goto objectFoundLabel;
        }
      }

    // --- PASS 2: MEDIUM PRIORITY - LINEAR ELEMENTS (EDGES) ---
    // If we missed vertices, check edges.
    
    // 4. Lines (Bodies)
    for (auto &linePtr : editor.lines) {
      if (linePtr && linePtr->isValid() && linePtr->isVisible() &&
          linePtr->contains(worldPos_sfml, tolerance)) {
        std::cout << "Found Line Body (Priority 3)" << std::endl;
        potentialSelection = linePtr.get();
        potentialDragMode = DragMode::TranslateLine;
        editor.dragStart_sfml = worldPos_sfml;
        goto objectFoundLabel;
      }
    }

    // 5. Shape Edges (Explicit Edge Check)
    // This allows selecting a shape by its edge even if the fill is transparent or hit logic is strictly boundary-based
    {
       auto checkShapeEdges = [&](auto& container) -> bool {
           for(auto& shape : container) {
             if(!shape || !shape->isValid() || !shape->isVisible()) continue;
               auto edges = shape->getEdges();
               for(const auto& edge : edges) {
                   Point_2 proj;
                   double relPos;
                   // Use standard tolerance
                   double dist = PointUtils::projectPointOntoSegment(
                       editor.toCGALPoint(worldPos_sfml), edge, proj, relPos);
                   
                   // Compare squared distance for efficiency? PointUtils returns double distance.
                   if(dist < tolerance) { 
                       std::cout << "Found Shape Edge (Priority 3.5)" << std::endl;
                       potentialSelection = shape.get();
                       potentialDragMode = DragMode::TranslateShape;
                       return true;
                   }
               }
           }
           return false;
       };
       
       if(checkShapeEdges(editor.rectangles) || checkShapeEdges(editor.polygons) ||
          checkShapeEdges(editor.regularPolygons) || checkShapeEdges(editor.triangles)) {
          goto objectFoundLabel;
       }
    }

    // --- PASS 3: LOWEST PRIORITY - INTERIORS / BODIES ---
    // Only if nothing else was hit.
    {
        // 6. Circles (Center then Body)
        for (auto &circlePtr : editor.circles) {
          if (!circlePtr || !circlePtr->isValid() || !circlePtr->isVisible()) continue;
          // Center check (already covered by Point check if center is a Point obj, but Point might be hidden)
          float centerTolerance = tolerance * 4.0f; 
          if (circlePtr->isCenterPointHovered(worldPos_sfml, centerTolerance)) {
            potentialSelection = circlePtr.get();
            potentialDragMode = DragMode::InteractWithCircle; // Needs specific mode?
            editor.dragStart_sfml = worldPos_sfml;
             // Special case: if we want to move center, maybe drag mode TranslateShape? 
             // But logic elsewhere handles InteractWithCircle
            goto objectFoundLabel;
          } 
          else if (circlePtr->contains(worldPos_sfml, tolerance)) {
            potentialSelection = circlePtr.get();
            potentialDragMode = DragMode::InteractWithCircle;
            editor.dragStart_sfml = worldPos_sfml;
            goto objectFoundLabel;
          }
        }
    
        // 7. Shape Interiors (Global Contains)
        auto checkShapeContains = [&](auto& container) -> bool {
            for (auto &ptr : container) {
            if (!ptr || !ptr->isValid() || !ptr->isVisible()) continue;
                if (ptr->contains(worldPos_sfml, tolerance)) {
                    std::cout << "Found Shape Interior (Priority 4)" << std::endl;
                    potentialSelection = ptr.get();
                    potentialDragMode = DragMode::TranslateShape;
                    return true;
                }
            }
            return false;
        };
    
        if (checkShapeContains(editor.rectangles) || checkShapeContains(editor.polygons) ||
            checkShapeContains(editor.regularPolygons) || checkShapeContains(editor.triangles)) {
          goto objectFoundLabel;
        }
    }

    if (potentialSelection) {


      // We found an object under the cursor
      std::cout << "Object found! Type: " << static_cast<int>(potentialSelection->getType())
                << std::endl;

      // Only deselect all if we're not clicking on the currently selected
      // object
      if (potentialSelection != previousSelection) {
        deselectAllAndClearInteractionState(editor);
      }

      // Set the selected object
      editor.selectedObject = potentialSelection;
      editor.selectedObjects.clear();
      if (editor.selectedObject) {
        editor.selectedObjects.push_back(editor.selectedObject);
      }
      if (editor.selectedObject) {
        try {
          std::cout << "Setting object as selected" << std::endl;
          editor.selectedObject->setSelected(true);
          sf::Color selectedColor = editor.selectedObject->getColor();
          editor.setCurrentColor(selectedColor);
          editor.gui.setCurrentColor(selectedColor);
          if (auto &picker = editor.gui.getColorPicker()) {
            picker->setCurrentColor(selectedColor);
          }
        } catch (const std::exception &e) {
          std::cerr << "Error setting selection state: " << e.what() << std::endl;
        }
      }

      // Configure drag mode
      editor.dragMode = potentialDragMode;
      editor.m_selectedEndpoint = potentialEndpointSelection;
      editor.isDragging = true;
      editor.lastMousePos_sfml = worldPos_sfml;

      std::cout << "DEBUG: Before MoveShapeVertex check - potentialDragMode=" << static_cast<int>(potentialDragMode)
                << " (6=MoveShapeVertex)" << std::endl;
      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        std::cout << "DEBUG handleMousePress: Set activeVertexShape=" << editor.activeVertexShape 
                  << " activeVertexIndex=" << editor.activeVertexIndex << std::endl;
        if (auto *rect = dynamic_cast<Rectangle *>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto *poly = dynamic_cast<Polygon *>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto *reg = dynamic_cast<RegularPolygon *>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        } else if (auto *tri = dynamic_cast<Triangle *>(potentialSelection)) {
          tri->setActiveVertex(potentialVertexIndex);
        }
      }

      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        if (auto *rect = dynamic_cast<Rectangle *>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto *poly = dynamic_cast<Polygon *>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto *reg = dynamic_cast<RegularPolygon *>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        } else if (auto *tri = dynamic_cast<Triangle *>(potentialSelection)) {
          tri->setActiveVertex(potentialVertexIndex);
        }
      }

      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        if (auto *rect = dynamic_cast<Rectangle *>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto *poly = dynamic_cast<Polygon *>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto *reg = dynamic_cast<RegularPolygon *>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        }
      }

      std::cout << "Object selected with drag mode: " << static_cast<int>(editor.dragMode)
                << std::endl;
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
      std::cout << "Object found! Type: " << static_cast<int>(potentialSelection->getType())
                << std::endl;

      if (potentialSelection != previousSelection) {
        deselectAllAndClearInteractionState(editor);
      }

      editor.selectedObject = potentialSelection;
      if (editor.selectedObject) {
        try {
          std::cout << "Setting object as selected" << std::endl;
          editor.selectedObject->setSelected(true);
        } catch (const std::exception &e) {
          std::cerr << "Error setting selection state: " << e.what() << std::endl;
        }
      }

      editor.dragMode = potentialDragMode;
      editor.m_selectedEndpoint = potentialEndpointSelection;
      editor.isDragging = true;
      editor.lastMousePos_sfml = worldPos_sfml;

      // Set vertex state for MoveShapeVertex mode (CRITICAL FIX)
      if (potentialDragMode == DragMode::MoveShapeVertex) {
        editor.activeVertexIndex = potentialVertexIndex;
        editor.activeVertexShape = potentialSelection;
        std::cout << "DEBUG objectFoundLabel: Set activeVertexShape=" << editor.activeVertexShape 
                  << " activeVertexIndex=" << editor.activeVertexIndex << std::endl;
        // Set the active vertex visual on the shape
        if (auto *rect = dynamic_cast<Rectangle *>(potentialSelection)) {
          rect->setActiveVertex(potentialVertexIndex);
        } else if (auto *poly = dynamic_cast<Polygon *>(potentialSelection)) {
          poly->setActiveVertex(potentialVertexIndex);
        } else if (auto *reg = dynamic_cast<RegularPolygon *>(potentialSelection)) {
          reg->setActiveVertex(potentialVertexIndex);
        } else if (auto *tri = dynamic_cast<Triangle *>(potentialSelection)) {
          tri->setActiveVertex(potentialVertexIndex);
        }
      }

      std::cout << "Object selected with drag mode: " << static_cast<int>(editor.dragMode)
                << std::endl;
      if (editor.dragMode == DragMode::MoveLineEndpointStart ||
          editor.dragMode == DragMode::MoveLineEndpointEnd) {
        std::cout << "Dragging Line Endpoint. Selected Line: " << editor.selectedObject
                  << std::endl;
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
void handleKeyPress(GeometryEditor &editor, const sf::Event::KeyEvent &keyEvent) {
  // Handle key press events
  if (keyEvent.code == sf::Keyboard::Escape) {
    // Close color picker if it's open
    auto &colorPickerPtr = editor.gui.getColorPicker();
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
      } catch (const std::exception &e) {
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
             auto newPoly = std::make_shared<Polygon>(editor.polygonVertices, editor.getCurrentColor(),
                                        editor.objectIdCounter++);
             editor.polygons.push_back(newPoly);
             editor.isCreatingPolygon = false;
             editor.polygonVertices.clear();
             editor.previewPolygon.reset();
             editor.setGUIMessage("Polygon created via Enter key.");
             std::cout << "Polygon created via Enter key." << std::endl;
         } else {
             editor.setGUIMessage("Cannot create polygon: Need at least 3 vertices.");
         }
     }
  } else if (keyEvent.code == sf::Keyboard::Delete) {
    editor.deleteSelected();
    std::cout << "Delete key pressed - deleting selected object(s)" << std::endl;
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
  }
  // Add other key handlers as needed
}

void handleMouseMove(GeometryEditor &editor, const sf::Event::MouseMoveEvent &moveEvent) {
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
    editor.isResizingAngle = false; // Ensure flag is cleared if object is lost
  }

  sf::Vector2i pixelPos(moveEvent.x, moveEvent.y);
  sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
  
  // Angle Resizing Logic
  if (editor.isDragging && editor.isResizingAngle && editor.selectedObject && 
      editor.selectedObject->getType() == ObjectType::Angle) {
      
      auto angle = dynamic_cast<Angle*>(editor.selectedObject);
      if (angle) {
          Point_2 vertexPos = angle->getCGALPosition();
          double dx = worldPos.x - CGAL::to_double(vertexPos.x());
          double dy = worldPos.y - CGAL::to_double(vertexPos.y());
          double newRadius = std::sqrt(dx*dx + dy*dy);
          
          // Clamp radius to reasonable values
          newRadius = std::max(10.0, std::min(newRadius, 2000.0));
          
          angle->setRadius(newRadius);
      }
      return; // Consume event
  }
  sf::Vector2f guiPos = editor.window.mapPixelToCoords(pixelPos, editor.guiView);

  // Grid snapping (Shift): adjust world position before further processing
  bool gridSnapActive = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) ||
                        sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
  if (gridSnapActive) {
    float g = Constants::GRID_SIZE;
    worldPos.x = std::round(worldPos.x / g) * g;
    worldPos.y = std::round(worldPos.y / g) * g;
  }

  // Snapping Status Feedback (Ctrl)
  static bool wasSnapping = false;
  bool isSnapping = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl);
  if (isSnapping != wasSnapping) {
      if (isSnapping) editor.setGUIMessage("Vertex Snapping: Enabled");
      else editor.setGUIMessage("Vertex Snapping: Disabled");
      wasSnapping = isSnapping;
  }

  // Add bounds checking FIRST
  const float MAX_WORLD_COORD = 1e10f;
  if (!std::isfinite(worldPos.x) || !std::isfinite(worldPos.y) ||
      std::abs(worldPos.x) > MAX_WORLD_COORD || std::abs(worldPos.y) > MAX_WORLD_COORD) {
    return;  // Skip processing invalid coordinates
  }

  Point_2 cgalWorldPos;
  try {
    cgalWorldPos = Point_2(worldPos.x, worldPos.y);
    if (!CGAL::is_finite(cgalWorldPos.x()) || !CGAL::is_finite(cgalWorldPos.y())) {
      return;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error in CGAL point conversion: " << e.what() << std::endl;
    return;
  }

  static bool justFinishedDrag = false;
  static bool hasSetDeferredThisDrag = false;

  // Get the current zoom factor for point construction
  float currentActualZoomFactor = 1.0f;
  try {
    if (Constants::WINDOW_HEIGHT > 0) {
      currentActualZoomFactor =
          editor.drawingView.getSize().y / static_cast<float>(Constants::WINDOW_HEIGHT);
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

  float moveDistance = std::sqrt(std::pow(worldPos.x - lastPreviewPos.x, 2) +
                                 std::pow(worldPos.y - lastPreviewPos.y, 2));
  sf::Int32 timeSinceLastPreview = previewUpdateClock.getElapsedTime().asMilliseconds();

  // Update preview every mouse move for overlay-based tools (GeoGebra-like)
  // Fallback to minimal gating for other modes
  bool shouldUpdatePreview = (moveDistance > 5.0f) || (timeSinceLastPreview > 16);
  if (editor.m_currentToolType == ObjectType::ParallelLine ||
      editor.m_currentToolType == ObjectType::PerpendicularLine) {
    shouldUpdatePreview = true;
  }

  if (shouldUpdatePreview) {
    previewUpdateClock.restart();
    lastPreviewPos = worldPos;
  }

  // Safe cleanup of preview lines ONLY when the tool is not actively placing them
  try {
    const bool parallelPreviewActive =
        (editor.m_currentToolType == ObjectType::ParallelLine && editor.m_isPlacingParallel);
    const bool perpendicularPreviewActive =
        (editor.m_currentToolType == ObjectType::PerpendicularLine &&
         editor.m_isPlacingPerpendicular);

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
  } catch (const std::exception &e) {
    std::cerr << "Error cleaning preview lines: " << e.what() << std::endl;
    editor.m_parallelPreviewLine.reset();
    editor.m_perpendicularPreviewLine.reset();
  }

  editor.currentMousePos_sfml = worldPos;

  // === LINE TOOL VERTEX/EDGE SNAP PREVIEW ===
  // When Line Tool is active, highlight nearby vertices/edges to give visual feedback
  if (editor.m_currentToolType == ObjectType::Line || 
      editor.m_currentToolType == ObjectType::LineSegment) {
    float tolerance = editor.getScaledTolerance(editor.drawingView) * 2.0f;
    
    // Clear previous hover state
    if (editor.hoveredVertexShape) {
      if (auto *rect = dynamic_cast<Rectangle *>(editor.hoveredVertexShape)) rect->setHoveredVertex(-1);
      if (auto *poly = dynamic_cast<Polygon *>(editor.hoveredVertexShape)) poly->setHoveredVertex(-1);
      if (auto *reg = dynamic_cast<RegularPolygon *>(editor.hoveredVertexShape)) reg->setHoveredVertex(-1);
      if (auto *tri = dynamic_cast<Triangle *>(editor.hoveredVertexShape)) tri->setHoveredVertex(-1);
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
      
      if (auto *rect = dynamic_cast<Rectangle *>(outShape)) rect->setHoveredVertex(static_cast<int>(outVertexIndex));
      else if (auto *poly = dynamic_cast<Polygon *>(outShape)) poly->setHoveredVertex(static_cast<int>(outVertexIndex));
      else if (auto *reg = dynamic_cast<RegularPolygon *>(outShape)) reg->setHoveredVertex(static_cast<int>(outVertexIndex));
      else if (auto *tri = dynamic_cast<Triangle *>(outShape)) tri->setHoveredVertex(static_cast<int>(outVertexIndex));
      
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
    auto updatePreviewLine = [&](std::shared_ptr<Line> &previewLine, const Point_2 &p1,
                                 const Point_2 &p2) {
      if (!CGAL::is_finite(p1.x()) || !CGAL::is_finite(p1.y()) || !CGAL::is_finite(p2.x()) ||
          !CGAL::is_finite(p2.y())) {
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
        if (auto *p1Obj = previewLine->getStartPointObject()) {
          p1Obj->setCGALPosition(p1);
        }
        if (auto *p2Obj = previewLine->getEndPointObject()) {
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
              if (editor.m_parallelReference.edgeIndex >= 0 &&
                  editor.m_parallelReference.edgeIndex < static_cast<int>(edges.size())) {
                currentRefVec = edges[editor.m_parallelReference.edgeIndex].to_vector();
              }
            }
          }
        }

        if (currentRefVec != Vector_2(0, 0)) {
          Vector_2 unitDir = CGALSafeUtils::normalize_vector_robust(
              currentRefVec, "ParallelPreview_normalize_ref_dir");
          Kernel::FT length = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
          Point_2 p1_preview = cgalWorldPos;
          Point_2 p2_preview = cgalWorldPos + (unitDir * length * 0.5);
          updatePreviewLine(editor.m_parallelPreviewLine, p1_preview, p2_preview);
        } else if (editor.m_parallelPreviewLine) {
          editor.m_parallelPreviewLine.reset();
        }
      } catch (const std::exception &e) {
        std::cerr << "Error in parallel preview update: " << e.what() << std::endl;
        editor.m_parallelPreviewLine.reset();
      }
    }

    // Handle perpendicular line preview with persistent objects
    if (editor.m_currentToolType == ObjectType::PerpendicularLine &&
        editor.m_isPlacingPerpendicular) {
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
              if (editor.m_perpendicularReference.edgeIndex >= 0 &&
                  editor.m_perpendicularReference.edgeIndex < static_cast<int>(edges.size())) {
                currentRefVec = edges[editor.m_perpendicularReference.edgeIndex].to_vector();
              }
            }
          }
        }

        if (currentRefVec != Vector_2(0, 0)) {
          Vector_2 perpVec(-currentRefVec.y(), currentRefVec.x());
          Vector_2 unitDir = CGALSafeUtils::normalize_vector_robust(
              perpVec, "PerpendicularPreview_normalize_perp_dir");
          Kernel::FT length = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
          Point_2 p1_preview = cgalWorldPos;
          Point_2 p2_preview = cgalWorldPos + (unitDir * length * 0.5);
          updatePreviewLine(editor.m_perpendicularPreviewLine, p1_preview, p2_preview);
        } else if (editor.m_perpendicularPreviewLine) {
          editor.m_perpendicularPreviewLine.reset();
        }
      } catch (const std::exception &e) {
        std::cerr << "Error in perpendicular preview update: " << e.what() << std::endl;
        editor.m_perpendicularPreviewLine.reset();
      }
    }
  }

        // === UNIVERSAL SMART SNAPPING ===
        if (editor.m_currentToolType == ObjectType::Point ||
          editor.m_currentToolType == ObjectType::Line ||
          editor.m_currentToolType == ObjectType::LineSegment ||
          editor.m_currentToolType == ObjectType::Circle ||
          editor.m_currentToolType == ObjectType::Intersection) {
        float tolerance = editor.getScaledTolerance(editor.drawingView);
        editor.m_snapState = PointUtils::checkSnapping(editor, worldPos, tolerance);
        } else {
        editor.m_snapState = PointUtils::SnapState{};
        }
  
  // Update selection box if we're drawing one
  if (editor.isDrawingSelectionBox) {
    sf::Vector2f size = worldPos - editor.selectionBoxStart_sfml;
    editor.selectionBoxShape.setSize(size);
    return;
  } else if (sf::Mouse::isButtonPressed(sf::Mouse::Left) &&
             !editor.isDragging &&                            // Not dragging an existing object
             editor.m_currentToolType == ObjectType::None &&  // No tool active
             !editor.isDrawingSelectionBox) {                 // And not already drawing a
    // selection box

    // Check if the mouse has moved significantly from the initial press to
    // start selection box
    const float minDragDistance = 3.0f;  // Minimum pixels moved to initiate drag-select
    sf::Vector2f moveDelta = worldPos - editor.potentialSelectionBoxStart_sfml;

    if (std::abs(moveDelta.x) > minDragDistance || std::abs(moveDelta.y) > minDragDistance) {
      editor.isDrawingSelectionBox = true;
      editor.selectionBoxStart_sfml =
          editor.potentialSelectionBoxStart_sfml;  // Use the stored press
      // position
      editor.selectionBoxShape.setPosition(editor.selectionBoxStart_sfml);
      editor.selectionBoxShape.setFillColor(Constants::SELECTION_BOX_FILL_COLOR);
      editor.selectionBoxShape.setOutlineColor(Constants::SELECTION_BOX_OUTLINE_COLOR);
      editor.selectionBoxShape.setOutlineThickness(Constants::SELECTION_BOX_OUTLINE_THICKNESS);
      std::cout << "Selection box drag started." << std::endl;

      // Update size immediately based on current mouse position
      sf::Vector2f currentSize = worldPos - editor.selectionBoxStart_sfml;
      editor.selectionBoxShape.setSize(currentSize);
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
      double dist = std::sqrt(CGAL::to_double(
          CGAL::squared_distance(editor.toCGALPoint(worldPos), editor.createStart_cgal)));
      editor.previewCircle->setRadius(dist);
    } catch (const std::exception &e) {
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
         editor.previewTriangle = std::make_shared<Triangle>(
             editor.triangleVertices[0], editor.triangleVertices[1], cgalWorldPos,
             editor.getCurrentColor());
       } else {
         editor.previewTriangle->setVertexPosition(2, cgalWorldPos);
       }
    }
  }
  
  // Handle preview regular polygon
  if (shouldUpdatePreview && editor.isCreatingRegularPolygon && editor.regularPolygonPhase == 1) {
       if (!editor.previewRegularPolygon) {
            editor.previewRegularPolygon = std::make_shared<RegularPolygon>(
                editor.regularPolygonCenter, cgalWorldPos, editor.regularPolygonNumSides,
                editor.getCurrentColor());
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
        double width = side; // square-like preview for clarity
        editor.previewRectangle = std::make_shared<Rectangle>(
          editor.rectangleCorner1, cgalWorldPos, width, editor.getCurrentColor(),
          editor.objectIdCounter /* temp id */);
      } catch (const std::exception &e) {
        std::cerr << "Error updating rotatable rectangle preview: " << e.what() << std::endl;
        editor.previewRectangle.reset();
      }
    }

  // Handle dragging
  if (editor.isDragging) {
    QUICK_PROFILE("DragOperations");
    // 1. Calculate Raw World Position from Mouse
    Point_2 targetPos = editor.toCGALPoint(worldPos);

    // 2. SNAP LOGIC (Must happen BEFORE applying position)
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) &&
        (editor.dragMode == DragMode::MoveFreePoint ||
         editor.dragMode == DragMode::MoveLineEndpointStart ||
         editor.dragMode == DragMode::MoveLineEndpointEnd)) {
      // std::cout << "Ctrl held. Searching for snap..." << std::endl;

      double bestDistSq = 1000.0;
      bool foundSnap = false;
      Point_2 snapLocation;
      std::shared_ptr<Point> bestSnapPoint;

      Point *draggedPointRaw = nullptr;
      if (editor.dragMode == DragMode::MoveFreePoint && editor.selectedObject &&
          editor.selectedObject->getType() == ObjectType::Point) {
        draggedPointRaw = static_cast<Point *>(editor.selectedObject);
      } else if ((editor.dragMode == DragMode::MoveLineEndpointStart ||
                  editor.dragMode == DragMode::MoveLineEndpointEnd) &&
                 editor.selectedObject &&
                 (editor.selectedObject->getType() == ObjectType::Line ||
                  editor.selectedObject->getType() == ObjectType::LineSegment)) {
        Line *selectedLine = static_cast<Line *>(editor.selectedObject);
        draggedPointRaw = (editor.dragMode == DragMode::MoveLineEndpointStart)
                              ? selectedLine->getStartPointObject()
                              : selectedLine->getEndPointObject();
      }

      // A. Check Independent Points
      for (const auto &pt : editor.points) {
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
      for (const auto &line : editor.lines) {
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
      Line *currentlyManipulatedLine = nullptr;
      if (editor.selectedObject && (editor.selectedObject->getType() == ObjectType::Line ||
                                    editor.selectedObject->getType() == ObjectType::LineSegment)) {
        currentlyManipulatedLine = static_cast<Line *>(editor.selectedObject);
      }

      // Defer SFML updates for all lines EXCEPT the one being manipulated
      for (auto &linePtr : editor.lines) {
        if (linePtr && linePtr->isValid() && linePtr.get() != currentlyManipulatedLine) {
          std::cout << "  NOT deferring SFML updates - keeping real-time updates enabled" << std::endl;
          // REMOVED: linePtr->setDeferSFMLUpdates(true); - keep updates real-time
        }
      }
      if (editor.dragMode == DragMode::MoveFreePoint) {
        for (auto &linePtr : editor.lines) {
          if (linePtr && linePtr->isValid()) {
            // REMOVED deferring - enable real-time updates during point drag
            linePtr->setDeferSFMLUpdates(false);  // Enable immediate visual updates
          }
        }
      }

      // DEFER CONSTRAINT UPDATES FOR POINTS DURING LINE TRANSLATION
      if (editor.dragMode == DragMode::TranslateLine && currentlyManipulatedLine) {
        Point *startPoint = currentlyManipulatedLine->getStartPointObject();
        Point *endPoint = currentlyManipulatedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
      }

      // DEFER CONSTRAINT UPDATES FOR ENDPOINT DRAGGING
      if (editor.dragMode == DragMode::MoveLineEndpointStart ||
          editor.dragMode == DragMode::MoveLineEndpointEnd) {
        for (auto &pointPtr : editor.points) {
          if (pointPtr && pointPtr->isValid()) {
            pointPtr->setDeferConstraintUpdates(false);  // Real-time updates
          }
        }
      }

      // REAL-TIME UPDATES: Don't defer constraint updates for dragged points
      if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
        Point *selectedPoint = static_cast<Point *>(editor.selectedObject);
        selectedPoint->setDeferConstraintUpdates(false);  // Keep real-time updates
      }

      hasSetDeferredThisDrag = true;
      std::cout << "SFML DEFERRING SETUP COMPLETE" << std::endl;
    }
    try {
      if (editor.dragMode == DragMode::MoveFreePoint) {
        // Cast to Point and update position
        if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
          Point *selectedPoint = static_cast<Point *>(editor.selectedObject);
          if (selectedPoint->isIntersectionPoint()) {
            return;
          }
          selectedPoint->setCGALPosition(targetPos);
          selectedPoint->update();
          selectedPoint->setDeferConstraintUpdates(false);  // Enable real-time updates
          std::cout << "Point position updated to: (" << worldPos.x << ", " << worldPos.y << ")"
                    << std::endl;
          
          // Update any circles that use this point as their center or radius point
          for (auto &circlePtr : editor.circles) {
            if (circlePtr && circlePtr->isValid()) {
              // Check if this circle's center Point is the selected point (by pointer)
              if (circlePtr->getCenterPointObject() == selectedPoint ||
                  circlePtr->getRadiusPointObject() == selectedPoint) {
                circlePtr->update();  // Trigger circle to re-render in real-time
              }
            }
          }
        }
      } else if (editor.dragMode == DragMode::DragObjectPoint) {
        // Cast to ObjectPoint and handle movement
        if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::ObjectPoint) {
          ObjectPoint *selectedObjPoint = static_cast<ObjectPoint *>(editor.selectedObject);
          selectedObjPoint->updateFromMousePos(worldPos);
        }
      } else if (editor.dragMode == DragMode::TranslateLine) {
        // Cast to Line and handle translation
        if (editor.selectedObject &&
            (editor.selectedObject->getType() == ObjectType::Line ||
             editor.selectedObject->getType() == ObjectType::LineSegment)) {
          Line *selectedLine = static_cast<Line *>(editor.selectedObject);
          selectedLine->setIsUnderDirectManipulation(true);

          sf::Vector2f delta_sfml = worldPos - editor.lastMousePos_sfml;
          Vector_2 cgal_Delta = editor.toCGALVector(delta_sfml);

          // Keep constraint updates real-time during translation
          Point *startPoint = selectedLine->getStartPointObject();
          Point *endPoint = selectedLine->getEndPointObject();

          if (startPoint) {
            startPoint->setDeferConstraintUpdates(false);  // Real-time updates
          }
          if (endPoint) {
            endPoint->setDeferConstraintUpdates(false);  // Real-time updates
          }

          if (Constants::DEBUG_CGAL_POINT) {
            std::cout << "HandleEvents (Line Drag): sfml_delta: (" << delta_sfml.x << ", "
                      << delta_sfml.y << ")" << std::endl;
            std::cout << "HandleEvents (Line Drag): cgal_delta before translate: ("
                      << CGAL::to_double(cgal_Delta.x()) << ", " << CGAL::to_double(cgal_Delta.y())
                      << ")" << std::endl;
            std::cout << "  cgal_delta.x finite: " << CGAL::is_finite(cgal_Delta.x())
                      << ", cgal_delta.y finite: " << CGAL::is_finite(cgal_Delta.y()) << std::endl;
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
      } else if (editor.dragMode == DragMode::MoveLineEndpointStart ||
                 editor.dragMode == DragMode::MoveLineEndpointEnd) {
        // Cast to Line and handle endpoint movement
        if (editor.selectedObject &&
            (editor.selectedObject->getType() == ObjectType::Line ||
             editor.selectedObject->getType() == ObjectType::LineSegment)) {
          Line *selectedLine = static_cast<Line *>(editor.selectedObject);

          if (editor.dragMode == DragMode::MoveLineEndpointStart) {
            Point *startPoint = selectedLine->getStartPointObject();
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
            Point *endPoint = selectedLine->getEndPointObject();
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
        std::cout << "DEBUG MoveShapeVertex: activeVertexShape=" << editor.activeVertexShape 
                  << " activeVertexIndex=" << editor.activeVertexIndex << std::endl;
        if (editor.activeVertexShape && editor.activeVertexIndex >= 0) {
          Point_2 newPos = editor.toCGALPoint(worldPos);
          std::cout << "DEBUG MoveShapeVertex: Moving vertex " << editor.activeVertexIndex 
                    << " to (" << CGAL::to_double(newPos.x()) << "," << CGAL::to_double(newPos.y()) << ")" << std::endl;
          switch (editor.activeVertexShape->getType()) {
            case ObjectType::Rectangle:
            case ObjectType::RectangleRotatable: {
              auto *rect = static_cast<Rectangle *>(editor.activeVertexShape);
              std::cout << "DEBUG MoveShapeVertex: Calling rect->setVertexPosition" << std::endl;
              rect->setVertexPosition(static_cast<size_t>(editor.activeVertexIndex), newPos);
              rect->setActiveVertex(editor.activeVertexIndex);
              break;
            }
            case ObjectType::Polygon: {
              auto *poly = static_cast<Polygon *>(editor.activeVertexShape);
              poly->setVertexPosition(static_cast<size_t>(editor.activeVertexIndex), newPos);
              poly->setActiveVertex(editor.activeVertexIndex);
              break;
            }
            case ObjectType::RegularPolygon: {
              auto *reg = static_cast<RegularPolygon *>(editor.activeVertexShape);
              // Use creation point method: 0=center (translate), 1=first vertex (scale)
              reg->setCreationPointPosition(static_cast<size_t>(editor.activeVertexIndex), newPos);
              reg->setActiveVertex(editor.activeVertexIndex);
              break;
            }
            case ObjectType::Triangle: {
              auto *tri = static_cast<Triangle *>(editor.activeVertexShape);
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
              auto *rect = static_cast<Rectangle *>(editor.selectedObject);
              rect->translate(delta_cgal);
              break;
            }
            case ObjectType::Polygon: {
              auto *poly = static_cast<Polygon *>(editor.selectedObject);
              poly->translate(delta_cgal);
              break;
            }
            case ObjectType::RegularPolygon: {
              auto *reg = static_cast<RegularPolygon *>(editor.selectedObject);
              reg->translate(delta_cgal);
              break;
            }
            case ObjectType::Triangle: {
              auto *tri = static_cast<Triangle *>(editor.selectedObject);
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
          Circle *selectedCircle = static_cast<Circle *>(editor.selectedObject);
          if (!selectedCircle || !selectedCircle->isValid()) return;
          
          float tolerance = editor.getScaledTolerance(editor.drawingView);
          
          // Determine if we're dragging the center or circumference
          if (selectedCircle->isCenterPointHovered(worldPos, tolerance * 4.0f)) {
            // Dragging the center - move the circle
            Vector_2 offset = cgalWorldPos - selectedCircle->getCenterPoint();
            selectedCircle->setCenter(selectedCircle->getCenterPoint() + offset);
            std::cout << "[CIRCLE] Moving center" << std::endl;
          } else {
            // Dragging the circumference - resize the circle
            Point_2 center = selectedCircle->getCenterPoint();
            double newRadius = std::sqrt(CGAL::to_double(CGAL::squared_distance(cgalWorldPos, center)));
            if (newRadius > 1.0) {  // Minimum radius
              selectedCircle->setRadius(newRadius);
              std::cout << "[CIRCLE] Resizing to radius: " << newRadius << std::endl;
            }
          }
        }
      }
    } catch (const std::exception &e) {
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

    float moveDistance = std::sqrt(std::pow(worldPos.x - lastHoverCheckPos.x, 2) +
                                   std::pow(worldPos.y - lastHoverCheckPos.y, 2));
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
    float baseTolerance = editor.getScaledTolerance(editor.drawingView);
    float tolerance = baseTolerance * 1.5f;  // Slightly larger tolerance for
    // hover detection
    const float MIN_TOLERANCE = 8.0f;  // Minimum 8 pixels
    if (tolerance < MIN_TOLERANCE) {
      tolerance = MIN_TOLERANCE;
    }
    // IMPROVED: Only reset hover if something was previously hovered

    // Reset only the previously hovered object if it's still valid and not being deleted
    if (g_lastHoveredObject && !isObjectBeingDeleted(g_lastHoveredObject)) {
      try {
        g_lastHoveredObject->setHovered(false);
      } catch (const std::exception &e) {
        std::cerr << "Error clearing hover (object may be deleted): " << e.what() << std::endl;
      }
    }
    g_lastHoveredObject = nullptr;

    // Clear vertex hover state
    if (editor.hoveredVertexShape) {
      if (auto *rect = dynamic_cast<Rectangle *>(editor.hoveredVertexShape)) rect->setHoveredVertex(-1);
      if (auto *poly = dynamic_cast<Polygon *>(editor.hoveredVertexShape)) poly->setHoveredVertex(-1);
      if (auto *reg = dynamic_cast<RegularPolygon *>(editor.hoveredVertexShape)) reg->setHoveredVertex(-1);
      if (auto *tri = dynamic_cast<Triangle *>(editor.hoveredVertexShape)) tri->setHoveredVertex(-1);
    }
    editor.hoveredVertexShape = nullptr;
    editor.hoveredVertexIndex = -1;

    // Check for hover, prioritizing smaller objects
    GeometricObject *newHoveredObject = nullptr;
    const float QUICK_REJECT_DISTANCE = tolerance * 3.0f;

    // Check vertex handles first
    auto checkVertexHover = [&](auto &container) -> bool {
      for (auto &ptr : container) {
        if (!ptr || !ptr->isValid() || !ptr->isVisible()) continue;
        auto verts = ptr->getVerticesSFML();
        for (size_t i = 0; i < verts.size(); ++i) {
          if (auto *regPtr = dynamic_cast<RegularPolygon *>(ptr.get())) {
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

    if (checkVertexHover(editor.rectangles) || checkVertexHover(editor.polygons) ||
        checkVertexHover(editor.regularPolygons) || checkVertexHover(editor.triangles)) {
      // continue to set hover state; still allow color set below
    }

    // Check ObjectPoints first (smallest, highest priority)
    if (!newHoveredObject) {
      for (auto &objPointPtr : editor.ObjectPoints) {
        if (objPointPtr && objPointPtr->isVisible() && !isObjectBeingDeleted(objPointPtr.get())) {
          // Quick distance check before expensive contains()
          sf::Vector2f objPos = objPointPtr->getSFMLPosition();
          float quickDist = std::abs(worldPos.x - objPos.x) +
                            std::abs(worldPos.y - objPos.y);  // Manhattan distance (faster)

          if (quickDist <= QUICK_REJECT_DISTANCE && objPointPtr->contains(worldPos, tolerance)) {
            newHoveredObject = objPointPtr.get();
            break;
          }
        }
      }
    }

    // Check for Free Points explicitly if not found yet
    if (!newHoveredObject) {
      for (auto &pointPtr : editor.points) {
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
           for(auto& shape : container) {
             if(!shape || !shape->isValid() || !shape->isVisible()) continue;
               
               auto edges = shape->getEdges();
               for(size_t i=0; i<edges.size(); ++i) {
                   Point_2 proj;
                   double relPos;
                   // Use PointUtils helper for consistent math
                   double dist = PointUtils::projectPointOntoSegment(
                       editor.toCGALPoint(worldPos), edges[i], proj, relPos);
                   
                   // Convert distance back to float/pixels approx for check
                   if(dist < tolerance) { // tolerance is in view units? no, getScaledTolerance is in world units
                       return shape.get();
                   }
               }
           }
           return nullptr;
       };
       
       if(auto* hit = checkShapeEdges(editor.rectangles)) newHoveredObject = hit;
       else if(auto* hit = checkShapeEdges(editor.polygons)) newHoveredObject = hit;
       else if(auto* hit = checkShapeEdges(editor.regularPolygons)) newHoveredObject = hit;
       else if(auto* hit = checkShapeEdges(editor.triangles)) newHoveredObject = hit;
    }

    // Lines and Circles with safety checks
    if (!newHoveredObject) {
      for (auto &linePtr : editor.lines) {
        if (linePtr && linePtr->isVisible() && !isObjectBeingDeleted(linePtr.get()) &&
            linePtr->contains(worldPos, tolerance)) {
          newHoveredObject = linePtr.get();
          break;
        }
      }
    }

    if (!newHoveredObject) {
      for (auto &anglePtr : editor.angles) {
        if (anglePtr && anglePtr->isVisible() && !isObjectBeingDeleted(anglePtr.get()) &&
            anglePtr->contains(worldPos, tolerance)) {
          newHoveredObject = anglePtr.get();
          break;
        }
      }
    }

    if (!newHoveredObject) {
      for (auto &circlePtr : editor.circles) {
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
        } catch (const std::exception &e) {
          std::cerr << "Error clearing hover on object: " << e.what() << std::endl;
        }
      }

      // Set new hover safely
      if (newHoveredObject && !isObjectBeingDeleted(newHoveredObject)) {
        try {
          newHoveredObject->setHovered(true);
        } catch (const std::exception &e) {
          std::cerr << "Error setting hover on object: " << e.what() << std::endl;
          newHoveredObject = nullptr;  // Clear the reference if it failed
        }
      }

      g_lastHoveredObject = newHoveredObject;
    }

  } catch (const std::exception &e) {
    std::cerr << "Error in hover detection: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown error in hover detection" << std::endl;
  }

  // === EDGE HOVER DETECTION (Universal Snapping) ===
  // Only check for edge hover if no object is currently hovered
  try {
    if (!g_lastHoveredObject && !editor.isDragging) {
      float tolerance = editor.getScaledTolerance(editor.drawingView);
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
  } catch (const std::exception &e) {
    std::cerr << "Error in edge hover detection: " << e.what() << std::endl;
  }

  editor.lastMousePos_sfml = worldPos;
}

void handleMouseRelease(GeometryEditor &editor, const sf::Event::MouseButtonEvent &mouseEvent) {
  sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
  sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

  // Clear active vertex drag state
  if (editor.activeVertexShape) {
    if (auto *rect = dynamic_cast<Rectangle *>(editor.activeVertexShape)) rect->setActiveVertex(-1);
    if (auto *poly = dynamic_cast<Polygon *>(editor.activeVertexShape)) poly->setActiveVertex(-1);
    if (auto *reg = dynamic_cast<RegularPolygon *>(editor.activeVertexShape)) reg->setActiveVertex(-1);
    if (auto *tri = dynamic_cast<Triangle *>(editor.activeVertexShape)) tri->setActiveVertex(-1);
  }
  editor.activeVertexShape = nullptr;
  editor.activeVertexIndex = -1;

  // Check if GUI (especially color picker) should handle this release event
  sf::Vector2f guiPos = editor.window.mapPixelToCoords(pixelPos, editor.guiView);
  
  sf::Event eventToForward;
  eventToForward.type = sf::Event::MouseButtonReleased;
  eventToForward.mouseButton = mouseEvent;
  if (editor.gui.handleEvent(editor.window, eventToForward, editor)) {
    return;  // GUI consumed the event
  }

  // Check if we're releasing a selection box drag
  if (editor.isDrawingSelectionBox && mouseEvent.button == sf::Mouse::Left) {
    editor.isDrawingSelectionBox = false;

    sf::FloatRect selectionBox(std::min(editor.selectionBoxStart_sfml.x, worldPos.x),
                               std::min(editor.selectionBoxStart_sfml.y, worldPos.y),
                               std::abs(worldPos.x - editor.selectionBoxStart_sfml.x),
                               std::abs(worldPos.y - editor.selectionBoxStart_sfml.y));

    try {
      // Deselect everything first. This also sets editor.selectedObject to
      // nullptr.
      deselectAllAndClearInteractionState(editor);

      std::vector<GeometricObject *> newlySelectedObjects;

      for (auto &pointPtr : editor.points) {
        if (pointPtr && pointPtr->isValid() && pointPtr->isVisible() &&
            selectionBox.intersects(pointPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(pointPtr.get());
        }
      }
      for (auto &linePtr : editor.lines) {
        if (linePtr && linePtr->isValid() && linePtr->isVisible() &&
            selectionBox.intersects(linePtr->getGlobalBounds())) {
          if (Constants::DEBUG_SELECTION) {  // Use your DEBUG_SELECTION flag
            std::cout << "SelectionBox Check: Line " << linePtr.get() << " bounds: ("
                      << linePtr->getGlobalBounds().left << "," << linePtr->getGlobalBounds().top
                      << "," << linePtr->getGlobalBounds().width << ","
                      << linePtr->getGlobalBounds().height << ")"
                      << " intersects: " << selectionBox.intersects(linePtr->getGlobalBounds())
                      << std::endl;
          }
          newlySelectedObjects.push_back(linePtr.get());
        }
      }
      for (auto &circlePtr : editor.circles) {
        if (circlePtr && circlePtr->isValid() && circlePtr->isVisible() &&
            selectionBox.intersects(circlePtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(circlePtr.get());
        }
      }
      for (auto &objPointPtr : editor.ObjectPoints) {
        if (objPointPtr && objPointPtr->isValid() && objPointPtr->isVisible() &&
            selectionBox.intersects(objPointPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(objPointPtr.get());
        }
      }
      // Add Rectangle selection
      for (auto &rectPtr : editor.rectangles) {
        if (rectPtr && rectPtr->isValid() && rectPtr->isVisible() &&
            selectionBox.intersects(rectPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(rectPtr.get());
        }
      }
      // Add Polygon selection
      for (auto &polyPtr : editor.polygons) {
        if (polyPtr && polyPtr->isValid() && polyPtr->isVisible() &&
            selectionBox.intersects(polyPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(polyPtr.get());
        }
      }
      // Add RegularPolygon selection
      for (auto &regPolyPtr : editor.regularPolygons) {
        if (regPolyPtr && regPolyPtr->isValid() && regPolyPtr->isVisible() &&
            selectionBox.intersects(regPolyPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(regPolyPtr.get());
        }
      }
      // Add Triangle selection
      for (auto &triPtr : editor.triangles) {
        if (triPtr && triPtr->isValid() && triPtr->isVisible() &&
            selectionBox.intersects(triPtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(triPtr.get());
        }
      }
      for (auto &anglePtr : editor.angles) {
        if (anglePtr && anglePtr->isValid() && anglePtr->isVisible() &&
            selectionBox.intersects(anglePtr->getGlobalBounds())) {
          newlySelectedObjects.push_back(anglePtr.get());
        }
      }

      // Apply selection to all found objects
      for (GeometricObject *obj : newlySelectedObjects) {
        if (obj) {  // Basic safety check
          obj->setSelected(true);
        }
      }

      editor.selectedObjects = newlySelectedObjects;

      // If exactly one object was selected by the box, set it as the
      // primary selected object
      if (newlySelectedObjects.size() == 1) {
        editor.selectedObject = newlySelectedObjects[0];
        if (editor.selectedObject) {
          sf::Color selectedColor = editor.selectedObject->getColor();
          editor.setCurrentColor(selectedColor);
          editor.gui.setCurrentColor(selectedColor);
          if (auto &picker = editor.gui.getColorPicker()) {
            picker->setCurrentColor(selectedColor);
          }
        }
      }
      // If multiple objects are selected, editor.selectedObject remains
      // nullptr (as set by deselectAllAndClearInteractionState)

    } catch (const std::exception &e) {
      std::cerr << "Error processing selection box: " << e.what() << std::endl;
    }

    std::cout << "Selection box completed" << std::endl;
    return;  // Selection box handling is exclusive
  }

  // Handle end of dragging an object
  if (editor.isDragging && mouseEvent.button == sf::Mouse::Left) {
    // ✅ CRITICAL FIX: Defer constraint updates DURING the cleanup phase
    std::cout << "PREPARING FOR COORDINATED UPDATE" << std::endl;

    // Keep constraint updates deferred while we clean up visual deferring
    if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
      Point *draggedPoint = static_cast<Point *>(editor.selectedObject);
      // Keep constraint updates deferred for now
      draggedPoint->setDeferConstraintUpdates(false);  // This will trigger update
      for (auto &linePtr : editor.lines) {
        if (linePtr && linePtr->isValid()) {
          // REMOVED deferring - enable real-time updates during point drag
          linePtr->setDeferSFMLUpdates(false);  // Enable immediate visual updates
        }
      }
      editor.updateAllGeometry();
      for (auto &linePtr : editor.lines) {
        if (linePtr && linePtr->isValid()) {
          linePtr->setDeferSFMLUpdates(false);
          linePtr->updateSFMLShape();  // Single final update
        }
      }
    } else {
      // Handle line translation points
      if (editor.dragMode == DragMode::TranslateLine && editor.selectedObject &&
          (editor.selectedObject->getType() == ObjectType::Line ||
           editor.selectedObject->getType() == ObjectType::LineSegment)) {
        Line *selectedLine = static_cast<Line *>(editor.selectedObject);
        Point *startPoint = selectedLine->getStartPointObject();
        Point *endPoint = selectedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);  // Real-time updates
        }
      }

      // Handle endpoint dragging
      if ((editor.dragMode == DragMode::MoveLineEndpointStart ||
           editor.dragMode == DragMode::MoveLineEndpointEnd) &&
          editor.selectedObject &&
          (editor.selectedObject->getType() == ObjectType::Line ||
           editor.selectedObject->getType() == ObjectType::LineSegment)) {
        Line *selectedLine = static_cast<Line *>(editor.selectedObject);
        Point *startPoint = selectedLine->getStartPointObject();
        Point *endPoint = selectedLine->getEndPointObject();

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
      for (auto &linePtr : editor.lines) {
        if (linePtr && linePtr->isValid()) {
          linePtr->setDeferSFMLUpdates(false);  // Enable visual updates
          // Don't call updateSFMLShape yet - let constraint update handle it
        }
      }

      GeometricObject *draggedObject = editor.selectedObject;
      DragMode previousDragMode = editor.dragMode;

      // Topological merge on snap release
      if (editor.m_wasSnapped && editor.m_snapTargetPoint &&
          (previousDragMode == DragMode::MoveFreePoint ||
           previousDragMode == DragMode::MoveLineEndpointStart ||
           previousDragMode == DragMode::MoveLineEndpointEnd)) {
        std::shared_ptr<Point> draggedPointShared = nullptr;

        if (previousDragMode == DragMode::MoveFreePoint && draggedObject &&
            draggedObject->getType() == ObjectType::Point) {
          Point *draggedPointRaw = static_cast<Point *>(draggedObject);
          for (auto &pt : editor.points) {
            if (pt && pt.get() == draggedPointRaw) {
              draggedPointShared = pt;
              break;
            }
          }
        } else if ((previousDragMode == DragMode::MoveLineEndpointStart ||
                    previousDragMode == DragMode::MoveLineEndpointEnd) &&
                   draggedObject &&
                   (draggedObject->getType() == ObjectType::Line ||
                    draggedObject->getType() == ObjectType::LineSegment)) {
          Line *selectedLine = static_cast<Line *>(draggedObject);
          draggedPointShared = (previousDragMode == DragMode::MoveLineEndpointStart)
                                   ? selectedLine->getStartPointObjectShared()
                                   : selectedLine->getEndPointObjectShared();
        }

        if (draggedPointShared && draggedPointShared != editor.m_snapTargetPoint) {
          editor.replacePoint(draggedPointShared, editor.m_snapTargetPoint);
          draggedPointShared = nullptr;
        }
      }

      editor.m_wasSnapped = false;
      editor.m_snapTargetPoint = nullptr;

      editor.isDragging = false;  // Stop dragging state

      // Reset m_isUnderDirectManipulation flag for lines
      if (draggedObject && (draggedObject->getType() == ObjectType::Line ||
                            draggedObject->getType() == ObjectType::LineSegment)) {
        if (previousDragMode == DragMode::TranslateLine ||
            previousDragMode == DragMode::MoveLineEndpointStart ||
            previousDragMode == DragMode::MoveLineEndpointEnd) {
          static_cast<Line *>(draggedObject)->setIsUnderDirectManipulation(false);
          if (Constants::DEBUG_DRAGGING) {
            std::cout << "Line " << draggedObject
                      << ": m_isUnderDirectManipulation set to false (drag end)" << std::endl;
          }
        }
      }

      // ✅ NOW enable constraint updates - this will trigger coordinated
      // geometry update
      std::cout << "ENABLING CONSTRAINT UPDATES FOR FINAL UPDATE" << std::endl;

      if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
        Point *draggedPoint = static_cast<Point *>(editor.selectedObject);
        draggedPoint->setDeferConstraintUpdates(false);  // This will trigger update
      }

      // Handle line translation points
      if (previousDragMode == DragMode::TranslateLine && draggedObject &&
          (draggedObject->getType() == ObjectType::Line ||
           draggedObject->getType() == ObjectType::LineSegment)) {
        Line *selectedLine = static_cast<Line *>(draggedObject);
        Point *startPoint = selectedLine->getStartPointObject();
        Point *endPoint = selectedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);
        }
      }

      // Handle endpoint dragging
      if ((previousDragMode == DragMode::MoveLineEndpointStart ||
           previousDragMode == DragMode::MoveLineEndpointEnd) &&
          draggedObject &&
          (draggedObject->getType() == ObjectType::Line ||
           draggedObject->getType() == ObjectType::LineSegment)) {
        Line *selectedLine = static_cast<Line *>(draggedObject);
        Point *startPoint = selectedLine->getStartPointObject();
        Point *endPoint = selectedLine->getEndPointObject();

        if (startPoint) {
          startPoint->setDeferConstraintUpdates(false);
        }
        if (endPoint) {
          endPoint->setDeferConstraintUpdates(false);
        }
      }

      // Update geometry if a relevant drag operation occurred
      if (previousDragMode == DragMode::MoveFreePoint ||
          previousDragMode == DragMode::DragObjectPoint ||
          previousDragMode == DragMode::TranslateLine ||
          previousDragMode == DragMode::MoveLineEndpointStart ||
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
    }

    // Reset general drag interaction state
    editor.isDragging = false;
    editor.dragMode = DragMode::None;
    editor.m_selectedEndpoint = EndpointSelection::None;
    editor.isResizingAngle = false;
  }

  // Handle end of panning
  if (editor.isPanning && (mouseEvent.button == sf::Mouse::Middle ||  // Original
                           mouseEvent.button == sf::Mouse::Right ||   // New options
                           (mouseEvent.button == sf::Mouse::Left &&
                            (sf::Keyboard::isKeyPressed(sf::Keyboard::Space) ||
                             sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) ||
                             sf::Keyboard::isKeyPressed(sf::Keyboard::LControl))))) {
    editor.isPanning = false;
    std::cout << "Panning ended" << std::endl;
  }
}

void handleZoom(GeometryEditor &editor, float scrollDelta, const sf::Vector2i &mousePixelPos) {
  sf::View &view = editor.drawingView;

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

  if (std::abs(zoomDirection - 1.0f) <
      Constants::EPSILON) {  // Avoid zooming if factor is effectively 1
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
void handleMouseWheelScroll(GeometryEditor &editor,
                            const sf::Event::MouseWheelScrollEvent &wheelEvent) {
  // wheelEvent.delta will be positive for scroll up (zoom in for many)
  // or negative for scroll down (zoom out for many)
  // SFML typically: scroll up is positive delta, scroll down is negative.
  // If your mouse is inverted, adjust the sign of wheelEvent.delta.
  // Get mouse position in pixel coordinates from the event
  sf::Vector2i mousePixelPos(wheelEvent.x, wheelEvent.y);
  handleZoom(editor, wheelEvent.delta, mousePixelPos);
}

// Add this implementation at the end of the file
void handleEvents(GeometryEditor &editor) {
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
      case sf::Event::KeyPressed:
        handleKeyPress(editor, event.key);
        break;
      default:
        break;
    }
  }
}
