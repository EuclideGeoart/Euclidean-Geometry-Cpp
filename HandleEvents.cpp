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
#include "ConstructionObjects.h"
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
#include "HandleMousePress.h"
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
#include <imgui.h>
#include <imgui-SFML.h>

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
// (Keeping existing code)


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

  // Font Size Adjustment (F +/-)
    ImGuiIO& io = ImGui::GetIO();

    // Check for PLUS keys (Numpad Add or Standard Equal/Plus)
    if (keyEvent.code == sf::Keyboard::Add || keyEvent.code == sf::Keyboard::Equal) {
        // ONLY if 'F' is held down
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::F)) {
            io.FontGlobalScale = std::min(1.5f, io.FontGlobalScale + 0.05f); 
        editor.setGUIMessage("UI Scale: " + std::to_string(io.FontGlobalScale));
        }
    }

    // Check for MINUS keys (Numpad Subtract or Standard Dash)
    if (keyEvent.code == sf::Keyboard::Subtract || keyEvent.code == sf::Keyboard::Hyphen) {
        // ONLY if 'F' is held down
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::F)) {
            io.FontGlobalScale = std::max(0.1f, io.FontGlobalScale - 0.05f);
            editor.setGUIMessage("UI Scale: " + std::to_string(io.FontGlobalScale)); 
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
      auto xAxisPtr = editor.getXAxis();
      auto yAxisPtr = editor.getYAxis();
      for (const auto& obj : container) {
        if (!obj || !obj->isSelected()) continue;
        if ((GeometricObject*)obj.get() == (GeometricObject*)xAxisPtr || (GeometricObject*)obj.get() == (GeometricObject*)yAxisPtr) continue;
        if (!obj->isLocked() || obj->isDependent()) {
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

      // Keep tiny radii valid; avoid hard clamp that snaps size
      newRadius = std::max(1e-9, newRadius);

      angle->setRadius(newRadius);
    }
    return;  // Consume event
  }

  if (editor.isDraggingLabel && editor.labelDragObject) {
    sf::Vector2f mouseScreenPos(static_cast<float>(moveEvent.x), static_cast<float>(moveEvent.y));
    auto clampOffsetToRadius = [](const sf::Vector2f& offset, float radius) {
      float distSq = offset.x * offset.x + offset.y * offset.y;
      float radiusSq = radius * radius;
      if (distSq <= radiusSq || distSq <= 0.0f) {
        return offset;
      }
      float dist = std::sqrt(distSq);
      float scale = radius / dist;
      return sf::Vector2f(offset.x * scale, offset.y * scale);
    };

    if (auto* pt = dynamic_cast<Point*>(editor.labelDragObject)) {
      sf::Vector2f worldPos = pt->getSFMLPosition();
      sf::Vector2i pointScreen = editor.window.mapCoordsToPixel(worldPos, editor.drawingView);
      sf::Vector2f pointScreenPos(static_cast<float>(pointScreen.x), static_cast<float>(pointScreen.y));
      sf::Vector2f labelTopLeft = mouseScreenPos - editor.labelDragGrabOffset;
      sf::Vector2f newOffset = labelTopLeft - pointScreenPos;
      newOffset = clampOffsetToRadius(newOffset, Constants::LABEL_DRAG_RADIUS_PIXELS);
      pt->setLabelOffset(newOffset);
    } else if (auto* rect = dynamic_cast<Rectangle*>(editor.labelDragObject)) {
      if (editor.labelDragVertexIndex >= 0 && editor.labelDragVertexIndex < 4) {
        auto verts = rect->getVerticesSFML();
        if (editor.labelDragVertexIndex < static_cast<int>(verts.size())) {
          sf::Vector2f worldPos = verts[editor.labelDragVertexIndex];
          sf::Vector2i vertexScreen = editor.window.mapCoordsToPixel(worldPos, editor.drawingView);
          sf::Vector2f vertexScreenPos(static_cast<float>(vertexScreen.x), static_cast<float>(vertexScreen.y));
          sf::Vector2f labelPos = mouseScreenPos - editor.labelDragGrabOffset;
          sf::Vector2f newOffset = labelPos - vertexScreenPos;
          newOffset = clampOffsetToRadius(newOffset, Constants::LABEL_DRAG_RADIUS_PIXELS);
          rect->setVertexLabelOffset(editor.labelDragVertexIndex, newOffset);
        }
      }
    } else if (auto* angle = dynamic_cast<Angle*>(editor.labelDragObject)) {
      // Calculate angle label's default position (without offset)
      auto pointA = angle->getPointA().lock();
      auto vertex = angle->getVertex().lock();
      auto pointB = angle->getPointB().lock();
      
      if (pointA && vertex && pointB) {
        Point_2 v = vertex->getCGALPosition();
        Point_2 a = pointA->getCGALPosition();
        Point_2 b = pointB->getCGALPosition();
        
        double vx = CGAL::to_double(v.x());
        double vy = CGAL::to_double(v.y());
        double ax = CGAL::to_double(a.x());
        double ay = CGAL::to_double(a.y());
        double bx = CGAL::to_double(b.x());
        double by = CGAL::to_double(b.y());
        
        double angle1 = std::atan2(ay - vy, ax - vx);
        double angle2 = std::atan2(by - vy, bx - vx);
        
        double sweep = angle2 - angle1;
        if (angle->isReflex()) {
          if (sweep >= 0) sweep -= 2.0 * 3.14159265358979323846;
          else sweep += 2.0 * 3.14159265358979323846;
        } else {
          if (sweep < 0) sweep += 2.0 * 3.14159265358979323846;
        }
        
        double midAngle = angle1 + sweep * 0.5;
        double textRadius = angle->getRadius() + 12.0;
        
        // Calculate default label position in world coordinates
        sf::Vector2f labelWorldPos(
          static_cast<float>(vx + textRadius * std::cos(midAngle)),
          static_cast<float>(vy + textRadius * std::sin(midAngle))
        );
        
        // Convert to screen coordinates
        sf::Vector2i labelScreen = editor.window.mapCoordsToPixel(labelWorldPos, editor.drawingView);
        sf::Vector2f labelDefaultScreenPos(static_cast<float>(labelScreen.x), static_cast<float>(labelScreen.y));
        
        // Calculate new offset
        sf::Vector2f labelPos = mouseScreenPos - editor.labelDragGrabOffset;
        sf::Vector2f newOffset = labelPos - labelDefaultScreenPos;
        
        // Constrain label movement to a circular boundary around the vertex (screen pixels)
        newOffset = clampOffsetToRadius(newOffset, Constants::LABEL_DRAG_RADIUS_PIXELS);
        
        angle->setLabelOffset(newOffset);
      }
    }
    return;
  }
  sf::Vector2f guiPos = editor.window.mapPixelToCoords(pixelPos, editor.guiView);

  // Grid snapping (Shift): adjust world position before further processing
  bool gridSnapActive = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
  if (gridSnapActive) {
    float g = editor.getCurrentGridSpacing();
    if (g <= 0.0f) {
      g = Constants::GRID_SIZE;
    }
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

  // === FREE POINT TOOL STICKY PROJECTION ===
  // When Free Point tool is active, project cursor onto nearby lines/circles for visual feedback
  if (editor.m_currentToolType == ObjectType::Point) {
    float snapTolerance = getDynamicSelectionTolerance(editor);
    bool isAltPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LAlt) || sf::Keyboard::isKeyPressed(sf::Keyboard::RAlt);

    // Check for existing points ONLY if ALT is pressed (merge mode)
    bool snappedToPoint = false;
    if (isAltPressed) {
      GeometricObject* nearbyPoint = editor.lookForObjectAt(worldPos, snapTolerance, {ObjectType::Point, ObjectType::ObjectPoint});
      if (nearbyPoint) {
        Point_2 pointPos;
        if (auto* pt = dynamic_cast<Point*>(nearbyPoint)) {
          pointPos = pt->getCGALPosition();
        } else if (auto* objPt = dynamic_cast<ObjectPoint*>(nearbyPoint)) {
          pointPos = objPt->getCGALPosition();
        }

        // Snap to existing point
        editor.m_snapState = PointUtils::SnapState{};
        editor.m_snapState.kind = PointUtils::SnapState::Kind::ExistingPoint;
        editor.m_snapState.position = pointPos;
        nearbyPoint->setHovered(true);
        snappedToPoint = true;
      }
    }

    // If not snapped to a point, allow snapping to lines/circles (projection)
    if (!snappedToPoint) {
      GeometricObject* nearbyObj = editor.lookForObjectAt(worldPos, snapTolerance, {ObjectType::Line, ObjectType::LineSegment, ObjectType::Ray, ObjectType::Vector, ObjectType::Circle});

      if (nearbyObj) {
        Point_2 cgalPos = editor.toCGALPoint(worldPos);
        Point_2 projectedPos = cgalPos; // Default to mouse position

        // Project onto the object
        if (nearbyObj->getType() == ObjectType::Line || nearbyObj->getType() == ObjectType::LineSegment ||
            nearbyObj->getType() == ObjectType::Ray || nearbyObj->getType() == ObjectType::Vector) {
          auto* line = dynamic_cast<Line*>(nearbyObj);
          if (line && line->isValid()) {
            projectedPos = projectPointOntoLine(cgalPos, line, line->isSegment());
          }
        } else if (nearbyObj->getType() == ObjectType::Circle) {
          auto* circle = dynamic_cast<Circle*>(nearbyObj);
          if (circle && circle->isValid()) {
            Point_2 center = circle->getCenterPoint();
            double radius = circle->getRadius();
            projectedPos = projectPointOntoCircle(cgalPos, center, radius);
          }
        }

        // Update snap state for visual feedback (cyan preview point)
        editor.m_snapState = PointUtils::SnapState{};
        editor.m_snapState.kind = PointUtils::SnapState::Kind::Line; // Use Line kind for object snapping
        editor.m_snapState.position = projectedPos;
        nearbyObj->setHovered(true);
      } else {
        editor.m_snapState = PointUtils::SnapState{};
      }
    }
  } else {
    editor.m_snapState = PointUtils::SnapState{};
  }

  // === LINE TOOL VERTEX/EDGE SNAP PREVIEW ===
  // When Line Tool is active, highlight nearby vertices/edges to give visual feedback
  if (editor.m_currentToolType == ObjectType::Line || editor.m_currentToolType == ObjectType::LineSegment ||
      editor.m_currentToolType == ObjectType::Ray || editor.m_currentToolType == ObjectType::Vector) {
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

    // Handle standard line/segment preview overlay
    if ((editor.m_currentToolType == ObjectType::Line || editor.m_currentToolType == ObjectType::LineSegment) &&
        editor.lineCreationPoint1 && editor.dragMode == DragMode::CreateLineP1) {
      editor.previewLineOverlay.setPrimitiveType(sf::Lines);
      editor.previewLineOverlay.resize(2);
      editor.previewLineOverlay[0].position = editor.toSFMLVector(editor.lineCreationPoint1->getCGALPosition());
      editor.previewLineOverlay[1].position = worldPos;
      editor.previewLineOverlay[0].color = Constants::PREVIEW_COLOR;
      editor.previewLineOverlay[1].color = Constants::PREVIEW_COLOR;
      editor.hasPreviewLineOverlay = true;
    } else if (editor.m_currentToolType == ObjectType::Line || editor.m_currentToolType == ObjectType::LineSegment) {
      editor.hasPreviewLineOverlay = false;
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
    if (!std::isfinite(left) || !std::isfinite(top) || !std::isfinite(width) ||
        !std::isfinite(height)) {
      return;
    }
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

  // Handle preview rotatable rectangle (3-step: base then height)
  if (shouldUpdatePreview && editor.isCreatingRotatableRectangle) {
    try {
      if (editor.dragMode == DragMode::RotatedRectP2) {
        double dx = CGAL::to_double(cgalWorldPos.x() - editor.rectangleCorner1.x());
        double dy = CGAL::to_double(cgalWorldPos.y() - editor.rectangleCorner1.y());
        double baseLen = std::sqrt(dx * dx + dy * dy);
        editor.previewRectangle = std::make_shared<Rectangle>(
            editor.rectangleCorner1, cgalWorldPos, baseLen,
            editor.getCurrentColor(), editor.objectIdCounter /* temp id */);
      } else if (editor.dragMode == DragMode::RotatedRectHeight && editor.previewRectangle) {
        editor.previewRectangle->setCorners(editor.rectangleCorner1, editor.rectangleCorner2);

        Point_2 baseStart = editor.rectangleCorner1;
        Point_2 baseEnd = editor.rectangleCorner2;
        double dx = CGAL::to_double(baseEnd.x() - baseStart.x());
        double dy = CGAL::to_double(baseEnd.y() - baseStart.y());
        double baseLen = std::sqrt(dx * dx + dy * dy);
        if (baseLen > Constants::MIN_CIRCLE_RADIUS) {
          double ux = -dy / baseLen;
          double uy = dx / baseLen;
          double vx = CGAL::to_double(cgalWorldPos.x() - baseStart.x());
          double vy = CGAL::to_double(cgalWorldPos.y() - baseStart.y());
          double signedHeight = vx * ux + vy * uy;
          editor.previewRectangle->setHeight(signedHeight);
        }
      }
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
          case ObjectType::LineSegment:
          case ObjectType::Ray:
          case ObjectType::Vector: {
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
        pt->move(delta);
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
             (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment ||
          editor.selectedObject->getType() == ObjectType::Ray || editor.selectedObject->getType() == ObjectType::Vector)) {
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
          (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment ||
           editor.selectedObject->getType() == ObjectType::Ray || editor.selectedObject->getType() == ObjectType::Vector)) {
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
                rectPtr->setVertexPosition(2, targetPos);
              } else {
                rectPtr->setVertexPosition(1, targetPos);
              }
              return;
            }
            if (d && d.get() == selectedPoint) {
              if (rectPtr->isRotatable()) {
                rectPtr->setVertexPosition(3, targetPos);
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
            (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment ||
             editor.selectedObject->getType() == ObjectType::Ray || editor.selectedObject->getType() == ObjectType::Vector)) {
          Line* selectedLine = static_cast<Line*>(editor.selectedObject);

          Point* startPoint = selectedLine->getStartPointObject();
          Point* endPoint = selectedLine->getEndPointObject();

          if (dynamic_cast<ObjectPoint*>(startPoint) || dynamic_cast<ObjectPoint*>(endPoint)) {
            return;
          }
          if ((startPoint && startPoint->isDependent()) || (endPoint && endPoint->isDependent())) {
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
            (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment ||
             editor.selectedObject->getType() == ObjectType::Ray || editor.selectedObject->getType() == ObjectType::Vector)) {
          Line* selectedLine = static_cast<Line*>(editor.selectedObject);

          if (editor.dragMode == DragMode::MoveLineEndpointStart) {
            Point* startPoint = selectedLine->getStartPointObject();
            if (startPoint) {
              if (startPoint->isDependent()) return;
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
              if (endPoint->isDependent()) return;
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
              rect->move(delta_cgal);
              break;
            }
            case ObjectType::Polygon: {
              auto* poly = static_cast<Polygon*>(editor.selectedObject);
              poly->move(delta_cgal);
              break;
            }
            case ObjectType::RegularPolygon: {
              auto* reg = static_cast<RegularPolygon*>(editor.selectedObject);
              reg->move(delta_cgal);
              break;
            }
            case ObjectType::Triangle: {
              auto* tri = static_cast<Triangle*>(editor.selectedObject);
              tri->move(delta_cgal);
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

          if (selectedCircle->isSemicircle()) {
            selectedCircle->move(delta_cgal);
          } else {
            Point* centerPoint = selectedCircle->getCenterPointObject();
            Point* radiusPoint = selectedCircle->getRadiusPointObject();

            // Move BOTH to achieve rigid body translation
            if (centerPoint) {
              centerPoint->move(delta_cgal);
            }
            if (radiusPoint) {
              radiusPoint->move(delta_cgal);
            }
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
      int bestEdgeIndex = -1; // New: Track which edge triggered the hover

      auto considerCandidate = [&](GeometricObject* obj, double dist, int priority, int edgeIndex = -1) {
        if (!obj) return;
        if (dist > tolerance) return;

        // Always prefer non-dependent (source) objects over dependent results
        if (bestObj) {
          if (bestObj->isDependent() && !obj->isDependent()) {
            bestObj = obj;
            bestDist = dist;
            bestPriority = priority;
            bestEdgeIndex = edgeIndex;
            return;
          }
          if (!bestObj->isDependent() && obj->isDependent()) {
            return;
          }
        }
        
        // First candidate becomes best by default
        if (!bestObj) {
          bestObj = obj;
          bestDist = dist;
          bestPriority = priority;
          bestEdgeIndex = edgeIndex;
          return;
        }
        
        // NEAREST NEIGHBOR: Higher priority always wins
        if (priority > bestPriority) {
          bestObj = obj;
          bestDist = dist;
          bestPriority = priority;
          bestEdgeIndex = edgeIndex;
          return;
        }
        
        // Same priority: pick the closer one (true nearest neighbor)
        if (priority == bestPriority && dist < bestDist) {
          bestObj = obj;
          bestDist = dist;
          bestPriority = priority;
          bestEdgeIndex = edgeIndex;
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
          int nearestEdge = -1;
          for (size_t i = 0; i < edges.size(); ++i) {
            Point_2 proj;
            double relPos;
            double dist = PointUtils::projectPointOntoSegment(cgalWorldPos, edges[i], proj, relPos);
            if (dist < minDist) {
                 minDist = dist;
                 nearestEdge = static_cast<int>(i);
            }
          }
          considerCandidate(shape.get(), minDist, 1, nearestEdge);
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
        if (linePtr.get() == editor.getXAxis()) {
          // X-axis is at Y=0, so distance is simply |mouseY|
          dist = std::abs(static_cast<double>(worldPos.y));
        } else if (linePtr.get() == editor.getYAxis()) {
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
          bestObj->setHoveredEdge(bestEdgeIndex);
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
          (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment ||
           editor.selectedObject->getType() == ObjectType::Ray || editor.selectedObject->getType() == ObjectType::Vector)) {
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
          (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment ||
           editor.selectedObject->getType() == ObjectType::Ray || editor.selectedObject->getType() == ObjectType::Vector)) {
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
             (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment ||
              draggedObject->getType() == ObjectType::Ray || draggedObject->getType() == ObjectType::Vector)) {
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
      if (draggedObject && (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment ||
                            draggedObject->getType() == ObjectType::Ray || draggedObject->getType() == ObjectType::Vector)) {
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
          (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment ||
           draggedObject->getType() == ObjectType::Ray || draggedObject->getType() == ObjectType::Vector)) {
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
          (draggedObject->getType() == ObjectType::Line || draggedObject->getType() == ObjectType::LineSegment ||
           draggedObject->getType() == ObjectType::Ray || draggedObject->getType() == ObjectType::Vector)) {
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
    ImGui::SFML::ProcessEvent(editor.window, event);
    // Check if ImGui wants the mouse
    if (ImGui::GetIO().WantCaptureMouse) {
      // IGNORE these events if hovering UI
      if (event.type == sf::Event::MouseButtonPressed ||
          event.type == sf::Event::MouseButtonReleased ||
          event.type == sf::Event::MouseWheelScrolled) {
        continue;
      }

      // CRITICAL: Also ignore MouseMoved if it involves dragging (button held)
      // This stops the selection box from updating while dragging a slider
      if (event.type == sf::Event::MouseMoved) {
        // Optional: You might still want hover effects, but definitely STOP dragging logic
        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
          continue;
        }
      }
    }

    // Check if ImGui wants the keyboard (for text inputs)
    if (ImGui::GetIO().WantCaptureKeyboard) {
      if (event.type == sf::Event::KeyPressed ||
          event.type == sf::Event::KeyReleased ||
          event.type == sf::Event::TextEntered) {
        continue;
      }
    }
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
