// ObjectPoint.cpp
#include "ObjectPoint.h"

#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#pragma message("CGAL_USE_SSE2 was defined, now undefined locally for testing in " __FILE__)
#endif
#include "CGALSafeUtils.h"
#include "Circle.h"  // For m_hostCircle
#include "Constants.h"
#include "Line.h"  // For m_hostLine
#include "Polygon.h"        // For Polygon cast in updateFromMousePos
#include "Rectangle.h"      // For Rectangle cast in updateFromMousePos
#include "RegularPolygon.h" // For RegularPolygon cast in updateFromMousePos
#include "Triangle.h"       // For Triangle cast in updateFromMousePos
#include "ObjectPoint.h"
#include "Transforms.h"                                        // For toSFMLVector
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>  // For Point_2, Vector_2
#include <CGAL/number_utils.h>                                 // For CGAL::to_double
#include <cmath>  // For std::atan2, std::sqrt, std::isfinite
#include <iostream>
#include <memory>

// Forward declaration for conversion function from SFML to CGAL.
Point_2 ObjectPoint_sfmlToCGAL(const sf::Vector2f &p_sfml);
// Static helper functions for checking finiteness (similar to HandleEvents.cpp)
// It's better to move these to a shared utility header.
namespace {  // Anonymous namespace for file-local static functions

static bool is_cgal_ft_finite_local(const Kernel::FT &value) {
  try {
    if (!CGAL::is_finite(value)) return false;  // CGAL's own check
    return std::isfinite(CGAL::to_double(value));
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "ObjectPoint::is_cgal_ft_finite_local: Uncertain conversion: " << e.what()
              << std::endl;
    return false;  // Treat uncertain as potentially non-finite for safety
  } catch (const std::exception &e) {
    std::cerr << "ObjectPoint::is_cgal_ft_finite_local: Exception: " << e.what() << std::endl;
    return false;
  }
}

static bool is_cgal_point_finite_local(const Point_2 &point) {
  try {
    // CGAL::is_finite for Point_2 checks if coordinates are finite Kernel::FT
    // The direct call CGAL::is_finite(point) was causing issues with Epeck.
    // Rely on checking individual components.
    return is_cgal_ft_finite_local(point.x()) && is_cgal_ft_finite_local(point.y());
  } catch (const std::exception &e) {
    std::cerr << "ObjectPoint::is_cgal_point_finite_local: Exception: " << e.what() << std::endl;
    return false;
  }
}

}  // anonymous namespace
// --- Constructors ---
ObjectPoint::ObjectPoint(std::shared_ptr<Line> hostLine, double relativePosition,
                         const sf::Color &color)
    : Point(Point_2(0, 0), 1.0f, color),
      m_hostObject(hostLine),
      m_hostType(ObjectType::Line),
      m_hostLine(hostLine),
      m_hostCircle(),
      m_relativePositionOnLine(relativePosition),
      m_angleOnCircleRad(0) {
  if (!hostLine) throw std::invalid_argument("Host line cannot be null");
  // no shared_from_this() here
}
// In ObjectPoint.cpp - REMOVE shared_from_this from constructor:
ObjectPoint::ObjectPoint(std::shared_ptr<Circle> hostCircle, double angleRad,
                         const sf::Color &color)
    : Point(Point_2(0, 0), 1.0f, color),
      m_hostObject(hostCircle),
      m_hostType(ObjectType::Circle),
      m_hostLine(),
      m_hostCircle(hostCircle),
      m_relativePositionOnLine(0.0),
      m_angleOnCircleRad(angleRad),
      m_isBeingDragged(false) {
  std::cout << "  ObjectPoint constructor (Circle*, double) BODY entered. Host: "
            << m_hostObject.lock() << ", AngleRad: " << angleRad << std::endl;

  if (!m_hostObject.lock()) {
    std::cerr << "  ERROR: m_hostObject is NULL in ObjectPoint constructor body." << std::endl;
    throw std::runtime_error("ObjectPoint (Circle*, double) created with null circle host.");
  }

  m_cgalPosition = calculatePositionOnHost();
  m_color = color;
  updateSFMLShape();

  std::cout << "  SFML shape updated. Position: (" << CGAL::to_double(m_cgalPosition.x()) << ", "
            << CGAL::to_double(m_cgalPosition.y()) << ")" << std::endl;

  /*
  if (m_hostObject.lock()) {
    Circle *actualCircleHost = static_cast<Circle *>(m_hostObject.lock().get());
    std::cout << "  Adding self to host circle " << actualCircleHost << " child
  list..." << std::endl; actualCircleHost->addChildPoint(
        std::static_pointer_cast<ObjectPoint>(Point::shared_from_this())); // ←
  REMOVE THIS! std::cout << "  Added self to host circle child list." <<
  std::endl;
  }
  */

  std::cout << "  ObjectPoint constructor (Circle*, double) finished successfully." << std::endl;
}

// factory that _does_ the registration
std::shared_ptr<ObjectPoint> ObjectPoint::create(std::shared_ptr<Line> hostLine,
                                                 double relativePosition, const sf::Color &color) {
  auto self = std::shared_ptr<ObjectPoint>(new ObjectPoint(hostLine, relativePosition, color));
  // now safe to do shared_from_this()
  hostLine->addChildPoint(self);
  try {
    self->updatePositionFromHost();
  } catch (...) {
    // handle errors
  }
  return self;
}
std::shared_ptr<ObjectPoint> ObjectPoint::create(std::shared_ptr<Circle> hostCircle,
                                                 double angleRad, const sf::Color &color) {
  if (!hostCircle) {
    std::cerr << "ObjectPoint::create (Circle): hostCircle is null" << std::endl;
    return nullptr;
  }

  try {
    // Create the ObjectPoint first
    auto objPoint = std::shared_ptr<ObjectPoint>(new ObjectPoint(hostCircle, angleRad, color));

    if (!objPoint || !objPoint->isValid()) {
      std::cerr << "ObjectPoint::create (Circle): Created ObjectPoint is invalid" << std::endl;
      return nullptr;
    }

    // NOW register with host (this will call your Circle::addChildPoint)
    if (hostCircle) {
      std::cout << "ObjectPoint::create: About to call addChildPoint on hostCircle.get() = "
                << hostCircle.get() << ", use_count = " << hostCircle.use_count() << std::endl;
      if (hostCircle.use_count() == 0) {  // Should ideally be at least 1 if valid
        std::cerr << "ObjectPoint::create: CRITICAL WARNING - hostCircle.use_count() is 0 before "
                     "calling addChildPoint!"
                  << std::endl;
      }
      std::cout << "  Adding ObjectPoint to host circle child list..." << std::endl;
      hostCircle->addChildPoint(objPoint);  // ← This calls your method
      std::cout << "  Added ObjectPoint to host circle child list." << std::endl;
    }

    std::cout << "ObjectPoint::create (Circle): Successfully created ObjectPoint" << std::endl;
    return objPoint;

  } catch (const std::exception &e) {
    std::cerr << "ObjectPoint::create (Circle): Exception: " << e.what() << std::endl;
    return nullptr;
  }
}
// ObjectPoint::ObjectPoint(Line *hostLine, double relativePosition,
//                          const sf::Color &color)
//     : Point(Point_2(0, 0), 1.0f, color), m_hostObject(hostLine),
//       m_hostType(ObjectType::Line), m_hostLine(hostLine),
//       m_hostCircle(nullptr), m_relativePositionOnLine(relativePosition),
//       m_angleOnCircleRad(0.0), m_isBeingDragged(false) {
//   if (!m_hostLine) {
//     throw std::invalid_argument("Host line cannot be null for ObjectPoint.");
//   }
//   m_hostLine->addChildPoint(
//       std::static_pointer_cast<ObjectPoint>(Point::shared_from_this()));
//   try {
//     updatePositionFromHost(); // Initial position update
//   } catch (const std::exception &e) {
//     std::cerr << "ObjectPoint Constructor (Line): Exception during initial "
//                  "updatePositionFromParameters: "
//               << e.what() << std::endl;
//     // Set to a default safe state if update fails
//     m_cgalPosition = Point_2(0, 0);
//     if (m_hostLine && m_hostLine->getStartPointObject()) {
//       try {
//         m_cgalPosition =
//         m_hostLine->getStartPointObject()->getCGALPosition();
//       } catch (...) { /* already tried to set to 0,0 */
//       }
//     }
//     m_relativePosition = 0.0;
//     std::cerr << "ObjectPoint (Line) might be at a default/invalid position "
//                  "due to construction error."
//               << std::endl;
//   }
//   // m_shape setup is now handled by Point's constructor
//   // updateShape() is also called by Point's constructor
// }

void ObjectPoint::setHost(std::shared_ptr<GeometricObject> host, ObjectType hostType) {
  // Clear any existing host first
  clearHost();

  if (!host) {
    std::cerr << "ObjectPoint::setHost - Cannot set null host" << std::endl;
    return;
  }

  m_hostObject = host;
  m_hostType = hostType;

  if (hostType == ObjectType::Line || hostType == ObjectType::LineSegment) {
    // Use dynamic_pointer_cast for safe casting
    auto lineHost = std::dynamic_pointer_cast<Line>(host);
    if (lineHost) {
      m_hostLine = lineHost;  // Store as weak_ptr to avoid circular references
      std::cout << "ObjectPoint::setHost - Line host set successfully" << std::endl;
    } else {
      std::cerr << "ObjectPoint::setHost - Failed to cast to Line" << std::endl;
      m_hostObject.reset();
      m_hostType = ObjectType::None;
    }
  } else if (hostType == ObjectType::Circle) {
    // Use dynamic_pointer_cast for safe casting
    auto circleHost = std::dynamic_pointer_cast<Circle>(host);
    if (circleHost) {
      m_hostCircle = circleHost;  // Store as weak_ptr to avoid circular references
      std::cout << "ObjectPoint::setHost - Circle host set successfully" << std::endl;
    } else {
      std::cerr << "ObjectPoint::setHost - Failed to cast to Circle" << std::endl;
      m_hostObject.reset();
      m_hostType = ObjectType::None;
    }
  }
}

void ObjectPoint::setHost(std::shared_ptr<Line> lineHost) {
  clearHost();

  if (!lineHost) {
    std::cerr << "ObjectPoint::setHost - Cannot set null Line host" << std::endl;
    return;
  }

  m_hostObject = lineHost;
  m_hostType = ObjectType::Line;
  m_hostLine = lineHost;  // Store as weak_ptr

  std::cout << "ObjectPoint::setHost - Line host set successfully" << std::endl;
}

void ObjectPoint::setHost(std::shared_ptr<Circle> circleHost) {
  clearHost();

  if (!circleHost) {
    std::cerr << "ObjectPoint::setHost - Cannot set null Circle host" << std::endl;
    return;
  }

  m_hostObject = circleHost;
  m_hostType = ObjectType::Circle;
  m_hostCircle = circleHost;  // Store as weak_ptr

  std::cout << "ObjectPoint::setHost - Circle host set successfully" << std::endl;
}
// Fix the destructor to ensure proper cleanup
ObjectPoint::~ObjectPoint() {
  std::cout << "ObjectPoint::~ObjectPoint: ENTERED for ObjectPoint " << this << std::endl;

  try {
    // ✅ CORRECT: Check if weak_ptr is NOT expired
    if (!m_hostObject.expired()) {
      // Get raw pointer before clearing weak_ptr
      GeometricObject *rawHost = m_hostObject.lock().get();

      // ✅ Clear the host reference FIRST to prevent circular calls
      m_hostObject.reset();
      m_hostType = ObjectType::None;

      // ✅ Then remove from host's collection
      // Use the base class method which handles all host types (Line, Circle, Shape)
      if (rawHost) {
          rawHost->removeChildPoint(this);
          std::cout << "ObjectPoint destructor: Removed self from host" << std::endl;
      }
    }

    // ✅ Clear weak pointers (they should already be cleared but just to be
    // safe)
    m_hostLine.reset();
    m_hostCircle.reset();

    std::cout << "ObjectPoint::~ObjectPoint: COMPLETED for ObjectPoint " << this << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "ObjectPoint destructor: Exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "ObjectPoint destructor: Unknown exception" << std::endl;
  }
}

void ObjectPoint::clearHost() {
  // ✅ CORRECT: Check if weak_ptr is NOT expired
  if (m_hostObject.expired()) return;

  try {
    // Store the host type before clearing
    ObjectType hostType = m_hostType;

    // Clear the host reference first to prevent recursion
    m_hostObject.reset();  // Clear the weak_ptr
    m_hostType = ObjectType::None;

    // Now safely remove from host's child list
    if (hostType == ObjectType::Line || hostType == ObjectType::LineSegment) {
      if (auto line = m_hostLine.lock()) {
        // Cast the shared_ptr<Point> to shared_ptr<ObjectPoint>
        auto selfPtr = std::static_pointer_cast<ObjectPoint>(Point::shared_from_this());
        line->removeChildPoint(selfPtr.get());
      }
    } else if (hostType == ObjectType::Circle) {
      if (auto circle = m_hostCircle.lock()) {
        // Cast the shared_ptr<Point> to shared_ptr<ObjectPoint> first, then get
        // raw pointer
        auto selfPtr = std::static_pointer_cast<ObjectPoint>(Point::shared_from_this());
        circle->removeChildPoint(selfPtr.get());
      }
    }

    // Clear the weak pointers
    m_hostLine.reset();
    m_hostCircle.reset();

  } catch (const std::exception &e) {
    std::cerr << "Exception in ObjectPoint::clearHost: " << e.what() << std::endl;
  }
}

void ObjectPoint::projectOntoHost(const Point_2 &clickPos) {
  if (!m_hostObject.lock()) return;

  if (!is_cgal_point_finite_local(clickPos)) {
    std::cerr << "ObjectPoint::projectOntoHost: Input clickPos is not finite. "
                 "Aborting projection."
              << std::endl;
    return;
  }

  Point_2 projectedPos;

  if (m_hostType == ObjectType::Line || m_hostType == ObjectType::LineSegment) {
    if (auto hostLine = m_hostLine.lock()) {
      try {
        // Lock the weak_ptr to get a shared_ptr
        auto line = m_hostLine.lock();
        if (!line) {
          throw std::runtime_error("Host line pointer expired");
        }
        // Ensure host line endpoints are valid before projection
        Point_2 startP = line->getStartPoint();
        Point_2 endP = line->getEndPoint();
        if (!is_cgal_point_finite_local(startP) || !is_cgal_point_finite_local(endP)) {
          std::cerr << "ObjectPoint::projectOntoHost: Host line endpoints are "
                       "not finite. Aborting projection."
                    << std::endl;
          return;
        }
        if (startP == endP) {
          std::cerr << "ObjectPoint::projectOntoHost: Host line endpoints are "
                       "coincident. Using start point."
                    << std::endl;
          projectedPos = startP;
        } else {
          Line_2 hostCGALLine(startP, endP);
          projectedPos = hostCGALLine.projection(clickPos);
        }
      } catch (const std::exception &e) {
        std::cerr << "ObjectPoint::projectOntoHost (Line): Exception during "
                     "projection: "
                  << e.what() << std::endl;
        return;  // Abort on error
      }
    }
  } else if (m_hostType == ObjectType::Circle) {
    if (auto circle = m_hostCircle.lock()) {
      try {
        Point_2 centerP = circle->getCenterPoint();  // Use 'circle' not 'm_hostCircle'
        if (!is_cgal_point_finite_local(centerP)) {
          std::cerr << "ObjectPoint::projectOntoHost: Host circle center is "
                       "not finite. Aborting projection."
                    << std::endl;
          return;
        }
        projectedPos = circle->projectOntoCircumference(clickPos);
      } catch (const std::exception &e) {
        std::cerr << "ObjectPoint::projectOntoHost (Circle): Exception during "
                     "projection: "
                  << e.what() << std::endl;
        return;  // Abort on error
      }
    }
  }

  if (!is_cgal_point_finite_local(projectedPos)) {
    std::cerr << "ObjectPoint::projectOntoHost: Resulting projectedPos is not "
                 "finite. Aborting update."
              << std::endl;
    return;
  }

  m_cgalPosition = projectedPos;    // Update internal position
  calculateAttachmentParameters();  // Recalculate relativePosition or angle
  updateSFMLShape();
}

void ObjectPoint::updateSFMLShape() {
  // Get the current SFML position
  sf::Vector2f position = cgalToSFML(m_cgalPosition);
  // Call the version that takes a position
  updateSFMLShape(position);
}

void ObjectPoint::updateSFMLShape(const sf::Vector2f &position) {
  // Call the base class implementation first
  Point::updateSFMLShape(position);

  // Then apply any ObjectPoint-specific styling
  // For example, use a different color to distinguish ObjectPoints
  if (!m_selected && !m_hovered) {  // Only modify if not already highlighted
    m_sfmlShape.setFillColor(Constants::OBJECT_POINT_DEFAULT_COLOR);
    m_sfmlShape.setOutlineColor(Constants::OBJECT_POINT_OUTLINE_COLOR);
  }

  // You may want to add specific styling for when the point is attached to
  // different host types
  if (m_hostType == ObjectType::Line) {
    // Line-specific styling
    if (!m_selected) {  // Don't override selection styling
      // Maybe use a different outline thickness or pattern for line-hosted
      // points
    }
  } else if (m_hostType == ObjectType::Circle) {
    // Circle-specific styling
    if (!m_selected) {  // Don't override selection styling
      // Maybe use a different outline thickness or pattern for circle-hosted
      // points
    }
  }
}

void ObjectPoint::calculateAttachmentParameters() {
  if (m_hostObject.expired()) {
    std::cerr << "ObjectPoint::calculateAttachmentParameters: No host object." << std::endl;
    return;
  }

  if (!is_cgal_point_finite_local(m_cgalPosition)) {
    std::cerr << "ObjectPoint::calculateAttachmentParameters: m_cgalPosition "
                 "is not finite. Cannot calculate parameters."
              << std::endl;
    // Set safe defaults without referencing undefined variables
    if (m_hostType == ObjectType::Line || m_hostType == ObjectType::LineSegment) {
      m_relativePositionOnLine = 0.5;  // Use correct member variable name
    } else if (m_hostType == ObjectType::Circle) {
      m_angleOnCircleRad = 0.0;
    }
    return;
  }

  if (m_hostType == ObjectType::Line || m_hostType == ObjectType::LineSegment) {
    if (auto hostLine = m_hostLine.lock()) {
      Point_2 hostStartPos, hostEndPos;
      try {
        hostStartPos = hostLine->getStartPoint();
        hostEndPos = hostLine->getEndPoint();
      } catch (const std::exception &e) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters (Line): "
                     "Exception getting host endpoints: "
                  << e.what() << std::endl;
        m_relativePositionOnLine = 0.5;
        return;
      }

      if (!is_cgal_point_finite_local(hostStartPos) || !is_cgal_point_finite_local(hostEndPos)) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters: Host line "
                     "endpoints are not finite."
                  << std::endl;
        m_relativePositionOnLine = 0.5;
        return;
      }

      if (hostStartPos == hostEndPos) {
        m_relativePositionOnLine = 0.5;
        return;
      }

      Vector_2 lineVector = hostEndPos - hostStartPos;
      if (!is_cgal_ft_finite_local(lineVector.x()) || !is_cgal_ft_finite_local(lineVector.y())) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters: lineVector "
                     "components are not finite."
                  << std::endl;
        m_relativePositionOnLine = 0.5;
        return;
      }

      Vector_2 pointVector = m_cgalPosition - hostStartPos;
      if (!is_cgal_ft_finite_local(pointVector.x()) || !is_cgal_ft_finite_local(pointVector.y())) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters: pointVector "
                     "components are not finite."
                  << std::endl;
        m_relativePositionOnLine = 0.5;
        return;
      }

      // NOW define these variables in the correct scope
      Kernel::FT lineSqLength_ft = lineVector.squared_length();
      if (!is_cgal_ft_finite_local(lineSqLength_ft) || CGAL::is_zero(lineSqLength_ft)) {
        m_relativePositionOnLine = 0.5;
        return;
      }

      Kernel::FT dotProduct_ft = pointVector * lineVector;
      if (!is_cgal_ft_finite_local(dotProduct_ft)) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters: dotProduct "
                     "is not finite."
                  << std::endl;
        m_relativePositionOnLine = 0.5;
        return;
      }

      // Calculate relative position using defined variables
      m_relativePositionOnLine = CGAL::to_double(dotProduct_ft / lineSqLength_ft);

      // Clamp for segments
      if (hostLine->isSegment()) {
        m_relativePositionOnLine = std::max(0.0, std::min(1.0, m_relativePositionOnLine));
      }
    }
  } else if (m_hostType == ObjectType::Circle) {
    if (auto circle = m_hostCircle.lock()) {
      Point_2 hostCenterPos;
      try {
        hostCenterPos = circle->getCenterPoint();
      } catch (const std::exception &e) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters (Circle): "
                     "Exception getting host center: "
                  << e.what() << std::endl;
        m_angleOnCircleRad = 0.0;
        return;
      }

      if (!is_cgal_point_finite_local(hostCenterPos)) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters: Host circle "
                     "center is not finite."
                  << std::endl;
        m_angleOnCircleRad = 0.0;
        return;
      }

      Vector_2 vectorToPoint = m_cgalPosition - hostCenterPos;
      if (!is_cgal_ft_finite_local(vectorToPoint.x()) ||
          !is_cgal_ft_finite_local(vectorToPoint.y())) {
        std::cerr << "ObjectPoint::calculateAttachmentParameters: "
                     "vectorToPoint components for circle are not finite."
                  << std::endl;
        m_angleOnCircleRad = 0.0;
        return;
      }

      // Ensure components are not both zero before atan2
      if (CGAL::is_zero(vectorToPoint.x()) && CGAL::is_zero(vectorToPoint.y())) {
        m_angleOnCircleRad = 0.0;
      } else {
        try {
          m_angleOnCircleRad = std::atan2(CGAL::to_double(vectorToPoint.y()),
                                          CGAL::to_double(vectorToPoint.x()));  //
        } catch (const CGAL::Uncertain_conversion_exception &e) {
          std::cerr << "ObjectPoint::calculateAttachmentParameters (Circle): "
                       "Uncertain conversion for atan2: "
                    << e.what() << std::endl;
          m_angleOnCircleRad = 0.0;
        }
      }
    }
  }
}

void ObjectPoint::updatePositionFromHost() {
  if (m_hostObject.expired()) {
    if (Constants::DEBUG_OBJECT_POINT_UPDATE) {  // Add a new debug flag if
                                                 // needed
      std::cout << "ObjectPoint " << this << " updatePositionFromHost: No host object."
                << std::endl;
    }
    // If no host, its position is fixed (or should be handled as an error)
    // Call Point::update() to ensure its own SFML shape is updated if it's
    // standalone.
    Point::update();
    return;
  }

  try {
    Point_2 newPos = calculatePositionOnHost();
    if (Constants::DEBUG_OBJECT_POINT_UPDATE) {
      std::cout << "ObjectPoint " << this << " updatePositionFromHost: Calculated new CGAL Pos: ("
                << CGAL::to_double(newPos.x()) << ", " << CGAL::to_double(newPos.y())
                << "), Finite: " << (CGAL::is_finite(newPos.x()) && CGAL::is_finite(newPos.y()))
                << std::endl;
    }
    Point::setCGALPosition(newPos);  // This will call Point::updateSFMLShape()

    // Fix for propagation lag: Force immediate visual update of connected lines
    // This ensures lines attached to the object point follow smoothly during parent shape updates
    const auto& connected = getConnectedLines();
    for (const auto& weakLine : connected) {
      if (auto line = weakLine.lock()) {
        line->update();
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "ObjectPoint " << this << " updatePositionFromHost: Exception: " << e.what()
              << std::endl;
    m_isValid = false;  // Mark as invalid if position calculation/setting fails
  }
}

void ObjectPoint::updatePositionFromParameters() {
  if (!m_hostObject.lock()) {
    return;  // No host, nothing to update from
  }

  // Call updatePositionFromHost which already has the logic to position
  // the point based on attachment parameters
  updatePositionFromHost();
}

Point_2 ObjectPoint::calculatePositionOnHost() const {
  if (!m_hostObject.lock()) {
    std::cerr << "ObjectPoint::calculatePositionOnHost - No host object" << std::endl;
    return Point::getCGALPosition();  // Default fallback position
  }

  if (m_hostType == ObjectType::Line || m_hostType == ObjectType::LineSegment) {
    //  OLD: Line *line = dynamic_cast<Line *>(m_hostObject.get());
    //  NEW: Use the weak_ptr directly
    if (auto line = m_hostLine.lock()) {
      if (!line->getStartPointObject() || !line->getEndPointObject()) {
        std::cerr << "ObjectPoint::calculatePositionOnHost - Line has invalid "
                     "endpoints"
                  << std::endl;
        return Point::getCGALPosition();
      }
      Point_2 p1 = line->getStartPoint();
      Point_2 p2 = line->getEndPoint();
      if (p1 == p2) {
        return p1;  // Degenerate case, return the point
      }
      // Linear interpolation between p1 and p2
      double t = m_relativePositionOnLine;
      Kernel::FT p1x = p1.x();
      Kernel::FT p1y = p1.y();
      Kernel::FT p2x = p2.x();
      Kernel::FT p2y = p2.y();
      Kernel::FT t_ft(t);  // Convert double 't' to Kernel::FT if not already

      Point_2 resultPos(p1x + t_ft * (p2x - p1x), p1y + t_ft * (p2y - p1y));
      return resultPos;
    } else {
      std::cerr << "ObjectPoint::calculatePositionOnHost - Line host expired" << std::endl;
      return Point::getCGALPosition();
    }
  } else if (m_hostType == ObjectType::Circle) {
    // OLD: Circle *circle = dynamic_cast<Circle *>(m_hostObject);
    if (auto circle = m_hostCircle.lock()) {
      Point_2 center = circle->getCenterPoint();
      if (m_isCircleCenterAttachment) {
        // Attach directly to the circle center
        return center;
      } else {
        double radius = circle->getRadius();
        double x = CGAL::to_double(center.x()) + radius * std::cos(m_angleOnCircleRad);
        double y = CGAL::to_double(center.y()) + radius * std::sin(m_angleOnCircleRad);
        return circle->projectOntoCircumference(Point_2(x, y));
      }
    } else {
      std::cerr << "ObjectPoint::calculatePositionOnHost - Circle host expired" << std::endl;
      return Point::getCGALPosition();
    }
  } else if (m_isShapeEdgeAttachment) {
      // Generic shape edge logic
      if (auto shape = m_hostShape.lock()) {
          auto edges = shape->getEdges();
          if (m_edgeIndex < edges.size()) {
              const Segment_2& edge = edges[m_edgeIndex];
              Point_2 a = edge.source();
              Point_2 b = edge.target();

              Kernel::FT t_ft(m_edgeRelativePos);
              return Point_2(
                  a.x() + t_ft * (b.x() - a.x()),
                  a.y() + t_ft * (b.y() - a.y())
              );
          }
      }
  }
  // Fallback in case host type is not recognized.
  return Point::getCGALPosition();
}

// Static factory to create an ObjectPoint attached at the circle's center
std::shared_ptr<ObjectPoint> ObjectPoint::createCenter(std::shared_ptr<Circle> hostCircle,
                                                       const sf::Color &color) {
  if (!hostCircle) return nullptr;
  try {
    auto obj = std::shared_ptr<ObjectPoint>(new ObjectPoint(hostCircle, 0.0, color));
    obj->m_isCircleCenterAttachment = true;
    // Set host explicitly to ensure proper registration
    obj->setHost(hostCircle);
    obj->updatePositionFromHost();
    return obj;
  } catch (const std::exception &e) {
    std::cerr << "ObjectPoint::createCenter: exception: " << e.what() << std::endl;
    return nullptr;
  }
}

// Static factory to create an ObjectPoint attached to any shape's edge
std::shared_ptr<ObjectPoint> ObjectPoint::createOnShapeEdge(
    std::shared_ptr<GeometricObject> hostShape,
    size_t edgeIndex,
    double relativePosition,
    const sf::Color &color) {
  
  if (!hostShape) {
    std::cerr << "ObjectPoint::createOnShapeEdge: hostShape is null" << std::endl;
    return nullptr;
  }
  
  try {
    // Get the edges from the shape using the generic interface
    auto edges = hostShape->getEdges();
    if (edgeIndex >= edges.size()) {
      std::cerr << "ObjectPoint::createOnShapeEdge: edgeIndex " << edgeIndex 
                << " out of bounds (shape has " << edges.size() << " edges)" << std::endl;
      return nullptr;  
    }
    
    // Calculate position on the edge using linear interpolation
    const Segment_2& edge = edges[edgeIndex];
    Point_2 a = edge.source();
    Point_2 b = edge.target();
    
    // Clamp relative position to [0, 1]
    double t = std::max(0.0, std::min(1.0, relativePosition));
    
    // Interpolate position along edge
    Kernel::FT t_ft(t);
    Point_2 initialPos(
      a.x() + t_ft * (b.x() - a.x()),
      a.y() + t_ft * (b.y() - a.y())
    );
    
    // Create the ObjectPoint using private constructor
    auto objPoint = std::shared_ptr<ObjectPoint>(new ObjectPoint());
    
    // Set up shape edge attachment
    objPoint->m_isShapeEdgeAttachment = true;
    objPoint->m_hostShape = hostShape;
    objPoint->m_edgeIndex = edgeIndex;
    objPoint->m_edgeRelativePos = t;
    objPoint->m_hostObject = hostShape;
    objPoint->m_hostType = hostShape->getType();
    objPoint->m_color = color;
    
    // Set position
    objPoint->setCGALPosition(initialPos);
    objPoint->updateSFMLShape();
    
    // Register with host
    hostShape->addChildPoint(objPoint);
    
    std::cout << "[ObjPoint] Created on shape edge " << edgeIndex 
              << " at relative pos " << t << std::endl;
    
    return objPoint;
    
  } catch (const std::exception &e) {
    std::cerr << "ObjectPoint::createOnShapeEdge: exception: " << e.what() << std::endl;
    return nullptr;
  }
}

void ObjectPoint::update() {
  if (m_hostObject.lock()) {
    updatePositionFromHost();  // This updates m_cgalPosition and calls
                               // Point::updateSFMLShape()
  } else {
    // If it's a free point (host cleared), it might still need its SFML
    // shape updated if its state (like selection/hover) changed.
    // Point::updateSFMLShape(); // Base Point::update() is empty, so direct
    // call if needed. However, updatePositionFromHost already triggers it
    // via setCGALPosition. If no host, selection/hover changes are main
    // drivers for updateSFMLShape.
  }
  // Point::update(); // Base Point::update() is currently empty.
  // updateSFMLShape() is called by updatePositionFromHost via
  // Point::setCGALPosition or by direct interaction methods like
  // setSelected.
}

void ObjectPoint::setPosition(const sf::Vector2f &newSfmlPos) {
  Point::setCGALPosition(ObjectPoint_sfmlToCGAL(newSfmlPos));  // Set m_cgalPosition in base, this
                                                               // calls Point::updateSFMLShape()
  if (m_hostObject.lock()) {
    // If moved freely, recalculate attachment parameters to the host
    calculateAttachmentParameters();
    // Then snap back to the host based on new parameters
    updatePositionFromHost();  // This will again call Point::setCGALPosition
                               // and Point::updateSFMLShape
  }
}

void ObjectPoint::setCGALPosition(const Point_2 &newPos) {
  Point::setCGALPosition(newPos);  // Set m_cgalPosition in base, this calls
                                   // Point::updateSFMLShape()
  if (m_hostObject.lock()) {
    // If moved freely, recalculate attachment parameters to the host
    calculateAttachmentParameters();
    // Then snap back to the host based on new parameters
    updatePositionFromHost();  // This will again call Point::setCGALPosition
                               // and Point::updateSFMLShape
  }
}

void ObjectPoint::dragTo(const sf::Vector2f &mouseWorldPos_sfml) {
  m_isBeingDragged = true;
  Point_2 clickPos_cgal(CGAL::to_double(mouseWorldPos_sfml.x),
                        CGAL::to_double(mouseWorldPos_sfml.y));

  if (!is_cgal_point_finite_local(clickPos_cgal)) {
    std::cerr << "ObjectPoint::dragTo: Mouse click position (clickPos_cagal) "
                 "is not finite. Aborting drag."
              << std::endl;
    m_isBeingDragged = false;  // Reset drag state
    return;
  }

  projectOntoHost(clickPos_cgal);  // This now includes more safety checks
                                   // and updates m_cgalPosition & parameters
  m_isBeingDragged = false;        // Reset drag state after update
}

// --- SFML / CGAL Conversion ---
sf::Vector2f ObjectPoint_cgalToSFML(const Point_2 &p) {
  return sf::Vector2f(static_cast<float>(CGAL::to_double(p.x())),
                      static_cast<float>(CGAL::to_double(p.y())));
}
Point_2 ObjectPoint_sfmlToCGAL(const sf::Vector2f &p_sfml) { return Point_2(p_sfml.x, p_sfml.y); }

// --- Debugging ---
void ObjectPoint::validate() const {
#ifdef _DEBUG
  std::cout << "ObjectPoint validation: " << this << std::endl;
  // Print the state of m_hostObject (weak_ptr) safely
  if (auto hostShared = m_hostObject.lock()) {
    std::cout << "  Host: " << hostShared.get() << std::endl;
  } else {
    std::cout << "  Host: (expired/null)" << std::endl;
  }
  std::cout << "  Host Type: " << static_cast<int>(m_hostType) << std::endl;

  if (!m_hostObject.expired()) {
    // Verify my host knows about me
    bool foundInHost = false;

    try {
      if (m_hostType == ObjectType::Line || m_hostType == ObjectType::LineSegment) {
        auto hostShared = m_hostObject.lock();
        Line *hostLine = dynamic_cast<Line *>(hostShared.get());
        if (hostLine) {
          const auto &hostedPoints = hostLine->getHostedObjectPoints();
          foundInHost = std::find_if(hostedPoints.begin(), hostedPoints.end(),
                                     [this](const std::weak_ptr<ObjectPoint> &weakPtr) {
                                       if (auto sharedPtr = weakPtr.lock()) {
                                         return sharedPtr.get() == this;
                                       }
                                       return false;  // expired weak_ptr can't match
                                     }) != hostedPoints.end();
        }
      } else if (m_hostType == ObjectType::Circle) {
        // Similar check for Circle hosts (implement if needed)
      }

      std::cout << "  Found in host's list: " << (foundInHost ? "Yes" : "No") << std::endl;
    } catch (...) {
      std::cout << "  Exception while checking host relationship" << std::endl;
    }
  }
#endif
}

// --- SFML Drawable ---
// --- SFML Drawable ---
void ObjectPoint::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  // Delegate to base Point drawing for now, which supports scaling
  Point::draw(window, scale, forceVisible);
}


// --- Interaction ---
sf::FloatRect ObjectPoint::getGlobalBounds() const {
  float interactionRadius =
      Constants::OBJECT_POINT_RADIUS + Constants::OBJECT_POINT_INTERACTION_PADDING;
  sf::Vector2f pos = ObjectPoint_cgalToSFML(Point::getCGALPosition());
  return sf::FloatRect(pos.x - interactionRadius, pos.y - interactionRadius, 2 * interactionRadius,
                       2 * interactionRadius);
}

bool ObjectPoint::contains(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  // Effective radius for interaction, potentially larger than visual radius
  float interactionRadius = Constants::OBJECT_POINT_RADIUS +
                            Constants::OBJECT_POINT_INTERACTION_PADDING;  // Use new constants
  sf::Vector2f shapePos_sfml = m_sfmlShape.getPosition();
  sf::Vector2f diff = worldPos_sfml - shapePos_sfml;
  float distSq = diff.x * diff.x + diff.y * diff.y;
  return distSq <= (interactionRadius + tolerance) * (interactionRadius + tolerance);
}

void ObjectPoint::setSelected(bool sel) {
  Point::setSelected(sel);
  // updateSFMLShape(); // Point::setSelected already calls updateSFMLShape
}

double ObjectPoint::getRelativePositionOnLine() const { return m_relativePositionOnLine; }

double ObjectPoint::getAngleOnCircle() const { return m_angleOnCircleRad; }

void ObjectPoint::translate(const Vector_2 &offset) {
  // When an ObjectPoint is translated, it implies its host is also moving,
  // or it's being moved along its host.
  // If the host itself is translated, the host should call
  // updatePositionFromHost() on its ObjectPoints. If the ObjectPoint is
  // dragged (which is a form of translation), dragTo() handles it. A direct
  // call to translate() on an ObjectPoint is ambiguous. For now, assume it
  // means to try and move freely and then re-attach.
  Point::translate(offset);  // This updates m_cgalPosition and calls
                             // Point::updateSFMLShape()
  if (m_hostObject.lock()) {
    calculateAttachmentParameters();
    updatePositionFromHost();
  }
}

Point_2 ObjectPoint::getPositionCGAL() const {
  // Return the CGAL position from the base class
  return Point::getCGALPosition();
}

sf::Vector2f ObjectPoint::getSfmlPosition() const {
  // Convert the CGAL position to SFML coordinates
  return ObjectPoint_cgalToSFML(Point::getCGALPosition());
}

bool ObjectPoint::isValid() const {
  if (!Point::isValid()) return false;  // Base class validity
  if (!m_hostObject.lock()) return false;
  if (m_hostType == ObjectType::Line || m_hostType == ObjectType::LineSegment) {
    if (auto line = m_hostLine.lock()) {
      if (!line) {
        return false;
      }
      // Add checks for host line validity if necessary, e.g., finite
      // endpoints
      try {
        if (!is_cgal_point_finite_local(line->getStartPoint()) ||
            !is_cgal_point_finite_local(line->getEndPoint()))
          return false;
      } catch (...) {
        return false;
      }
    } else {
      return false;
    }

  } else if (m_hostType == ObjectType::Circle) {
    if (auto circle = m_hostCircle.lock()) {
      try {
        if (!is_cgal_point_finite_local(circle->getCenterPoint()) ||
            !std::isfinite(circle->getRadius()))
          return false;
      } catch (...) {
        return false;
      }
    } else {
      return false;  // Circle host expired
    }
  } else if (m_isShapeEdgeAttachment) {
    // Shape edge attachment - check host shape still exists
    if (auto hostShape = m_hostShape.lock()) {
      if (!hostShape->isValid()) return false;
      // Verify edge index is still valid
      auto edges = hostShape->getEdges();
      if (m_edgeIndex >= edges.size()) return false;
    } else {
      return false;  // Host shape expired
    }
  } else {
    // Unknown host type - allow it if hostObject is valid
    // (This can happen for other attachment types)
    if (!m_hostObject.lock()) return false;
  }
  return is_cgal_point_finite_local(m_cgalPosition);
}

void ObjectPoint::updateFromMousePos(const sf::Vector2f &mousePos) {
  // This function should update the object point position based on mouse
  // position

  if (auto line = m_hostLine.lock()) {
    // For a line, calculate the projection of mousePos onto the line
    try {
      // Convert SFML position to CGAL
      Point_2 cgalMousePos(mousePos.x, mousePos.y);

      // Get the line's endpoints
      Point_2 lineStart = line->getStartPoint();
      Point_2 lineEnd = line->getEndPoint();

      // Create a CGAL line
      Line_2 cgalLine(lineStart, lineEnd);

      // Project mouse position onto the line
      Point_2 projectedPoint = cgalLine.projection(cgalMousePos);

      // Calculate the new relative position along the line
      Vector_2 lineVector(lineStart, lineEnd);
      Vector_2 pointVector(lineStart, projectedPoint);

      Kernel::FT lineLength = lineVector.squared_length();
      if (!CGAL::is_zero(lineLength)) {
        m_relativePositionOnLine = CGAL::to_double((pointVector * lineVector) / lineLength);

        // Clamp between 0 and 1 for line segments
        if (line->isSegment()) {
          m_relativePositionOnLine = std::max(0.0, std::min(1.0, m_relativePositionOnLine));
        }

        // Update the object position
        updatePositionFromParameters();
        updateSFMLShape();
        std::cout << "ObjectPoint updated on line, relative position: " << m_relativePositionOnLine
                  << std::endl;
      }
    } catch (const std::exception &e) {
      std::cerr << "Error updating ObjectPoint on line: " << e.what() << std::endl;
    }
  } else if (auto circle = m_hostCircle.lock()) {
    try {
      // For a circle, calculate the angle to the mouse position
      Point_2 center = circle->getCenterPoint();
      Point_2 cgalMousePos(mousePos.x, mousePos.y);

      Vector_2 mouseVector(center, cgalMousePos);
      m_angleOnCircleRad =
          std::atan2(CGAL::to_double(mouseVector.y()), CGAL::to_double(mouseVector.x()));

      // Update the object position
      updatePositionFromParameters();
      updateSFMLShape();
      std::cout << "ObjectPoint updated on circle, angle: " << m_angleOnCircleRad << " radians"
                << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error updating ObjectPoint on circle: " << e.what() << std::endl;
    }
  } else if (m_isShapeEdgeAttachment) {
    // === SHAPE EDGE / VERTEX ATTACHMENT ===
    if (auto hostShape = m_hostShape.lock()) {
      try {
        Point_2 cgalMousePos(mousePos.x, mousePos.y);

        // *** CRITICAL: Check CREATION TYPE, not current position ***
        // m_isVertexAnchor = true  -> this is a vertex anchor that resizes shape
        // m_isVertexAnchor = false -> this is an edge slider that only slides

        if (m_isVertexAnchor) {
          // === VERTEX FORWARDING: Forward drag to parent shape ===
          // Vertex anchors are always created at m_edgeIndex (t=0.0 = start of edge)
          size_t vertexIndex = m_edgeIndex;

          std::cout << "[ObjPoint] Forwarding drag to shape vertex " << vertexIndex << std::endl;

          // Call shape-specific setVertexPosition
          switch (hostShape->getType()) {
            case ObjectType::Rectangle:
            case ObjectType::RectangleRotatable: {
              auto* rect = static_cast<Rectangle*>(hostShape.get());
              rect->setVertexPosition(vertexIndex, cgalMousePos);
              break;
            }
            case ObjectType::Polygon: {
              auto* poly = static_cast<Polygon*>(hostShape.get());
              poly->setVertexPosition(vertexIndex, cgalMousePos);
              break;
            }
            case ObjectType::RegularPolygon: {
              auto* reg = static_cast<RegularPolygon*>(hostShape.get());
              reg->setCreationPointPosition(vertexIndex, cgalMousePos);
              break;
            }
            case ObjectType::Triangle: {
              auto* tri = static_cast<Triangle*>(hostShape.get());
              tri->setVertexPosition(vertexIndex, cgalMousePos);
              break;
            }
            default:
              std::cerr << "[ObjPoint] Unknown shape type for vertex forwarding" << std::endl;
              break;
          }

          // Shape's setVertexPosition should call updateHostedPoints() which updates us
          // But force an update just in case
          updatePositionFromHost();

        } else {
          // === EDGE SLIDING: Project onto edge and update t parameter ===
          auto edges = hostShape->getEdges();
          if (m_edgeIndex < edges.size()) {
            const Segment_2& edge = edges[m_edgeIndex];
            Point_2 a = edge.source();
            Point_2 b = edge.target();

            Vector_2 ab = b - a;
            double lenSq = CGAL::to_double(ab.squared_length());

            if (lenSq > 1e-10) {
              Vector_2 ap = cgalMousePos - a;
              double t = CGAL::to_double(ap * ab) / lenSq;

              // Clamp t to [0, 1]
              t = std::max(0.0, std::min(1.0, t));
              m_edgeRelativePos = t;

              // Calculate new position on edge
              Kernel::FT t_ft(t);
              m_cgalPosition = Point_2(
                a.x() + t_ft * (b.x() - a.x()),
                a.y() + t_ft * (b.y() - a.y())
              );

              updateSFMLShape();
              updateConnectedLines();

              std::cout << "[ObjPoint] Slid to edge position t=" << t << std::endl;
            }
          }
        }
      } catch (const std::exception &e) {
        std::cerr << "Error updating ObjectPoint on shape edge: " << e.what() << std::endl;
      }
    }
  }
}
