#pragma message("--- In Line.cpp: Checking CGAL preprocessor flags ---")

#ifdef CGAL_HAS_THREADS
#pragma message("Line.cpp: CGAL_HAS_THREADS is DEFINED before any action.")
#else
#pragma message("Line.cpp: CGAL_HAS_THREADS is NOT DEFINED before any action.")
#endif

// CGAL_USE_SSE2 check removed

// Attempt to set the desired state
#define CGAL_HAS_THREADS 1
// #undef CGAL_USE_SSE2 removed
#pragma message("Line.cpp: Action: Defined CGAL_HAS_THREADS. CGAL_USE_SSE2 not modified.")

// Verify after action
#ifdef CGAL_HAS_THREADS
#pragma message("Line.cpp: CGAL_HAS_THREADS is DEFINED after action.")
#else
#pragma message("Line.cpp: CGAL_HAS_THREADS is NOT DEFINED after action. (Problem!)")
#endif

#pragma message("--- End of preprocessor checks in Line.cpp ---")

#include "Line.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "CGALSafeUtils.h"
#include "Constants.h"
#include "ObjectPoint.h"
#include "Point.h"
#include <set>

using namespace CGALSafeUtils;

thread_local std::set<Line *> Line::s_updatingLines;
// Helper function to debug CGAL point coordinates
/* void debug_cgal_point(const Point_2 &pt, const std::string &point_name,
                      const std::string &context) {
  try {
    double x = CGAL::to_double(pt.x());
    double y = CGAL::to_double(pt.y());
    bool x_finite = CGAL::is_finite(pt.x()); // Check original Kernel::FT
    bool y_finite = CGAL::is_finite(pt.y()); // Check original Kernel::FT

    std::cerr << "DEBUG_CGAL_POINT [" << context << "] " << point_name << ": ("
              << x << ", " << y << ")"
              << " | Finite: (x: " << (x_finite ? "yes" : "NO")
              << ", y: " << (y_finite ? "yes" : "NO") << ")" << std::endl;
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "DEBUG_CGAL_POINT [" << context << "] " << point_name
              << ": Uncertain conversion to double: " << e.what() << std::endl;
    // Optionally, try to print approximate interval if using Epeck
  } catch (const std::exception &e) {
    std::cerr << "DEBUG_CGAL_POINT [" << context << "] " << point_name
              << ": Exception converting to double: " << e.what() << std::endl;
  }
}
 */
Line::Line(std::shared_ptr<Point> startPoint, std::shared_ptr<Point> endPoint, bool isSegment,
           const sf::Color &color)
    : GeometricObject(isSegment ? ObjectType::LineSegment : ObjectType::Line, color),
      m_lineType(isSegment ? LineType::Segment : LineType::Infinite),
      m_startPoint(startPoint),
      m_endPoint(endPoint),
      m_isSegment(isSegment),
      m_color(color),
      m_isUpdatingInternally(false),
      m_isUnderDirectManipulation(false),
      m_isParallelLine(false),
      m_isPerpendicularLine(false),
      m_constraintRefObject(),
      m_constraintRefEdgeIndex(-1),
      m_dashed(false),
      m_dashLength(Constants::LINE_DASH_LENGTH),
      m_gapLength(Constants::LINE_DASH_GAP),
      m_constraintDirection(0, 0),
      m_thickness(Constants::LINE_THICKNESS),
      m_inConstraint(false),
      m_externallyMovedEndpoint(nullptr) {
  bool localDebug = Constants::DEBUG_LINE_CREATION;

  if (localDebug) {
    std::cout << "Line Constructor: Creating " << (isSegment ? "segment" : "infinite line")
              << " with color (" << static_cast<int>(color.r) << ", " << static_cast<int>(color.g)
              << ", " << static_cast<int>(color.b) << ")" << std::endl;
  }

  // Enhanced validation with better error messages
  if (!startPoint || !endPoint) {
    std::cerr << "Line Constructor: FATAL - Null endpoint provided" << std::endl;
    m_canonicalLength = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
    throw std::invalid_argument("Line constructor requires valid (non-null) Point objects");
  }

  if (!startPoint->isValid() || !endPoint->isValid()) {
    std::cerr << "Line Constructor: ERROR - Invalid Point objects provided" << std::endl;
    m_canonicalLength = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
    throw std::runtime_error("Line constructor: Input Point objects are not valid");
  }

  // Calculate canonical length with better error handling
  try {
    calculateCanonicalLength();
    if (localDebug) {
      std::cout << "Line Constructor: Canonical length = " << CGAL::to_double(m_canonicalLength)
                << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Line Constructor: Exception calculating canonical length: " << e.what()
              << ". Using default." << std::endl;
    m_canonicalLength = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
  }

  // Initialize SFML shape
  initializeSFMLShape();

  // Build geometry and register connections
  try {
    updateCGALLine();
    updateSFMLShape();

    // CRITICAL: Register this line with its endpoints
    // registerWithEndpoints();
    if (localDebug) {
      std::cout << "Line Constructor: Successfully created line between P" << m_startPoint->getID()
                << " and P" << m_endPoint->getID() << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Line Constructor: Exception during initialization: " << e.what() << std::endl;
    throw;
  }
}



// ------ctr cgal with ID------
Line::Line(std::shared_ptr<Point> start, std::shared_ptr<Point> end, bool isSegment,
           const sf::Color &color, unsigned int id)
    : GeometricObject(isSegment ? ObjectType::LineSegment : ObjectType::Line, color, id),
      m_lineType(isSegment ? LineType::Segment : LineType::Infinite),
      m_startPoint(start),
      m_endPoint(end),
      m_isSegment(isSegment),
      m_color(color),
      m_dashLength(Constants::LINE_DASH_LENGTH),
      m_gapLength(Constants::LINE_DASH_GAP) {
  if (!m_startPoint || !m_endPoint) {
    throw std::invalid_argument("Line constructor requires valid Point objects");
  }

  if (!m_startPoint->isValid() || !m_endPoint->isValid()) {
    throw std::runtime_error("Line constructor: Input Point objects are not valid");
  }

  // Calculate canonical length
  try {
    calculateCanonicalLength();
  } catch (const std::exception &e) {
    std::cerr << "Line Constructor (ID variant): Exception calculating "
                 "canonical length: "
              << e.what() << ". Setting to default." << std::endl;
    m_canonicalLength = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
  }

  // Initialize SFML shape
  initializeSFMLShape();

  // Build geometry and register connections
  try {
    update();  // This calls updateCGALLine and updateSFMLShape

    // CRITICAL: Register this line with its endpoints
    // registerWithEndpoints();

    std::cout << "Line/Segment created with ID: " << m_id << " between P" << m_startPoint->getID()
              << " and P" << m_endPoint->getID() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Line Constructor (ID variant): Exception during initialization: " << e.what()
              << std::endl;
    throw;
  }
}
void Line::prepareForDestruction() {
    try {
        std::cout << "Line::prepareForDestruction: ENTERED for Line " << this << std::endl;
        
        // Only do cleanup that requires shared_from_this() BEFORE destruction starts
        auto self = shared_from_this();

        // CRITICAL: Clear constraint relationships safely
        
        // 1. Remove this line from any reference line's observers
        if (!m_constraintRefObject.expired()) {
            if (auto refObj = m_constraintRefObject.lock()) {
                // Only Lines support generic observer list for now, we'll cast to check
                if (auto refLine = std::dynamic_pointer_cast<Line>(refObj)) {
                    std::cout << "Removing self from reference line's observers" << std::endl;
                    refLine->removeConstraintObserver(self);
                }
            }
        }
        
        // 2. Notify all observers that this line is being destroyed
        if (!m_constraintObservers.empty()) {
            std::cout << "Notifying " << m_constraintObservers.size() << " constraint observers of destruction" << std::endl;
            
            // Create a copy of the observers list to avoid modification during iteration
            auto observersCopy = m_constraintObservers;
            
            for (const auto &observer_weak : observersCopy) {
                if (auto observer = observer_weak.lock()) {
                    try {
                        // Clear the observer's reference to this line
                        observer->clearConstraintReference(self);
                    } catch (const std::exception &e) {
                        std::cerr << "Error notifying observer of destruction: " << e.what() << std::endl;
                    }
                }
            }
        }
        
        // 3. Clear our own relationships
        m_constraintRefObject.reset();
        m_constraintRefEdgeIndex = -1;
        m_constraintObservers.clear();
        
        // 4. Reset constraint flags
        m_isParallelLine = false;
        m_isPerpendicularLine = false;
        m_constraintDirection = Vector_2(0, 0);
        
        std::cout << "Line::prepareForDestruction: COMPLETED for Line " << this << std::endl;

    } catch (const std::bad_weak_ptr &e) {
        // Object already being destroyed, skip cleanup
        std::cout << "Line::prepareForDestruction: Object already being destroyed" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "prepareForDestruction error: " << e.what() << std::endl;
    }
}
// ------dtor------
Line::~Line() {
  if (Constants::LIFECYCLE) {
  std::cout << "Line::~Line: ENTERED for Line " << this << std::endl;
  }
  try {
    // Clear hosted ObjectPoints first
    if (!m_hostedObjectPoints.empty()) {
      std::cout << "Line destructor: Clearing " << m_hostedObjectPoints.size()
                << " hosted ObjectPoints" << std::endl;
      m_hostedObjectPoints.clear();
    }

    // Clear constraint relationships WITHOUT calling shared_from_this()
    // All cleanup requiring shared_from_this() should have been done in prepareForDestruction()
    m_constraintRefObject.reset();
    m_constraintRefEdgeIndex = -1;
    m_constraintObservers.clear();
    
    // Reset constraint state
    m_isParallelLine = false;
    m_isPerpendicularLine = false;
    m_constraintDirection = Vector_2(0, 0);

    std::cout << "Line::~Line: COMPLETED for Line " << this << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Line destructor: Exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Line destructor: Unknown exception" << std::endl;
  }
}

void Line::setDeferSFMLUpdates(bool defer) {
  m_deferSFMLUpdates = defer;
  if (!defer && m_pendingSFMLUpdate) {
    updateSFMLShape();
    m_pendingSFMLUpdate = false;
  }
}

void Line::forceSFMLUpdate() {
  m_deferSFMLUpdates = false;
  updateSFMLShape();
  m_pendingSFMLUpdate = false;
}
/* Line::~Line() {
  // Make a copy of the hosted points to avoid iterator invalidation
  std::vector<ObjectPoint *> pointsCopy = m_hostedObjectPoints;

  // Clear the list first
  m_hostedObjectPoints.clear();

  // Notify each object point that their host is going away
  for (auto *point : pointsCopy) {
    if (point) {
      try {
        point->clearHost();
      } catch (const std::exception &e) {
        std::cerr << "Exception in Line destructor while clearing host: "
                  << e.what() << std::endl;
      }
    }
  }
} */
/* Direction_2 Line::getDirection() const {
    if (!m_startPointObject || !m_endPointObject ||
!m_startPointObject->isValid() || !m_endPointObject->isValid()) { std::cerr <<
"Warning: Line::getDirection() called on line with invalid points." <<
std::endl; return Direction_2(1.0, 0.0); // Default horizontal direction
    }
    // Construct Direction_2 from the vector between the two points
    return Direction_2(m_endPointObject->getCGALPosition() -
m_startPointObject->getCGALPosition());
} */
Line_2 Line::getCGALLine() const {
  if (!m_startPoint || !m_endPoint) {
    throw std::runtime_error("Line endpoints are null");
  }

  Point_2 p1_pos = m_startPoint->getCGALPosition();
  Point_2 p2_pos = m_endPoint->getCGALPosition();

  if (p1_pos == p2_pos) {
    throw std::runtime_error("Line endpoints are coincident");
  }

  return Line_2(p1_pos, p2_pos);
}
//
void Line::calculateCanonicalLength() {
  Point_2 cgal_p1 = m_startPoint->getCGALPosition();
  Point_2 cgal_p2 = m_endPoint->getCGALPosition();

  if (cgal_p1 == cgal_p2) {
    m_canonicalLength = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
    if (Constants::DEBUG_LINE_CREATION) {
      std::cout << "Line Constructor: Endpoints coincident, using default length" << std::endl;
    }
    return;
  }

  Kernel::FT sq_len = CGAL::squared_distance(cgal_p1, cgal_p2);
  double d_sq_len = CGAL::to_double(sq_len);
  if (d_sq_len < 0) {
    d_sq_len = 0;  // Safety for sqrt
  }

  double d_len = std::sqrt(d_sq_len);
  m_canonicalLength = Kernel::FT(d_len);

  // If calculated length is too small, use default
  if (CGAL::abs(m_canonicalLength) < Kernel::FT(Constants::EPSILON_LENGTH_CONSTRUCTION)) {
    if (Constants::DEBUG_LINE_CREATION) {
      std::cout << "Line Constructor: Calculated length too small ("
                << CGAL::to_double(m_canonicalLength)
                << "), using default: " << Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH << std::endl;
    }
    m_canonicalLength = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
  }
}

void Line::initializeSFMLShape() {
  m_sfmlShape.setPrimitiveType(sf::Lines);
  m_sfmlShape.resize(2);
}

void Line::registerWithEndpoints() {
  std::cout << "Line::registerWithEndpoints: ENTERED for Line " << getID() << std::endl;

  if (m_startPoint) {
    try {
      std::cout << "Registering line with start point" << std::endl;
      m_startPoint->addConnectedLine(shared_from_this());  // ✅ UNCOMMENT THIS!
      std::cout << "Successfully registered line " << getID() << " with start point "
                << m_startPoint->getID() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error registering with start point: " << e.what() << std::endl;
    }
  }

  if (m_endPoint) {
    try {
      std::cout << "Registering line with end point" << std::endl;
      m_endPoint->addConnectedLine(shared_from_this());  // ✅ UNCOMMENT THIS!
      std::cout << "Successfully registered line " << getID() << " with end point "
                << m_endPoint->getID() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error registering with end point: " << e.what() << std::endl;
    }
  }

  std::cout << "Line::registerWithEndpoints: COMPLETED for Line " << getID() << std::endl;
}

void Line::setExternallyMovedEndpoint(Point *movedPoint) {
  m_externallyMovedEndpoint = movedPoint;

  // If this line has constraint observers, notify them immediately
  if (!m_constraintObservers.empty() && !m_inConstraint) {
    if (Constants::DEBUG_CONSTRAINTS) {
      if (Constants::DEBUG_CONSTRAINTS) {
        std::cout << "Line " << getID() << " notified that Point " << movedPoint
                  << " was externally moved" << std::endl;
      }
      notifyConstraintObserversOfChange();
    }
  }
}

/* void Line::updateCGALLine() {
  if (!m_startPoint || !m_endPoint) {
    // std::cerr << "Line::updateCGALLine: Start or end point is null." <<
    // std::endl; // Commented out to reduce noise
    m_cgalLine = Line_2(Point_2(0, 0), Point_2(1, 0)); // Default invalid line
    return;
  }

  try {
    Point_2 p1 = m_startPoint->getCGALPosition();
    Point_2 p2 = m_endPoint->getCGALPosition();

    if (p1 == p2) {
      // std::cerr << "Line::updateCGALLine: Points are coincident. Line is
      // degenerate." << std::endl; // Commented out Create a degenerate line
      // (or handle as error) For now, let's make it a line through p1 with an
      // arbitrary direction if p1 is finite
      if (CGAL::is_finite(p1.x()) && CGAL::is_finite(p1.y())) {
        m_cgalLine = Line_2(p1, Point_2(p1.x() + 1, p1.y()));
      } else {
        m_cgalLine =
            Line_2(Point_2(0, 0), Point_2(1, 0)); // Default if p1 is not
finite
      }
      return;
    }

    // Construct the line directly from two points
    m_cgalLine = Line_2(p1, p2);

    // Normalize the coefficients to keep them manageable
    // ax + by + c = 0
    Kernel::FT a = m_cgalLine.a();
    Kernel::FT b = m_cgalLine.b();
    Kernel::FT c = m_cgalLine.c();

    // std::cout << "Line::updateCGALLine: Initial coeffs: a=" << a << ", b="
    // << b << ", c=" << c << std::endl; // Commented out

    // Attempt to normalize by dividing by sqrt(a^2 + b^2)
    // This helps keep coefficients in a reasonable range and is standard.
    Kernel::FT det = CGAL::square(a) + CGAL::square(b);
    if (CGAL::sign(det) > 0) {
      // Use CGAL::sqrt_extension for exact kernel compatibility
      double sqrt_det_approx = std::sqrt(CGAL::to_double(det));
      Kernel::FT norm_factor(1.0 / sqrt_det_approx);

      // Normalize using the approximate factor
      Kernel::FT norm_a = a * norm_factor;
      Kernel::FT norm_b = b * norm_factor;
      Kernel::FT norm_c = c * norm_factor;

      // Check if normalized coefficients are finite. Epeck can handle large
      // numbers, but extreme values or non-finite results from division are
      // problematic.
      if (CGAL::is_finite(norm_a) && CGAL::is_finite(norm_b) &&
          CGAL::is_finite(norm_c)) {
        m_cgalLine = Line_2(norm_a, norm_b, norm_c);
        // std::cout << "Line::updateCGALLine: Normalized coeffs: a=" <<
        // norm_a << ", b=" << norm_b << ", c=" << norm_c << std::endl; //
        // Commented out
      } else {
        // std::cerr << "Line::updateCGALLine: WARNING - Normalization
        // resulted in non-finite coefficients. Using original." << std::endl;
        // // Commented out Fallback to original if normalization fails
        m_cgalLine = Line_2(a, b, c);
      }
    } else {
      // std::cerr << "Line::updateCGALLine: WARNING - sqrt(a^2+b^2) is zero,
      // cannot normalize. Using original." << std::endl; // Commented out
      m_cgalLine = Line_2(a, b, c); // Should not happen if p1 != p2
    }

    // Additional check for very large coefficients even after potential
    // normalization
    const double MAX_COEFF_MAGNITUDE = 1e12; // Example threshold
    if (CGAL::to_double(CGAL::abs(m_cgalLine.a())) > MAX_COEFF_MAGNITUDE ||
        CGAL::to_double(CGAL::abs(m_cgalLine.b())) > MAX_COEFF_MAGNITUDE ||
        CGAL::to_double(CGAL::abs(m_cgalLine.c())) > MAX_COEFF_MAGNITUDE) {
      // std::cerr << "Line::updateCGALLine: WARNING - Line has very large
      // coefficients after update: a="
      //   << m_cgalLine.a() << ", b=" << m_cgalLine.b() << ", c=" <<
      //   m_cgalLine.c()
      //   << std::endl; // Commented out
    }

  } catch (const CGAL::Precondition_exception &e) {
    std::cerr << "CGAL Precondition_exception in Line::updateCGALLine: "
              << e.what() << std::endl;
    // Handle error, e.g., by setting a default/invalid line state
    m_cgalLine = Line_2(Point_2(0, 0), Point_2(1, 0)); // Default invalid line
  } catch (const std::exception &e) {
    std::cerr << "Exception in Line::updateCGALLine: " << e.what() <<
std::endl; m_cgalLine = Line_2(Point_2(0, 0), Point_2(1, 0)); // Default
invalid line
  }
  updateSFMLShape();
} */

// In Line.cpp, Line::updateCGALLine() method (around line 535)
void Line::updateCGALLine() {
  try {
    // Early validation BEFORE any CGAL operations
    if (!m_startPoint || !m_endPoint || !m_startPoint->isValid() || !m_endPoint->isValid()) {
      std::cerr << "Line::updateCGALLine: Invalid endpoints" << std::endl;
      return;
    }

    // Get positions but avoid triggering exact arithmetic yet
    Point_2 start_pos, end_pos;
    try {
      start_pos = m_startPoint->getCGALPosition();
      end_pos = m_endPoint->getCGALPosition();
    } catch (const std::exception &e) {
      std::cerr << "Line::updateCGALLine: Error getting endpoint positions: " << e.what()
                << std::endl;
      return;
    }

    // Pre-validate using double conversion to avoid exact arithmetic crashes
    double start_x, start_y, end_x, end_y;
    try {
      start_x = CGAL::to_double(start_pos.x());
      start_y = CGAL::to_double(start_pos.y());
      end_x = CGAL::to_double(end_pos.x());
      end_y = CGAL::to_double(end_pos.y());
    } catch (const std::exception &e) {
      std::cerr << "Line::updateCGALLine: Cannot convert coordinates to double: " << e.what()
                << std::endl;
      return;
    }

    // Validate converted coordinates
    if (!std::isfinite(start_x) || !std::isfinite(start_y) || !std::isfinite(end_x) ||
        !std::isfinite(end_y)) {
      std::cerr << "Line::updateCGALLine: Non-finite double coordinates" << std::endl;
      return;
    }

    // Check for degenerate line
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double dist_sq = dx * dx + dy * dy;

    if (!std::isfinite(dist_sq) || dist_sq < 1e-20) {
      std::cerr << "Line::updateCGALLine: Degenerate line (distance² = " << dist_sq << ")"
                << std::endl;
      return;
    }

    // Only now that we've validated everything, create the CGAL line
    m_cgalLine = Line_2(start_pos, end_pos);
    m_canonicalLength = Kernel::FT(std::sqrt(dist_sq));

  } catch (const std::exception &e) {
    std::cerr << "Line::updateCGALLine: Exception: " << e.what() << std::endl;
  }
}

// Helper function to convert CGAL Point_2 to SFML Vector2f
sf::Vector2f cgalToSFML(const Point_2 &cgalPoint) {
  double x_val, y_val;
  try {
    x_val = CGAL::to_double(cgalPoint.x());
    y_val = CGAL::to_double(cgalPoint.y());
  } catch (const CGAL::Uncertain_conversion_exception &e) {
    std::cerr << "cgalToSFML: Uncertain conversion for point. what(): " << e.what() << std::endl;
    // Consider logging exact values if helpful for debugging:
    // try {
    //   std::cerr << "Exact X: " << CGAL::exact(cgalPoint.x()) << " Exact Y:
    //   "
    //   << CGAL::exact(cgalPoint.y()) << std::endl;
    // } catch (...) {} // CGAL::exact might also throw
    throw;  // Re-throw to signal error to caller
  }

  if (std::isnan(x_val) || std::isinf(x_val) || std::isnan(y_val) || std::isinf(y_val)) {
    std::cerr << "cgalToSFML: Resulting double is NaN/Inf. Original CGAL point "
                 "might be problematic."
              << std::endl;
    throw std::runtime_error("NaN/Inf encountered in cgalToSFML after CGAL::to_double");
  }
  return sf::Vector2f(static_cast<float>(x_val), static_cast<float>(y_val));
}

// Helper function to convert SFML Vector2f to CGAL Point_2
Point_2 sfmlToCGAL(const sf::Vector2f &sfmlPoint) {
  if (std::isnan(sfmlPoint.x) || std::isinf(sfmlPoint.x) || std::isnan(sfmlPoint.y) ||
      std::isinf(sfmlPoint.y)) {
    std::cerr << "Error: sfmlToCGAL called with NaN/Inf coordinates: (" << sfmlPoint.x << ", "
              << sfmlPoint.y << ")" << std::endl;
    throw std::runtime_error("Invalid coordinates (NaN/Inf) in sfmlToCGAL");
  }
  return Point_2(sfmlPoint.x, sfmlPoint.y);
}

// Define a threshold for squared length. E.g., (0.001 units)^2 = 1e-6.
// Points closer than 0.001 units are considered "too close" to define a
// robust line.
const Kernel::FT MIN_SQ_LENGTH_THRESHOLD(
    1e-6);  // Adjust as needed based on typical coordinate scale

// Constructor for Line with CGAL points
Line::Line(const Point_2 &start, const Point_2 &end, bool isSegment, const sf::Color &color)
    : GeometricObject(isSegment ? ObjectType::LineSegment : ObjectType::Line, color),
      m_lineType(LineType::Infinite),
      m_startPoint(nullptr),  // Initialize before other members that might
      // depend on it or for order
      m_endPoint(nullptr),
      m_isSegment(isSegment),
      m_color(color) {

  if (start == end) {
    throw std::invalid_argument("Line (CGAL constructor) endpoints are coincident (exact).");
  }
  Vector_2 dir = end - start;
  if (CGAL::compare(dir.squared_length(), MIN_SQ_LENGTH_THRESHOLD) == CGAL::SMALLER) {
    throw std::invalid_argument("Line (CGAL constructor) endpoints are too close.");
  }

  m_cgalLine = Line_2(start, end);
  // For this constructor, m_sfmlShape cannot be updated from
  // m_startPoint/m_endPoint It should be updated if these points are later
  // associated with actual Point objects or if SFML representation is derived
  // directly from m_cgalLine's points (if needed). For now, SFML shape will
  // be default until Point objects are linked or SFML shape is explicitly
  // set.
  m_sfmlShape.setPrimitiveType(sf::Lines);
  m_sfmlShape.resize(2);
  // If you have a way to set m_sfmlShape from 'start' and 'end' CGAL points:
  // m_sfmlShape[0].position = cgalToSFML(start);
  // m_sfmlShape[1].position = cgalToSFML(end);
  // m_sfmlShape[0].color = m_color;
  // m_sfmlShape[1].color = m_color;
}
void Line::setTemporaryPreviewPoints(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2) {
  m_tempPreviewP1 = p1;
  m_tempPreviewP2 = p2;
  // The raw m_startPoint and m_endPoint are already set by the constructor
  // that takes Point* when the preview Line is created.
  // This method just ensures the shared_ptrs keep the temporary points alive.
  m_startPoint = p1;
  m_endPoint = p2;
  // For preview, avoid full constraint maintenance; update only SFML shape
  updateSFMLShape();
}
// --- GeometricObject Overrides ---
void Line::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  try {
    if ((!m_visible && !forceVisible) || (!m_startPoint || !m_startPoint->isValid()) ||
        (m_endPoint && !m_endPoint->isValid())) {
      return;
    }

    // --- COLOR & STATE SETUP ---
    sf::Color drawColor = m_color;
    if (m_selected) {
      drawColor = Constants::SELECTION_COLOR;
    } else if (m_hovered) {
      drawColor = Constants::HOVER_COLOR;
    }
    // Ghost mode transparency
    if (!m_visible && forceVisible) {
      drawColor.a = 50;
    }

    float basePixelThickness = Constants::LINE_THICKNESS_DEFAULT;
    if (m_selected) basePixelThickness = 4.0f;
    else if (m_hovered) basePixelThickness = 3.0f;
    
    // Convert thickness to world units
    float worldThickness = basePixelThickness * scale;

    // --- COORDINATE CALCULATION ---
    const Point_2 startCgal = m_startPoint->getCGALPosition();
    const Point_2 endCgal = m_endPoint->getCGALPosition();
    sf::Vector2f p1(static_cast<float>(CGAL::to_double(startCgal.x())),
                    static_cast<float>(CGAL::to_double(startCgal.y())));
    sf::Vector2f p2(static_cast<float>(CGAL::to_double(endCgal.x())),
                    static_cast<float>(CGAL::to_double(endCgal.y())));

    sf::Vector2f dir = p2 - p1;
    float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (len < 1e-9f) return; // Degenerate
    sf::Vector2f unitDir = dir / len;

    // --- TYPE-SPECIFIC DRAWING LOGIC ---
    sf::Vector2f drawStart = p1;
    sf::Vector2f drawEnd = p2;

    ObjectType type = getType(); // Line, LineSegment, Ray, Vector

    if (m_lineType == LineType::Infinite || type == ObjectType::Line) {
        // Infinite Line: Extend both ways
        sf::View currentView = window.getView();
        sf::Vector2f viewSize = currentView.getSize();
        float viewDiagonal = std::sqrt(viewSize.x * viewSize.x + viewSize.y * viewSize.y);
        float extension = viewDiagonal * 2.0f;
        drawStart = p1 - unitDir * extension;
        drawEnd = p2 + unitDir * extension;
    } 
    else if (m_lineType == LineType::Ray || type == ObjectType::Ray) {
        // Ray: Extend only from Start towards End
        sf::View currentView = window.getView();
        sf::Vector2f viewSize = currentView.getSize();
        float viewDiagonal = std::sqrt(viewSize.x * viewSize.x + viewSize.y * viewSize.y);
        float extension = viewDiagonal * 2.0f;
        drawStart = p1; // Start is fixed
        drawEnd = p2 + unitDir * extension; // Extend past P2
    }
    // Else: Segment or Vector (draw directly from p1 to p2)

    // --- DRAW MAIN LINE BODY ---
    sf::Vector2f normal(-unitDir.y, unitDir.x);
    sf::Vector2f offset = normal * (worldThickness * 0.5f);

    sf::VertexArray quad(sf::Quads, 4);
    quad[0].position = drawStart + offset;
    quad[1].position = drawEnd + offset;
    quad[2].position = drawEnd - offset;
    quad[3].position = drawStart - offset;
    
    for(int i=0; i<4; ++i) quad[i].color = drawColor;
    window.draw(quad);

    // --- DRAW VECTOR ARROWHEAD ---
    if (type == ObjectType::Vector) {
        float arrowSize = 15.0f * scale; // Adjust size as needed
        sf::Vector2f arrowTip = p2;
        sf::Vector2f arrowBase = arrowTip - unitDir * arrowSize;
        sf::Vector2f arrowLeft = arrowBase + normal * (arrowSize * 0.35f);
        sf::Vector2f arrowRight = arrowBase - normal * (arrowSize * 0.35f);

        sf::VertexArray arrow(sf::Triangles, 3);
        arrow[0].position = arrowTip;
        arrow[1].position = arrowLeft;
        arrow[2].position = arrowRight;
        
        for(int i=0; i<3; ++i) arrow[i].color = drawColor;
        window.draw(arrow);
    }

  } catch (const std::exception &e) {
    std::cerr << "Exception in Line::draw: " << e.what() << std::endl;
  }
}
/* void Line::draw(sf::RenderWindow &window) const {
  // Get the current view
  sf::View currentView = window.getView();

  // Calculate zoom factor
  float viewHeight = currentView.getSize().y;
  float zoomFactor = viewHeight /
static_cast<float>(Constants::WINDOW_HEIGHT);

  // Set colors based on state
  sf::Color drawColor = m_color;
  if (m_selected) {
    drawColor = Constants::SELECTION_COLOR;
  } else if (m_hovered) {
    drawColor = Constants::HOVER_COLOR;
  }

  if (m_isSegment) {
    // For line segments, draw just the segment between endpoints
    sf::VertexArray drawShape = m_sfmlShape;

    // Apply color to both vertices
    drawShape[0].color = drawColor;
    drawShape[1].color = drawColor;

    // Draw the line segment
    window.draw(drawShape);
  } else {
    // For infinite lines, calculate very long extension beyond visible area

    // Get screen dimensions in world coordinates
    sf::Vector2f viewSize = currentView.getSize();
    sf::Vector2f viewCenter = currentView.getCenter();

    // Calculate the diagonal length of the view (to ensure lines extend
beyond
    // corners)
    float diagonalLength =
        std::sqrt(viewSize.x * viewSize.x + viewSize.y * viewSize.y);
    float extensionLength =
        diagonalLength * 2.0f; // Double the diagonal for safety

    if (!m_startPoint || !m_endPoint)
      return;

    sf::Vector2f startPos = m_startPoint->getSFMLPosition();
    sf::Vector2f endPos = m_endPoint->getSFMLPosition();

    // Calculate direction vector
    sf::Vector2f direction = endPos - startPos;
    float length =
        std::sqrt(direction.x * direction.x + direction.y * direction.y);

    if (length < 0.0001f) {
      // Handle degenerate case
      window.draw(m_sfmlShape);
      return;
    }

    // Normalize
    direction /= length;

    // Extend in both directions
    sf::Vector2f extendedStart = startPos - direction * extensionLength;
    sf::Vector2f extendedEnd = endPos + direction * extensionLength;

    // Create an extended line
    sf::VertexArray extendedLine(sf::Lines, 2);
    extendedLine[0].position = extendedStart;
    extendedLine[1].position = extendedEnd;
    extendedLine[0].color = drawColor;
    extendedLine[1].color = drawColor;

    // Draw the extended line
    window.draw(extendedLine);

    // Optionally, draw the segment between endpoints in a different color or
    // thickness to distinguish the defining segment
    sf::VertexArray definingSeg(sf::Lines, 2);
    definingSeg[0].position = startPos;
    definingSeg[1].position = endPos;

    // Use slightly bolder/brighter color for the defining segment
    sf::Color definingColor = drawColor;
    if (!m_selected && !m_hovered) {
      definingColor.a = 255; // Make the core segment fully opaque
    }

    definingSeg[0].color = definingColor;
    definingSeg[1].color = definingColor;

    // Draw the defining segment
    window.draw(definingSeg);
  }
}
 */
bool Line::contains(const sf::Vector2f &worldPos_sfml, float tolerance) const {
  if (!m_startPoint || !m_endPoint) {
    // std::cerr << "Line::contains: Null endpoints." << std::endl; //
    // Optional debug
    return false;
  }

  sf::Vector2f p1_sfml, p2_sfml;
  try {
    p1_sfml = m_startPoint->getSFMLPosition();
    p2_sfml = m_endPoint->getSFMLPosition();
  } catch (const std::exception &e) {
    std::cerr << "Line::contains: Exception getting SFML endpoint positions: " << e.what()
              << std::endl;
    return false;
  }

  float dx_line = p2_sfml.x - p1_sfml.x;
  float dy_line = p2_sfml.y - p1_sfml.y;
  float lineSqLength = dx_line * dx_line + dy_line * dy_line;

  // Case 1: Line is effectively a point (degenerate)
  if (lineSqLength < (Constants::EPSILON * Constants::EPSILON)) {
    float distSqToP1 = (worldPos_sfml.x - p1_sfml.x) * (worldPos_sfml.x - p1_sfml.x) +
                       (worldPos_sfml.y - p1_sfml.y) * (worldPos_sfml.y - p1_sfml.y);
    return distSqToP1 <= (tolerance * tolerance);
  }

  if (m_isSegment) {
    // Parameter t for projection of worldPos_sfml onto the line defined by
    // p1_sfml p2_sfml t = dot(P1_World, P1_P2) / |P1_P2|^2
    float t_numerator =
        (worldPos_sfml.x - p1_sfml.x) * dx_line + (worldPos_sfml.y - p1_sfml.y) * dy_line;
    float t = t_numerator / lineSqLength;

    float distSqToClosest;  // Squared distance to the closest part of the
    // segment

    if (t < 0.0f) {  // Closest point on segment is p1_sfml
      distSqToClosest = (worldPos_sfml.x - p1_sfml.x) * (worldPos_sfml.x - p1_sfml.x) +
                        (worldPos_sfml.y - p1_sfml.y) * (worldPos_sfml.y - p1_sfml.y);
    } else if (t > 1.0f) {  // Closest point on segment is p2_sfml
      distSqToClosest = (worldPos_sfml.x - p2_sfml.x) * (worldPos_sfml.x - p2_sfml.x) +
                        (worldPos_sfml.y - p2_sfml.y) * (worldPos_sfml.y - p2_sfml.y);
    } else {  // Closest point is on the segment body (perpendicular
      // projection)
      // Distance from point (x0, y0) to line Ax + By + C = 0 is |Ax0 + By0 +
      // C| / sqrt(A^2 + B^2) A = p2_sfml.y - p1_sfml.y = dy_line B =
      // p1_sfml.x - p2_sfml.x = -dx_line C = -(A*p1_sfml.x + B*p1_sfml.y) =
      // dx_line*p1_sfml.y - dy_line*p1_sfml.x C can also be p2_sfml.x *
      // p1_sfml.y - p1_sfml.x * p2_sfml.y
      float dist_num_abs = std::abs(dy_line * worldPos_sfml.x - dx_line * worldPos_sfml.y +
                                    (p2_sfml.x * p1_sfml.y - p1_sfml.x * p2_sfml.y));
      // distSqToClosest = (dist_num_abs * dist_num_abs) / lineSqLength; //
      // This is distance squared No, the above is wrong for distSq. It should
      // be (dist_num_abs / sqrt(lineSqLength))^2
      return (dist_num_abs / std::sqrt(lineSqLength)) <= tolerance;
    }
    return distSqToClosest <= (tolerance * tolerance);

  } else {  // Infinite Line
    // Perpendicular distance to the infinite line
    float dist_num_abs = std::abs(dy_line * worldPos_sfml.x - dx_line * worldPos_sfml.y +
                                  (p2_sfml.x * p1_sfml.y - p1_sfml.x * p2_sfml.y));
    float dist_den = std::sqrt(lineSqLength);
    // if (dist_den < Constants::EPSILON) return false; // Should be caught by
    // degenerate check
    return (dist_num_abs / dist_den) <= tolerance;
  }
}

void Line::setSelected(bool selected) {
  // ONLY update if the state actually changed
  if (m_selected != selected) {
    m_selected = selected;
    if (m_startPoint && m_endPoint) {
      try {
        updateSFMLShape();  // Only update when state changes
      } catch (const std::exception &e) {
        std::cerr << "Exception in Line::setSelected: " << e.what() << std::endl;
      }
    }
  }
}

void Line::setHovered(bool hoveredStatus) {
  if (m_hovered != hoveredStatus) {
    m_hovered = hoveredStatus;
    updateSFMLShape();  // Only update when state changes
  }
}

sf::FloatRect Line::getGlobalBounds() const {
  if (!m_startPoint || !m_endPoint) {
    return sf::FloatRect(0, 0, 0,
                         0);  // Return an empty rect if points are invalid
  }

  sf::Vector2f startPos_sfml, endPos_sfml;
  try {
    startPos_sfml = m_startPoint->getSFMLPosition();
    endPos_sfml = m_endPoint->getSFMLPosition();
  } catch (const std::exception &e) {
    std::cerr << "Line::getGlobalBounds: Error getting SFML positions from points: " << e.what()
              << std::endl;
    return sf::FloatRect(0, 0, 0, 0);  // Return empty on error
  }

  float left = std::min(startPos_sfml.x, endPos_sfml.x);
  float top = std::min(startPos_sfml.y, endPos_sfml.y);
  float width = std::abs(startPos_sfml.x - endPos_sfml.x);
  float height = std::abs(startPos_sfml.y - endPos_sfml.y);

  float padding = Constants::LINE_INTERACTION_RADIUS;

  left -= padding;
  top -= padding;
  width += (2 * padding);
  height += (2 * padding);

  // Ensure minimum dimensions if the line is extremely short or axis-aligned
  // AFTER padding This logic is slightly different from your original to
  // better handle centering when minimums are applied.
  if (std::abs(startPos_sfml.x - endPos_sfml.x) < Constants::EPSILON) {  // Vertical line
    if (width < Constants::LINE_INTERACTION_RADIUS * 2) {  // Check if padding wasn't enough
      left = startPos_sfml.x - Constants::LINE_INTERACTION_RADIUS;
      width = Constants::LINE_INTERACTION_RADIUS * 2;
    }
  }
  if (std::abs(startPos_sfml.y - endPos_sfml.y) < Constants::EPSILON) {  // Horizontal line
    if (height < Constants::LINE_INTERACTION_RADIUS * 2) {  // Check if padding wasn't enough
      top = startPos_sfml.y - Constants::LINE_INTERACTION_RADIUS;
      height = Constants::LINE_INTERACTION_RADIUS * 2;
    }
  }
  // For very short (point-like) lines, ensure both width and height have
  // minimums
  if (width < Constants::LINE_INTERACTION_RADIUS * 2 &&
      height < Constants::LINE_INTERACTION_RADIUS * 2 &&
      (std::abs(startPos_sfml.x - endPos_sfml.x) < Constants::EPSILON &&
       std::abs(startPos_sfml.y - endPos_sfml.y) < Constants::EPSILON)) {
    left = startPos_sfml.x - Constants::LINE_INTERACTION_RADIUS;
    top = startPos_sfml.y - Constants::LINE_INTERACTION_RADIUS;
    width = Constants::LINE_INTERACTION_RADIUS * 2;
    height = Constants::LINE_INTERACTION_RADIUS * 2;
  }

  return sf::FloatRect(left, top, width, height);
}
void Line::setIsUnderDirectManipulation(bool isManipulated) {
  m_isUnderDirectManipulation = isManipulated;
  if (Constants::DEBUG_DRAGGING) {  // Or your preferred debug flag
    std::cout << "Line " << this << ": m_isUnderDirectManipulation set to "
              << (isManipulated ? "true" : "false") << std::endl;
  }
}

void Line::update() {
  if (m_isUpdatingInternally) {
    if (Constants::DEBUG_CONSTRAINTS) {
      std::cout << "Line::update: m_isUpdatingInternally is true, returning "
                   "early. Line: "
                << this << std::endl;
    }
    return;
  }
  m_isUpdatingInternally = true;

  if (Constants::DEBUG_CONSTRAINTS) {
    std::cout << "Line::update: Entered. Line: " << this
              << ", m_isUnderDirectManipulation: " << m_isUnderDirectManipulation << std::endl;
  }

  try {
    updateCGALLine();

    if ((m_isParallelLine || m_isPerpendicularLine) && !m_isUnderDirectManipulation) {
      bool has_ref_obj = false;
      if (auto ref_sp = m_constraintRefObject.lock()) {
         if (ref_sp.get() != this) { // Valid non-self reference
            has_ref_obj = true;
         }
      }

      // Condition: EITHER a valid reference OR a valid axis constraint direction
      if (has_ref_obj || (m_constraintDirection != Vector_2(0, 0) && m_constraintRefObject.expired())) {
        if (Constants::DEBUG_CONSTRAINTS) {
          std::string refId = "None/Axis";
          if (auto ref = m_constraintRefObject.lock()) {
             refId = std::to_string(ref->getID());
          }
          std::cout << "Line::update: Calling maintainConstraints for Line " << getID() << " (Ref: "
                    << refId
                    << ")" << std::endl;
        }
        
        bool changed = maintainConstraints(); // Capture result
        
        if (changed) {
            updateCGALLine();  
            updateSFMLShape(); // Update visual only if changed
            updateHostedPoints();
            notifyConstraintObserversOfChange(); // Break loop if false
        }
      } else {
         // Standard update path for non-constrained or failed constraint lines
         updateSFMLShape();
         updateHostedPoints();
         notifyConstraintObserversOfChange();
      }
    } else {
         // Standard update path
         updateSFMLShape();
         updateHostedPoints();
         notifyConstraintObserversOfChange();
    }

  } catch (const std::exception &e) {
    std::cerr << "Line::update: Exception during update cycle: " << e.what() << " Line: " << getID()
              << " (" << this << ")" << std::endl;
  } catch (...) {
    std::cerr << "Line::update: Unknown exception during update cycle. Line: " << getID() << " ("
              << this << ")" << std::endl;
  }

  m_isUpdatingInternally = false;
  m_externallyMovedEndpoint = nullptr;
  if (Constants::DEBUG_CONSTRAINTS) {
    std::cout << "Line::update: Exiting. Line: " << this << std::endl;
  }
}
void Line::forceConstraintUpdate() {
  if ((m_isParallelLine || m_isPerpendicularLine) && !m_inConstraint) {
    if (Constants::DEBUG_CONSTRAINTS) {
      std::cout << "Line::forceConstraintUpdate: Forcing immediate constraint "
                   "update for Line "
                << getID() << std::endl;
    }

    // Temporarily disable the updating flag to force constraint processing
    bool wasUpdating = m_isUpdatingInternally;
    m_isUpdatingInternally = false;

    // CIRCUIT BREAKER: Only update visuals if geometry actually changed
    bool geometryChanged = maintainConstraints();
    if (geometryChanged) {
      updateCGALLine();
      updateSFMLShape();
      updateHostedPoints();
    } else if (Constants::DEBUG_CONSTRAINTS) {
      std::cout << "Line::forceConstraintUpdate: NO CHANGE - skipping visual updates for Line "
                << getID() << std::endl;
    }

    m_isUpdatingInternally = wasUpdating;
  }
}
// Add implementation for the notifyConstraintObserversOfChange method
void Line::notifyConstraintObserversOfChange() {
  if (Constants::DEBUG_CONSTRAINTS) {
    std::cout << "Line::notifyConstraintObserversOfChange: Notifying "
              << m_constraintObservers.size() << " observers from Line " << getID() << std::endl;
  }

  for (const auto &observer_weak : m_constraintObservers) {
    if (auto observer = observer_weak.lock()) {
      try {
        // Force immediate constraint update instead of queuing a regular
        // update
        observer->forceConstraintUpdate();
      } catch (const std::exception &e) {
        std::cerr << "Line::notifyConstraintObserversOfChange: Exception "
                     "notifying observer: "
                  << e.what() << std::endl;
      }
    }
  }
}

// Simplified Line::translate
void Line::translate(const Vector_2 &offset) {
  if (!CGAL::is_finite(offset.x()) || !CGAL::is_finite(offset.y())) {
    std::cerr << "Error: Line::translate attempting to translate with "
                 "non-finite offset. Line: "
              << this << std::endl;
    // Add debug_cgal_point for offset if you have a helper for Vector_2 or
    // print components
    return;
  }
  if (!m_startPoint || !m_endPoint) {
    std::cerr << "Error: Line::translate called on line with null "
                 "endpoint(s). Line: "
              << this << std::endl;
    return;
  }

  // Prevent re-entrant full updates from point setters for *this* line.
  bool was_already_updating_internally = m_isUpdatingInternally;
  m_isUpdatingInternally = true;

  try {
    if (m_startPoint) {
      Point_2 startPos = m_startPoint->getCGALPosition();
      Point_2 newStartPos(startPos.x() + offset.x(), startPos.y() + offset.y());
      m_startPoint->setCGALPosition(newStartPos);
      // m_startPoint->setCGALPosition calls its connected lines' update().
      // For *this* line, update() will see m_isUpdatingInternally=true and
      // return early. For *other* lines connected to this point, their
      // update() will run.
    }

    if (m_endPoint) {
      Point_2 endPos = m_endPoint->getCGALPosition();
      Point_2 newEndPos(endPos.x() + offset.x(), endPos.y() + offset.y());
      m_endPoint->setCGALPosition(newEndPos);
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception in Line::translate during endpoint setting: " << e.what()
              << " Line: " << this << std::endl;
    m_isUpdatingInternally = was_already_updating_internally;  // Restore flag on error
    return;
  }

  m_isUpdatingInternally = was_already_updating_internally;  // Restore the original state

  // If this translate operation was the primary action (not nested within
  // another update of this line), then finalize this line's state and notify
  // its observers.
  if (!m_isUpdatingInternally) {
    // The line's points have been directly translated.
    // Update its internal CGAL line, SFML shape, and hosted points.
    // Then, notify observers. Observers will run their full update cycle,
    // including their own maintainConstraints if applicable.

    updateCGALLine();  // Update m_cgalLine from the new m_startPoint and
    // m_endPoint
    updateSFMLShape();     // Update visual representation
    updateHostedPoints();  // Update points that live on this line

    if (Constants::DEBUG_CONSTRAINTS && !m_constraintObservers.empty()) {
      std::cout << "Line::translate: Notifying " << m_constraintObservers.size()
                << " observers post-translation. Line: " << this << std::endl;
    }
    notifyConstraintObserversOfChange();
  }
  // If m_isUpdatingInternally was true, it means this translate was called
  // from within an existing update cycle for *this same line*. The outer
  // update cycle will handle finalization and observer notification. We just
  // ensure its internal geometry is current.
  else {
    updateCGALLine();
    updateSFMLShape();
    updateHostedPoints();
  }
}
void Line::setColor(const sf::Color &color) {
  // NO CALL to GeometricObject::setColor here
  this->m_color = color;  // Set the inherited m_color from GeometricObject
  // If lines have their own distinct color members (e.g., m_lineColor), set
  // them. Otherwise, m_color will be used by updateSFMLShape.

  // Optional: Propagate color to endpoints if they are not shared or if
  // desired. This can be complex if points are shared. if (m_startPoint)
  // m_startPoint->setColor(color); // Be cautious if (m_endPoint)
  // m_endPoint->setColor(color);   // Be cautious

  this->updateSFMLShape();  // Update the SFML shape
}

void Line::setAsConstructionLine() {
  m_isConstructionLine = true;
}

bool Line::isConstructionLine() const {
  return m_isConstructionLine;
}

void Line::setVisible(bool visible) {
  m_visible = visible;
  m_hidden = !visible;
  updateSFMLShape();
}

void Line::setLocked(bool locked) {
  GeometricObject::setLocked(locked);
}

bool Line::isLocked() const {
  return GeometricObject::isLocked();
}

bool Line::isVisible() const { return m_visible; }

void Line::toggleVisibility() {
  setVisible(!m_visible);
}

void Line::setHidden(bool hidden) {
  m_hidden = hidden;
  m_visible = !hidden;
  updateSFMLShape();
}

bool Line::isHidden() const { return m_hidden; }
/* void Line::translate(const Vector_2 &offset) {
  std::cerr << "Line::translate (Simplified): Entered." << std::endl;
  try {
    // Ensure offset components are finite
    if (!CGAL::is_finite(offset.x()) || !CGAL::is_finite(offset.y())) {
      std::cerr
          << "Line::translate (Simplified): Offset contains non-finite values.
" "Translation aborted."
          << std::endl;
      return;
    }

    bool needsFullUpdate = false;

    // Translate start point
    if (m_startPoint) {
      try {
        std::cerr << "Line::translate (Simplified): Translating m_startPoint."
                  << std::endl;
        m_startPoint->translate(
            offset); // Assumes Point::translate updates its CGAL pos
                     // and notifies connected lines (or setCGALPosition does)
        needsFullUpdate = true;
      } catch (const std::exception &e) {
        std::cerr
            << "Line::translate (Simplified): Error translating start point: "
            << e.what() << std::endl;
      }
    } else {
      std::cerr << "Line::translate (Simplified): m_startPoint is null."
                << std::endl;
    }

    // Translate end point
    if (m_endPoint) {
      try {
        std::cerr << "Line::translate (Simplified): Translating m_endPoint."
                  << std::endl;
        m_endPoint->translate(
            offset); // Assumes Point::translate updates its CGAL pos
        needsFullUpdate = true;
      } catch (const std::exception &e) {
        std::cerr
            << "Line::translate (Simplified): Error translating end point: "
            << e.what() << std::endl;
      }
    } else {
      std::cerr << "Line::translate (Simplified): m_endPoint is null."
                << std::endl;
    }

    if (needsFullUpdate) {
      std::cerr << "Line::translate (Simplified): Calling update()."
                << std::endl;
      try {
        update(); // This calls updateCGALLine, maintainConstraints,
                  // updateSFMLShape, updateHostedPoints, notifyObservers
      } catch (const std::exception &e) {
        std::cerr << "Line::translate (Simplified): Error during update cycle
" "after translation: "
                  << e.what() << std::endl;
      }
    } else {
      std::cerr << "Line::translate (Simplified): No points translated, "
                   "update() skipped."
                << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "Line::translate (Simplified): Top-level unhandled exception:
"
              << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Line::translate (Simplified): Unknown top-level exception"
              << std::endl;
  }
} */

// Line::translateWithDependents remains structurally similar but calls the
// simplified translate
void Line::translateWithDependents(const Vector_2 &offset) {
  std::cerr << "Line::translateWithDependents: Entered." << std::endl;

  // --- BEGIN NEW PRE-CHECK for offset components ---
  try {
    std::cerr << "Line::translateWithDependents: Pre-accessing offset.x()..." << std::endl;
    // Attempt to access offset.x() to trigger lazy evaluation.
    // The type of offset.x() is Kernel::FT (which is Epeck_ft for Epeck
    // kernel)
    [[maybe_unused]] volatile Kernel::FT temp_x = offset.x();
    std::cerr << "Line::translateWithDependents: offset.x() pre-access OK." << std::endl;

    std::cerr << "Line::translateWithDependents: Pre-accessing offset.y()..." << std::endl;
    [[maybe_unused]] volatile Kernel::FT temp_y = offset.y();
    std::cerr << "Line::translateWithDependents: offset.y() pre-access OK." << std::endl;

    // If access was OK, then try to check finiteness with CGAL::is_finite
    // This is the original problematic line, now guarded by the pre-access
    // try-catch
    if (!CGAL::is_finite(offset.x()) ||
        !CGAL::is_finite(offset.y())) {  // This is line 1161 from your context
      std::cerr << "Line::translateWithDependents: Offset contains non-finite "
                   "values after CGAL::is_finite check. Aborting."
                << std::endl;
      return;
    }
    std::cerr << "Line::translateWithDependents: CGAL::is_finite(offset) check "
                 "passed."
              << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Line::translateWithDependents: CRITICAL - std::exception "
                 "during offset pre-access or CGAL::is_finite check: "
              << e.what() << std::endl;
    std::cerr << "Line::translateWithDependents: This likely indicates the "
                 "crash point from Lazy.h:442."
              << std::endl;
    // It's crucial to know if the sfml_delta values in HandleEvents.cpp were
    // finite. If they were, this points to an issue within Epeck's handling
    // of those finite doubles.
    return;  // Abort on any exception here
  } catch (...) {
    std::cerr << "Line::translateWithDependents: CRITICAL - Unknown exception "
                 "during offset pre-access or CGAL::is_finite check."
              << std::endl;
    return;  // Abort on any exception here
  }

  // Debug state of line's endpoints BEFORE the loop
  if (m_startPoint && m_endPoint) {
    std::cerr << "Line::translateWithDependents: BEFORE hostedObjectPoints "
                 "loop - Checking line endpoints."
              << std::endl;
    try {
      Point_2 sp_before_loop = m_startPoint->getCGALPosition();
      Point_2 ep_before_loop = m_endPoint->getCGALPosition();
      debug_cgal_point(sp_before_loop, "m_startPoint (before loop)",
                       "Line::translateWithDependents");
      debug_cgal_point(ep_before_loop, "m_endPoint (before loop)", "Line::translateWithDependents");
    } catch (const std::exception &e) {
      std::cerr << "Line::translateWithDependents: (BEFORE loop) Exception "
                   "getting initial endpoint positions: "
                << e.what() << std::endl;
    }
  } else {
    std::cerr << "Line::translateWithDependents: (BEFORE loop) Line has null "
                 "m_startPoint or m_endPoint."
              << std::endl;
  }
  // Store the *double* relative parameter for each hosted point
  std::vector<std::pair<std::shared_ptr<ObjectPoint>, double>> relativeParamsToRestore;
  if (!m_hostedObjectPoints.empty()) {
    std::cerr << "Line::translateWithDependents: Processing " << m_hostedObjectPoints.size()
              << " hosted points." << std::endl;
    for (const auto &objPoint_weak : m_hostedObjectPoints) {
      if (auto objPoint = objPoint_weak.lock()) {
        try {
          std::cerr << "Line::translateWithDependents: For ObjectPoint, "
                       "calling getRelativePositionOnLine()."
                    << std::endl;
          double relativeParam = objPoint->getRelativePositionOnLine();
          relativeParamsToRestore.push_back(std::make_pair(objPoint, relativeParam));
          std::cerr << "Line::translateWithDependents: Stored relativeParam: " << relativeParam
                    << " for ObjectPoint " << objPoint.get() << std::endl;
        } catch (const std::exception &e) {
          std::cerr << "Line::translateWithDependents: Exception in "
                       "getRelativePositionOnLine() for ObjectPoint: "
                    << e.what() << std::endl;
        }
      }
    }
  }

  // Debug state of line's endpoints AFTER the loop, BEFORE main translate
  // call
  if (m_startPoint && m_endPoint) {
    std::cerr << "Line::translateWithDependents: AFTER hostedObjectPoints "
                 "loop, BEFORE this->translate() - Checking line endpoints."
              << std::endl;
    try {
      Point_2 sp_after_loop = m_startPoint->getCGALPosition();
      Point_2 ep_after_loop = m_endPoint->getCGALPosition();
      debug_cgal_point(sp_after_loop, "m_startPoint (after loop)", "Line::translateWithDependents");
      debug_cgal_point(ep_after_loop, "m_endPoint (after loop)", "Line::translateWithDependents");
    } catch (const std::exception &e) {
      std::cerr << "Line::translateWithDependents: (AFTER loop) Exception "
                   "getting endpoint positions: "
                << e.what() << std::endl;
    }
  } else {
    std::cerr << "Line::translateWithDependents: (AFTER loop) Line has null "
                 "m_startPoint or m_endPoint."
              << std::endl;
  }

  // Translate the line itself using the new SIMPLIFIED translate method
  std::cerr << "Line::translateWithDependents: Calling this->translate(offset) "
               "[Simplified Version]."
            << std::endl;
  this->translate(offset);  // Line 970 in your call stack - now calls the
  // simplified version

  // Hosted points are updated via the Line::update() call within
  // this->translate(), which calls updateHostedPoints() ->
  // ObjectPoint::updatePositionFromHost().
  std::cerr << "Line::translateWithDependents: Exiting. Hosted points should "
               "have been updated by Line::update() via updateHostedPoints()."
            << std::endl;
}

// ... (rest of Line.cpp, including the original complex translate method if
// you want to keep it commented out or under a different name for now) Make
// sure that any other place that was calling the old `Line::translate` now
// calls this simplified one if this is the intended global change. Or, you
// can rename the old complex translate (e.g., to
// `translateWithConstraintLogic`) and have this new one be the primary
// `translate`.
void Line::moveEndpointToStart(const Point_2 &newPos) {
  try {
    if (!m_startPoint || !m_endPoint) {
      std::cerr << "Line::moveEndpointToStart: Missing endpoint(s)" << std::endl;
      return;
    }

    if (m_isParallelLine) {  // Implies axis-parallel if m_referenceLine is
      // null
      Point_2 startPos_current, endPos_current;
      try {
        startPos_current = m_startPoint->getCGALPosition();
        endPos_current = m_endPoint->getCGALPosition();
      } catch (const std::exception &e) {
        std::cerr << "Line::moveEndpointToStart: Error getting positions: " << e.what()
                  << std::endl;
        return;
      }

      Vector_2 lineDirection = endPos_current - startPos_current;
      Kernel::FT ft_epsilon(1e-9);

      bool isHorizontal =
          (CGAL::abs(lineDirection.y()) < ft_epsilon && CGAL::abs(lineDirection.x()) > ft_epsilon);
      bool isVertical =
          (CGAL::abs(lineDirection.x()) < ft_epsilon && CGAL::abs(lineDirection.y()) > ft_epsilon);

      if (isHorizontal) {
        try {
          Point_2 constrainedPos(newPos.x(), startPos_current.y());
          m_startPoint->setCGALPosition(constrainedPos);
          if (m_endPoint->getCGALPosition().y() != constrainedPos.y()) {
            m_endPoint->setCGALPosition(
                Point_2(m_endPoint->getCGALPosition().x(), constrainedPos.y()));
          }
          updateHostedPoints();
          update();
          return;
        } catch (const std::exception &e) {
          std::cerr << "Line::moveEndpointToStart: Error handling horizontal "
                       "constraint: "
                    << e.what() << std::endl;
        }
      } else if (isVertical) {
        try {
          Point_2 constrainedPos(startPos_current.x(), newPos.y());
          m_startPoint->setCGALPosition(constrainedPos);
          if (m_endPoint->getCGALPosition().x() != constrainedPos.x()) {
            m_endPoint->setCGALPosition(
                Point_2(constrainedPos.x(), m_endPoint->getCGALPosition().y()));
          }
          updateHostedPoints();
          update();
          return;
        } catch (const std::exception &e) {
          std::cerr << "Line::moveEndpointToStart: Error handling vertical "
                       "constraint: "
                    << e.what() << std::endl;
        }
      }
    }

    // Default handling when no special case applies or special case handling
    // failed
    try {
      m_startPoint->setCGALPosition(newPos);
      updateHostedPoints();  // This calls updateCGALLine, updateSFMLShape,
      // etc.
      update();  // Ensure full update cycle, including constraints
    } catch (const std::exception &e) {
      std::cerr << "Line::moveEndpointToStart: Error in default handling: " << e.what()
                << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Line::moveEndpointToStart: Unhandled exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Line::moveEndpointToStart: Unknown exception" << std::endl;
  }
}

void Line::moveEndpointToEnd(const Point_2 &newPos) {
  try {
    if (!m_startPoint || !m_endPoint) {
      std::cerr << "Line::moveEndpointToEnd: Missing endpoint(s)" << std::endl;
      return;
    }

    if (m_isParallelLine) {  // Implies axis-parallel if m_referenceLine is
      // null
      Point_2 startPos_current, endPos_current;
      try {
        startPos_current = m_startPoint->getCGALPosition();
        endPos_current = m_endPoint->getCGALPosition();
      } catch (const std::exception &e) {
        std::cerr << "Line::moveEndpointToEnd: Error getting positions: " << e.what() << std::endl;
        return;
      }

      Vector_2 lineDirection = endPos_current - startPos_current;
      Kernel::FT ft_epsilon(1e-9);

      bool isHorizontal =
          (CGAL::abs(lineDirection.y()) < ft_epsilon && CGAL::abs(lineDirection.x()) > ft_epsilon);
      bool isVertical =
          (CGAL::abs(lineDirection.x()) < ft_epsilon && CGAL::abs(lineDirection.y()) > ft_epsilon);

      if (isHorizontal) {
        try {
          Point_2 constrainedPos(newPos.x(), endPos_current.y());
          m_endPoint->setCGALPosition(constrainedPos);
          if (m_startPoint->getCGALPosition().y() != constrainedPos.y()) {
            m_startPoint->setCGALPosition(
                Point_2(m_startPoint->getCGALPosition().x(), constrainedPos.y()));
          }
          updateHostedPoints();
          update();
          return;
        } catch (const std::exception &e) {
          std::cerr << "Line::moveEndpointToEnd: Error handling horizontal "
                       "constraint: "
                    << e.what() << std::endl;
        }
      } else if (isVertical) {
        try {
          Point_2 constrainedPos(endPos_current.x(), newPos.y());
          m_endPoint->setCGALPosition(constrainedPos);
          if (m_startPoint->getCGALPosition().x() != constrainedPos.x()) {
            m_startPoint->setCGALPosition(
                Point_2(constrainedPos.x(), m_startPoint->getCGALPosition().y()));
          }
          updateHostedPoints();
          update();
          return;
        } catch (const std::exception &e) {
          std::cerr << "Line::moveEndpointToEnd: Error handling vertical "
                       "constraint: "
                    << e.what() << std::endl;
        }
      }
    }

    // Default handling when no special case applies or special case handling
    // failed
    try {
      m_endPoint->setCGALPosition(newPos);
      updateHostedPoints();  // This calls updateCGALLine, updateSFMLShape,
      // etc.
      update();  // Ensure full update cycle, including constraints
    } catch (const std::exception &e) {
      std::cerr << "Line::moveEndpointToEnd: Error in default handling: " << e.what() << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Line::moveEndpointToEnd: Unhandled exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Line::moveEndpointToEnd: Unknown exception" << std::endl;
  }
}

Point_2 Line::getCGALPosition() const {
  if (!m_startPoint || !m_endPoint) {
    throw std::runtime_error("Line::getCGALPosition: null endpoint(s)");
  }

  // Return midpoint of the line
  Point_2 startPos = m_startPoint->getCGALPosition();
  Point_2 endPos = m_endPoint->getCGALPosition();

  // Calculate midpoint
  return Point_2((startPos.x() + endPos.x()) / 2, (startPos.y() + endPos.y()) / 2);
}

void Line::setCGALPosition(const Point_2 &newPos) {
  // Move both endpoints to maintain the line's direction and length
  if (!m_startPoint || !m_endPoint) {
    throw std::runtime_error("Line::setCGALPosition: null endpoint(s)");
  }

  Point_2 currentPos = getCGALPosition();  // Uses the above method
  Vector_2 offset(newPos.x() - currentPos.x(), newPos.y() - currentPos.y());

  translate(offset);  // Uses the modified translate
}

void Line::setPosition(const sf::Vector2f &newSfmlPos) {
  // Convert SFML position to CGAL and call setCGALPosition
  Point_2 newPos(newSfmlPos.x, newSfmlPos.y);
  setCGALPosition(newPos);  // Uses the above method
}

std::vector<Segment_2> Line::getEdges() const {
  if (!m_isSegment) {
    return {};
  }
  try {
    return {Segment_2(getStartPoint(), getEndPoint())};
  } catch (...) {
    return {};
  }
}

std::vector<Segment_2> Line::getBoundarySegments() const {
  return getEdges();
}

bool Line::getClosestPointOnPerimeter(const Point_2 &query, Point_2 &outPoint) const {
  try {
    if (m_isSegment) {
      Segment_2 seg(getStartPoint(), getEndPoint());
      Point_2 a = seg.source();
      Point_2 b = seg.target();
      Vector_2 ab = b - a;
      FT ab2 = ab.squared_length();
      if (ab2 == FT(0)) {
        outPoint = a;
        return true;
      }
      Vector_2 ap = query - a;
      FT t = (ab * ap) / ab2;
      if (t < FT(0)) {
        t = FT(0);
      } else if (t > FT(1)) {
        t = FT(1);
      }
      outPoint = a + ab * t;
      return true;
    }

    Line_2 line = getCGALLine();
    outPoint = line.projection(query);
    return true;
  } catch (...) {
    return false;
  }
}

// --- Line Specific Methods ---
Point_2 Line::getStartPoint() const {
  if (!m_startPoint) {
    throw std::runtime_error("Line::getStartPoint: null start point");
  }
  return m_startPoint->getCGALPosition();
}

Point_2 Line::getEndPoint() const {
  if (!m_endPoint) {
    throw std::runtime_error("Line::getEndPoint: null end point");
  }
  return m_endPoint->getCGALPosition();
}

/* Vector_2 Line::getDirection() const {
  // Add more robustness
  static const Vector_2 DEFAULT_DIRECTION(1, 0);

  if (!m_startPoint || !m_endPoint) {
    std::cerr
        << "Line::getDirection: Null endpoint(s), returning default
direction."
        << std::endl;
    return DEFAULT_DIRECTION;
  }

  try {
    // Check if endpoints are properly initialized
    if (!m_startPoint->isInitialized() || !m_endPoint->isInitialized()) {
      std::cerr << "Line::getDirection: Endpoint not initialized, returning "
                   "default direction."
                << std::endl;
      return DEFAULT_DIRECTION;
    }

    Point_2 startPos = m_startPoint->getCGALPosition();
    Point_2 endPos = m_endPoint->getCGALPosition();

    if (!CGAL::is_finite(startPos.x()) || !CGAL::is_finite(startPos.y()) ||
        !CGAL::is_finite(endPos.x()) || !CGAL::is_finite(endPos.y())) {
      std::cerr << "Line::getDirection: Non-finite coordinate detected, "
                   "returning default direction."
                << std::endl;
      return DEFAULT_DIRECTION;
    }

    if (startPos == endPos) {
      std::cerr << "Line::getDirection: Endpoints are coincident, returning "
                   "default direction."
                << std::endl;
      return DEFAULT_DIRECTION;
    }

    Vector_2 dir(endPos.x() - startPos.x(), endPos.y() - startPos.y());

    if (!CGAL::is_finite(dir.x()) || !CGAL::is_finite(dir.y())) {
      std::cerr << "Line::getDirection: Non-finite direction component, "
                   "returning default direction."
                << std::endl;
      return DEFAULT_DIRECTION;
    }

    return dir;

  } catch (const std::exception &e) {
    std::cerr << "Line::getDirection: Exception: " << e.what()
              << ", returning default direction." << std::endl;
    return DEFAULT_DIRECTION;
  } catch (...) {
    std::cerr
        << "Line::getDirection: Unknown exception, returning default
direction."
        << std::endl;
    return DEFAULT_DIRECTION;
  }
} */
Direction_2 Line::getDirection() const {
  if (!m_startPoint || !m_endPoint || !m_startPoint->isValid() || !m_endPoint->isValid()) {
    std::cerr << "Warning: Line::getDirection() called on line with invalid points." << std::endl;
    return Direction_2(1.0, 0.0);  // Default horizontal direction
  }
  // Construct Direction_2 from the vector between the two points
  return Direction_2(m_endPoint->getCGALPosition() - m_startPoint->getCGALPosition());
}

void Line::setAsParallelLine(std::shared_ptr<GeometricObject> refObj, int edgeIndex,
                             const Vector_2 &referenceDirection) {
  try {
    // 1. Clear from old reference if it exists
    if (auto old_ref_sp = m_constraintRefObject.lock()) {
      if (m_constraintRefEdgeIndex == -1) {  // Was a whole object (Line) constraint
        if (auto old_line_sp = std::dynamic_pointer_cast<Line>(old_ref_sp)) {
            // Only remove observer if we are changing to a new reference or null
             if (!refObj || refObj != old_ref_sp) {
                 old_line_sp->removeConstraintObserver(shared_from_this());
             }
        }
      }
    }
    m_constraintRefObject.reset();

    // 2. Set new constraint type
    m_isParallelLine = true;
    m_isPerpendicularLine = false;

    // 3. Set new reference and direction
    if (refObj) {
        if (refObj.get() == this) {
            std::cerr << "Line::setAsParallelLine: Cannot set self as reference. Constraint not applied." << std::endl;
            m_isParallelLine = false;
            m_constraintDirection = Vector_2(0, 0);
            update();
            return;
        }

        m_constraintRefObject = refObj;
        m_constraintRefEdgeIndex = edgeIndex;
        m_constraintDirection = referenceDirection; // Initial direction (will be updated)

        // 4. Add self as observer IF it's a Line object (optimization)
        if (edgeIndex == -1) {
            if (auto lineRef = std::dynamic_pointer_cast<Line>(refObj)) {
                lineRef->addConstraintObserver(shared_from_this());
            }
        }

        if (Constants::DEBUG_CONSTRAINTS) {
            std::cout << "Line " << getID() << " setAsParallelLine to Object Type " 
                      << static_cast<int>(refObj->getType()) << ". Index: " << edgeIndex << std::endl;
        }

    } else {
        // Axis-Parallel logic (unchanged essentially, just using m_constraintDirection)
         m_constraintDirection = referenceDirection;
          if (m_constraintDirection == Vector_2(0, 0)) {
            std::cerr << "Line::setAsParallelLine: No reference and zero direction." << std::endl;
             m_isParallelLine = false;
          }
    }

    if (!m_startPoint || !m_endPoint || !m_startPoint->isValid() || !m_endPoint->isValid()) {
       // ... error handling
       m_isParallelLine = false; 
       m_constraintRefObject.reset();
       update();
       return;
    }

    // 5. Immediately apply constraint
    update();

  } catch (const std::exception &e) {
    std::cerr << "Line::setAsParallelLine: Exception: " << e.what() << std::endl;
    m_isParallelLine = false;
    m_constraintRefObject.reset();
  }
}

void Line::setAsPerpendicularLine(std::shared_ptr<GeometricObject> refObj, int edgeIndex,
                                  const Vector_2 &referenceDirection) {
  try {
    // 1. Clear from old reference if it exists
    if (auto old_ref_sp = m_constraintRefObject.lock()) {
       if (m_constraintRefEdgeIndex == -1) {
           if (auto old_line_sp = std::dynamic_pointer_cast<Line>(old_ref_sp)) {
             if (!refObj || refObj != old_ref_sp) {
                 old_line_sp->removeConstraintObserver(shared_from_this());
             }
           }
       }
    }
    m_constraintRefObject.reset();

    // 2. Set new constraint type
    m_isParallelLine = false;
    m_isPerpendicularLine = true;

    // 3. Set new reference and direction
    if (refObj) {
       if (refObj.get() == this) {
            std::cerr << "Line::setAsPerpendicularLine: Cannot set self as reference." << std::endl;
            m_isPerpendicularLine = false;
            update();
            return;
        }

        m_constraintRefObject = refObj;
        m_constraintRefEdgeIndex = edgeIndex;
        m_constraintDirection = referenceDirection;

        // 4. Add self as observer
        if (edgeIndex == -1) {
            if (auto lineRef = std::dynamic_pointer_cast<Line>(refObj)) {
                lineRef->addConstraintObserver(shared_from_this());
            }
        }
        
        if (Constants::DEBUG_CONSTRAINTS) {
             std::cout << "Line " << getID() << " setAsPerpendicularLine to Object Type " 
                      << static_cast<int>(refObj->getType()) << ". Index: " << edgeIndex << std::endl;
        }

    } else {
        m_constraintDirection = referenceDirection;
         if (m_constraintDirection == Vector_2(0, 0)) {
            m_isPerpendicularLine = false;
         }
    }

    if (!m_startPoint || !m_endPoint || !m_startPoint->isValid() || !m_endPoint->isValid()) {
       m_isPerpendicularLine = false;
       m_constraintRefObject.reset();
       update();
       return;
    }

    update();
  } catch (const std::exception &e) {
    std::cerr << "Line::setAsPerpendicularLine: Exception: " << e.what() << std::endl;
    m_isPerpendicularLine = false;
    m_constraintRefObject.reset();
  }
}

bool Line::maintainConstraints() {
  if (s_updatingLines.find(this) != s_updatingLines.end()) {
    return false;  // Already updating this line in the current call stack
  }

  if (m_inConstraint) {
    return false;
  }

  s_updatingLines.insert(this);
  m_inConstraint = true;

  bool geometryChanged = false;

  bool localDebug = Constants::DEBUG_CONSTRAINTS;

  if (localDebug) {
    std::cout << "Line::maintainConstraints: ENTERED for Line " << this
              << " (isParallel: " << m_isParallelLine
              << ", isPerpendicular: " << m_isPerpendicularLine << ")" << std::endl;
  }

  try {
    if (!m_isParallelLine && !m_isPerpendicularLine) {
      if (localDebug) {
        std::cout << "  No constraint type active. Aborting." << std::endl;
      }
      m_inConstraint = false;
      s_updatingLines.erase(this);
      return false;
    }

    if (!m_startPoint || !m_endPoint || !m_startPoint->isValid() || !m_endPoint->isValid()) {
      if (localDebug) {
        std::cout << "  Null or invalid endpoint(s). Aborting." << std::endl;
      }
      m_inConstraint = false;
      s_updatingLines.erase(this);
      return false;
    }
    Vector_2 actual_ref_dir_for_constraint;
    bool has_valid_reference_for_constraint = false;

    // Get reference direction - ALWAYS get the CURRENT direction from reference
    if (auto ref_obj_sp = m_constraintRefObject.lock()) {
      if (ref_obj_sp.get() == this) {
         m_isParallelLine = false; m_isPerpendicularLine = false;
         m_constraintRefObject.reset();
         m_inConstraint = false;
         s_updatingLines.erase(this);
         return false;
      }

      if (ref_obj_sp->isValid()) {
        try {
            if (m_constraintRefEdgeIndex == -1) {
                // Was a Line/Segment object
                if (auto refLine = std::dynamic_pointer_cast<Line>(ref_obj_sp)) {
                    Point_2 refStart = refLine->getStartPoint();
                    Point_2 refEnd = refLine->getEndPoint();
                     if (refStart != refEnd) {
                        actual_ref_dir_for_constraint = Vector_2(refEnd.x() - refStart.x(), refEnd.y() - refStart.y());
                        has_valid_reference_for_constraint = true;
                     }
                }
            } else {
                // Edge of a complex shape
                std::vector<Segment_2> edges = ref_obj_sp->getEdges();
                if (m_constraintRefEdgeIndex >= 0 && m_constraintRefEdgeIndex < static_cast<int>(edges.size())) {
                    const Segment_2& seg = edges[m_constraintRefEdgeIndex];
                    actual_ref_dir_for_constraint = seg.to_vector();
                     if (actual_ref_dir_for_constraint != Vector_2(0,0)) {
                         has_valid_reference_for_constraint = true;
                     }
                }
            }
            
            if (has_valid_reference_for_constraint) {
                m_constraintDirection = actual_ref_dir_for_constraint;
                 if (localDebug) {
                    CGALSafeUtils::debug_cgal_vector(actual_ref_dir_for_constraint,
                                                "  CURRENT Reference Direction",
                                                "MaintainConstraints");
                 }
            }

        } catch (const std::exception &e) {
          std::cerr << "  Exception getting reference object edges: " << e.what() << std::endl;
        }
      }
    }

    if (!has_valid_reference_for_constraint) {
      if (m_constraintDirection != Vector_2(0, 0)) {
        actual_ref_dir_for_constraint = m_constraintDirection;
        has_valid_reference_for_constraint = true;
        if (localDebug)
          CGALSafeUtils::debug_cgal_vector(actual_ref_dir_for_constraint,
                                           "  Axis Constraint Direction", "MaintainConstraints");
      } else {
        if (localDebug) std::cout << "  No valid reference. Clearing constraints." << std::endl;
        m_isParallelLine = false;
        m_isPerpendicularLine = false;
        m_constraintRefObject.reset();
        m_inConstraint = false;
        s_updatingLines.erase(this);
        return false;
      }
    }

    if (actual_ref_dir_for_constraint.squared_length() <
        Kernel::FT(Constants::CGAL_EPSILON_SQUARED)) {
      if (localDebug) {
        std::cout << "  Reference direction is zero. Aborting." << std::endl;
      }
      m_inConstraint = false;
      s_updatingLines.erase(this);
      return false;
    }

    try {
      Point_2 authoritative_anchor_pos;
      std::shared_ptr<Point> point_to_be_adjusted_sp;

      // Determine which point to use as anchor and which to adjust
      if (m_externallyMovedEndpoint == m_startPoint.get()) {
        authoritative_anchor_pos = m_startPoint->getCGALPosition();
        point_to_be_adjusted_sp = m_endPoint;
        if (localDebug)
          std::cout << "  Anchor: StartPoint (externally moved). Adjusting: EndPoint." << std::endl;
      } else if (m_externallyMovedEndpoint == m_endPoint.get()) {
        authoritative_anchor_pos = m_endPoint->getCGALPosition();
        point_to_be_adjusted_sp = m_startPoint;
        if (localDebug)
          std::cout << "  Anchor: EndPoint (externally moved). Adjusting: StartPoint." << std::endl;
      } else {
        // Default: anchor on start, adjust end
        authoritative_anchor_pos = m_startPoint->getCGALPosition();
        point_to_be_adjusted_sp = m_endPoint;
        if (localDebug)
          std::cout << "  Anchor: StartPoint (default). Adjusting: EndPoint." << std::endl;
      }

      if (!point_to_be_adjusted_sp || !point_to_be_adjusted_sp->isValid()) {
        std::cerr << "  Point to be adjusted is null or invalid. Aborting." << std::endl;
        m_inConstraint = false;
        s_updatingLines.erase(this);
        return false;
      }

      if (!CGAL::is_finite(authoritative_anchor_pos.x()) ||
          !CGAL::is_finite(authoritative_anchor_pos.y())) {
        std::cerr << "  Authoritative anchor position is not finite. Aborting." << std::endl;
        m_inConstraint = false;
        s_updatingLines.erase(this);
        return false;
      }

      // Use current distance between points to maintain relative positioning
      Point_2 current_adjusted_pos = point_to_be_adjusted_sp->getCGALPosition();

      // Calculate the squared distance
      Kernel::FT sq_distance =
          CGAL::squared_distance(authoritative_anchor_pos, current_adjusted_pos);

      // Calculate distance using sqrt from double and convert back to
      // Kernel::FT
      double sq_distance_double = CGAL::to_double(sq_distance);
      double distance_double = std::sqrt(sq_distance_double);
      Kernel::FT current_distance = Kernel::FT(distance_double);

      // If distance is too small, use default
      if (current_distance < Kernel::FT(Constants::EPSILON_LENGTH_CONSTRUCTION)) {
        current_distance = Kernel::FT(Constants::DEFAULT_LINE_CONSTRUCTION_LENGTH);
      }

      Vector_2 unit_target_dir;
      if (m_isParallelLine) {
        unit_target_dir = CGALSafeUtils::normalize_vector_robust(actual_ref_dir_for_constraint,
                                                                 "MaintainConstraints_ParallelDir");
      } else {  // m_isPerpendicularLine
        Vector_2 perpDir(-actual_ref_dir_for_constraint.y(), actual_ref_dir_for_constraint.x());
        unit_target_dir =
            CGALSafeUtils::normalize_vector_robust(perpDir, "MaintainConstraints_PerpDir");
      }

      if (unit_target_dir.squared_length() < Kernel::FT(Constants::CGAL_EPSILON_SQUARED / 100.0)) {
        std::cerr << "  Unit target direction is zero after normalization. Aborting." << std::endl;
        m_inConstraint = false;
        s_updatingLines.erase(this);
        return false;
      }

      // Determine direction: if the current adjusted point is "behind" the
      // anchor relative to target direction, place it in the opposite direction
      Vector_2 current_vector = current_adjusted_pos - authoritative_anchor_pos;
      Kernel::FT dot_product =
          current_vector.x() * unit_target_dir.x() + current_vector.y() * unit_target_dir.y();

      Point_2 new_adjusted_pos;
      if (dot_product >= 0) {
        // Current point is in the positive direction, keep it there
        new_adjusted_pos = authoritative_anchor_pos + (unit_target_dir * current_distance);
      } else {
        // Current point is in the negative direction, keep it there
        new_adjusted_pos = authoritative_anchor_pos - (unit_target_dir * current_distance);
      }

      if (!CGAL::is_finite(new_adjusted_pos.x()) || !CGAL::is_finite(new_adjusted_pos.y())) {
        std::cerr << "  Calculated new_adjusted_pos is not finite. Aborting." << std::endl;
        m_inConstraint = false;
        s_updatingLines.erase(this);
        return false;
      }

      // EPSILON GUARD: Prevent infinite recursion by checking if position actually changed
      // Floating-point precision errors can cause tiny differences (e.g., 1e-15) that
      // trigger update cascades even when the position is mathematically identical.
      Kernel::FT distSq = CGAL::squared_distance(current_adjusted_pos, new_adjusted_pos);
      const double EPSILON_THRESHOLD = 1e-12; // Squared distance threshold (Loop Killer)
      
      if (CGAL::to_double(distSq) < EPSILON_THRESHOLD) {
        // NO MEANINGFUL CHANGE - break the update cycle
        if (localDebug) {
          std::cout << "  EPSILON GUARD: Position unchanged (distSq=" 
                    << CGAL::to_double(distSq) << " < " << EPSILON_THRESHOLD 
                    << "). Skipping update." << std::endl;
        }
        m_inConstraint = false;
        s_updatingLines.erase(this);
        return false;
      }
      
      if (localDebug) {
        CGALSafeUtils::debug_cgal_point(current_adjusted_pos, "  Old adjusted pos",
                                        "MaintainConstraints");
        CGALSafeUtils::debug_cgal_point(new_adjusted_pos, "  New adjusted pos",
                                        "MaintainConstraints");
        std::cout << "  Delta squared distance: " << CGAL::to_double(distSq) << std::endl;
      }

      // Temporarily clear m_externallyMovedEndpoint before this call
      Point *tempExternalMove = m_externallyMovedEndpoint;
      m_externallyMovedEndpoint = nullptr;

      point_to_be_adjusted_sp->setCGALPosition(new_adjusted_pos);
      geometryChanged = true;

      m_externallyMovedEndpoint = tempExternalMove;

    } catch (const std::exception &e) {
      std::cerr << "  Exception in Line::maintainConstraints: " << e.what() << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "Line::maintainConstraints: Top-level exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Line::maintainConstraints: Unknown exception" << std::endl;
  }

  if (localDebug) {
    std::cout << "Line::maintainConstraints: EXITED for Line " << this << std::endl;
  }

  m_inConstraint = false;
  s_updatingLines.erase(this);
  return geometryChanged;  // True only if geometry was actually updated
}

void Line::notifyObjectPoints() {
  // Backward compatibility wrapper
  updateHostedPoints(); 
}

void Line::updateSFMLShape() {
  QUICK_PROFILE("Line::updateSFMLShape");
  if (m_deferSFMLUpdates) {
    m_pendingSFMLUpdate = true;
    return;
  }
   try {
        // Test access to member variables to ensure object is still valid
        [[maybe_unused]] auto test = m_color;  // This will crash here if object is destroyed
        
        if (!m_startPoint || !m_endPoint) {
            m_sfmlShape.clear();
            return;
        }
        if ((m_startPoint && !m_startPoint->isValid()) ||
            (m_endPoint && !m_endPoint->isValid())) {
          m_sfmlShape.clear();
          return;
        }
  // clear if endpoints missing
  if (!m_startPoint || !m_endPoint) {
    m_sfmlShape.clear();
    return;
  }

  // Convert CGAL→SFML
  auto toSF = [&](const Point_2 &p) {
    return sf::Vector2f(static_cast<float>(CGAL::to_double(p.x())),
                        static_cast<float>(CGAL::to_double(p.y())));
  };

  sf::Vector2f p1, p2;
  try {
    p1 = toSF(m_startPoint->getCGALPosition());
    p2 = toSF(m_endPoint->getCGALPosition());
  } catch (const std::exception &e) {
    return;
  }

  // Set up vertex array
  m_sfmlShape.setPrimitiveType(sf::Lines);
  m_sfmlShape.resize(2);

  if (m_isSegment) {
    m_sfmlShape[0].position = p1;
    m_sfmlShape[1].position = p2;
  } else {
    // direction unit‐vector for infinite line
    sf::Vector2f dir = p2 - p1;
    float len = std::hypot(dir.x, dir.y);
    if (len > 1e-6f) {
      dir /= len;
      const float EXT = 10000.f;
      sf::Vector2f mid = (p1 + p2) * 0.5f;
      m_sfmlShape[0].position = mid - dir * EXT;
      m_sfmlShape[1].position = mid + dir * EXT;
    } else {
      return;
    }
  }

  // apply color
  for (std::size_t i = 0; i < m_sfmlShape.getVertexCount(); ++i) {
    m_sfmlShape[i].color = m_color;
  }
  } catch (const std::exception &e) {
        return;
    } catch (...) {
        return;
    }
}

void Line::addConstraintObserver(std::shared_ptr<Line> observer) {
  if (observer) {
    // Check if observer already exists
    bool exists = false;
    for (const auto &weak_obs : m_constraintObservers) {
      if (auto obs = weak_obs.lock()) {
        if (obs.get() == observer.get()) {
          exists = true;
          break;
        }
      }
    }

    if (!exists) {
      m_constraintObservers.push_back(observer);
    }
  }
}

void Line::removeConstraintObserver(std::shared_ptr<Line> observer) {
  if (!observer) {
    return;
  }

  m_constraintObservers.erase(
      std::remove_if(m_constraintObservers.begin(), m_constraintObservers.end(),
                     [observer](const std::weak_ptr<Line> &weak_obs) {
                       auto obs = weak_obs.lock();
                       return !obs || obs.get() == observer.get();
                     }),
      m_constraintObservers.end());
}

void Line::clearConstraintReference(Line *disappearingRef) {
  if (auto ref_obj = m_constraintRefObject.lock()) {
    if (ref_obj.get() == disappearingRef) {
      m_isParallelLine = false;
      m_isPerpendicularLine = false;
      m_constraintRefObject.reset();
      m_constraintRefEdgeIndex = -1;
    }
  }
}

// Methods addChildPoint, removeChildPoint, updateHostedPoints, getHostedObjectPoints 
// are now inherited from GeometricObject
// removeChildPointByRawPointer is removed (use removeChildPoint generic)

void Line::attemptDataRepair() {
  // Attempt to repair the line if it is in an invalid state
  try {
    // First check if endpoints exist
    if (!m_startPoint || !m_endPoint) {
      std::cerr << "Line::attemptDataRepair: Missing endpoint(s), cannot repair" << std::endl;
      return;
    }

    // Check if endpoints are valid (can access their positions)
    Point_2 startPos, endPos;
    try {
      startPos = m_startPoint->getCGALPosition();
      endPos = m_endPoint->getCGALPosition();
    } catch (const std::exception &e) {
      std::cerr << "Line::attemptDataRepair: Error accessing endpoint positions: " << e.what()
                << std::endl;
      return;
    }

    // Handle coincident endpoints
    if (startPos == endPos) {
      std::cerr << "Line::attemptDataRepair: Coincident endpoints detected, "
                << "attempting fix" << std::endl;

      // Try horizontal offset first
      try {
        Point_2 newEndPos(startPos.x() + 10.0, startPos.y());
        m_endPoint->setCGALPosition(newEndPos);

        // Verify the fix worked
        if (m_endPoint->getCGALPosition() == startPos) {
          // If horizontal shift didn't work, try vertical shift
          newEndPos = Point_2(startPos.x(), startPos.y() + 10.0);
          m_endPoint->setCGALPosition(newEndPos);
        }

        // Update CGAL line and SFML representation
        updateCGALLine();
        updateSFMLShape();
        std::cout << "Line::attemptDataRepair: Successfully separated endpoints" << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "Line::attemptDataRepair: Failed to fix coincident endpoints: " << e.what()
                  << std::endl;
      }
    }

    // Make sure the line equation is valid
    try {
      if (m_cgalLine.is_degenerate()) {
        std::cerr << "Line::attemptDataRepair: Degenerate line detected, "
                  << "attempting fix" << std::endl;

        // Re-create the line with current endpoints
        m_cgalLine = Line_2(m_startPoint->getCGALPosition(), m_endPoint->getCGALPosition());

        // If still degenerate, force endpoints to be distinct
        if (m_cgalLine.is_degenerate()) {
          Point_2 startPos = m_startPoint->getCGALPosition();
          Point_2 newEndPos(startPos.x() + 15.0, startPos.y() + 15.0);
          m_endPoint->setCGALPosition(newEndPos);
          m_cgalLine = Line_2(startPos, newEndPos);
        }
      }
    } catch (const std::exception &e) {
      std::cerr << "Line::attemptDataRepair: Failed to fix line equation: " << e.what()
                << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Line::attemptDataRepair: Unhandled exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Line::attemptDataRepair: Unknown exception" << std::endl;
  }
}

void Line::clearEndpointIfMatches(Point *p) {
  if (!p) return;

  try {
    // Clear the endpoints if they match the given point
    if (m_startPoint && m_startPoint.get() == p) {
      m_startPoint.reset();
    }

    if (m_endPoint && m_endPoint.get() == p) {
      m_endPoint.reset();
    }

    // Don't trigger additional notifications during this cleanup
    // as that could lead to circular destruction crashes
  } catch (...) {
    // Suppress all exceptions during this emergency cleanup
  }
}
void Line::clearConstraintReference(std::shared_ptr<Line> lineBeingDestroyed) {
    try {
        if (!lineBeingDestroyed) return;
        
        std::cout << "Line::clearConstraintReference: Clearing reference to Line " 
                  << lineBeingDestroyed.get() << " from Line " << this << std::endl;
        
        // Check if the line being destroyed is our reference line
        if (auto currentRef = m_constraintRefObject.lock()) {
            if (currentRef == lineBeingDestroyed) {
                std::cout << "Clearing reference line relationship" << std::endl;
                m_constraintRefObject.reset();
                m_constraintRefEdgeIndex = -1;
                
                // Reset constraint state
                m_isParallelLine = false;
                m_isPerpendicularLine = false;
                
                // Keep the direction constraint for axis-based constraints
                // m_constraintDirection remains unchanged
            }
        }
        
        // Remove from our observers list
        auto it = std::remove_if(m_constraintObservers.begin(), m_constraintObservers.end(),
            [&lineBeingDestroyed](const std::weak_ptr<Line> &weak_observer) {
                if (auto observer = weak_observer.lock()) {
                    return observer == lineBeingDestroyed;
                }
                return true; // Remove expired weak_ptrs too
            });
        
        if (it != m_constraintObservers.end()) {
            m_constraintObservers.erase(it, m_constraintObservers.end());
            std::cout << "Removed destroyed line from constraint observers" << std::endl;
        }
        
    } catch (const std::exception &e) {
        std::cerr << "Error in clearConstraintReference: " << e.what() << std::endl;
    }
}
// void Line::clearEndpointIfMatches(Point *p) {
//   if (!p)
//     return; // Safety check

//   bool needsUpdate = false;

//   // Check if either endpoint matches and clear if it does
//   if (m_startPoint.get() == p) {
//     std::cerr << "Line::clearEndpointIfMatches: Cleared start point"
//               << std::endl;
//     m_startPoint = nullptr;
//     needsUpdate = true;
//   }

//   if (m_endPoint.get() == p) {
//     std::cerr << "Line::clearEndpointIfMatches: Cleared end point" <<
//     std::endl; m_endPoint = nullptr; needsUpdate = true;
//   }

//   if (needsUpdate) {
//     // Remove this line from any constraint relationships
//     // FIX: Check if m_referenceLine is NOT expired before locking it
//     if (!m_referenceLine.expired()) {
//       if (auto refLine = m_referenceLine.lock()) {
//         refLine->removeConstraintObserver(shared_from_this());
//       }
//       m_referenceLine.reset();
//     }

//     // Notify all lines that depend on this one
//     for (const auto &observer_weak : m_constraintObservers) {
//       if (auto observer = observer_weak.lock()) {
//         observer->clearConstraintReference(this);
//       }
//     }
//     m_constraintObservers.clear();

//     // Notify hosted object points
//     for (const auto &objPoint_weak : m_hostedObjectPoints) {
//       if (auto objPoint = objPoint_weak.lock()) {
//         objPoint->clearHost();
//       }
//     }
//     m_hostedObjectPoints.clear();
//   }
// }
void Line::setPoints(std::shared_ptr<Point> start, std::shared_ptr<Point> end) {
    if (!start || !end) {
        throw std::invalid_argument("Line::setPoints: Pointers cannot be null");
    }
    
    // Cleanup old connections if necessary (though registerWithEndpoints handles new ones)
    safeResetEndpoints(); // Clear old connections safely

    m_startPoint = start;
    m_endPoint = end;
    
    // CRITICAL: Register as listener
    registerWithEndpoints();

    updateCGALLine();
    updateSFMLShape();
}
