#pragma once
#include "CharTraitsFix.h" // For char_traits<unsigned int> specialization
                           // 
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>  // Include CGAL Kernel

#include <SFML/Graphics.hpp>  // Then SFML headers
#include <SFML/Graphics/Color.hpp>
#include <string>  // Include standard string first



// Define the Kernel type
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;

namespace Constants {

// --- Font Paths ---
// If your executable is in the project root (where arial.ttf is), this is fine.
// If your executable is in a 'build' subdirectory, and arial.ttf is in the
// project root, you might need "../arial.ttf". For now, assuming arial.ttf will
// be placed next to the executable.

// --- Font Paths ---
// TEMPORARY TEST: Use an absolute path. Adjust this path to where your
// arial.ttf ACTUALLY IS. Make sure to use double backslashes or a single
// forward slash. Example:
// "C:/Users/Mario_Geometry2/Desktop/Coding/Geometry_Tool_2/arial.ttf" OR:
// "C:\\Users\\Mario_Geometry2\\Desktop\\Coding\\Geometry_Tool_2\\arial.ttf"
//
// Original relative path:
// inline const std::string DEFAULT_FONT_PATH = "arial.ttf";
//
// Replace with your actual absolute path for testing:
#ifdef _WIN32
const std::string DEFAULT_FONT_PATH =
    "C:/Windows/Fonts/arial.ttf";
    //"C:/Users/Mario_Geometry2/Desktop/Coding/Geometry_Tool_2/arial.ttf";
#else
const std::string DEFAULT_FONT_PATH =
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf";
#endif

// --- Window & View ---
inline unsigned int WINDOW_WIDTH = sf::VideoMode::getDesktopMode().width;
inline unsigned int WINDOW_HEIGHT = sf::VideoMode::getDesktopMode().height;
const sf::Color BACKGROUND_COLOR = sf::Color(255, 255, 255);  // White background
inline constexpr float ZOOM_FACTOR = 1.1f;
const float PAN_SPEED = 100.0f;        // Consider making this pixels per second or similar
const float MIN_PAN_BOUND = -2000.0f;  // Example pan boundary
const float MAX_PAN_BOUND = 2000.0f;   // Example pan boundary
const float MIN_VISIBLE_SIZE = 0.5f;   // Minimum visible size in world units for culling/LOD

// --- Grid & Axes ---
constexpr float GRID_SIZE = 30.0f;                           // Base size for grid cells
const sf::Color GRID_COLOR = sf::Color(200, 200, 200, 150);  // Light grey, semi-transparent
const sf::Color GRID_AXIS_COLOR = sf::Color(0, 0, 0, 200);   // Darker grey for axes
constexpr unsigned int GRID_LABEL_FONT_SIZE = 18;
const sf::Color AXIS_LABEL_COLOR = sf::Color(0, 0, 0, 255);  // Solid black for visibility
const sf::Color LOCKED_COLOR = sf::Color(128, 0, 128, 200);  // Changed to const

// --- Fonts ---
// inline const std::string DEFAULT_FONT_PATH =
//     "arial.ttf"; // Ensure path is correct relative to executable

// --- General Colors ---
const sf::Color PREVIEW_COLOR = sf::Color(100, 100, 100, 128);  // General preview
const sf::Color SELECTION_UNIVERSAL_COLOR =
    sf::Color(255, 165, 0,
              180);  // Orange, semi-transparent for general selection highlight
const sf::Color HOVER_UNIVERSAL_COLOR =
    sf::Color(0, 255, 255, 180);  // Cyan, semi-transparent for general hover

// --- Object Visuals & Interaction ---
// (General outline thicknesses for selected/hovered objects)
const float SELECTION_OUTLINE_THICKNESS = 1.5f;
const float HOVER_OUTLINE_THICKNESS = 1.5f;

// --- Point Visuals & Interaction ---
constexpr float POINT_RADIUS = 5.0f;  // Make points larger for visibility
constexpr float POINT_DRAW_RADIUS_SCREEN_PIXELS = 5.0f;
constexpr float OBJECT_POINT_RADIUS = 4.0f;      // Slightly smaller for object points
constexpr float POINT_OUTLINE_THICKNESS = 1.0f;  // Thickness of point outlines
const sf::Color POINT_DEFAULT_COLOR = sf::Color(0, 0, 0);
const sf::Color POINT_FILL_COLOR = sf::Color(255, 0, 0);  // Red for better visibility
constexpr float POINT_RADIUS_SELECTED = 7.0f;             // Selected point radius

const sf::Color LINE_DEFAULT_COLOR = sf::Color(0, 0, 255);            // Blue for lines
constexpr float LINE_THICKNESS_DEFAULT = 2.0f; // Scale invariant thickness
const sf::Color CONSTRUCTION_LINE_COLOR = sf::Color(0, 0, 255, 150);  // Blue
const sf::Color CIRCLE_DEFAULT_COLOR = sf::Color(255, 105, 180);      // Hot Pink
const sf::Color OBJECT_POINT_DEFAULT_COLOR = sf::Color(255, 165, 0);  // Orange
const sf::Color OBJECT_POINT_OUTLINE_COLOR = sf::Color(50, 75, 200);

const float OBJECT_POINT_INTERACTION_PADDING = 2.0f;
const sf::Color INTERSECTION_POINT_COLOR = sf::Color(255, 0, 255);  // Magenta
const sf::Color POINT_INTERSECTION_COLOR = sf::Color(255, 0, 255);  // Alias for code compatibility
const sf::Color SELECTION_COLOR = sf::Color::Red;
const sf::Color HOVER_COLOR = sf::Color(0, 255, 0, 100);  // Semi-transparent Green
const float GUI_MESSAGE_POSITION = 10.0f;                 // Example position for GUI messages
const unsigned int GUI_MESSAGE_FONT_SIZE = 16;            // Message font size
const sf::Color GUI_MESSAGE_COLOR = sf::Color::Black;

// ============================================================================
// SCREEN-PIXEL INTERACTION CONSTANTS
// ============================================================================
// These define how many SCREEN PIXELS are used for various user interactions.
// They should be converted to WORLD UNITS at runtime using:
//   worldTolerance = SCREEN_PIXELS * (viewWidth / windowWidth)
//
// Use getDynamicSelectionTolerance() or getDynamicSnapTolerance() in
// HandleEvents.cpp to get properly scaled world-unit tolerances.
// ============================================================================

constexpr float SELECTION_SCREEN_PIXELS = 7.0f;      // Click/selection precision
constexpr float SNAP_SCREEN_PIXELS = 12.0f;          // Snapping "magnetism" radius
constexpr float HOVER_SCREEN_PIXELS = 10.0f;         // Hover detection radius
constexpr float DRAG_THRESHOLD_PIXELS = 5.0f;        // Drag initiation threshold

// ============================================================================
// DEPRECATED: Old world-unit constants (for backwards compatibility)
// Prefer using getDynamicSelectionTolerance() instead of these constants
// ============================================================================

// [DEPRECATED] Use getDynamicSelectionTolerance() instead
const float POINT_INTERACTION_RADIUS = 10.0f;  // Legacy: was used for point selection
const float POINT_SIZE = 5.0f;                 // Used for scaled drawing (still valid)

// Line specific visual constants
const float LINE_THICKNESS = 1.0f;
const float CONSTRUCTION_LINE_THICKNESS = 1.0f;  // Thinner for construction lines
// [DEPRECATED] Use getDynamicSelectionTolerance() instead  
const float LINE_INTERACTION_RADIUS = 5.0f;      // Legacy: was for clicking on lines
const float GRID_SNAP_INTERVAL = 2.0f;           // Snapping interval for grid alignment
const float LINE_ARROWHEAD_SIZE = 10.0f;         // Size of arrowheads on lines
const float LINE_DASH_LENGTH = 5.0f;             // Length of dashes in dashed lines
const float LINE_DASH_GAP = 3.0f;                // Gap between dashes in dashed lines

// Add this epsilon constant for small floating-point comparisons
const double EPSILON = 0.00001;  // Small value for floating-point comparisons
const double CGAL_EPSILON_SQUARED = 1e-18;
const double EPSILON_LENGTH_CONSTRUCTION = 1e-9;  // Length tolerance for
/* const float DEFAULT_PERPENDICULAR_LINE_LENGTH =
    50.0f; // Example default length, adjust as needed
const Kernel::FT DEFAULT_PERPENDICULAR_LINE_LENGTH_CGAL(50.0); // CGAL version
if needed elsewhere */
static constexpr double DEFAULT_LINE_CONSTRUCTION_LENGTH = 1000.0;
// Circle specific visual constants
const sf::Color CIRCLE_DEFAULT_FILL_COLOR = sf::Color::Transparent;  // Default transparent fill
// You could change it to something like:
// const sf::Color CIRCLE_DEFAULT_FILL_COLOR = sf::Color(255, 105, 180, 100); //
// Semi-transparent Hot Pink

const sf::Color CIRCLE_OUTLINE_COLOR =
    sf::Color(0, 100, 100);  // Example: Darker teal for non-selected, non-hovered outline
const float CIRCLE_OUTLINE_THICKNESS = 2.0f;    // Example: Default outline thickness for circles
const float CIRCLE_PERIMETER_TOLERANCE = 5.0f;  // Tolerance for clicking circle perimeter (pixels
                                                // or world units depending on use)
const float MIN_CIRCLE_RADIUS = 5.0f;           // Minimum radius for a circle
const bool ALWAYS_SHOW_CIRCLE_CENTERS = true;   // Always show circle centers

// Missing constants needed for selection box

const sf::Color SELECTION_BOX_FILL_COLOR = sf::Color(255, 255, 0, 50);  // Yellow with transparency
const sf::Color SELECTION_BOX_OUTLINE_COLOR = sf::Color::Yellow;
const float SELECTION_BOX_OUTLINE_THICKNESS = 1.0f;

// ADD THESE New circle-specific selection and hover constants
const sf::Color SELECTION_COLOR_CIRCLE_FILL =
    sf::Color(SELECTION_UNIVERSAL_COLOR.r, SELECTION_UNIVERSAL_COLOR.g, SELECTION_UNIVERSAL_COLOR.b,
              100);  // Semi-transparent selection fill
const sf::Color SELECTION_COLOR_CIRCLE_OUTLINE = SELECTION_UNIVERSAL_COLOR;
const float SELECTION_THICKNESS_CIRCLE = 2.0f;
const sf::Color HOVER_COLOR_CIRCLE_OUTLINE = HOVER_UNIVERSAL_COLOR;
const float HOVER_THICKNESS_CIRCLE = 1.5f;

// --- GUI ---
const sf::Color GUI_BACKGROUND_COLOR =
    sf::Color(255, 255, 255, 255);  // Dark gray with transparency
const float GUI_HEIGHT = 50.0f;     // Height of the GUI panel
const sf::Vector2f BUTTON_SIZE(100.f, 35.f);
const sf::Color BUTTON_DEFAULT_COLOR = sf::Color(200, 200, 200);
const sf::Color BUTTON_ACTIVE_COLOR = sf::Color(100, 100, 255, 200);    // Blueish, active tool
const sf::Color BUTTON_INACTIVE_COLOR = sf::Color(180, 180, 180, 200);  // Greyish, inactive
const sf::Color BUTTON_HOVER_COLOR = sf::Color(150, 150, 220, 220);     // Lighter blueish for hover
inline unsigned int BUTTON_TEXT_SIZE = 24;              // Helper for adjustable
const sf::Color BUTTON_TEXT_COLOR = sf::Color::Black;  // Example value
inline unsigned int GUI_MESSAGE_TEXT_SIZE = 18;         // Helper for adjustable
const float GUI_MESSAGE_DURATION = 3.0f;               // Seconds, example value

// --- General Interaction & Snapping ---
const float MOUSE_OVER_TOLERANCE = 5.0f;    // Tolerance in screen pixels
const float COORDINATE_PRECISION = 1e-6f;   // For comparing CGAL coordinates
const float MIN_GRID_SPACING_WORLD = 5.0f;  // Minimum grid spacing in world units
const int GRID_LABEL_OFFSET = 5;            // Offset for grid labels in pixels
// Magnet snapping multiplier: expands tolerance for snapping to existing points during creation
inline constexpr float SNAP_MAGNET_RADIUS_MULTIPLIER = 2.5f;

// -- ObjectPoint-specific constants
// (The duplicate definition of OBJECT_POINT_DEFAULT_COLOR was here and has been
// removed)

// -- Selection/Hover colors for different object types
const sf::Color SELECTION_COLOR_POINT_FILL = sf::Color(150, 150, 70);    // Yellow
const sf::Color SELECTION_COLOR_POINT_OUTLINE = sf::Color(255, 165, 0);  // Orange
const float SELECTION_THICKNESS_POINT = 2.0f;
const sf::Color HOVER_COLOR_POINT_OUTLINE = sf::Color(0, 255, 255);  // Cyan
const float HOVER_THICKNESS_POINT = 1.5f;

// -- Circle-specific constants
const float CIRCLE_CENTER_VISUAL_RADIUS = 3.0f;
const sf::Color CIRCLE_CENTER_VISUAL_COLOR = sf::Color(255, 255, 255);
const float CIRCLE_ALPHA = 30;  // Transparency for circle fills


// -- Coordinate precision for intersection points
// (The duplicate definition of COORDINATE_PRECISION was here and has been
// removed)

// -- Background color
// (The duplicate definition of BACKGROUND_COLOR was here and has been
// removed)

// --- Deprecated / To Review (Consolidated from previous versions) ---
// Remove or ensure these are not conflicting with the above definitions.
// For example, if SELECTION_UNIVERSAL_COLOR is used, other generic
// SELECTION_COLOR constants might be redundant. (The specific deprecated line
// 'const sf::Color SELECTION_COLOR = sf::Color(255, 255, 0, 128);' was here and
// has been removed)
// ... other deprecated constants ...

// Add these if you want more specific names for circle interactions
// const float CIRCLE_CENTER_HANDLE_RADIUS = 5.0f; // Could be same as
// POINT_INTERACTION_RADIUS const float CIRCLE_EDGE_INTERACTION_TOLERANCE
// = 5.0f; // Could be same as LINE_INTERACTION_RADIUS

// Add this line among other constants:
inline constexpr bool LIFECYCLE = false;
inline constexpr bool DEBUG_LINE_DRAWING = false;  // Set to true only when needed
inline constexpr bool DEBUG_POINT_DRAWING = false;
inline constexpr bool DEBUG_POINT_UPDATE = false;
inline constexpr bool DEBUG_CGAL_POINT = false;
inline constexpr bool DEBUG_CIRCLE_DRAWING = false;
inline constexpr bool DEBUG_OBJECT_POINT_DRAWING = false;
inline constexpr bool DEBUG_OBJECT_POINT_UPDATE = false;
inline constexpr bool DEBUG_GRID_DRAWING = false;
inline constexpr bool DEBUG_GEOMETRY_UPDATES = false;
inline constexpr bool DEBUG_CONSTRAINTS = false;
inline constexpr bool DEBUG_DRAGGING = false;
inline constexpr bool DEBUG_SELECTION = false;
inline constexpr bool DEBUG_DELETION = false;
inline constexpr bool DEBUG_OBJECT_CREATION = false;
inline constexpr bool DEBUG_LINE_CREATION = false;  // New flag for debugging line creation

// Add this new constant for circle center point color if it doesn't exist
inline const sf::Color CIRCLE_CENTER_COLOR = sf::Color(255, 165, 0, 255);  // Orange for visibility

// Add the missing constant for circle interaction radius
inline const float CIRCLE_INTERACTION_RADIUS = 5.0f;  // Tolerance for clicking on circle elements

// --- Zoom Constants ---
extern float CURRENT_ZOOM;  // This should not be a global constant, but tracked
                            // by GeometryEditor or view state
// REMOVE or COMMENT OUT the old MIN_ZOOM and MAX_ZOOM:
// inline float MIN_ZOOM = 5.0f; // Minimum zoom level -- OLD, PROBLEMATIC
// inline float MAX_ZOOM = 500.0f; // Maximum zoom level -- OLD, PROBLEMATIC

// REPLACE WITH THESE:
inline constexpr float VIEW_MIN_ZOOM_LEVEL =
    0.1f;  // Max zoom-in (e.g., view is 10% of original height)
inline constexpr float VIEW_MAX_ZOOM_LEVEL =
    15.0f;  // Max zoom-out (e.g., view is 1500% of original height)
inline constexpr float MOUSE_WHEEL_ZOOM_FACTOR =
    1.1f;  // Factor for each scroll tick (was ZOOM_FACTOR)

const float MIN_CENTER_POINT_VISUAL_RADIUS = 3.0f;  // Minimum size for center point visual
const float MAX_CENTER_POINT_VISUAL_RADIUS =
    10.0f;  // Maximum size for center point visual
            // Default length for constructing lines like perpendiculars

// Debug flags

const double MIN_DISTANCE_SQUARED_LINE_CREATION =
    1.0;  // Or a smaller value like 1.0e-6 if very close points are allowed
}  // namespace Constants