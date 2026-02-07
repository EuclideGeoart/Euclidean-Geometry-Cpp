#pragma once

#include <memory>
#include "GeometryEditor.h"
#include "Point.h"
#include "Line.h"

/// @brief Utility functions for checking geometric constraints
namespace ConstraintUtils {

/// @brief Check if a point is an endpoint of a constrained line (parallel/perpendicular)
/// @details Rectangles and other shapes should NOT reuse such points to avoid breaking constraints.
///          This prevents the "phantom vertex capture" bug where rectangles steal constrained points.
/// @param editor Reference to the geometry editor
/// @param point Point to check
/// @return true if point belongs to a constrained line, false otherwise
bool isPointConstrainedByLine(GeometryEditor& editor, const std::shared_ptr<Point>& point);

} // namespace ConstraintUtils
