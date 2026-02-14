#include "ConstraintUtils.h"
#include "Line.h"
#include "Point.h"
#include "GeometryEditor.h"
#include <iostream>
#include <cmath>

namespace ConstraintUtils {

bool isPointConstrainedByLine(GeometryEditor& editor, const std::shared_ptr<Point>& point) {
  if (!point) {
    std::cout << "[ConstraintUtils] Point is null, returning false" << std::endl;
    return false;
  }
  
  if (point->getType() == ObjectType::ObjectPoint || point->getType() == ObjectType::IntersectionPoint) {
    std::cout << "[ConstraintUtils] Point is an ObjectPoint or IntersectionPoint, returning true" << std::endl;
    return true;
  }

  Point* rawPtr = point.get();
  auto pointType = point->getType();
  auto pointPos = point->getCGALPosition();
  std::cout << "[ConstraintUtils] Checking if point " << rawPtr << " (type=" << static_cast<int>(pointType) 
            << ", pos=" << pointPos << ") is constrained" << std::endl;
  std::cout << "[ConstraintUtils] Total lines to check: " << editor.lines.size() << std::endl;
  
  // Get snap tolerance for distance-based comparison
  // This needs to be large enough to catch points created by snapping
  // Based on testing, snapping can occur from 10+ world units away
  const double snapTolerance = 20.0; // World units - covers typical snap range
  
  // Check all lines to see if this point is related to a constrained line
  for (const auto& line : editor.lines) {
    if (!line || !line->isValid()) continue;
    
    // Check if line has constraints (parallel or perpendicular)
    if (line->isParallelLine() || line->isPerpendicularLine()) {
      std::cout << "[ConstraintUtils] Found constrained line (parallel=" << line->isParallelLine() 
                << ", perpendicular=" << line->isPerpendicularLine() << ")" << std::endl;
      
      // Check #1: Is this point one of the line's endpoints (by pointer)?
      auto startPt = line->getStartPointObjectShared();
      auto endPt = line->getEndPointObjectShared();
      
      std::cout << "[ConstraintUtils] Line endpoints: start=" << startPt.get();
      if (startPt) std::cout << " (pos=" << startPt->getCGALPosition() << ")";
      std::cout << " end=" << endPt.get();
      if (endPt) std::cout << " (pos=" << endPt->getCGALPosition() << ")";
      std::cout << " (checking against " << rawPtr << " at " << pointPos << ")" << std::endl;
      
      if ((startPt && startPt.get() == rawPtr) || (endPt && endPt.get() == rawPtr)) {
        std::cout << "[ConstraintUtils] *** MATCH! Point IS an endpoint of constrained line (by pointer) ***" << std::endl;
        return true;
      }
      
      // Check #2: DISABLED - Distance-based checking was too aggressive
      // A free point near a constrained line should still be reusable!
      // Only exact endpoint matches (Check #1) should prevent reuse.
      /*
      if (startPt) {
        auto startPos = startPt->getCGALPosition();
        auto dx = CGAL::to_double(pointPos.x() - startPos.x());
        auto dy = CGAL::to_double(pointPos.y() - startPos.y());
        auto distSquared = dx * dx + dy * dy;
        auto snapToleranceSquared = snapTolerance * snapTolerance;
        
        std::cout << "[ConstraintUtils] Distance to START: " << std::sqrt(distSquared) << " (tolerance: " << snapTolerance << ")" << std::endl;
        
        if (distSquared < snapToleranceSquared) {
          std::cout << "[ConstraintUtils] *** MATCH! Point is NEAR constrained line START (within snap tolerance) ***" << std::endl;
          return true;
        }
      }
      */
      
      /*
      if (endPt) {
        auto endPos = endPt->getCGALPosition();
        auto dx = CGAL::to_double(pointPos.x() - endPos.x());
        auto dy = CGAL::to_double(pointPos.y() - endPos.y());
        auto distSquared = dx * dx + dy * dy;
        auto snapToleranceSquared = snapTolerance * snapTolerance;
        
        std::cout << "[ConstraintUtils] Distance to END: " << std::sqrt(distSquared) << " (tolerance: " << snapTolerance << ")" << std::endl;
        
        if (distSquared < snapToleranceSquared) {
          std::cout << "[ConstraintUtils] *** MATCH! Point is NEAR constrained line END (within snap tolerance) ***" << std::endl;
          return true;
        }
      }
      */
      
      // Check #3: Is this point an ObjectPoint hosted by this constrained line?
      auto hostedPoints = line->getHostedObjectPoints();
      std::cout << "[ConstraintUtils] Checking " << hostedPoints.size() << " hosted ObjectPoints..." << std::endl;
      for (const auto& weakObjPt : hostedPoints) {
        if (auto objPt = weakObjPt.lock()) {
          std::cout << "[ConstraintUtils]   Hosted ObjectPoint: " << objPt.get() << std::endl;
          if (objPt.get() == rawPtr) {
            std::cout << "[ConstraintUtils] *** MATCH! Point IS hosted by constrained line (ObjectPoint) ***" << std::endl;
            return true;
          }
        }
      }
    }
  }
  
  std::cout << "[ConstraintUtils] Point is NOT constrained, safe to reuse" << std::endl;
  return false;
}

} // namespace ConstraintUtils
