/**
 * ProjectSerializer.cpp
 *
 * Implements project save/load (JSON) and SVG export.
 *
 * IMPORTANT: This implementation uses nlohmann/json library.
 * Download json.hpp from: https://github.com/nlohmann/json/releases
 * Place it in the project directory or include path.
 */

#include "ProjectSerializer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <unordered_set>
#include <sstream>

#include "Angle.h"
#include "Circle.h"
#include "Constants.h"
#include "ConstructionObjects.h"
#include "Deserializer.h"
#include "GeometryEditor.h"
#include "Intersection.h"
#include "IntersectionPoint.h"
#include "Line.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "Polygon.h"
#include "Rectangle.h"
#include "RegularPolygon.h"
#include "SVGWriter.h"
#include "TransformationObjects.h"
#include "Triangle.h"

// Include nlohmann/json - user must download this
// Download from: https://github.com/nlohmann/json/releases/latest
// Place json.hpp in the project directory
#include "json.hpp"

using json = nlohmann::json;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

std::string ProjectSerializer::colorToHex(const sf::Color& color) {
  std::stringstream ss;
  ss << "#" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(color.r) << std::setw(2) << static_cast<int>(color.g) << std::setw(2)
     << static_cast<int>(color.b);
  return ss.str();
}

sf::Color ProjectSerializer::hexToColor(const std::string& hex) {
  if (hex.empty() || hex[0] != '#' || hex.length() < 7) {
    return sf::Color::Blue;  // Default
  }

  unsigned int r, g, b;
  std::stringstream ss;
  ss << std::hex << hex.substr(1, 2);
  ss >> r;
  ss.clear();
  ss << std::hex << hex.substr(3, 2);
  ss >> g;
  ss.clear();
  ss << std::hex << hex.substr(5, 2);
  ss >> b;

  return sf::Color(static_cast<sf::Uint8>(r), static_cast<sf::Uint8>(g), static_cast<sf::Uint8>(b));
}

void ProjectSerializer::worldToSVG(double worldX, double worldY, double& svgX, double& svgY, double viewHeight, double minY) {
  // WYSIWYG Export: Direct mapping to ViewBox coordinates
  // We assume the ViewBox matches the World Coordinates exactly.
  svgX = worldX;
  svgY = worldY;
}

void ProjectSerializer::calculateBounds(const GeometryEditor& editor, double& minX, double& minY, double& maxX, double& maxY) {
  minX = minY = std::numeric_limits<double>::max();
  maxX = maxY = std::numeric_limits<double>::lowest();

  bool hasObjects = false;

  // Check all points
  for (const auto& pt : editor.points) {
    if (pt && pt->isValid()) {
      Point_2 pos = pt->getCGALPosition();
      double x = CGAL::to_double(pos.x());
      double y = CGAL::to_double(pos.y());
      minX = std::min(minX, x);
      minY = std::min(minY, y);
      maxX = std::max(maxX, x);
      maxY = std::max(maxY, y);
      hasObjects = true;
    }
  }

  // Check all ObjectPoints
  for (const auto& op : editor.ObjectPoints) {
    if (op && op->isValid()) {
      Point_2 pos = op->getCGALPosition();
      double x = CGAL::to_double(pos.x());
      double y = CGAL::to_double(pos.y());
      minX = std::min(minX, x);
      minY = std::min(minY, y);
      maxX = std::max(maxX, x);
      maxY = std::max(maxY, y);
      hasObjects = true;
    }
  }

  // Check all lines (using endpoints for segments)
  for (const auto& ln : editor.lines) {
    if (ln && ln->isValid()) {
      Point_2 start = ln->getStartPoint();
      Point_2 end = ln->getEndPoint();

      double x1 = CGAL::to_double(start.x());
      double y1 = CGAL::to_double(start.y());
      double x2 = CGAL::to_double(end.x());
      double y2 = CGAL::to_double(end.y());

      minX = std::min({minX, x1, x2});
      minY = std::min({minY, y1, y2});
      maxX = std::max({maxX, x1, x2});
      maxY = std::max({maxY, y1, y2});
      hasObjects = true;
    }
  }

  // Check all circles
  for (const auto& ci : editor.circles) {
    if (ci && ci->isValid()) {
      Point_2 center = ci->getCGALPosition();
      double radius = ci->getRadius();
      double cx = CGAL::to_double(center.x());
      double cy = CGAL::to_double(center.y());

      minX = std::min(minX, cx - radius);
      minY = std::min(minY, cy - radius);
      maxX = std::max(maxX, cx + radius);
      maxY = std::max(maxY, cy + radius);
      hasObjects = true;
    }
  }

  // Check all Polygons (in case they have vertices not in editor.points)
  for (const auto& poly : editor.polygons) {
    if (poly && poly->isValid()) {
      auto vertices = poly->getVertices();
      for (const auto& v : vertices) {
        double x = CGAL::to_double(v.x());
        double y = CGAL::to_double(v.y());
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
        hasObjects = true;
      }
    }
  }

  // Check all Rectangles
  for (const auto& rect : editor.rectangles) {
    if (rect && rect->isValid()) {
      auto vertices = rect->getInteractableVertices();
      for (const auto& v : vertices) {
        double x = CGAL::to_double(v.x());
        double y = CGAL::to_double(v.y());
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
        hasObjects = true;
      }
    }
  }

  // Check triangles
  for (const auto& tri : editor.triangles) {
    if (tri && tri->isValid()) {
      auto vertices = tri->getInteractableVertices();
      for (const auto& v : vertices) {
        double x = CGAL::to_double(v.x());
        double y = CGAL::to_double(v.y());
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
      }
      hasObjects = true;
    }
  }

  // Check regular polygons
  for (const auto& rpoly : editor.regularPolygons) {
    if (rpoly && rpoly->isValid()) {
      auto vertices = rpoly->getInteractableVertices();
      for (const auto& v : vertices) {
        double x = CGAL::to_double(v.x());
        double y = CGAL::to_double(v.y());
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
      }
      hasObjects = true;
    }
  }

  // Include axes in bounds when visible
  if (editor.areAxesVisible()) {
    minX = std::min(minX, 0.0);
    maxX = std::max(maxX, 0.0);
    minY = std::min(minY, 0.0);
    maxY = std::max(maxY, 0.0);
    hasObjects = true;
  }

  // Default bounds if no objects
  if (!hasObjects) {
    minX = -100;
    minY = -100;
    maxX = 100;
    maxY = 100;
  }

  // Add 10% padding
  double padX = (maxX - minX) * 0.1;
  double padY = (maxY - minY) * 0.1;
  minX -= padX;
  minY -= padY;
  maxX += padX;
  maxY += padY;
}

// ============================================================================
// SAVE PROJECT
// ============================================================================

bool ProjectSerializer::saveProject(const GeometryEditor& editor, const std::string& filepath) {
  // Helper to add transformation metadata
  auto addTransformMetadata = [](json& jObj, const std::shared_ptr<GeometricObject>& obj) {
    if (!obj) return;
    jObj["isDependent"] = obj->isDependent();
    jObj["transformType"] = static_cast<int>(obj->getTransformType());
    if (obj->getTransformType() != TransformationType::None) {
      jObj["parentSourceID"] = obj->getParentSourceID();
      jObj["auxObjectID"] = obj->getAuxObjectID();
      jObj["transformValue"] = obj->getTransformValue();

      if (obj->getTransformType() == TransformationType::Translate) {
        Vector_2 v = obj->getTranslationVector();
        jObj["translationVectorX"] = CGAL::to_double(v.x());
        jObj["translationVectorY"] = CGAL::to_double(v.y());
      }
    }
  };

  try {
    json project;
    project["version"] = "1.0";
    project["objects"] = json::object();
    project["settings"] = json::object();
    project["settings"]["axesVisible"] = editor.areAxesVisible();

    // ------------------------------------------------------------
    // PASS 1: POINTS (Deep Harvesting & Merge Strategy)
    // ------------------------------------------------------------
    json pointsJson = json::array();
    
    // 1. COLLECT ALL UNIQUE POINTS
    // We use a Set to track IDs to avoid duplicates, and a Vector to preserve order.
    std::vector<std::shared_ptr<Point>> pointsToSave;
    std::set<unsigned int> savedPointIDs;

    auto markForSave = [&](std::shared_ptr<Point> p) {
        if (p && savedPointIDs.find(p->getID()) == savedPointIDs.end()) {
            savedPointIDs.insert(p->getID());
            pointsToSave.push_back(p);
        }
    };

    // A. Harvest from Main Editor List
    for (const auto& pt : editor.points) markForSave(pt);

    // B. Harvest from Rectangles (Crucial for "Ghost" Fix)
    for (const auto& r : editor.rectangles) {
        if (!r) continue;
        markForSave(r->getCorner1Point());
        markForSave(r->getCornerBPoint());
        markForSave(r->getCorner2Point());
        markForSave(r->getCornerDPoint());
    }

    // C. Harvest from Polygons
    for (const auto& poly : editor.polygons) {
        if (!poly) continue;
        for (size_t i = 0; i < poly->getVertexCount(); ++i) {
            markForSave(poly->getVertexPoint(i));
        }
    }

    // D. Harvest from Triangles
    for (const auto& tri : editor.triangles) {
        if (!tri) continue;
        for (size_t i = 0; i < 3; ++i) {
            markForSave(tri->getVertexPoint(i));
        }
    }

    // E. Harvest ObjectPoints (CRITICAL FIX: Include constrained points!)
    for (const auto& op : editor.ObjectPoints) {
        markForSave(op);
    }

    // 2. SERIALIZE THE UNIFIED LIST
    for (const auto& pt : pointsToSave) {
        if (!pt) continue;
        json jPt;
        jPt["id"] = pt->getID();
        Point_2 pos = pt->getCGALPosition();
        jPt["x"] = CGAL::to_double(pos.x());
        jPt["y"] = CGAL::to_double(pos.y());
        jPt["color"] = colorToHex(pt->getColor());
        jPt["visible"] = pt->isVisible();
        jPt["label"] = pt->getLabel();
        jPt["showLabel"] = pt->getShowLabel();
        sf::Vector2f offset = pt->getLabelOffset();
        jPt["labelOffsetX"] = offset.x; // Explicit separate fields per request
        jPt["labelOffsetY"] = offset.y;
        jPt["locked"] = pt->isLocked();
        jPt["isIntersectionPoint"] = pt->isIntersectionPoint();

        // Save Transformation Metadata (Essential for Transformed Shapes)
        if (pt->getTransformType() != TransformationType::None) {
            json t;
            t["transformType"] = static_cast<int>(pt->getTransformType());
            
            if (auto p = pt->getParentSource()) t["parentSourceID"] = p->getID();
            if (auto aux = pt->getAuxObject()) t["auxObjectID"] = aux->getID();
            
            // TRANSLATION VECTOR FIX
            if (pt->getTransformType() == TransformationType::Translate) {
                Vector_2 v = pt->getTranslationVector();
                t["translationVectorX"] = CGAL::to_double(v.x());
                t["translationVectorY"] = CGAL::to_double(v.y());
            }

            t["transformValue"] = pt->getTransformValue();
            t["isDependent"] = pt->isDependent();
            
            // Legacy Type Identification (For 3-Pass Loader compatibility)
            if (std::dynamic_pointer_cast<ReflectLine>(pt)) t["type"] = "ReflectLine";
            else if (std::dynamic_pointer_cast<ReflectPoint>(pt)) t["type"] = "ReflectPoint";
            else if (std::dynamic_pointer_cast<TranslateVector>(pt)) t["type"] = "TranslateVector";
            else if (std::dynamic_pointer_cast<RotatePoint>(pt)) t["type"] = "RotatePoint";
            else if (std::dynamic_pointer_cast<DilatePoint>(pt)) t["type"] = "DilatePoint";

            jPt["transform"] = t;
        }

        // CRITICAL FIX: Save ObjectPoint Metadata (for constrained points)
        if (auto op = std::dynamic_pointer_cast<ObjectPoint>(pt)) {
            jPt["isObjectPoint"] = true;
            jPt["hostType"] = static_cast<int>(op->getHostType());
            
            if (op->isShapeEdgeAttachment()) {
                jPt["edgeIndex"] = op->getEdgeIndex();
                jPt["edgeRel"] = op->getEdgeRelativePosition();
                jPt["t"] = op->getEdgeRelativePosition();
            } else if (op->getHostType() == ObjectType::Circle) {
                jPt["t"] = op->getAngleOnCircle();
            } else if (op->getHostType() == ObjectType::Line || op->getHostType() == ObjectType::LineSegment) {
                jPt["t"] = op->getRelativePositionOnLine();
            }
            
            if (auto host = op->getHostObject()) {
                jPt["hostId"] = host->getID();
            }
        }

        pointsJson.push_back(jPt);
    }
    project["objects"]["points"] = pointsJson;

    // Save Lines
    json linesArray = json::array();
    for (const auto& ln : editor.lines) {
      if (ln && ln->isValid()) {
        if (editor.getXAxisShared() && ln == editor.getXAxisShared()) continue;
        if (editor.getYAxisShared() && ln == editor.getYAxisShared()) continue;
        if (std::dynamic_pointer_cast<TangentLine>(ln)) continue;
        json lnJson;
        lnJson["id"] = ln->getID();
        lnJson["isSegment"] = ln->isSegment();
        lnJson["color"] = colorToHex(ln->getColor());
        lnJson["thickness"] = ln->getThickness();
        lnJson["decoration"] = static_cast<int>(ln->getDecoration());
        lnJson["lineStyle"] = static_cast<int>(ln->getLineStyle());
        lnJson["lineType"] = static_cast<int>(ln->getLineType());

        addTransformMetadata(lnJson, ln);

        if (auto startObj = ln->getStartPointObject(); startObj && startObj->getID() > 0 && savedPointIDs.find(startObj->getID()) != savedPointIDs.end()) {
          lnJson["startPointID"] = startObj->getID();
        } else {
          Point_2 start = ln->getStartPoint();
          lnJson["startX"] = CGAL::to_double(start.x());
          lnJson["startY"] = CGAL::to_double(start.y());
        }
        if (auto endObj = ln->getEndPointObject(); endObj && endObj->getID() > 0 && savedPointIDs.find(endObj->getID()) != savedPointIDs.end()) {
          lnJson["endPointID"] = endObj->getID();
        } else {
          Point_2 end = ln->getEndPoint();
          lnJson["endX"] = CGAL::to_double(end.x());
          lnJson["endY"] = CGAL::to_double(end.y());
        }

        if (auto pb = std::dynamic_pointer_cast<PerpendicularBisector>(ln)) {
          lnJson["constructionType"] = "PerpendicularBisector";
          if (auto p1 = pb->getFirstParentPoint()) lnJson["pbP1ID"] = p1->getID();
          if (auto p2 = pb->getSecondParentPoint()) lnJson["pbP2ID"] = p2->getID();
        } else if (auto ab = std::dynamic_pointer_cast<AngleBisector>(ln)) {
          lnJson["constructionType"] = "AngleBisector";
          lnJson["isExternalBisector"] = ab->isExternalBisector();
          if (ab->usesLineParents()) {
            lnJson["abMode"] = "lines";
            if (auto l1 = ab->getFirstParentLine()) lnJson["abLine1ID"] = l1->getID();
            if (auto l2 = ab->getSecondParentLine()) lnJson["abLine2ID"] = l2->getID();
          } else {
            lnJson["abMode"] = "points";
            if (auto v = ab->getVertexParentPoint()) lnJson["abVertexID"] = v->getID();
            if (auto a = ab->getFirstArmParentPoint()) lnJson["abArm1ID"] = a->getID();
            if (auto b = ab->getSecondArmParentPoint()) lnJson["abArm2ID"] = b->getID();
          }
        }

        lnJson["isParallel"] = ln->isParallelLine();
        lnJson["isPerpendicular"] = ln->isPerpendicularLine();
        if (ln->isParallelLine() || ln->isPerpendicularLine()) {
          if (auto refObj = ln->getConstraintRefObject()) {
            lnJson["constraintRefId"] = refObj->getID();
            lnJson["constraintRefEdgeIndex"] = ln->getConstraintRefEdgeIndex();
          }
        }

        linesArray.push_back(lnJson);
      }
    }
    project["objects"]["lines"] = linesArray;

    // Save Circles
    json circlesArray = json::array();
    for (const auto& ci : editor.circles) {
      if (ci && ci->isValid()) {
        json ciJson;
        ciJson["id"] = ci->getID();
        ciJson["color"] = colorToHex(ci->getColor());
        ciJson["thickness"] = ci->getThickness();
        ciJson["lineStyle"] = static_cast<int>(ci->getLineStyle());

        addTransformMetadata(ciJson, ci);

        ciJson["isSemicircle"] = ci->isSemicircle();
        if (ci->isSemicircle()) {
          if (auto p1 = ci->getDiameterP1()) {
            ciJson["diameterP1ID"] = p1->getID();
            ciJson["startArcID"] = p1->getID();
          }
          if (auto p2 = ci->getDiameterP2()) {
            ciJson["diameterP2ID"] = p2->getID();
            ciJson["endArcID"] = p2->getID();
          }
        }

        if (auto centerObj = ci->getCenterPointObject()) {
          ciJson["centerPointID"] = centerObj->getID();
        } else {
          Point_2 center = ci->getCGALPosition();
          ciJson["centerX"] = CGAL::to_double(center.x());
          ciJson["centerY"] = CGAL::to_double(center.y());
        }

        if (auto radiusObj = ci->getRadiusPointObject()) {
          ciJson["radiusPointID"] = radiusObj->getID();
        } else {
          ciJson["radiusValue"] = ci->getRadius();
        }

        circlesArray.push_back(ciJson);
      }
    }
    project["objects"]["circles"] = circlesArray;

    // Save Rectangles
    json rectanglesArray = json::array();
    for (const auto& rect : editor.rectangles) {
      if (rect && rect->isValid()) {
        json rectJson;
        rectJson["id"] = rect->getID();

        // Save corner Point IDs instead of just coordinates
        if (auto c1 = rect->getCorner1Point()) {
          rectJson["corner1ID"] = c1->getID();
        }
        if (auto c2 = rect->getCorner2Point()) {
          rectJson["corner2ID"] = c2->getID();
        }
        if (auto cb = rect->getCornerBPoint()) {
          rectJson["cornerBID"] = cb->getID();
        }
        if (auto cd = rect->getCornerDPoint()) {
          rectJson["cornerDID"] = cd->getID();
        }

        // Keep vertices for legacy compatibility
        auto vertices = rect->getInteractableVertices();
        json verticesJson = json::array();
        for (const auto& v : vertices) {
          verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        rectJson["vertices"] = verticesJson;

        // Save vertexIds in 5-arg constructor member order:
        // [corner1, corner2, cornerB, cornerD]
        // For Rotatable: [A, B(adjacent), C(diagonal), D]
        // For AA:        [A, C(diagonal), B(adjacent), D]
        json rectVertexIds = json::array();
        rectVertexIds.push_back(rect->getCorner1Point() ? (int)rect->getCorner1Point()->getID() : -1);
        rectVertexIds.push_back(rect->getCorner2Point() ? (int)rect->getCorner2Point()->getID() : -1);
        rectVertexIds.push_back(rect->getCornerBPoint() ? (int)rect->getCornerBPoint()->getID() : -1);
        rectVertexIds.push_back(rect->getCornerDPoint() ? (int)rect->getCornerDPoint()->getID() : -1);
        rectJson["vertexIds"] = rectVertexIds;
        rectJson["color"] = colorToHex(rect->getColor());
        rectJson["thickness"] = rect->getThickness();
        rectJson["lineStyle"] = static_cast<int>(rect->getLineStyle());
        rectJson["isRotatable"] = rect->isRotatable();
        rectJson["height"] = rect->getHeight();
        rectJson["width"] = rect->getWidth();
        // FIX: Save Label Data
        rectJson["label"] = rect->getLabel();
        rectJson["showLabel"] = rect->getShowLabel();
        sf::Vector2f lo = rect->getLabelOffset();
        rectJson["labelOffsetX"] = lo.x;
        rectJson["labelOffsetY"] = lo.y;

        addTransformMetadata(rectJson, rect);

        rectanglesArray.push_back(rectJson);
      }
    }
    project["objects"]["rectangles"] = rectanglesArray;

    // Save Polygons
    json polygonsArray = json::array();
    for (const auto& poly : editor.polygons) {
      if (poly && poly->isValid()) {
        json polyJson;
        polyJson["id"] = poly->getID();

        addTransformMetadata(polyJson, poly);

        auto vertices = poly->getInteractableVertices();
        json verticesJson = json::array();
        for (const auto& v : vertices) {
          verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        polyJson["vertices"] = verticesJson;
        json vertexIdsJson = json::array();
        for (size_t i = 0; i < poly->getVertexCount(); ++i) {
          auto vPtr = poly->getVertexPoint(i);
          if (vPtr) {
            vertexIdsJson.push_back(static_cast<int>(vPtr->getID()));
          } else {
            vertexIdsJson.push_back(-1);
          }
        }
        polyJson["vertexIds"] = vertexIdsJson;
        polyJson["color"] = colorToHex(poly->getColor());
        polyJson["thickness"] = poly->getThickness();
        polyJson["lineStyle"] = static_cast<int>(poly->getLineStyle());
        // FIX: Save Label Data
        polyJson["label"] = poly->getLabel();
        polyJson["showLabel"] = poly->getShowLabel();
        sf::Vector2f lo = poly->getLabelOffset();
        polyJson["labelOffsetX"] = lo.x;
        polyJson["labelOffsetY"] = lo.y;
        polygonsArray.push_back(polyJson);
      }
    }
    project["objects"]["polygons"] = polygonsArray;

    // Save Triangles
    json trianglesArray = json::array();
    for (const auto& tri : editor.triangles) {
      if (tri && tri->isValid()) {
        json triJson;
        triJson["id"] = tri->getID();

        addTransformMetadata(triJson, tri);

        // Save vertex Point IDs
        json vertexIdsJson = json::array();
        for (size_t i = 0; i < 3; ++i) {
          auto vPtr = tri->getVertexPoint(i);
          if (vPtr) {
            vertexIdsJson.push_back(static_cast<int>(vPtr->getID()));
          } else {
            vertexIdsJson.push_back(-1);
          }
        }
        triJson["vertexIds"] = vertexIdsJson;

        // Keep vertices for legacy compatibility
        auto vertices = tri->getInteractableVertices();
        json verticesJson = json::array();
        for (const auto& v : vertices) {
          verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        triJson["vertices"] = verticesJson;
        triJson["color"] = colorToHex(tri->getColor());
        triJson["thickness"] = tri->getThickness();
        triJson["lineStyle"] = static_cast<int>(tri->getLineStyle());
        // FIX: Save Label Data
        triJson["label"] = tri->getLabel();
        triJson["showLabel"] = tri->getShowLabel();
        sf::Vector2f lo = tri->getLabelOffset();
        triJson["labelOffsetX"] = lo.x;
        triJson["labelOffsetY"] = lo.y;
        trianglesArray.push_back(triJson);
      }
    }
    project["objects"]["triangles"] = trianglesArray;

    // Save Regular Polygons
    json regularPolygonsArray = json::array();
    for (const auto& rpoly : editor.regularPolygons) {
      if (rpoly && rpoly->isValid()) {
        json rpolyJson;
        rpolyJson["id"] = rpoly->getID();

        addTransformMetadata(rpolyJson, rpoly);

        // Save center and first vertex Point IDs
        if (auto centerPt = rpoly->getCenterPoint()) {
          rpolyJson["centerPointID"] = centerPt->getID();
        }
        if (auto firstVertPt = rpoly->getFirstVertexPoint()) {
          rpolyJson["firstVertexPointID"] = firstVertPt->getID();
        }

        // Keep vertices for legacy compatibility
        auto vertices = rpoly->getInteractableVertices();
        json verticesJson = json::array();
        for (const auto& v : vertices) {
          verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        rpolyJson["vertices"] = verticesJson;
        rpolyJson["sides"] = rpoly->getNumSides();
        rpolyJson["color"] = colorToHex(rpoly->getColor());
        rpolyJson["thickness"] = rpoly->getThickness();
        rpolyJson["lineStyle"] = static_cast<int>(rpoly->getLineStyle());
        regularPolygonsArray.push_back(rpolyJson);
      }
    }
    project["objects"]["regularPolygons"] = regularPolygonsArray;

    // Save Shapes (combined for convenience - logic kept same, just adding metadata)
    json shapesArray = json::array();
    for (const auto& rect : editor.rectangles) {
      if (rect && rect->isValid()) {
        json shapeJson;
        shapeJson["shapeType"] = "rectangle";
        shapeJson["id"] = rect->getID();
        // ... (Original shapesArray logic didn't seem to process metadata,
        // but since these are redundant/duplicates of specific arrays, we update mainly the specific arrays above)
        // We'll keep this block as is or minimal update since specific arrays are primary.
        // Actually, let's leave shapesArray as legacy or secondary view unchanged to avoid bloat,
        // the deserializer uses the specific arrays 'rectangles', 'circles' etc primarily.
        auto vertices = rect->getInteractableVertices();
        json verticesJson = json::array();
        for (const auto& v : vertices) {
          verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        shapeJson["vertices"] = verticesJson;
        shapeJson["color"] = colorToHex(rect->getColor());
        shapesArray.push_back(shapeJson);
      }
    }
    // ... (Repeating for other shapes in shapesArray - skipping explicit metadata here as it's for export view likely)

    project["objects"]["shapes"] = shapesArray;

    // Save ObjectPoints
    json objectPointsArray = json::array();
    for (const auto& op : editor.ObjectPoints) {
      if (op && op->isValid()) {
        json opJson;
        opJson["id"] = op->getID();
        opJson["hostType"] = static_cast<int>(op->getHostType());
        if (op->isShapeEdgeAttachment()) {
          opJson["edgeIndex"] = op->getEdgeIndex();
          opJson["edgeRel"] = op->getEdgeRelativePosition();
          opJson["t"] = op->getEdgeRelativePosition();
        } else if (op->getHostType() == ObjectType::Circle) {
          opJson["t"] = op->getAngleOnCircle();
        } else {
          opJson["t"] = op->getRelativePositionOnLine();
        }
        if (auto host = op->getHostObject()) {
          opJson["hostId"] = host->getID();
        }
        opJson["color"] = colorToHex(op->getColor());
        opJson["visible"] = op->isVisible();
        opJson["label"] = op->getLabel();
        opJson["showLabel"] = op->getShowLabel();
        sf::Vector2f offset = op->getLabelOffset();
        opJson["labelOffset"] = {offset.x, offset.y};
        addTransformMetadata(opJson, op);
        objectPointsArray.push_back(opJson);
      }
    }
    project["objects"]["objectPoints"] = objectPointsArray;

    // Save Tangent Lines
    json tangentsArray = json::array();
    for (const auto& ln : editor.lines) {
      auto tangent = std::dynamic_pointer_cast<TangentLine>(ln);
      if (tangent && tangent->isValid()) {
        json tJson;
        tJson["id"] = tangent->getID();
        if (auto ext = tangent->getExternalPoint()) {
          tJson["externalPointID"] = ext->getID();
          Point_2 p = ext->getCGALPosition();
          tJson["externalPointX"] = CGAL::to_double(p.x());
          tJson["externalPointY"] = CGAL::to_double(p.y());
        }
        if (auto cir = tangent->getCircle()) {
          tJson["circleID"] = cir->getID();
          Point_2 c = cir->getCGALPosition();
          tJson["circleCenterX"] = CGAL::to_double(c.x());
          tJson["circleCenterY"] = CGAL::to_double(c.y());
          tJson["circleRadius"] = cir->getRadius();
        }
        tJson["solutionIndex"] = tangent->getSolutionIndex();
        tJson["color"] = colorToHex(tangent->getColor());
        tJson["thickness"] = tangent->getThickness();
        tJson["lineStyle"] = static_cast<int>(tangent->getLineStyle());
        tangentsArray.push_back(tJson);
      }
    }
    project["objects"]["tangentLines"] = tangentsArray;

    // Save Angles
    json anglesArray = json::array();
    for (const auto& angle : editor.angles) {
      if (!angle) continue;
      auto pA = angle->getPointA().lock();
      auto v = angle->getVertex().lock();
      auto pB = angle->getPointB().lock();
      if (!pA || !v || !pB) continue;

      json aJson;
      aJson["id"] = angle->getID();
      aJson["pointAID"] = pA->getID();
      aJson["vertexID"] = v->getID();
      aJson["pointBID"] = pB->getID();
      aJson["reflex"] = angle->isReflex();
      aJson["radius"] = angle->getRadius();
      aJson["color"] = colorToHex(angle->getColor());
      aJson["showLabel"] = angle->getShowLabel();
      sf::Vector2f aOffset = angle->getLabelOffset();
      aJson["labelOffset"] = {aOffset.x, aOffset.y};
      anglesArray.push_back(aJson);
    }
    project["objects"]["angles"] = anglesArray;

    // Save Intersections (constraints + points)
    json intersectionsArray = json::array();
    for (const auto& constraint : DynamicIntersection::getActiveIntersectionConstraints()) {
      auto A = constraint.A.lock();
      auto B = constraint.B.lock();
      if (!A || !B) continue;

      json cJson;
      cJson["aId"] = A->getID();
      cJson["bId"] = B->getID();
      json ptsJson = json::array();
      for (const auto& wp : constraint.resultingPoints) {
        if (auto sp = wp.lock()) {
          json pJson;
          pJson["id"] = sp->getID();
          pJson["label"] = sp->getLabel();
          pJson["showLabel"] = sp->getShowLabel();
          sf::Vector2f offset = sp->getLabelOffset();
          pJson["labelOffset"] = {offset.x, offset.y};
          Point_2 pos = sp->getCGALPosition();
          pJson["x"] = CGAL::to_double(pos.x());
          pJson["y"] = CGAL::to_double(pos.y());
          ptsJson.push_back(pJson);
        }
      }
      cJson["points"] = ptsJson;
      intersectionsArray.push_back(cJson);
    }
    project["objects"]["intersections"] = intersectionsArray;

    // Write to file
    std::ofstream file(filepath);
    if (!file.is_open()) {
      std::cerr << "ProjectSerializer::saveProject: Failed to open file: " << filepath << std::endl;
      return false;
    }

    file << project.dump(2);
    file.close();

    std::cout << "Project saved successfully to: " << filepath << std::endl;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "ProjectSerializer::saveProject: Exception: " << e.what() << std::endl;
    return false;
  }
}

// ============================================================================
// LOAD PROJECT - Delegates to Deserializer for robust 4-pass loading
// ============================================================================

bool ProjectSerializer::loadProject(GeometryEditor& editor, const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "ProjectSerializer::loadProject: Failed to open file: " << filepath << std::endl;
    return false;
  }

  try {
    json j;
    file >> j;
    file.close();

    editor.clearScene();
    DynamicIntersection::clearAllIntersectionConstraints(editor);

    if (editor.getXAxisShared() && std::find(editor.lines.begin(), editor.lines.end(), editor.getXAxisShared()) == editor.lines.end()) {
      editor.lines.push_back(editor.getXAxisShared());
    }
    if (editor.getYAxisShared() && std::find(editor.lines.begin(), editor.lines.end(), editor.getYAxisShared()) == editor.lines.end()) {
      editor.lines.push_back(editor.getYAxisShared());
    }

    const json& data = j.contains("objects") ? j["objects"] : j;
    unsigned int maxId = 0;

    auto bumpMaxId = [&](unsigned int id) {
      if (id > maxId) maxId = id;
    };

    // PHASE 1: REGISTRY
    std::map<unsigned int, json> registry;
    std::map<unsigned int, std::pair<unsigned int, unsigned int>> vectorLineMap;

    auto addToRegistry = [&](const json& list) {
        if (!list.is_array()) return;
        for (const auto& item : list) {
            unsigned int id = item.value("id", 0u);
        if (id > 0) {
          registry[id] = item;
          bumpMaxId(id);
        }
        }
    };

    if (data.contains("points")) addToRegistry(data["points"]);
    if (data.contains("lines")) {
        const auto& lines = data["lines"];
        addToRegistry(lines);
        for (const auto& line : lines) {
            unsigned int id = line.value("id", 0u);
            unsigned int start = line.value("startPointID", 0u);
            unsigned int end = line.value("endPointID", 0u);
            if (start > 0 && end > 0) vectorLineMap[end] = {start, id};
        }
    }
    if (data.contains("circles")) addToRegistry(data["circles"]);
    if (data.contains("objectPoints")) addToRegistry(data["objectPoints"]);

    // PHASE 2: POLYMORPHIC POINTS (Pass 1 & Pass 2)
    std::map<unsigned int, std::shared_ptr<GeometricObject>> created;
    std::map<unsigned int, std::shared_ptr<Point>> pointMap;
    std::unordered_set<unsigned int> usedPointIds;
    std::vector<std::pair<const json*, std::shared_ptr<Point>>> loadedPointRecords;
    std::vector<std::shared_ptr<ObjectPoint>> pendingCircleHostIdZero;
    std::vector<std::shared_ptr<Point>> zeroIdCircleHostedPointCandidates;
    if (editor.getXAxisShared()) created[editor.getXAxisShared()->getID()] = editor.getXAxisShared();
    if (editor.getYAxisShared()) created[editor.getYAxisShared()->getID()] = editor.getYAxisShared();

    auto allocatePointId = [&](unsigned int requestedId) -> unsigned int {
      if (requestedId == 0u || usedPointIds.find(requestedId) != usedPointIds.end()) {
        unsigned int generated = ++maxId;
        usedPointIds.insert(generated);
        return generated;
      }
      usedPointIds.insert(requestedId);
      bumpMaxId(requestedId);
      return requestedId;
    };

    auto createSmartPoint = [&](const json& jPt) -> std::shared_ptr<Point> {
      unsigned int id = jPt.value("id", 0u);
      double x = jPt.value("x", 0.0);
      double y = jPt.value("y", 0.0);
      sf::Color color = hexToColor(jPt.value("color", "#000000"));

      if (jPt.value("isIntersectionPoint", false)) {
        auto ip = std::make_shared<IntersectionPoint>(nullptr, nullptr, Point_2(x, y), color, id);
        ip->setDependent(true);
        return ip;
      }

      if (jPt.contains("hostId")) {
        auto op = std::make_shared<ObjectPoint>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
        return op;
      }

      if (jPt.contains("transform") && jPt["transform"].is_object()) {
        const json& t = jPt["transform"];
        std::string type = t.value("type", "");
        double tVal = t.value("transformValue", t.value("angleDeg", t.value("factor", 0.0)));

        if (type == "ReflectLine") return std::make_shared<ReflectLine>(nullptr, nullptr, color, id);
        if (type == "ReflectPoint") return std::make_shared<ReflectPoint>(nullptr, nullptr, color, id);
        if (type == "ReflectCircle") return std::make_shared<ReflectCircle>(nullptr, nullptr, color, id);
        // RotatePoint expects degrees (see RotatePoint::update)
        if (type == "RotatePoint") return std::make_shared<RotatePoint>(nullptr, nullptr, tVal, color, id);
        if (type == "TranslateVector") return std::make_shared<TranslateVector>(nullptr, nullptr, nullptr, color, id);
        if (type == "DilatePoint") return std::make_shared<DilatePoint>(nullptr, nullptr, tVal, color, id);
      }

      return std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
    };

    if (data.contains("points") && data["points"].is_array()) {
      for (const auto& jPt : data["points"]) {
        unsigned int requestedId = jPt.value("id", 0u);
        unsigned int effectiveId = allocatePointId(requestedId);

        auto pt = createSmartPoint(jPt);
        if (!pt) continue;
        pt->setID(effectiveId);

        pt->setLabel(jPt.value("label", ""));
        pt->setShowLabel(jPt.value("showLabel", false));
        if (jPt.contains("visible")) pt->setVisible(jPt.value("visible", true));
        if (jPt.contains("locked")) pt->setLocked(jPt.value("locked", false));

        if (jPt.contains("transform")) {
          pt->setDependent(true);
        }

        pointMap[effectiveId] = pt;
        created[effectiveId] = pt;
        if (requestedId > 0u) {
          if (!pointMap.count(requestedId)) pointMap[requestedId] = pt;
          if (!created.count(requestedId)) created[requestedId] = pt;
        }
        editor.points.push_back(pt);
        if (std::dynamic_pointer_cast<ObjectPoint>(pt)) {
          editor.ObjectPoints.push_back(std::dynamic_pointer_cast<ObjectPoint>(pt));
        }
        loadedPointRecords.push_back({&jPt, pt});

        if (requestedId == 0u && jPt.contains("hostType")) {
          int hostTypeVal = jPt.value("hostType", -1);
          if (hostTypeVal == static_cast<int>(ObjectType::Circle)) {
            zeroIdCircleHostedPointCandidates.push_back(pt);
          }
        }
      }
    }

    if (data.contains("objectPoints") && data["objectPoints"].is_array()) {
      for (const auto& jOp : data["objectPoints"]) {
        unsigned int requestedId = jOp.value("id", 0u);

        if (requestedId > 0u && pointMap.count(requestedId)) {
          auto existingOp = std::dynamic_pointer_cast<ObjectPoint>(pointMap[requestedId]);
          if (existingOp) {
            std::string existingLabel = existingOp->getLabel();
            std::string incomingLabel = jOp.value("label", "");
            if (existingLabel == incomingLabel) {
              continue;
            }
          }
        }

        unsigned int effectiveId = allocatePointId(requestedId);
        double x = jOp.value("x", 0.0);
        double y = jOp.value("y", 0.0);
        sf::Color color = hexToColor(jOp.value("color", "#000000"));

        auto op = std::make_shared<ObjectPoint>(Point_2(x, y), Constants::CURRENT_ZOOM, color, effectiveId);
        op->setLabel(jOp.value("label", ""));
        op->setShowLabel(jOp.value("showLabel", false));
        if (jOp.contains("visible")) op->setVisible(jOp.value("visible", true));
        if (jOp.contains("locked")) op->setLocked(jOp.value("locked", false));

        pointMap[effectiveId] = op;
        created[effectiveId] = op;
        if (requestedId > 0u) {
          if (!pointMap.count(requestedId)) pointMap[requestedId] = op;
          if (!created.count(requestedId)) created[requestedId] = op;
        }

        editor.points.push_back(op);
        editor.ObjectPoints.push_back(op);
        loadedPointRecords.push_back({&jOp, op});

        if (requestedId == 0u) {
          int hostTypeVal = jOp.value("hostType", -1);
          if (hostTypeVal == static_cast<int>(ObjectType::Circle)) {
            zeroIdCircleHostedPointCandidates.push_back(op);
          }
        }
      }
    }

    // PHASE 2: RECURSIVE CONSTRUCTION (Non-point objects)
    std::vector<unsigned int> stack;

    std::function<std::shared_ptr<GeometricObject>(unsigned int)> getOrCreate = 
      [&](unsigned int id) -> std::shared_ptr<GeometricObject> {
      if (id == 0) return nullptr;
      if (created.count(id)) return created[id];
      if (!registry.count(id)) return nullptr;
      if (std::find(stack.begin(), stack.end(), id) != stack.end()) return nullptr; // Cycle

      stack.push_back(id);
      const json& jObj = registry[id];
      std::shared_ptr<GeometricObject> obj = nullptr;

      if (pointMap.count(id)) {
        obj = pointMap[id];
      } else if (jObj.contains("constructionType")) {
            std::string cType = jObj.value("constructionType", "");
            sf::Color color = hexToColor(jObj.value("color", "#000000"));

            if (cType == "PerpendicularBisector") {
              auto p1 = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("pbP1ID", 0u)));
              auto p2 = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("pbP2ID", 0u)));
              if (p1 && p2) {
                auto pb = std::make_shared<PerpendicularBisector>(p1, p2, id, color);
                pb->setDependent(true);
                pb->setThickness(jObj.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
                if (jObj.contains("lineStyle")) {
                  pb->setLineStyle(static_cast<LineStyle>(jObj.value("lineStyle", 0)));
                }
                p1->addDependent(pb);
                p2->addDependent(pb);
                obj = pb;
              }
            } else if (cType == "AngleBisector") {
              bool isExternal = jObj.value("isExternalBisector", false);
              std::string mode = jObj.value("abMode", "");

              if (mode == "lines") {
                auto l1 = std::dynamic_pointer_cast<Line>(getOrCreate(jObj.value("abLine1ID", 0u)));
                auto l2 = std::dynamic_pointer_cast<Line>(getOrCreate(jObj.value("abLine2ID", 0u)));
                if (l1 && l2) {
                  auto ab = std::make_shared<AngleBisector>(l1, l2, id, isExternal, color);
                  ab->setDependent(true);
                  ab->setThickness(jObj.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
                  if (jObj.contains("lineStyle")) {
                    ab->setLineStyle(static_cast<LineStyle>(jObj.value("lineStyle", 0)));
                  }
                  l1->addDependent(ab);
                  l2->addDependent(ab);
                  obj = ab;
                }
              } else {
                auto v = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("abVertexID", 0u)));
                auto a = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("abArm1ID", 0u)));
                auto b = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("abArm2ID", 0u)));
                if (v && a && b) {
                  auto ab = std::make_shared<AngleBisector>(v, a, b, id, isExternal, color);
                  ab->setDependent(true);
                  ab->setThickness(jObj.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
                  if (jObj.contains("lineStyle")) {
                    ab->setLineStyle(static_cast<LineStyle>(jObj.value("lineStyle", 0)));
                  }
                  a->addDependent(ab);
                  v->addDependent(ab);
                  b->addDependent(ab);
                  obj = ab;
                }
              }
            }
      } else if (jObj.contains("startPointID") && jObj.value("isSegment", false)) { // Line segment
            auto start = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("startPointID", 0u)));
            auto end = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("endPointID", 0u)));
            if (!start && jObj.contains("startX") && jObj.contains("startY")) {
              unsigned int pid = ++maxId;
              auto p = std::make_shared<Point>(Point_2(jObj.value("startX", 0.0), jObj.value("startY", 0.0)), Constants::CURRENT_ZOOM,
                                               Constants::POINT_DEFAULT_COLOR, pid);
              pointMap[pid] = p;
              created[pid] = p;
              editor.points.push_back(p);
              start = p;
            }
            if (!end && jObj.contains("endX") && jObj.contains("endY")) {
              unsigned int pid = ++maxId;
              auto p = std::make_shared<Point>(Point_2(jObj.value("endX", 0.0), jObj.value("endY", 0.0)), Constants::CURRENT_ZOOM,
                                               Constants::POINT_DEFAULT_COLOR, pid);
              pointMap[pid] = p;
              created[pid] = p;
              editor.points.push_back(p);
              end = p;
            }
            sf::Color color = hexToColor(jObj.value("color", "#000000"));
            int lType = jObj.value("lineType", 0);
            if (start && end) {
                // Fix: Line constructor takes bool for segment, not LineType
                auto ln = std::make_shared<Line>(start, end, true, color, id); 
                ln->setLineType((Line::LineType)lType); 
          ln->setThickness(jObj.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
          if (jObj.contains("lineStyle")) {
            ln->setLineStyle(static_cast<LineStyle>(jObj.value("lineStyle", 0)));
          }
          if (jObj.contains("decoration")) {
            ln->setDecoration(static_cast<DecorationType>(jObj.value("decoration", 0)));
          }
                ln->setLabel(jObj.value("label", ""));
                ln->setShowLabel(jObj.value("showLabel", false));
                obj = ln;
            }
        } else if (jObj.contains("startPointID")) { // Infinite Line
             auto start = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("startPointID", 0u)));
             auto end = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("endPointID", 0u)));
             if (!start && jObj.contains("startX") && jObj.contains("startY")) {
               unsigned int pid = ++maxId;
               auto p = std::make_shared<Point>(Point_2(jObj.value("startX", 0.0), jObj.value("startY", 0.0)), Constants::CURRENT_ZOOM,
                                                Constants::POINT_DEFAULT_COLOR, pid);
               pointMap[pid] = p;
               created[pid] = p;
               editor.points.push_back(p);
               start = p;
             }
             if (!end && jObj.contains("endX") && jObj.contains("endY")) {
               unsigned int pid = ++maxId;
               auto p = std::make_shared<Point>(Point_2(jObj.value("endX", 0.0), jObj.value("endY", 0.0)), Constants::CURRENT_ZOOM,
                                                Constants::POINT_DEFAULT_COLOR, pid);
               pointMap[pid] = p;
               created[pid] = p;
               editor.points.push_back(p);
               end = p;
             }
             sf::Color color = hexToColor(jObj.value("color", "#000000"));
             int lType = jObj.value("lineType", 0);
             if (start && end) {
                 auto ln = std::make_shared<Line>(start, end, false, color, id);
                 ln->setLineType((Line::LineType)lType);
           ln->setThickness(jObj.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
           if (jObj.contains("lineStyle")) {
             ln->setLineStyle(static_cast<LineStyle>(jObj.value("lineStyle", 0)));
           }
           if (jObj.contains("decoration")) {
             ln->setDecoration(static_cast<DecorationType>(jObj.value("decoration", 0)));
           }
                 obj = ln;
             }
        } else if (jObj.contains("centerPointID")) { // Circle
           auto center = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("centerPointID", 0u)));
           sf::Color color = hexToColor(jObj.value("color", "#000000"));
           unsigned int circleId = jObj.value("id", 0u);
           if (circleId == 0u) {
             circleId = ++maxId;
           } else {
             bumpMaxId(circleId);
           }
           if (center) {
             if (jObj.contains("radiusPointID")) {
               auto radPt = std::dynamic_pointer_cast<Point>(getOrCreate(jObj.value("radiusPointID", 0u)));
               if (radPt) {
                 obj = std::make_shared<Circle>(center.get(), radPt, 0.0, color);
                 obj->setID(circleId);
               }
             } else {
               double r = jObj.value("radiusValue", 1.0);
               obj = std::make_shared<Circle>(center.get(), nullptr, r, color);
               obj->setID(circleId);
             }
             if (auto ci = std::dynamic_pointer_cast<Circle>(obj)) {
               ci->setThickness(jObj.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
               if (jObj.contains("lineStyle")) {
                 ci->setLineStyle(static_cast<LineStyle>(jObj.value("lineStyle", 0)));
               }
             }
           }
        }

        if (obj) created[id] = obj;
        stack.pop_back();
        return obj;
    };

      // PHASE 2.5: POINT WIRING (Transformations + ObjectPoints)
      auto wirePoint = [&](const json& jPt, const std::shared_ptr<Point>& pt) {
        if (!pt) return;

        if (jPt.contains("hostId")) {
          unsigned int hostId = jPt.value("hostId", 0u);
          auto op = std::dynamic_pointer_cast<ObjectPoint>(pt);
          ObjectType hostType = static_cast<ObjectType>(jPt.value("hostType", static_cast<int>(ObjectType::None)));
          if (op && hostId == 0u && hostType == ObjectType::Circle) {
            pendingCircleHostIdZero.push_back(op);
          }

          auto host = std::dynamic_pointer_cast<GeometricObject>(getOrCreate(hostId));
          if (op && host) {
            if (hostType == ObjectType::None) {
              hostType = static_cast<ObjectType>(host->getType());
            }
            double tHost = jPt.value("t", 0.5);
            op->relinkHost(host, tHost, hostType);
            if (hostType == ObjectType::Line || hostType == ObjectType::LineSegment) {
              if (auto l = std::dynamic_pointer_cast<Line>(host)) l->addChildPoint(op);
            } else if (hostType == ObjectType::Circle) {
              if (auto c = std::dynamic_pointer_cast<Circle>(host)) c->addChildPoint(op);
            }
          }
        }

        if (!jPt.contains("transform") || !jPt["transform"].is_object()) return;

        const json& t = jPt["transform"];
        std::string type = t.value("type", "");

        unsigned int parentId = t.value("parentSourceID", jPt.value("parentSourceID", 0u));
        auto parent = (pointMap.count(parentId) ? pointMap[parentId] : nullptr);

        if (type == "TranslateVector") {
          unsigned int vecStartId = t.value("vecStartId", 0u);
          unsigned int vecEndId = t.value("vecEndId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
          if (vecStartId == 0u && vecEndId != 0u && vectorLineMap.count(vecEndId)) {
            vecStartId = vectorLineMap[vecEndId].first;
          }
          auto vecStart = (pointMap.count(vecStartId) ? pointMap[vecStartId] : nullptr);
          auto vecEnd = (pointMap.count(vecEndId) ? pointMap[vecEndId] : nullptr);
          auto tv = std::dynamic_pointer_cast<TranslateVector>(pt);
          if (tv && parent && vecStart && vecEnd) {
            tv->restoreTransformation(parent, vecEnd, TransformationType::Translate);
            tv->setVectorStart(vecStart);
            parent->addDependent(tv);
            tv->update();
          }
        } else if (type == "RotatePoint") {
          unsigned int centerId = t.value("centerId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
          auto center = (pointMap.count(centerId) ? pointMap[centerId] : nullptr);
          auto rp = std::dynamic_pointer_cast<RotatePoint>(pt);
          if (rp && parent && center) {
            rp->restoreTransformation(parent, center, TransformationType::Rotate);
            parent->addDependent(rp);
            rp->update();
          }
        } else if (type == "ReflectLine") {
          unsigned int lineId = t.value("lineId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
          auto line = std::dynamic_pointer_cast<Line>(getOrCreate(lineId));
          auto rl = std::dynamic_pointer_cast<ReflectLine>(pt);
          if (rl && parent && line) {
            rl->restoreTransformation(parent, line, TransformationType::Reflect);
            parent->addDependent(rl);
            rl->update();
          }
        } else if (type == "ReflectPoint") {
          unsigned int centerId = t.value("centerId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
          auto center = (pointMap.count(centerId) ? pointMap[centerId] : nullptr);
          auto rp = std::dynamic_pointer_cast<ReflectPoint>(pt);
          if (rp && parent && center) {
            rp->restoreTransformation(parent, center, TransformationType::ReflectPoint);
            parent->addDependent(rp);
            rp->update();
          }
        } else if (type == "ReflectCircle") {
          unsigned int circleId = t.value("circleId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
          auto circle = std::dynamic_pointer_cast<Circle>(getOrCreate(circleId));
          auto rc = std::dynamic_pointer_cast<ReflectCircle>(pt);
          if (rc && parent && circle) {
            rc->restoreTransformation(parent, circle, TransformationType::ReflectCircle);
            parent->addDependent(rc);
            rc->update();
          }
        } else if (type == "DilatePoint") {
          unsigned int centerId = t.value("centerId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
          auto center = (pointMap.count(centerId) ? pointMap[centerId] : nullptr);
          auto dp = std::dynamic_pointer_cast<DilatePoint>(pt);
          if (dp && parent && center) {
            dp->restoreTransformation(parent, center, TransformationType::Dilate);
            parent->addDependent(dp);
            dp->update();
          }
        }
      };

      for (const auto& rec : loadedPointRecords) {
        if (!rec.first || !rec.second) continue;
        wirePoint(*rec.first, rec.second);
      }

      // PHASE 2.6: STABILIZE TRANSFORM CHAINS (Dependency-ordered)
      // Ensures centers/parents that are themselves transformed are updated first.
      if (data.contains("points") && data["points"].is_array()) {
        std::unordered_set<unsigned int> updated;
        std::vector<unsigned int> pendingIds;

        for (const auto& jPt : data["points"]) {
          unsigned int id = jPt.value("id", 0u);
          if (id == 0u) continue;
          if (jPt.contains("transform") && jPt["transform"].is_object()) {
            pendingIds.push_back(id);
          } else {
            updated.insert(id);
          }
        }

        auto pointReady = [&](unsigned int pid) -> bool {
          return (pid == 0u) || (updated.find(pid) != updated.end()) || (pointMap.find(pid) == pointMap.end());
        };
        auto objectReady = [&](unsigned int oid) -> bool {
          return (oid == 0u) || (created.find(oid) != created.end());
        };

        bool progressed = true;
        int safetyPasses = 0;
        while (progressed && safetyPasses < 8) {
          progressed = false;
          ++safetyPasses;

          for (const auto& jPt : data["points"]) {
            if (!jPt.contains("transform") || !jPt["transform"].is_object()) continue;
            unsigned int id = jPt.value("id", 0u);
            if (id == 0u || updated.find(id) != updated.end()) continue;

            const json& t = jPt["transform"];
            std::string type = t.value("type", "");
            unsigned int parentId = t.value("parentSourceID", jPt.value("parentSourceID", 0u));
            if (!pointReady(parentId)) continue;

            if (type == "TranslateVector") {
              unsigned int vecStartId = t.value("vecStartId", 0u);
              unsigned int vecEndId = t.value("vecEndId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
              if (vecStartId == 0u && vecEndId != 0u && vectorLineMap.count(vecEndId)) {
                vecStartId = vectorLineMap[vecEndId].first;
              }
              if (!pointReady(vecStartId) || !pointReady(vecEndId)) continue;
            } else if (type == "ReflectLine") {
              unsigned int lineId = t.value("lineId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
              if (!objectReady(lineId)) continue;
            } else if (type == "ReflectCircle") {
              unsigned int circleId = t.value("circleId", t.value("auxObjectID", jPt.value("auxObjectID", 0u)));
              if (!objectReady(circleId)) continue;
            } else {
              unsigned int auxId = t.value("auxObjectID", jPt.value("auxObjectID", 0u));
              if (!pointReady(auxId)) continue;
            }

            auto pt = (pointMap.count(id) ? pointMap[id] : nullptr);
            if (pt) {
              pt->update();
              updated.insert(id);
              progressed = true;
            }
          }
        }

        // Final safety sweep to settle any remaining transforms.
        for (const auto& jPt : data["points"]) {
          if (!jPt.contains("transform") || !jPt["transform"].is_object()) continue;
          unsigned int id = jPt.value("id", 0u);
          auto pt = (pointMap.count(id) ? pointMap[id] : nullptr);
          if (pt) pt->update();
        }
      }

    // Populate Editor Lists (Non-point objects only)
    if (data.contains("lines")) {
        for (const auto& item : data["lines"]) {
            auto obj = getOrCreate(item.value("id", 0u));
            if (auto ln = std::dynamic_pointer_cast<Line>(obj)) {
              ln->setThickness(item.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
              if (item.contains("lineStyle")) {
                ln->setLineStyle(static_cast<LineStyle>(item.value("lineStyle", 0)));
              }
              if (item.contains("decoration")) {
                ln->setDecoration(static_cast<DecorationType>(item.value("decoration", 0)));
              }
              editor.lines.push_back(ln);
            }
        }

        for (const auto& item : data["lines"]) {
          bool isParallel = item.value("isParallel", false);
          bool isPerpendicular = item.value("isPerpendicular", false);
          if (!isParallel && !isPerpendicular) continue;

          auto ln = std::dynamic_pointer_cast<Line>(getOrCreate(item.value("id", 0u)));
          if (!ln) continue;

          auto refObj = getOrCreate(item.value("constraintRefId", 0u));
          int edgeIndex = item.value("constraintRefEdgeIndex", -1);
          Vector_2 refDir(0, 0);

          if (refObj) {
            if (edgeIndex >= 0) {
              auto edges = refObj->getEdges();
              if (edgeIndex < static_cast<int>(edges.size())) {
                refDir = edges[static_cast<size_t>(edgeIndex)].to_vector();
              }
            } else if (auto refLine = std::dynamic_pointer_cast<Line>(refObj)) {
              refDir = refLine->getEndPoint() - refLine->getStartPoint();
            }
          }

          if (isParallel) {
            ln->setAsParallelLine(refObj, edgeIndex, refDir);
          } else if (isPerpendicular) {
            ln->setAsPerpendicularLine(refObj, edgeIndex, refDir);
          }
        }
    }

    std::shared_ptr<Circle> zeroIdCircleFallback;
    if (data.contains("circles")) {
        for (const auto& item : data["circles"]) {
          unsigned int jsonCircleId = item.value("id", 0u);
          auto obj = (jsonCircleId == 0u) ? nullptr : getOrCreate(jsonCircleId);
          auto ci = std::dynamic_pointer_cast<Circle>(obj);

          if (!ci) {
            auto center = std::dynamic_pointer_cast<Point>(getOrCreate(item.value("centerPointID", 0u)));
            if (center) {
              sf::Color color = hexToColor(item.value("color", "#000000"));
              unsigned int assignedCircleId = (jsonCircleId == 0u) ? ++maxId : jsonCircleId;
              bumpMaxId(assignedCircleId);

              if (item.contains("radiusPointID")) {
                auto radiusPt = std::dynamic_pointer_cast<Point>(getOrCreate(item.value("radiusPointID", 0u)));
                if (radiusPt) {
                  ci = std::make_shared<Circle>(center.get(), radiusPt, 0.0, color);
                }
              } else {
                double r = item.value("radiusValue", 1.0);
                ci = std::make_shared<Circle>(center.get(), nullptr, r, color);
              }

              if (ci) {
                ci->setID(assignedCircleId);
                created[assignedCircleId] = ci;
                if (jsonCircleId > 0u && !created.count(jsonCircleId)) created[jsonCircleId] = ci;
              }
            }
          }

          if (ci) {
            ci->setThickness(item.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
            if (item.contains("lineStyle")) {
              ci->setLineStyle(static_cast<LineStyle>(item.value("lineStyle", 0)));
            }
            // Ensure circle is in editor.circles only once
            if (std::find(editor.circles.begin(), editor.circles.end(), ci) == editor.circles.end()) {
              editor.circles.push_back(ci);
            }
            if (jsonCircleId == 0u && !zeroIdCircleFallback) {
              zeroIdCircleFallback = ci;
            }
          }
        }

        if (zeroIdCircleFallback) {
          for (const auto& op : pendingCircleHostIdZero) {
            if (!op) continue;
            op->relinkHost(zeroIdCircleFallback, op->getAngleOnCircle(), ObjectType::Circle);
            zeroIdCircleFallback->addChildPoint(op);
          }
        }
    }

    // Tangent Lines
    if (data.contains("tangentLines")) {
      for (const auto& item : data["tangentLines"]) {
        unsigned int extId = item.value("externalPointID", 0u);
        auto ext = std::dynamic_pointer_cast<Point>(getOrCreate(extId));
        if (!ext && extId == 0u && !zeroIdCircleHostedPointCandidates.empty()) {
          ext = zeroIdCircleHostedPointCandidates.front();
        }
        if (!ext && item.contains("externalPointX") && item.contains("externalPointY")) {
          unsigned int pid = ++maxId;
          ext = std::make_shared<Point>(Point_2(item.value("externalPointX", 0.0), item.value("externalPointY", 0.0)),
                                        Constants::CURRENT_ZOOM, Constants::POINT_DEFAULT_COLOR, pid);
          pointMap[pid] = ext;
          created[pid] = ext;
          editor.points.push_back(ext);
        }
        unsigned int tangentCircleId = item.value("circleID", 0u);
        auto cir = (tangentCircleId == 0u)
                     ? zeroIdCircleFallback
                     : std::dynamic_pointer_cast<Circle>(getOrCreate(tangentCircleId));
        if (!cir && item.contains("circleCenterX") && item.contains("circleCenterY") && item.contains("circleRadius")) {
          unsigned int centerPid = ++maxId;
          auto centerPt = std::make_shared<Point>(Point_2(item.value("circleCenterX", 0.0), item.value("circleCenterY", 0.0)),
                                                  Constants::CURRENT_ZOOM, Constants::POINT_DEFAULT_COLOR, centerPid);
          pointMap[centerPid] = centerPt;
          created[centerPid] = centerPt;
          editor.points.push_back(centerPt);

          cir = std::make_shared<Circle>(centerPt.get(), nullptr, item.value("circleRadius", 1.0), sf::Color::Black);
          cir->setID(++maxId);
          created[cir->getID()] = cir;
          if (std::find(editor.circles.begin(), editor.circles.end(), cir) == editor.circles.end()) {
            editor.circles.push_back(cir);
          }
        }
        // If circle has id==0, assign new id and bump maxId
        if (cir && cir->getID() == 0u) {
            cir->setID(++maxId);
            bumpMaxId(cir->getID());
            if (std::find(editor.circles.begin(), editor.circles.end(), cir) == editor.circles.end()) {
                editor.circles.push_back(cir);
            }
        }
        if (!ext || !cir) continue;
        auto tan = std::make_shared<TangentLine>(ext, cir, item.value("solutionIndex", 0),
                                                 item.value("id", 0u),
                                                 hexToColor(item.value("color", "#00FFFF")));
        tan->setThickness(item.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        if (item.contains("lineStyle")) {
          tan->setLineStyle(static_cast<LineStyle>(item.value("lineStyle", 0)));
        }
        editor.lines.push_back(tan);
      }
    }

    // PHASE 3: SHAPES (Strict vertexId linkage)
    auto hasTransformParent = [&](const json& jObj) -> bool {
      unsigned int parentId = jObj.value("parentSourceID", 0u);
      if (parentId != 0u) return true;
      if (jObj.contains("transform") && jObj["transform"].is_object()) {
        const json& t = jObj["transform"];
        parentId = t.value("parentSourceID", t.value("sourceId", 0u));
      }
      return parentId != 0u;
    };

    auto processPolyShape = [&](const json& jShape, bool isRect) {
      unsigned int id = jShape.value("id", 0u);
      if (id == 0) {
        id = ++maxId;
      } else {
        bumpMaxId(id);
      }
      
      // 1. Collect Points
      if (!jShape.contains("vertexIds") || !jShape["vertexIds"].is_array()) return;
        
      std::vector<std::shared_ptr<Point>> vPts;
      for (const auto& vidVal : jShape["vertexIds"]) {
        unsigned int vid = vidVal.get<unsigned int>();
        if (vid == 0) continue; 
        auto it = pointMap.find(vid);
        if (it != pointMap.end() && it->second) {
            vPts.push_back(it->second);
        }
      }

      // FIX 1: Allow 2 points for Rectangles (Implicit Diagonal case)
      if (isRect) {
          if (vPts.size() != 4 && vPts.size() != 2) return;
      } else {
          if (vPts.size() < 3) return;
      }

      sf::Color color = hexToColor(jShape.value("color", "#000000"));

      if (isRect) {
        bool isRot = jShape.value("isRotatable", false);

        // FIX 2: Detect Hidden Rotation
        // If the shape is dependent and has a Rotate/Reflect transform, we MUST treat it as Rotatable
        // even if the file says "false" (because AA Rects become Rotatable when rotated).
        if (!vPts.empty() && vPts[0]->isDependent()) {
            TransformationType tType = vPts[0]->getTransformType();
            if (tType == TransformationType::Rotate || tType == TransformationType::Reflect || 
                tType == TransformationType::ReflectPoint || tType == TransformationType::ReflectCircle) {
                isRot = true;
            }
        }

        std::shared_ptr<Rectangle> rect;

        // CASE A: 4 Points (Explicit)
        if (vPts.size() == 4) {
             // 1. Sort points CCW (A -> B -> C -> D)
             // This ensures we have the perimeter order correct before passing to constructor.
             std::vector<std::shared_ptr<Point>> ordered = vPts;
             double cx = 0.0; double cy = 0.0;
             for (const auto& p : ordered) {
               Point_2 pos = p->getCGALPosition();
               cx += CGAL::to_double(pos.x());
               cy += CGAL::to_double(pos.y());
             }
             cx *= 0.25; cy *= 0.25;

             std::vector<std::pair<double, std::shared_ptr<Point>>> sorted;
             sorted.reserve(4);
             for (const auto& p : ordered) {
               Point_2 pos = p->getCGALPosition();
               double dx = CGAL::to_double(pos.x()) - cx;
               double dy = CGAL::to_double(pos.y()) - cy;
               sorted.push_back({std::atan2(dy, dx), p});
             }
             std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

             ordered.clear();
             for (const auto& pair : sorted) ordered.push_back(pair.second);
             
             // 2. CONSTRUCTOR SELECTION (CRITICAL FIX)
             if (isRot) {
                 // Rotatable Constructor: Expects Perimeter Order (Corner1, Side1, Side2, Side3)
                 // We pass: A, B, C, D (Indices 0, 1, 2, 3)
                 rect = std::make_shared<Rectangle>(
                     ordered[0], ordered[1], ordered[2], ordered[3], 
                     true, color, id
                 );
             } else {
                 // Standard Constructor: Expects Diagonal Pair First (Corner1, OppositeCorner, ...)
                 // We pass: A, C, B, D (Indices 0, 2, 1, 3)
                 rect = std::make_shared<Rectangle>(
                     ordered[0], ordered[2], ordered[1], ordered[3], 
                     false, color, id
                 );
             }
        }
        // CASE B: 2 Points (Implicit Diagonal - Dilation/Translation of AA Rects)
        else if (vPts.size() == 2) {
             // 2-point constructor handles the rest.
             rect = std::make_shared<Rectangle>(vPts[0], vPts[1], isRot, color, id);
        }

        if (rect) {
            bool jsonDependent = jShape.value("isDependent", false);
            if (jsonDependent || hasTransformParent(jShape)) {
                rect->setDependent(true);
            }
            
            rect->setLabel(jShape.value("label", ""));
            rect->setShowLabel(jShape.value("showLabel", false));
            rect->setThickness(jShape.value("thickness", 2.0f));
            if (jShape.contains("lineStyle")) {
              rect->setLineStyle(static_cast<LineStyle>(jShape.value("lineStyle", 0)));
            }
            if (jShape.contains("labelOffsetX") && jShape.contains("labelOffsetY")) {
              rect->setLabelOffset(sf::Vector2f(jShape["labelOffsetX"].get<float>(),
                                                jShape["labelOffsetY"].get<float>()));
            }
            editor.rectangles.push_back(rect);
        }

      } else {
        // Polygons (Unchanged)
        auto poly = std::make_shared<Polygon>(vPts, color, id);
        bool jsonDependent = jShape.value("isDependent", false);
        poly->setDependent(jsonDependent && hasTransformParent(jShape));
        poly->setLabel(jShape.value("label", ""));
        poly->setShowLabel(jShape.value("showLabel", false));
        poly->setThickness(jShape.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        if (jShape.contains("lineStyle")) {
          poly->setLineStyle(static_cast<LineStyle>(jShape.value("lineStyle", 0)));
        }
        if (jShape.contains("labelOffsetX") && jShape.contains("labelOffsetY")) {
          poly->setLabelOffset(sf::Vector2f(jShape["labelOffsetX"].get<float>(),
                                            jShape["labelOffsetY"].get<float>()));
        }
        editor.polygons.push_back(poly);
      }
    };

    if (data.contains("rectangles")) for (const auto& item : data["rectangles"]) processPolyShape(item, true);
    if (data.contains("polygons")) for (const auto& item : data["polygons"]) processPolyShape(item, false);

    if (data.contains("triangles")) {
      for (const auto& item : data["triangles"]) {
        unsigned int id = item.value("id", 0u);
        if (id == 0u) {
          id = ++maxId;
        } else {
          bumpMaxId(id);
        }
        if (!item.contains("vertexIds") || !item["vertexIds"].is_array()) continue;
        if (item["vertexIds"].size() != 3) continue;

        unsigned int v1Id = item["vertexIds"][0].get<unsigned int>();
        unsigned int v2Id = item["vertexIds"][1].get<unsigned int>();
        unsigned int v3Id = item["vertexIds"][2].get<unsigned int>();
        auto it1 = pointMap.find(v1Id);
        auto it2 = pointMap.find(v2Id);
        auto it3 = pointMap.find(v3Id);
        if (it1 == pointMap.end() || it2 == pointMap.end() || it3 == pointMap.end()) continue;
        auto v1 = it1->second;
        auto v2 = it2->second;
        auto v3 = it3->second;
        if (!v1 || !v2 || !v3) continue;

        sf::Color color = hexToColor(item.value("color", "#000000"));
        auto tri = std::make_shared<Triangle>(v1, v2, v3, color, id);
        bool jsonDependent = item.value("isDependent", false);
        tri->setDependent(jsonDependent && hasTransformParent(item));
        tri->setLabel(item.value("label", ""));
        tri->setShowLabel(item.value("showLabel", false));
        tri->setThickness(item.value("thickness", 2.0));
        if (item.contains("lineStyle")) {
          tri->setLineStyle(static_cast<LineStyle>(item.value("lineStyle", 0)));
        }
        if (item.contains("labelOffsetX") && item.contains("labelOffsetY")) {
          tri->setLabelOffset(sf::Vector2f(item["labelOffsetX"].get<float>(),
                                           item["labelOffsetY"].get<float>()));
        }
        editor.triangles.push_back(tri);
      }
    }

    if (data.contains("regularPolygons")) {
      for (const auto& item : data["regularPolygons"]) {
        unsigned int id = item.value("id", 0u);
        if (id == 0u) {
          id = ++maxId;
        } else {
          bumpMaxId(id);
        }
        unsigned int centerId = item.value("centerPointID", 0u);
        unsigned int v1Id = item.value("firstVertexPointID", 0u);
        auto centerIt = pointMap.find(centerId);
        auto v1It = pointMap.find(v1Id);
        if (centerIt == pointMap.end() || v1It == pointMap.end()) continue;
        auto center = centerIt->second;
        auto v1 = v1It->second;
        if (!center || !v1) continue;
        int sides = item.value("sides", 5);
        sf::Color color = hexToColor(item.value("color", "#000000"));
        auto rp = std::make_shared<RegularPolygon>(center, v1, sides, color, id);
        bool jsonDependent = item.value("isDependent", false);
        rp->setDependent(jsonDependent && hasTransformParent(item));
        rp->setThickness(item.value("thickness", 2.0));
        if (item.contains("lineStyle")) {
          rp->setLineStyle(static_cast<LineStyle>(item.value("lineStyle", 0)));
        }
        rp->setLabel(item.value("label", ""));
        rp->setShowLabel(item.value("showLabel", false));
        if (item.contains("labelOffsetX") && item.contains("labelOffsetY")) {
          rp->setLabelOffset(sf::Vector2f(item["labelOffsetX"].get<float>(),
                                          item["labelOffsetY"].get<float>()));
        }
        editor.regularPolygons.push_back(rp);
      }
    }

    // Final geometry sync for non-point objects after dependency stabilization.
    for (auto& ln : editor.lines) if (ln) ln->update();
    for (auto& ci : editor.circles) if (ci) ci->update();
    for (auto& rect : editor.rectangles) if (rect) rect->update();
    for (auto& poly : editor.polygons) if (poly) poly->update();
    for (auto& tri : editor.triangles) if (tri) tri->update();
    for (auto& rp : editor.regularPolygons) if (rp) rp->update();

    editor.objectIdCounter = maxId + 1;
    editor.enforceLabelPolicyOnAll(false);
    std::cout << "[Construction Protocol] Load Complete. Objects: " << created.size() << std::endl;
    return true;
    } catch (const std::exception& e) {
        std::cerr << "ProjectSerializer::loadProject: Exception: " << e.what() << std::endl;
        return false;
    }
}

#if 0 
bool ProjectSerializer::loadProject_OLD(GeometryEditor& editor, const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "ProjectSerializer::loadProject: Failed to open file: " << filepath << std::endl;
    return false;
  }

  try {
    json j;
    file >> j;
    file.close();

    // Clear existing scene completely
    editor.clearScene();
    DynamicIntersection::clearAllIntersectionConstraints(editor);

    // CRITICAL: Re-add axes to lines vector after clearScene (they were removed but still exist)
    if (editor.getXAxisShared() && std::find(editor.lines.begin(), editor.lines.end(), editor.getXAxisShared()) == editor.lines.end()) {
      editor.lines.push_back(editor.getXAxisShared());
    }
    if (editor.getYAxisShared() && std::find(editor.lines.begin(), editor.lines.end(), editor.getYAxisShared()) == editor.lines.end()) {
      editor.lines.push_back(editor.getYAxisShared());
    }

    const json& data = j.contains("objects") ? j["objects"] : j;
    bool axesVisible = j.value("settings", json::object()).value("axesVisible", true);

    // ========================================================================
    // THE MASTER ID MAP (Identity Store)
    // ========================================================================
    std::map<unsigned int, std::shared_ptr<GeometricObject>> masterMap;
    unsigned int maxId = 0;
    json deferredTransformPoints = json::array();
    json pendingPolygons = json::array();
    struct PendingLineConstraint {
      std::shared_ptr<Line> line;
      bool isParallel = false;
      bool isPerpendicular = false;
      unsigned int refId = 0u;
      int edgeIndex = -1;
    };
    std::vector<PendingLineConstraint> pendingLineConstraints;
    
    // CRITICAL FIX: Structure for pending ObjectPoint host relinking
    struct PendingObjectPoint {
      std::shared_ptr<ObjectPoint> objectPoint;
      unsigned int hostId = 0u;
      ObjectType hostType = ObjectType::None;
      double t = 0.0;
      bool isShapeEdge = false;
      size_t edgeIndex = 0;
    };
    std::vector<PendingObjectPoint> pendingObjectPoints;

    auto registerInMap = [&](unsigned int id, const std::shared_ptr<GeometricObject>& obj) {
      if (!obj || id == 0) return;
      if (masterMap.find(id) != masterMap.end()) {
        return;
      }
      masterMap[id] = obj;
      if (id > maxId) maxId = id;
    };

    auto getFromMap = [&](unsigned int id) -> std::shared_ptr<GeometricObject> {
      if (id == 0) return nullptr;
      auto it = masterMap.find(id);
      return (it != masterMap.end()) ? it->second : nullptr;
    };

    auto getPointFromMap = [&](unsigned int id) -> std::shared_ptr<Point> { return std::dynamic_pointer_cast<Point>(getFromMap(id)); };

    auto getPointByPosition = [&](const Point_2& pos, double tolSq) -> std::shared_ptr<Point> {
      std::shared_ptr<Point> best;
      double bestDist = tolSq;
      for (const auto& [objId, obj] : masterMap) {
        auto pt = std::dynamic_pointer_cast<Point>(obj);
        if (!pt) continue;
        Point_2 ptPos = pt->getCGALPosition();
        double dist = CGAL::to_double(CGAL::squared_distance(pos, ptPos));
        if (dist <= bestDist) {
          bestDist = dist;
          best = pt;
        }
      }
      return best;
    };

    // Helper: Parse color from JSON
    auto colorFromJson = [&](const json& jCol, const sf::Color& fallback) -> sf::Color {
      if (jCol.is_string()) return hexToColor(jCol.get<std::string>());
      if (jCol.is_object()) {
        return sf::Color(jCol.value("r", (int)fallback.r), jCol.value("g", (int)fallback.g), jCol.value("b", (int)fallback.b),
                         jCol.value("a", (int)fallback.a));
      }
      return fallback;
    };

    // Helper: Apply common point fields
    auto applyCommonPointFields = [&](const json& jPt, const std::shared_ptr<Point>& pt) {
      if (jPt.contains("label")) pt->setLabel(jPt.value("label", ""));
      if (jPt.contains("showLabel")) pt->setShowLabel(jPt.value("showLabel", true));
      if (jPt.contains("labelOffset") && jPt["labelOffset"].is_array() && jPt["labelOffset"].size() >= 2) {
        pt->setLabelOffset(sf::Vector2f(jPt["labelOffset"][0], jPt["labelOffset"][1]));
      } else if (jPt.contains("labelOffsetX") && jPt.contains("labelOffsetY")) {
        pt->setLabelOffset(sf::Vector2f(jPt["labelOffsetX"].get<float>(), jPt["labelOffsetY"].get<float>()));
      }
      if (jPt.contains("fixed")) pt->setLocked(jPt.value("fixed", false));
      if (jPt.contains("locked")) pt->setLocked(jPt.value("locked", false));
      if (jPt.contains("isDependent")) pt->setDependent(jPt.value("isDependent", false));
      if (jPt.contains("visible")) pt->setVisible(jPt.value("visible", true));
    };

    // Helper: Apply transform metadata
    // IMPORTANT: For shapes (Rectangle, Polygon, Triangle), only set isDependent if there's a
    // valid parent shape to transform from. Shapes with transformed VERTICES don't need shape-level
    // dependency - the vertices themselves handle the transformation chain.
    auto applyTransformMetadata = [&](const json& jObj, const std::shared_ptr<GeometricObject>& obj) {
      if (!obj) return;

      const json* tObj = nullptr;
      if (jObj.contains("transform") && jObj["transform"].is_object()) {
        tObj = &jObj["transform"];
      }

      auto mapTypeString = [](const std::string& type) -> TransformationType {
        if (type == "ReflectLine") return TransformationType::Reflect;
        if (type == "ReflectPoint") return TransformationType::ReflectPoint;
        if (type == "ReflectCircle") return TransformationType::ReflectCircle;
        if (type == "RotatePoint") return TransformationType::Rotate;
        if (type == "TranslateVector") return TransformationType::Translate;
        if (type == "DilatePoint") return TransformationType::Dilate;
        return TransformationType::None;
      };

      unsigned int parentId = jObj.value("parentSourceID", 0u);
      if (parentId == 0u && tObj) {
        parentId = tObj->value("parentSourceID", tObj->value("sourceId", 0u));
      }
      bool hasValidParent = (parentId > 0);

      // Only set isDependent if there's actually a parent to depend on
      // Shapes with isDependent:true but parentSourceID:0 use vertex-based transformation
      if (jObj.contains("isDependent") || (tObj && tObj->contains("isDependent"))) {
        bool jsonDependent = jObj.value("isDependent", tObj ? tObj->value("isDependent", false) : false);
        // For shapes, only mark dependent if parent exists
        ObjectType type = obj->getType();
        bool isShapeType = (type == ObjectType::Rectangle || type == ObjectType::Polygon || type == ObjectType::Triangle ||
                            type == ObjectType::RegularPolygon || type == ObjectType::RectangleRotatable);
        if (isShapeType && !hasValidParent) {
          // Fix for "Jelly" transformations:
          // Transformed shapes (created via tools) often have dependent VERTICES but no explicit parent shape ID.
          // If the JSON explicitly says it's dependent, trust it!
          // This ensures the shape remains 'rigid' (locked) because its vertices are dependent (Translate/Rotate points).
          if (jsonDependent) {
             obj->setDependent(true);
          } else {
             obj->setDependent(false);
          }
        } else {
          obj->setDependent(jsonDependent);
        }
      }

      if (jObj.contains("transformType") || (tObj && (tObj->contains("transformType") || tObj->contains("type")))) {
        TransformationType tType = TransformationType::None;
        if (jObj.contains("transformType")) {
          tType = static_cast<TransformationType>(jObj["transformType"].get<int>());
        } else if (tObj) {
          if (tObj->contains("transformType")) {
            tType = static_cast<TransformationType>((*tObj)["transformType"].get<int>());
          } else if (tObj->contains("type")) {
            tType = mapTypeString(tObj->value("type", ""));
          }
        }
        obj->setTransformType(tType);
        obj->setParentSourceID(parentId);

        // FORCE DEPENDENCY: If it has a transform type, it MUST be dependent.
        if (tType != TransformationType::None) {
            obj->setDependent(true);
        }

        unsigned int auxId = jObj.value("auxObjectID", 0u);
        if (auxId == 0u && tObj) {
          auxId = tObj->value("auxObjectID", 0u);
          if (auxId == 0u) auxId = tObj->value("lineId", 0u);
          if (auxId == 0u) auxId = tObj->value("centerId", 0u);
          if (auxId == 0u) auxId = tObj->value("circleId", 0u);
          if (auxId == 0u) auxId = tObj->value("vecEndId", 0u);
        }
        obj->setAuxObjectID(auxId);

        double tVal = jObj.value("transformValue", 0.0);
        if (tVal == 0.0 && tObj) {
          tVal = tObj->value("transformValue", tObj->value("angleDeg", tObj->value("factor", 0.0)));
        }
        obj->setTransformValue(tVal);
      }
      if ((jObj.contains("translationVectorX") && jObj.contains("translationVectorY")) ||
          (tObj && tObj->contains("translationVectorX") && tObj->contains("translationVectorY"))) {
        double tx = jObj.value("translationVectorX", tObj ? tObj->value("translationVectorX", 0.0) : 0.0);
        double ty = jObj.value("translationVectorY", tObj ? tObj->value("translationVectorY", 0.0) : 0.0);
        obj->setTranslationVector(Vector_2(tx, ty));
      }
    };

    auto finalizeExplicitRectangle = [&](const json& jRect, const std::shared_ptr<Rectangle>& rect, const std::vector<std::shared_ptr<Point>>& vPts) {
      bool vertexTransformed = false;
      for (const auto& p : vPts) {
        if (p && p->getTransformType() != TransformationType::None) {
          vertexTransformed = true;
          break;
        }
      }

      bool hasTransformMeta = jRect.contains("transformType") && jRect["transformType"].get<int>() != static_cast<int>(TransformationType::None);

      // FIX 11: CORRECTED LOGIC.
      // If vertices are transformed OR the JSON says it's a transformed shape,
      // then this rectangle IS dependent. The old code had this inverted.
      if (vertexTransformed || hasTransformMeta) {
        applyTransformMetadata(jRect, rect);
        rect->setDependent(true);
        std::cout << "    [finalizeRect] ID=" << rect->getID() << " DEPENDENT (vertexTx=" << vertexTransformed << " meta=" << hasTransformMeta << ")" << std::endl;
      } else {
        rect->setTransformType(TransformationType::None);
        rect->setParentSourceID(0u);
        rect->setAuxObjectID(0u);
        rect->setDependent(false);
      }
    };

    // ========================================================================
    // PRE-PASS: Register Axes in masterMap (for transformation references)
    // Transformations may reference axes by their original IDs (e.g., Y-axis for ReflectLine)
    // ========================================================================
    if (editor.getXAxisShared()) {
      registerInMap(editor.getXAxisShared()->getID(), editor.getXAxisShared());
      // Also register axis endpoint points if they exist
      if (editor.getXAxisShared()->getStartPointObjectShared())
        registerInMap(editor.getXAxisShared()->getStartPointObjectShared()->getID(), editor.getXAxisShared()->getStartPointObjectShared());
      if (editor.getXAxisShared()->getEndPointObjectShared())
        registerInMap(editor.getXAxisShared()->getEndPointObjectShared()->getID(), editor.getXAxisShared()->getEndPointObjectShared());
    }
    if (editor.getYAxisShared()) {
      registerInMap(editor.getYAxisShared()->getID(), editor.getYAxisShared());
      if (editor.getYAxisShared()->getStartPointObjectShared())
        registerInMap(editor.getYAxisShared()->getStartPointObjectShared()->getID(), editor.getYAxisShared()->getStartPointObjectShared());
      if (editor.getYAxisShared()->getEndPointObjectShared())
        registerInMap(editor.getYAxisShared()->getEndPointObjectShared()->getID(), editor.getYAxisShared()->getEndPointObjectShared());
    }

    // ========================================================================
    // PASS 1: Identity Registration (Points & ObjectPoints)
    // ========================================================================
    std::cout << "[3-Pass Engine] Pass 1: Identity Registration..." << std::endl;

    // 1a. Points (Standard, Transformation-derivied, Intersections, AND ObjectPoints)
    if (data.contains("points")) {
      for (const auto& jPt : data["points"]) {
        unsigned int id = jPt.value("id", 0u);
        if (id == 0) continue;

        double x = jPt.value("x", 0.0);
        double y = jPt.value("y", 0.0);
        sf::Color color = colorFromJson(jPt.contains("color") ? jPt["color"] : json(nullptr), sf::Color::Black);

        std::shared_ptr<Point> pt;
        
        // CRITICAL FIX: Detect and instantiate ObjectPoints from unified array
        bool isObjectPoint = jPt.value("isObjectPoint", false) || jPt.contains("hostId");
        if (isObjectPoint) {
          // Create ObjectPoint stub for Pass 3 relinking
          auto op = std::make_shared<ObjectPoint>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
          pt = op;
          
          // Store host relinking data for Pass 3
          PendingObjectPoint pending;
          pending.objectPoint = op;
          pending.hostId = jPt.value("hostId", 0u);
          pending.hostType = static_cast<ObjectType>(jPt.value("hostType", 0));
          pending.t = jPt.value("t", 0.0);
          pending.isShapeEdge = jPt.value("edgeIndex", -1) >= 0;
          if (pending.isShapeEdge) {
            pending.edgeIndex = jPt.value("edgeIndex", 0);
          }
          pendingObjectPoints.push_back(pending);
          
          // Add to ObjectPoints list
          editor.ObjectPoints.push_back(op);
        }
        // PASS 1: TYPE-AWARE INSTANTIATION (Prevent Type Erasure)
        // Instantiate correct derived class based on transform metadata
        else if (jPt.contains("transform") && jPt["transform"].is_object()) {
          const json& t = jPt["transform"];
          std::string type = t.value("type", "");
          
          // CRITICAL FIX: Read transform parameters from correct fields
          // The save code uses "transformValue" - use that with legacy fallbacks
          double transformValue = t.value("transformValue", 0.0);
          double angleDeg = t.value("angleDeg", transformValue);  // Legacy fallback
          double factor = t.value("factor", transformValue);      // Legacy fallback
          
          // Create the CORRECT derived class to preserve vtable
          if (type == "ReflectLine") {
            pt = std::make_shared<ReflectLine>(nullptr, nullptr, color, id);
          } else if (type == "ReflectPoint") {
            pt = std::make_shared<ReflectPoint>(nullptr, nullptr, color, id);
          } else if (type == "ReflectCircle") {
             pt = std::make_shared<ReflectCircle>(nullptr, nullptr, color, id);
          } else if (type == "RotatePoint") {
            pt = std::make_shared<RotatePoint>(nullptr, nullptr, angleDeg, color, id);
          } else if (type == "TranslateVector") {
            pt = std::make_shared<TranslateVector>(nullptr, nullptr, nullptr, color, id);
          } else if (type == "DilatePoint") {
            pt = std::make_shared<DilatePoint>(nullptr, nullptr, factor, color, id);
          } else {
            pt = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
          }
           
           // CRITICAL: Transformed points MUST be dependent.
           // JSON might say "isDependent": false if it was saved before the fix, or applyCommonPointFields might overwrite it.
           // We explicitly set it to true for these types.
           pt->setDependent(true); 

        } else if (jPt.value("isIntersectionPoint", false)) {
          pt = std::make_shared<IntersectionPoint>(nullptr, nullptr, Point_2(x, y), color, id);
          pt->setDependent(true); // Intersections are dependent
        } else {
          pt = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
        }

        if (pt) {
          pt->setCGALPosition(Point_2(x, y));
        }

        applyCommonPointFields(jPt, pt);
        
        // FIX: Re-enforce dependency if it was a transformed type, 
        // in case applyCommonPointFields overwrote it with 'false' from JSON.
        if (jPt.contains("transform") || jPt.value("isIntersectionPoint", false)) {
            pt->setDependent(true);
        }

        bool legacyTranslateWithoutStart = false;
        if (jPt.contains("transform") && jPt["transform"].is_object()) {
          const json& t = jPt["transform"];
          std::string type = t.value("type", "");
          if (type == "TranslateVector" && !t.contains("vecStartId") && !jPt.contains("transformType")) {
            legacyTranslateWithoutStart = true;
          }
        }

        if (!legacyTranslateWithoutStart) {
          applyTransformMetadata(jPt, pt);
        }
        if (jPt.contains("transform") && jPt["transform"].is_object()) {
          const json& t = jPt["transform"];
          std::string type = t.value("type", "");
          if (type == "TranslateVector") {
            json dp;
            dp["id"] = id;
            dp["vecStartId"] = t.value("vecStartId", 0u);
            dp["vecEndId"] = t.value("vecEndId", t.value("auxObjectID", 0u));
            deferredTransformPoints.push_back(dp);
          }
        }
        
        // Only add non-ObjectPoints to editor.points (ObjectPoints already added above)
        if (!isObjectPoint) {
          editor.points.push_back(pt);
        }
        registerInMap(id, pt);
      }
    }

    // 1b. ObjectPoints (Stubs)
    if (data.contains("objectPoints")) {
      for (const auto& jOp : data["objectPoints"]) {
        unsigned int id = jOp.value("id", 0u);
        sf::Color color = colorFromJson(jOp.contains("color") ? jOp["color"] : json(nullptr), sf::Color::Yellow);
        auto op = std::make_shared<ObjectPoint>(Point_2(0, 0), Constants::CURRENT_ZOOM, color, id);
        applyCommonPointFields(jOp, op);
        applyTransformMetadata(jOp, op);
        editor.ObjectPoints.push_back(op);
        registerInMap(id, op);
      }
    }

    // ========================================================================
    // PASS 2: Structural Pointer Re-linking (Lines, Circles, Shapes)
    // Constraint: ABSOLUTELY NO NEW POINTS. Fetch pointers from masterMap.
    // ========================================================================
    std::cout << "[3-Pass Engine] Pass 2: Structural Re-linking..." << std::endl;

    // 2a. Circles
    if (data.contains("circles")) {
      for (const auto& jCi : data["circles"]) {
        unsigned int id = jCi.value("id", 0u);
        auto center = getPointFromMap(jCi.value("centerPointID", 0u));
        if (!center) continue;

        auto radPt = getPointFromMap(jCi.value("radiusPointID", 0u));
        double r = jCi.value("radiusValue", jCi.value("radius", 10.0));
        sf::Color color = colorFromJson(jCi.contains("color") ? jCi["color"] : json(nullptr), sf::Color::Blue);

        auto ci = std::make_shared<Circle>(center.get(), radPt, r, color);
        ci->setID(id);
        ci->setThickness(jCi.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        if (jCi.contains("lineStyle")) {
          ci->setLineStyle(static_cast<LineStyle>(jCi.value("lineStyle", 0)));
        }
        applyTransformMetadata(jCi, ci);
        editor.circles.push_back(ci);
        registerInMap(id, ci);
      }
    }

    // 2b. Lines
    if (data.contains("lines")) {
      for (const auto& jLn : data["lines"]) {
        unsigned int id = jLn.value("id", 0u);
        auto start = getPointFromMap(jLn.value("startPointID", jLn.value("startId", 0u)));
        auto end = getPointFromMap(jLn.value("endPointID", jLn.value("endId", 0u)));
        if (!start || !end) continue;

        sf::Color color = colorFromJson(jLn.contains("color") ? jLn["color"] : json(nullptr), sf::Color::Green);
        auto ln = std::make_shared<Line>(start, end, jLn.value("isSegment", false), color, id);
        ln->registerWithEndpoints();
        ln->setThickness(jLn.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        if (jLn.contains("lineStyle")) {
          ln->setLineStyle(static_cast<LineStyle>(jLn.value("lineStyle", 0)));
        }
        ln->setLineType(static_cast<Line::LineType>(jLn.value("lineType", 1)));
        applyTransformMetadata(jLn, ln);
        editor.lines.push_back(ln);
        registerInMap(id, ln);

        bool isParallel = jLn.value("isParallel", false);
        bool isPerpendicular = jLn.value("isPerpendicular", false);
        if (isParallel || isPerpendicular) {
          PendingLineConstraint pending;
          pending.line = ln;
          pending.isParallel = isParallel;
          pending.isPerpendicular = isPerpendicular;
          pending.refId = static_cast<unsigned int>(jLn.value("constraintRefId", 0u));
          pending.edgeIndex = jLn.value("constraintRefEdgeIndex", -1);
          pendingLineConstraints.push_back(pending);
        }
      }
    }
    // Helper to check if any vertex ID is in deferredTransformPoints
    auto isVertexPending = [&](int vId) -> bool {
      for (const auto& dp : deferredTransformPoints) {
        if (dp.value("id", -1) == vId) return true;
      }
      return false;
    };

    auto orderRectangleVertices = [&](std::vector<std::shared_ptr<Point>>& pts) {
      if (pts.size() != 4) return;
      double cx = 0.0;
      double cy = 0.0;
      for (const auto& p : pts) {
        Point_2 pos = p->getCGALPosition();
        cx += CGAL::to_double(pos.x());
        cy += CGAL::to_double(pos.y());
      }
      cx *= 0.25;
      cy *= 0.25;

      std::vector<std::pair<double, std::shared_ptr<Point>>> sorted;
      sorted.reserve(4);
      for (const auto& p : pts) {
        Point_2 pos = p->getCGALPosition();
        double dx = CGAL::to_double(pos.x()) - cx;
        double dy = CGAL::to_double(pos.y()) - cy;
        double angle = std::atan2(dy, dx);
        sorted.push_back({angle, p});
      }

      std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

      std::vector<std::shared_ptr<Point>> ordered;
      ordered.reserve(4);
      for (const auto& pair : sorted) ordered.push_back(pair.second);

      int bestStart = 0;
      double bestScore = -1e18;
      for (int i = 0; i < 4; ++i) {
        Point_2 pos = ordered[i]->getCGALPosition();
        double score = CGAL::to_double(pos.y()) - CGAL::to_double(pos.x());
        if (score > bestScore) {
          bestScore = score;
          bestStart = i;
        }
      }

      std::rotate(ordered.begin(), ordered.begin() + bestStart, ordered.end());
      std::swap(ordered[1], ordered[3]);
      pts = ordered;
    };
    
    // 2c. Rectangles (Corrected Strategy C to handle Legacy Files)
    if (data.contains("rectangles")) {
        for (const auto& jRect : data["rectangles"]) {
            unsigned int id = jRect.value("id", 0u);
            
            // --------------------------------------------------------
            // STEP 1: Gather Vertices (Try 3 Strategies)
            // --------------------------------------------------------
            std::vector<std::shared_ptr<Point>> vPts;
            bool usedIdStrategy = false; // Track if IDs resolved the vertices
            
            // Strategy A: Explicit Corner IDs (Preferred)
            // 5-arg constructor expects: [corner1, corner2, cornerB, cornerD]
            // We need to support both legacy keys (corner1ID...) and new format (vertexIds).
            
            // Try Legacy Keys First (Most reliable for corner mapping)
            if (jRect.contains("corner1ID")) {
                auto c1 = getPointFromMap(jRect.value("corner1ID", 0u));
                auto c2 = getPointFromMap(jRect.value("corner2ID", 0u));
                // FIX 11: Save writes "cornerBID" and "cornerDID" — use those directly.
                auto cb = getPointFromMap(jRect.value("cornerBID", 0u));
                auto cd = getPointFromMap(jRect.value("cornerDID", 0u));
                
                if (c1 && c2 && cb && cd) {
                    vPts = {c1, c2, cb, cd};
                    usedIdStrategy = true;
                    std::cout << "    [RectLoad] ID=" << id << " Strategy A (cornerXID): OK" << std::endl;
                } else {
                    std::cout << "    [RectLoad] ID=" << id << " Strategy A FAILED: c1=" << (bool)c1 << " c2=" << (bool)c2 << " cb=" << (bool)cb << " cd=" << (bool)cd << std::endl;
                }
            } 
            // Fallback to vertexIds array if present and legacy keys failed
            else if (jRect.contains("vertexIds") && jRect["vertexIds"].is_array()) {
                 std::vector<std::shared_ptr<Point>> tempPts;
                 for (const auto& idVal : jRect["vertexIds"]) {
                    int vid = idVal.get<int>();
                    if (vid > 0) {
                        auto p = getPointFromMap(static_cast<unsigned int>(vid));
                        if(p) tempPts.push_back(p);
                    }
                }
                if (tempPts.size() == 4) {
                    vPts = tempPts; // Assume they are saved in constructor order [c1, c2, cb, cd]
                    usedIdStrategy = true;
                }
            }

            // Strategy C: Raw Coordinates ("vertices" array) - Legacy fallback
            if (vPts.size() != 4 && jRect.contains("vertices") && jRect["vertices"].is_array()) {
                vPts.clear();
                const double tolSq = 1e-6;
                for (size_t i = 0; i < 4; ++i) {
                    const auto& v = jRect["vertices"][i];
                    if (!v.is_array() || v.size() < 2) continue;
                    Point_2 pos(FT(v[0].get<double>()), FT(v[1].get<double>()));
                    
                    auto match = getPointByPosition(pos, tolSq);
                    
                    if (match) {
                        vPts.push_back(match);
                    } else {
                        auto newPt = std::make_shared<Point>(pos, 1.0);
                        unsigned int newPtID = editor.objectIdCounter++;
                        newPt->setID(newPtID);
                        newPt->setVisible(true);
                        editor.points.push_back(newPt);
                        registerInMap(newPtID, newPt);
                        vPts.push_back(newPt);
                    }
                }
                // usedIdStrategy stays false — angular sorting needed
            }

            // --------------------------------------------------------
            // STEP 2: Create Rectangle
            // --------------------------------------------------------
            if (vPts.size() == 4) {
                bool isRot = jRect.value("isRotatable", true);
                std::shared_ptr<Rectangle> rect;

                if (usedIdStrategy) {
                    // Strategy A/B: vPts is in 5-arg constructor member order
                    // [corner1, corner2, cornerB, cornerD]
                    // Pass directly — NO angular sorting, NO swaps.
                    rect = std::make_shared<Rectangle>(
                        vPts[0], vPts[1], vPts[2], vPts[3],
                        isRot,
                        colorFromJson(jRect["color"], sf::Color::Black),
                        id
                    );
                    // FIX: Load Label Data
                    if (jRect.contains("label")) rect->setLabel(jRect.value("label", ""));
                    if (jRect.contains("showLabel")) rect->setShowLabel(jRect.value("showLabel", false));
                    if (jRect.contains("labelOffsetX") && jRect.contains("labelOffsetY")) {
                        rect->setLabelOffset(sf::Vector2f(jRect["labelOffsetX"], jRect["labelOffsetY"]));
                    }
                } else {
                    // Strategy C: raw coordinates — need angular sort for perimeter order
                    orderRectangleVertices(vPts);

                    // Sanity check for crossed edges
                    Point_2 p0 = vPts[0]->getCGALPosition();
                    Point_2 p1 = vPts[1]->getCGALPosition();
                    Point_2 p2 = vPts[2]->getCGALPosition();
                    Vector_2 v1 = p1 - p0;
                    Vector_2 v2 = p2 - p0;
                    double crossZ = CGAL::to_double(v1.x() * v2.y() - v1.y() * v2.x());
                    if (std::abs(crossZ) < 1e-3) {
                        std::swap(vPts[1], vPts[2]);
                    }

                    // Raw coords are in perimeter order [A,B,C,D]
                    // Use vector constructor which expects perimeter order
                    rect = std::make_shared<Rectangle>(
                        vPts,
                        isRot,
                        colorFromJson(jRect["color"], sf::Color::Black),
                        id
                    );
                }

                rect->setThickness(jRect.value("thickness", 2.0f));
                if (jRect.contains("lineStyle")) {
                  rect->setLineStyle(static_cast<LineStyle>(jRect.value("lineStyle", 0)));
                }
                finalizeExplicitRectangle(jRect, rect, vPts);
                
                // CRITICAL FIX: Ensure vertices are visible for Rotatable Rectangles
                // Deep Harvesting might have saved them as hidden, but they must be visible to be editable.
                if (isRot) {
                    for (auto& p : vPts) {
                        if (p) p->setVisible(true);
                    }
                }

                for(auto& p : vPts) p->addDependent(rect);
                
                editor.rectangles.push_back(rect);
                registerInMap(id, rect);
            }
        }
    }
    // 2d. Load Polygons
    if (data.contains("polygons")) {
      for (const auto& polyJson : data["polygons"]) {
        std::vector<std::shared_ptr<Point>> vertexPoints;

        bool deferPoly = false;
        if (polyJson.contains("vertexIds") && polyJson["vertexIds"].is_array()) {
          for (const auto& idJson : polyJson["vertexIds"]) {
            int vId = idJson.get<int>();
            if (isVertexPending(vId)) {
              deferPoly = true;
              break;
            }
            auto vPtr = getPointFromMap(static_cast<unsigned int>(vId));
            if (!vPtr) {
              // vertex missing entirely (yet), so must defer
              deferPoly = true;
              break;
            }
            vertexPoints.push_back(vPtr);
          }
        } else if (polyJson.contains("vertices")) {
          // STRICT LEGACY FALLBACK: Only if vertexIds is missing
           if (!polyJson.contains("vertexIds")) {
              for (const auto& vJson : polyJson["vertices"]) {
                 std::shared_ptr<Point> vPtr = nullptr;
                 Point_2 pos;
                 bool validCoord = false;

                 if (vJson.is_array() && vJson.size() >= 2) {
                   pos = Point_2(vJson[0].get<double>(), vJson[1].get<double>());
                   validCoord = true;
                 } else if (vJson.is_object()) {
                   pos = Point_2(vJson.value("x", 0.0), vJson.value("y", 0.0));
                   validCoord = true;
                 }
                 
                 if (validCoord) {
                     vPtr = getPointByPosition(pos, 1e-6);
                     if (!vPtr) {
                        // Not found? Create it! (Fixing Locked Vertices / Missing Data)
                        auto newPt = std::make_shared<Point>(pos, 1.0);
                        unsigned int newPtID = editor.objectIdCounter++;
                        newPt->setID(newPtID);
                        newPt->setVisible(true); // Default to visible
                        
                        editor.points.push_back(newPt);
                        registerInMap(newPtID, newPt);
                        vPtr = newPt;
                     }
                     vertexPoints.push_back(vPtr);
                 }
              }
           }
        }

        if (deferPoly || vertexPoints.size() < 3) {
          pendingPolygons.push_back(polyJson);
          continue;
        }

        sf::Color color = sf::Color::Black;
        if (polyJson.contains("color")) {
          color = colorFromJson(polyJson["color"], color);
        }

        int id = polyJson.value("id", -1);
        auto polygon = std::make_shared<Polygon>(vertexPoints, color, static_cast<unsigned int>(std::max(id, 0)));
        
        // CRITICAL FIX: Ensure all polygon vertices are visible
        for (auto& p : vertexPoints) {
            if (p) p->setVisible(true);
        }

        polygon->setThickness(polyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        if (polyJson.contains("lineStyle")) {
          polygon->setLineStyle(static_cast<LineStyle>(polyJson.value("lineStyle", 0)));
        }

        applyTransformMetadata(polyJson, polygon);

        // FIX: Load Label Data
        if (polyJson.contains("label")) polygon->setLabel(polyJson.value("label", ""));
        if (polyJson.contains("showLabel")) polygon->setShowLabel(polyJson.value("showLabel", false));
        if (polyJson.contains("labelOffsetX") && polyJson.contains("labelOffsetY")) {
             polygon->setLabelOffset(sf::Vector2f(polyJson["labelOffsetX"], polyJson["labelOffsetY"]));
        }

        editor.polygons.push_back(polygon);
        if (id > 0) registerInMap(static_cast<unsigned int>(id), polygon);
      }
    }


    // 2e. Triangles
    if (data.contains("triangles")) {
      for (const auto& jTri : data["triangles"]) {
        unsigned int id = jTri.value("id", 0u);

        if (jTri.contains("vertexIds") && jTri["vertexIds"].size() >= 3) {
          auto p1 = getPointFromMap(jTri["vertexIds"][0]);
          auto p2 = getPointFromMap(jTri["vertexIds"][1]);
          auto p3 = getPointFromMap(jTri["vertexIds"][2]);

          if (p1 && p2 && p3) {
            auto tri = std::make_shared<Triangle>(p1, p2, p3, colorFromJson(jTri["color"], sf::Color::Black), id);
            tri->setThickness(jTri.value("thickness", 2.0));
            if (jTri.contains("lineStyle")) {
              tri->setLineStyle(static_cast<LineStyle>(jTri.value("lineStyle", 0)));
            }
            // FIX: Load Label Data
            if (jTri.contains("label")) tri->setLabel(jTri.value("label", ""));
            if (jTri.contains("showLabel")) tri->setShowLabel(jTri.value("showLabel", false));
            if (jTri.contains("labelOffsetX") && jTri.contains("labelOffsetY")) {
                 tri->setLabelOffset(sf::Vector2f(jTri["labelOffsetX"], jTri["labelOffsetY"]));
            }
            applyTransformMetadata(jTri, tri);

            p1->addDependent(tri);
            p2->addDependent(tri);
            p3->addDependent(tri);

            editor.triangles.push_back(tri);
            registerInMap(id, tri);
          }
        }
      }
    }

    // 2f. Regular Polygons (e.g. Hexagons)
    if (data.contains("regularPolygons")) {
      for (const auto& jRp : data["regularPolygons"]) {
        unsigned int id = jRp.value("id", 0u);
        // These depend on Center + First Vertex, NOT a list of vertices
        auto center = getPointFromMap(jRp.value("centerPointID", 0u));
        auto v1 = getPointFromMap(jRp.value("firstVertexPointID", 0u));

        if (center && v1) {
          int sides = jRp.value("sides", 5);
          auto rp = std::make_shared<RegularPolygon>(center, v1, sides, colorFromJson(jRp["color"], sf::Color::Black), 0);
          rp->setID(id);  // Set ID before map registration
          rp->setThickness(jRp.value("thickness", 2.0));
          if (jRp.contains("lineStyle")) {
            rp->setLineStyle(static_cast<LineStyle>(jRp.value("lineStyle", 0)));
          }
          applyTransformMetadata(jRp, rp);
          editor.regularPolygons.push_back(rp);
          registerInMap(id, rp);
        }
      }
    }

    // 2e. Tangents
    if (data.contains("tangentLines")) {
      for (const auto& jTan : data["tangentLines"]) {
        auto ext = getPointFromMap(jTan.value("externalPointID", 0u));
        auto cir = std::dynamic_pointer_cast<Circle>(getFromMap(jTan.value("circleID", 0u)));
        if (!ext || !cir) continue;
        auto tan = std::make_shared<TangentLine>(ext, cir, jTan.value("solutionIndex", 0), jTan.value("id", 0u),
                                                 colorFromJson(jTan["color"], sf::Color::Cyan));
        editor.lines.push_back(tan);
        registerInMap(tan->getID(), tan);
      }
    }

    // 2f. Angles
    if (data.contains("angles")) {
      for (const auto& jAngle : data["angles"]) {
        auto p1 = getPointFromMap(jAngle.value("vertexID", 0u));
        auto p2 = getPointFromMap(jAngle.value("pointAID", 0u));
        auto p3 = getPointFromMap(jAngle.value("pointBID", 0u));
        if (!p1 || !p2 || !p3) continue;
        auto angle = std::make_shared<Angle>(p2, p1, p3, jAngle.value("reflex", false));
        angle->setID(jAngle.value("id", 0u));
        angle->setRadius(jAngle.value("radius", 20.0));
        angle->setColor(colorFromJson(jAngle["color"], sf::Color::Yellow));
        editor.angles.push_back(angle);
        registerInMap(angle->getID(), angle);
      }
    }

    // ========================================================================
    // PASS 3: Constraint & Dependency Wake-up (Live Relinking)
    // ========================================================================
    std::cout << "[3-Pass Engine] Pass 3: Dependency Wake-up..." << std::endl;

    // 3a. Re-link ObjectPoints host pointers (Moved to START of Pass 3)
    // Must be done before dependency wake-up so dependent objects (like PerpendicularLines) see valid host positions.
    
    // CRITICAL FIX: Re-link ObjectPoints loaded from unified "points" array
    std::cout << "  Linking ObjectPoints from unified array..." << std::endl;
    for (const auto& pending : pendingObjectPoints) {
      if (!pending.objectPoint) continue;
      auto host = getFromMap(pending.hostId);
      if (host) {
        pending.objectPoint->relinkHost(host, pending.t, pending.hostType);
        
        // Manually register with host since setHost() doesn't call addChildPoint()
        if (pending.hostType == ObjectType::Line || pending.hostType == ObjectType::LineSegment) {
          if (auto l = std::dynamic_pointer_cast<Line>(host)) l->addChildPoint(pending.objectPoint);
        } else if (pending.hostType == ObjectType::Circle) {
          if (auto c = std::dynamic_pointer_cast<Circle>(host)) c->addChildPoint(pending.objectPoint);
        }
      }
    }
    
    // Legacy ObjectPoints array (for backward compatibility)
    if (data.contains("objectPoints")) {
      std::cout << "  Linking ObjectPoints from legacy array..." << std::endl;
      for (const auto& jOp : data["objectPoints"]) {
        auto op = std::dynamic_pointer_cast<ObjectPoint>(getFromMap(jOp.value("id", 0u)));
        if (!op) continue;
        auto host = getFromMap(jOp.value("hostId", 0u));
        if (host) {
            ObjectType type = static_cast<ObjectType>(jOp.value("hostType", 0));
            op->relinkHost(host, jOp.value("t", 0.0), type);
            
            // Manually register with host since setHost() doesn't call addChildPoint()
            if (type == ObjectType::Line || type == ObjectType::LineSegment) {
                if (auto l = std::dynamic_pointer_cast<Line>(host)) l->addChildPoint(op);
            } else if (type == ObjectType::Circle) {
                if (auto c = std::dynamic_pointer_cast<Circle>(host)) c->addChildPoint(op);
            }
        }
      }
    }

    // PASS 3a: COMPREHENSIVE TRANSFORMATION RE-LINKING
    // This is where we "wake up" transformed objects by restoring their source/aux pointers
    std::cout << "  Re-linking transformations..." << std::endl;
    
    int transformCount = 0;
    // Strategy: Iterate all objects and reconnect transformation dependencies
    for (auto& [id, obj] : masterMap) {
      if (!obj) continue;
      if (obj->getTransformType() == TransformationType::None && !obj->isDependent()) continue;
      
      // Get source and auxiliary objects from IDs
      auto parent = getFromMap(obj->getParentSourceID());
      auto aux = getFromMap(obj->getAuxObjectID());
      
      if (parent) {
        // Generic relinking (handles most cases)
        obj->relinkTransformation(parent, aux);
        transformCount++;
        
        // Type-specific post-linking for complex transformations
        if (auto pt = std::dynamic_pointer_cast<Point>(obj)) {
          // Ensure position is calculated from transform
          pt->update();
          
          // Debug output for first few points
          if (transformCount <= 3) {
            std::cout << "    [DEBUG] Restored transform for Point ID=" << id 
                      << " type=" << static_cast<int>(obj->getTransformType())
                      << " parent=" << (parent ? parent->getID() : 0)
                      << " aux=" << (aux ? aux->getID() : 0) << std::endl;
          }
          
          // Special handling for TranslateVector (needs vector endpoints)
          if (auto tv = std::dynamic_pointer_cast<TranslateVector>(pt)) {
            // TranslateVector might have vecStart/vecEnd stored separately
            // (These are handled by deferredTransformPoints below)
          }
        }
      }
    }
    std::cout << "    Relinked " << transformCount << " transformed objects." << std::endl;

    // PASS 3a.1: TranslateVector Special Case (Legacy & New Format)
    // TranslateVector needs the vector start/end points restored
    std::cout << "  Re-linking TranslateVector points..." << std::endl;
    for (const auto& dp : deferredTransformPoints) {
      unsigned int pid = dp.value("id", 0u);
      if (pid == 0u) continue;
      
      auto p = getPointFromMap(pid);
      auto tv = std::dynamic_pointer_cast<TranslateVector>(p);
      if (!tv) continue;
      
      auto vStart = getPointFromMap(dp.value("vecStartId", 0u));
      auto vEnd = getPointFromMap(dp.value("vecEndId", 0u));
      
      if (vStart) tv->setVectorStart(vStart);
      if (vEnd) tv->setVectorEnd(vEnd);
      
      // Force update to calculate position from vector
      tv->update();
    }



    // 3b.1. Restore line constraints (parallel/perpendicular)
    for (const auto& pending : pendingLineConstraints) {
      if (!pending.line) continue;
      auto refObj = getFromMap(pending.refId);
      Vector_2 refDir(0, 0);
      if (refObj) {
        if (pending.edgeIndex >= 0) {
          auto edges = refObj->getEdges();
          if (pending.edgeIndex < static_cast<int>(edges.size())) {
            refDir = edges[static_cast<size_t>(pending.edgeIndex)].to_vector();
          }
        } else if (auto refLine = std::dynamic_pointer_cast<Line>(refObj)) {
          refDir = refLine->getEndPoint() - refLine->getStartPoint();
        }
      }
      if (pending.isParallel) {
        pending.line->setAsParallelLine(refObj, pending.edgeIndex, refDir);
      } else if (pending.isPerpendicular) {
        pending.line->setAsPerpendicularLine(refObj, pending.edgeIndex, refDir);
      }
    }

    // 3c. Register Intersections
    if (data.contains("intersections")) {
      for (const auto& iJson : data["intersections"]) {
        auto A = getFromMap(iJson.value("aId", 0u));
        auto B = getFromMap(iJson.value("bId", 0u));
        if (!A || !B) continue;
        std::vector<std::shared_ptr<Point>> pts;
        for (const auto& pJ : iJson["points"]) {
          auto p = getPointFromMap(pJ.value("id", 0u));
          if (p) pts.push_back(p);
        }
        if (!pts.empty()) DynamicIntersection::registerIntersectionConstraint(A, B, pts);
      }
    }

    // Finalize: Restore visuals & global state
    for (auto& [id, obj] : masterMap)
      if (obj) obj->update();
    if (editor.getXAxisShared()) editor.getXAxisShared()->setVisible(axesVisible);
    if (editor.getYAxisShared()) editor.getYAxisShared()->setVisible(axesVisible);
    editor.objectIdCounter = maxId + 1;

    // CRITICAL: Enforce label policy on all loaded objects (respects saved user overrides)
    editor.enforceLabelPolicyOnAll(false);  // false = don't clear user overrides

    std::cout << "[3-Pass Engine] Success. Restored " << masterMap.size() << " objects." << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "ProjectSerializer::loadProject: Exception: " << e.what() << std::endl;
    return false;
  }
}
#endif

// ============================================================================
// EXPORT SVG
// ============================================================================

// Helper to clip infinite lines to the view box
static std::vector<std::pair<double, double>> getBoxIntersections(const Line_2& line, double minX, double minY, double maxX, double maxY) {
  std::vector<std::pair<double, double>> intersections;

  Point_2 p1(minX, minY);
  Point_2 p2(maxX, minY);
  Point_2 p3(maxX, maxY);
  Point_2 p4(minX, maxY);

  Segment_2 segments[4] = {Segment_2(p1, p2), Segment_2(p2, p3), Segment_2(p3, p4), Segment_2(p4, p1)};

  for (int i = 0; i < 4; ++i) {
    auto result = CGAL::intersection(line, segments[i]);
    if (result) {
      if (const Point_2* p = std::get_if<Point_2>(&*result)) {
        intersections.push_back({CGAL::to_double(p->x()), CGAL::to_double(p->y())});
      } else if (const Segment_2* s = std::get_if<Segment_2>(&*result)) {
        intersections.push_back({CGAL::to_double(s->source().x()), CGAL::to_double(s->source().y())});
        intersections.push_back({CGAL::to_double(s->target().x()), CGAL::to_double(s->target().y())});
      }
    }
  }
  return intersections;
}

bool ProjectSerializer::exportSVG(const GeometryEditor& editor, const std::string& filepath) {
  try {
    // STEP 1: Compute bounds from CURRENT VIEW (WYSIWYG)
    sf::View view = editor.drawingView;
    sf::Vector2f center = view.getCenter();
    sf::Vector2f size = view.getSize();

    // Calculate visible bounds in World Coordinates
    // (minX, minY) is Top-Left of the visible area
    double width = size.x;
    double height = size.y;  // Can be negative if flipped, take abs
    if (width < 0) width = -width;
    if (height < 0) height = -height;

    double minX = center.x - width / 2.0;
    double minY = center.y - height / 2.0;
    double maxX = minX + width;
    double maxY = minY + height;

    // Initialize SVG Writer
    SVGWriter svg(width, height);
    // Use visible world bounds as viewBox directly
    svg.setViewBox(minX, minY, width, height);

    // // ========================================================================
    // // STEP 4: Write SVG header with viewBox
    // // ========================================================================
    // // Ensure aspect ratio of the output image matches the view bounds
    // double outputWidth = 800.0;
    // double outputHeight = 800.0;

    // if (width > 0 && height > 0) {
    //   double aspectRatio = width / height;
    //   if (aspectRatio > 1.0) {
    //     outputWidth = 800.0;
    //     outputHeight = 800.0 / aspectRatio;
    //   } else {
    //     outputHeight = 800.0;
    //     outputWidth = 800.0 * aspectRatio;
    //   }
    // }
    // svg.setOutputSize(outputWidth, outputHeight);

    // ========================================================================
    // STEP 4: Dynamically determine SVG output size from the program window
    // ========================================================================
    sf::Vector2u windowSize = editor.window.getSize();
    double outputWidth = static_cast<double>(windowSize.x);
    double outputHeight = static_cast<double>(windowSize.y);

    // Update the SVG Writer with these dynamic dimensions
    svg.setOutputSize(outputWidth, outputHeight);

    // Background
    SVGWriter::Style bgStyle;
    bgStyle.fill = "white";
    bgStyle.stroke = "none";
    svg.drawRect(minX, minY, width, height, bgStyle);

    // Scaling Factor (now accurately reflects pixels-to-world units)
    double pixelToWorldScale = width / outputWidth;
    double strokeWidth = 1.0 * pixelToWorldScale;
    double pointRadius = 2.0 * pixelToWorldScale;
    double gridStrokeWidth = 0.5 * pixelToWorldScale;

    // Grid (Adaptive Spacing matched to Editor)
    double adaptiveGridStep = Constants::GRID_SIZE;
    // Logic ported from Grid::update
    double minScreenGap = 50.0;
    double minWorldGap = minScreenGap * pixelToWorldScale;

    double magnitude = std::pow(10.0, std::floor(std::log10(minWorldGap)));
    double residual = minWorldGap / magnitude;

    if (residual > 5.0)
      adaptiveGridStep = 10.0 * magnitude;
    else if (residual > 2.0)
      adaptiveGridStep = 5.0 * magnitude;
    else if (residual > 1.0)
      adaptiveGridStep = 2.0 * magnitude;
    else
      adaptiveGridStep = 1.0 * magnitude;

    // Draw Grid Lines (Direct World Coordinates)
    if (editor.isGridVisible()) {
      SVGWriter::Style gridStyle;
      gridStyle.stroke = "#e0e0e0";
      gridStyle.strokeWidth = gridStrokeWidth;
      gridStyle.fill = "none";

      // 1. Vertical Lines (X is unchanged in flip)
      // We draw from Top (minY) to Bottom (maxY) coverage
      double startX = std::floor(minX / adaptiveGridStep) * adaptiveGridStep;
      for (double x = startX; x <= maxX; x += adaptiveGridStep) {
        svg.drawLine(x, minY, x, maxY, gridStyle);
      }

      // 2. Horizontal Lines (FIX: Apply Y-Flip)
      double startY = std::floor(minY / adaptiveGridStep) * adaptiveGridStep;
      for (double y = startY; y <= maxY; y += adaptiveGridStep) {
        // BUG FIX: Map World Y to SVG Screen Y
        double sy = (minY + maxY) - y;

        svg.drawLine(minX, sy, maxX, sy, gridStyle);
      }
    }

    // Axes
    SVGWriter::Style axesStyle;
    axesStyle.stroke = "#000000";
    axesStyle.strokeWidth = gridStrokeWidth;

    // 1. Calculate the SVG Screen Y position for the World X-Axis (y=0)
    // Formula: ScreenY = (ViewTop + ViewBottom) - WorldY
    double xAxisScreenY = (minY + maxY) - 0.0;

    // 2. Calculate the SVG Screen X position for the World Y-Axis (x=0)
    double yAxisScreenX = 0.0;  // World X=0 is already SVG X=0 relative to the viewBox

    // 3. Draw the X-Axis line using Screen Coordinates
    if (editor.getXAxisShared() && editor.getXAxisShared()->isVisible()) {
      // We use minX and maxX for the horizontal span, but xAxisScreenY for the vertical position
      svg.drawLine(minX, xAxisScreenY, maxX, xAxisScreenY, axesStyle);
    }

    // 4. Draw the Y-Axis line
    if (editor.getYAxisShared() && editor.getYAxisShared()->isVisible()) {
      // Y-axis spans from minY to maxY in the vertical, positioned at yAxisScreenX
      svg.drawLine(yAxisScreenX, minY, yAxisScreenX, maxY, axesStyle);
    }

    // Axis Labels
    SVGWriter::Style axisLabelStyle;
    axisLabelStyle.fill = "black";
    axisLabelStyle.stroke = "none";
    axisLabelStyle.fontSize = 18.0 * pixelToWorldScale;
    axisLabelStyle.textAnchor = "middle";
    // Axis Labels (X-Axis numbers)
    // X-Axis Numbering (1, 2, 3...)
    double startX = std::floor(minX / adaptiveGridStep) * adaptiveGridStep;
    for (double x = startX; x <= maxX; x += adaptiveGridStep) {
      if (std::abs(x) < 0.001) continue;  // Skip origin

      // Map World X to SVG X, and Y=0 to the screen Y position of the X-axis
      double sx = x;
      double sy = (minY + maxY) - 0.0;  // The Y-axis is at world 0

      // Add a small vertical offset so numbers don't sit ON the line
      svg.drawText(sx, sy + (15.0 * pixelToWorldScale), std::to_string((int)x), axisLabelStyle);
    }

    // Y-Axis Numbering
    axisLabelStyle.textAnchor = "end";  // Align numbers to the right of their position
    double startY = std::floor(minY / adaptiveGridStep) * adaptiveGridStep;
    for (double y = startY; y <= maxY; y += adaptiveGridStep) {
      if (std::abs(y) < 0.001) continue;

      double sx = 0.0;  // The X-axis is at world 0
      double sy = (minY + maxY) - y;

      // Add a small horizontal offset
      svg.drawText(sx - (5.0 * pixelToWorldScale), sy, std::to_string((int)y), axisLabelStyle);
    }
    // NO FLIPPED GROUP - Direct Rendering
    // Geometry Group: Apply Flip (Y-Up World -> Y-Down SVG)
    // We translate by (minY + maxY) and scale Y by -1 to flip around the center of the view.
    svg.beginGroup("translate(0," + std::to_string(minY + maxY) + ") scale(1,-1)");
    // Draw Rectangles
    for (const auto& rect : editor.rectangles) {
      if (rect && rect->isValid() && rect->isVisible()) {
        SVGWriter::Style style;
        style.stroke = colorToHex(rect->getColor());
        style.strokeWidth = rect->getThickness() * pixelToWorldScale;
        sf::Color fillCol = rect->getFillColor();

        // If Alpha is 0, it's effectively "none"
        if (fillCol.a > 0) {
          style.fill = colorToHex(fillCol);
          style.opacity = fillCol.a / 255.0f;  // Convert 0-255 to 0.0-1.0
        } else {
          style.fill = "none";
        }

        std::vector<std::pair<double, double>> coords;
        // Use Interactable Vertices (World Coords)
        auto vertices = rect->getInteractableVertices();
        for (const auto& v : vertices) {
          coords.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        if (!coords.empty()) {
          svg.drawPolygon(coords, style);
        }
      }
    }

    // Draw Polygons
    for (const auto& poly : editor.polygons) {
      if (poly && poly->isValid() && poly->isVisible()) {
        SVGWriter::Style style;
        style.stroke = colorToHex(poly->getColor());
        style.strokeWidth = poly->getThickness() * pixelToWorldScale;
        sf::Color fillCol = poly->getFillColor();
        if (fillCol.a > 0) {
          style.fill = colorToHex(fillCol);
          style.opacity = fillCol.a / 255.0f;  // Ensure SVGWriter supports fillOpacity
        } else {
          style.fill = "none";
        }

        std::vector<std::pair<double, double>> coords;
        auto vertices = poly->getVertices();
        for (const auto& v : vertices) {
          coords.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        if (!coords.empty()) {
          svg.drawPolygon(coords, style);
        }
      }
    }

    // Draw Lines
    for (const auto& ln : editor.lines) {
      if (ln && ln->isValid() && ln->isVisible()) {
        // Skip axes (already drawn)
        if (ln == editor.getXAxisShared() || ln == editor.getYAxisShared()) continue;

        SVGWriter::Style style;
        style.stroke = colorToHex(ln->getColor());
        style.strokeWidth = ln->getThickness() * pixelToWorldScale;

        Point_2 start = ln->getStartPoint();
        Point_2 end = ln->getEndPoint();

        // Clip to view? Not strictly necessary, SVG handles large coords.
        // But for infinite lines, we MUST clip.
        if (ln->isSegment()) {
          svg.drawLine(CGAL::to_double(start.x()), CGAL::to_double(start.y()), CGAL::to_double(end.x()), CGAL::to_double(end.y()), style);
        } else {
          // Infinite line clipping to bounding box
          Point_2 p1(minX, minY), p2(maxX, minY), p3(maxX, maxY), p4(minX, maxY);
          auto intersections = getBoxIntersections(ln->getCGALLine(), minX, minY, maxX, maxY);
          if (intersections.size() >= 2) {
            svg.drawLine(intersections[0].first, intersections[0].second, intersections[1].first, intersections[1].second, style);
          }
        }

        // Decorations (Arrows etc) - Omitted for brevity/safety unless crucial
      }
    }

    // Draw Circles
    for (const auto& ci : editor.circles) {
      if (ci && ci->isValid() && ci->isVisible()) {
        Point_2 center = ci->getCGALPosition();
        double r = ci->getRadius();
        SVGWriter::Style style;
        style.stroke = colorToHex(ci->getColor());
        style.strokeWidth = 2.0 * pixelToWorldScale;
        sf::Color fillCol = ci->getFillColor();

        // If Alpha is 0, it's effectively "none"
        if (fillCol.a > 0) {
          style.fill = colorToHex(fillCol);
          style.opacity = fillCol.a / 255.0f;  // Convert 0-255 to 0.0-1.0
        } else {
          style.fill = "none";
        }

        svg.drawCircle(CGAL::to_double(center.x()), CGAL::to_double(center.y()), r, style);
      }
    }

    // Draw Triangles
    for (const auto& tri : editor.triangles) {
      if (tri && tri->isValid() && tri->isVisible()) {
        SVGWriter::Style style;
        style.stroke = colorToHex(tri->getColor());
        style.strokeWidth = tri->getThickness() * pixelToWorldScale;
        sf::Color fillCol = tri->getFillColor();
        // If Alpha is 0, it's effectively "none"
        if (fillCol.a > 0) {
          style.fill = colorToHex(fillCol);
          style.opacity = fillCol.a / 255.0f;  // Convert 0-255 to 0.0-1.0
        } else {
          style.fill = "none";
        }

        std::vector<Point_2> verts = tri->getVertices();
        std::vector<std::pair<double, double>> coords;
        for (const auto& v : verts) {
          coords.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        if (coords.size() == 3) {
          svg.drawPolygon(coords, style);
        }
      }
    }

    // Draw Regular Polygons
    for (const auto& rpoly : editor.regularPolygons) {
      if (rpoly && rpoly->isValid() && rpoly->isVisible()) {
        SVGWriter::Style style;
        style.stroke = colorToHex(rpoly->getColor());
        style.strokeWidth = 0.5 * rpoly->getThickness() * pixelToWorldScale;
        sf::Color fillCol = rpoly->getFillColor();
        // If Alpha is 0, it's effectively "none"
        if (fillCol.a > 0) {
          style.fill = colorToHex(fillCol);
          style.opacity = fillCol.a / 255.0f;  // Convert 0-255 to 0.0-1.0
        } else {
          style.fill = "none";
        }

        std::vector<Point_2> verts = rpoly->getVertices();
        std::vector<std::pair<double, double>> coords;
        for (const auto& v : verts) {
          coords.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
        }
        if (!coords.empty()) {
          svg.drawPolygon(coords, style);
        }
      }
    }

    // Collect ALL points for unified rendering (Points, ObjectPoints, Shape Corners)
    std::set<std::shared_ptr<Point>> uniquePoints;
    for (const auto& pt : editor.points) uniquePoints.insert(pt);
    for (const auto& pt : editor.ObjectPoints) uniquePoints.insert(pt);
    for (const auto& rect : editor.rectangles) {
      if (rect && rect->isValid() && rect->isVisible()) {
        if (auto p = rect->getCorner1Point()) uniquePoints.insert(p);
        if (auto p = rect->getCorner2Point()) uniquePoints.insert(p);
        if (auto p = rect->getCornerBPoint()) uniquePoints.insert(p);
        if (auto p = rect->getCornerDPoint()) uniquePoints.insert(p);
      }
    }
    for (const auto& tri : editor.triangles) {
      if (tri && tri->isValid() && tri->isVisible()) {
        for (int i = 0; i < 3; ++i) {
          if (auto p = tri->getVertexPoint(i)) uniquePoints.insert(p);
        }
      }
    }
    for (const auto& poly : editor.polygons) {
      if (poly && poly->isValid() && poly->isVisible()) {
        for (size_t i = 0; i < poly->getVertexCount(); ++i) {
          if (auto p = poly->getVertexPoint(i)) uniquePoints.insert(p);
        }
      }
    }
    for (const auto& rpoly : editor.regularPolygons) {
      if (rpoly && rpoly->isValid() && rpoly->isVisible()) {
        if (auto p = rpoly->getCenterPoint()) uniquePoints.insert(p);
        if (auto p = rpoly->getFirstVertexPoint()) uniquePoints.insert(p);
      }
    }
    // --- Draw Angles ---
    for (const auto& angle : editor.angles) {
      if (angle && angle->isValid() && angle->isVisible()) {
        SVGWriter::Style style;
        style.stroke = colorToHex(angle->getColor());
        style.strokeWidth = 2.0 * pixelToWorldScale;
        sf::Color fillCol = angle->getFillColor();
        // If Alpha is 0, it's effectively "none"
        if (fillCol.a > 0) {
          style.fill = colorToHex(fillCol);
          style.opacity = fillCol.a / 255.0f;  // Convert 0-255 to 0.0-1.0
        } else {
          style.fill = "none";
        }

        // FIX 1: Lock weak_ptrs to shared_ptrs before using them
        auto pA = angle->getPointA().lock();
        auto pB = angle->getPointB().lock();
        auto pV = angle->getVertex().lock();

        if (pA && pB && pV) {
          Point_2 center = pV->getCGALPosition();
          Point_2 start = pA->getCGALPosition();
          Point_2 end = pB->getCGALPosition();
          double radius = angle->getRadius();

          // FIX 2: Convert CGAL types to double for std::atan2
          double startY = CGAL::to_double(start.y() - center.y());
          double startX = CGAL::to_double(start.x() - center.x());
          double endY = CGAL::to_double(end.y() - center.y());
          double endX = CGAL::to_double(end.x() - center.x());

          double startAngle = std::atan2(startY, startX);
          double endAngle = std::atan2(endY, endX);

          // Calculate Sweep
          double diff = endAngle - startAngle;
          // Normalize to [0, 2PI)
          while (diff < 0) diff += 2 * 3.14159265359;
          while (diff >= 2 * 3.14159265359) diff -= 2 * 3.14159265359;

          // Handle Reflex Logic
          if (!angle->isReflex()) {
            if (diff > 3.14159265359) diff -= 2 * 3.14159265359;
          } else {
            if (diff < 3.14159265359) diff -= 2 * 3.14159265359;
          }

          // Generate Arc Points
          std::vector<std::pair<double, double>> arcPoints;
          int steps = 20;
          for (int i = 0; i <= steps; ++i) {
            double theta = startAngle + diff * (double(i) / steps);
            // Use conversion for coordinate math just to be safe, though radius is usually double
            double px = CGAL::to_double(center.x()) + radius * std::cos(theta);
            double py = CGAL::to_double(center.y()) + radius * std::sin(theta);
            arcPoints.push_back({px, py});
          }

          // Draw as connected lines
          if (arcPoints.size() > 1) {
            for (size_t i = 0; i < arcPoints.size() - 1; ++i) {
              svg.drawLine(arcPoints[i].first, arcPoints[i].second, arcPoints[i + 1].first, arcPoints[i + 1].second, style);
            }
          }
        }
      }
    }
    // Draw All Points
    for (const auto& pt : uniquePoints) {
      if (pt && pt->isValid() && pt->isVisible()) {
        Point_2 pos = pt->getCGALPosition();
        SVGWriter::Style style;
        style.fill = colorToHex(pt->getColor());
        style.stroke = "none";
        svg.drawCircle(CGAL::to_double(pos.x()), CGAL::to_double(pos.y()), pointRadius, style);
      }
    }

    svg.endGroup();  // End Geometry Group (Labels are drawn OUTSIDE to keep text upright)

    // Labels for All Points
    for (const auto& pt : uniquePoints) {
      if (!pt || !pt->isValid() || !pt->isVisible()) continue;

      // Export labels for points that have a non-empty label and are visible.
      // (Shape vertices often have m_showLabel=false to avoid SFML double-draw, but in SVG we want them).
      bool hasLabel = !pt->getLabel().empty();
      if (!hasLabel) continue;

      const std::string& label = pt->getLabel();
      if (label.empty()) continue;

      Point_2 pos = pt->getCGALPosition();
      sf::Vector2f offset = pt->getLabelOffset();

      // Calculate Screen Coordinates directly
      // Point is flipped: screenY = (minY + maxY) - worldY
      // Label Offset is in screen pixels (relative to point)
      // We MUST scale pixels to world units for the SVG viewBox.
      double sx = CGAL::to_double(pos.x()) + (offset.x + 5.0f) * pixelToWorldScale;
      double sy = (minY + maxY - CGAL::to_double(pos.y())) + (offset.y - 5.0f) * pixelToWorldScale;

      SVGWriter::Style textStyle;
      textStyle.fill = colorToHex(pt->getColor());
      textStyle.stroke = "none";
      textStyle.fontSize = 12.0 * pixelToWorldScale;
      textStyle.textAnchor = "start";  // Align text above the point
      textStyle.dominantBaseline = "central";
      svg.drawText(sx, sy, label, textStyle);
    }
    // 2. Angle Labels
    for (const auto& angle : editor.angles) {
      if (angle && angle->isValid() && angle->isVisible() && !angle->getLabel().empty()) {
        // FIX 1: Lock pointers
        auto pA = angle->getPointA().lock();
        auto pB = angle->getPointB().lock();
        auto pV = angle->getVertex().lock();

        if (pV && pA && pB) {
          Point_2 center = pV->getCGALPosition();
          Point_2 start = pA->getCGALPosition();
          Point_2 end = pB->getCGALPosition();
          double radius = angle->getRadius();

          // FIX 2: Explicit double conversion
          double startAngle = std::atan2(CGAL::to_double(start.y() - center.y()), CGAL::to_double(start.x() - center.x()));
          double endAngle = std::atan2(CGAL::to_double(end.y() - center.y()), CGAL::to_double(end.x() - center.x()));

          double diff = endAngle - startAngle;
          while (diff < 0) diff += 2 * 3.14159265359;
          while (diff >= 2 * 3.14159265359) diff -= 2 * 3.14159265359;

          if (!angle->isReflex()) {
            if (diff > 3.14159265359) diff -= 2 * 3.14159265359;
          } else {
            if (diff < 3.14159265359) diff -= 2 * 3.14159265359;
          }

          // Calculate Mid-Angle
          double midAngle = startAngle + diff / 2.0;

          // Position Label
          double labelRadius = radius + (15.0 * pixelToWorldScale);
          double lx = CGAL::to_double(center.x()) + labelRadius * std::cos(midAngle);
          double ly = CGAL::to_double(center.y()) + labelRadius * std::sin(midAngle);

          // Convert to Screen Space
          double sx = lx;
          double sy = (minY + maxY) - ly;

          SVGWriter::Style textStyle;
          textStyle.fill = colorToHex(angle->getColor());
          textStyle.stroke = "none";
          textStyle.fontSize = 12.0 * pixelToWorldScale;
          textStyle.textAnchor = "middle";
          textStyle.dominantBaseline = "central";
          svg.drawText(sx, sy, angle->getLabel(), textStyle);
        }
      }
    }
    if (svg.save(filepath)) {
      std::cout << "SVG exported successfully (WYSIWYG)." << std::endl;
      return true;
    } else {
      return false;
    }

  } catch (const std::exception& e) {
    std::cerr << "Export Error: " << e.what() << std::endl;
    return false;
  }
}

// End exportSVG

// Standard geometric labels for a quad
// const char* labels[] = {"A", "B", "C", "D"};
