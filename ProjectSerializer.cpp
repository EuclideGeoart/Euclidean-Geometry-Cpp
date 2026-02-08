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

    // Save Points
    json pointsArray = json::array();
    for (const auto& pt : editor.points) {
      if (pt && pt->isValid()) {
        json ptJson;
        ptJson["id"] = pt->getID();
        Point_2 pos = pt->getCGALPosition();
        ptJson["x"] = CGAL::to_double(pos.x());
        ptJson["y"] = CGAL::to_double(pos.y());
        ptJson["color"] = colorToHex(pt->getColor());
        ptJson["label"] = pt->getLabel();
        ptJson["showLabel"] = pt->getShowLabel();
        ptJson["hasUserOverride"] = pt->hasUserOverride();  // NEW: Persist manual toggle state
        sf::Vector2f offset = pt->getLabelOffset();
        ptJson["labelOffset"] = {offset.x, offset.y};
        ptJson["visible"] = pt->isVisible();
        ptJson["locked"] = pt->isLocked();
        ptJson["fixed"] = pt->isLocked();
        ptJson["isIntersectionPoint"] = pt->isIntersectionPoint();

        // Use new helper for consistency (Points handled differently in legacy, but good to have base data)
        // Note: Points rely on "transform" object block for specific reconstruction,
        // but adding base metadata doesn't hurt.
        addTransformMetadata(ptJson, pt);

        // IntersectionPoint dependency (line-line)
        if (auto ip = std::dynamic_pointer_cast<IntersectionPoint>(pt)) {
          if (auto l1 = ip->getLine1()) ptJson["line1Id"] = l1->getID();
          if (auto l2 = ip->getLine2()) ptJson["line2Id"] = l2->getID();
        }

        // Transformation-derived points (Legacy Explicit Structure)
        if (auto refL = std::dynamic_pointer_cast<ReflectLine>(pt)) {
          json t;
          t["type"] = "ReflectLine";
          if (auto src = refL->getSourcePoint()) t["sourceId"] = src->getID();
          if (auto ln = refL->getReflectLine()) t["lineId"] = ln->getID();
          ptJson["transform"] = t;
        } else if (auto refP = std::dynamic_pointer_cast<ReflectPoint>(pt)) {
          json t;
          t["type"] = "ReflectPoint";
          if (auto src = refP->getSourcePoint()) t["sourceId"] = src->getID();
          if (auto c = refP->getCenterPoint()) t["centerId"] = c->getID();
          ptJson["transform"] = t;
        } else if (auto refC = std::dynamic_pointer_cast<ReflectCircle>(pt)) {
          json t;
          t["type"] = "ReflectCircle";
          if (auto src = refC->getSourcePoint()) t["sourceId"] = src->getID();
          if (auto c = refC->getCircle()) t["circleId"] = c->getID();
          ptJson["transform"] = t;
        } else if (auto rot = std::dynamic_pointer_cast<RotatePoint>(pt)) {
          json t;
          t["type"] = "RotatePoint";
          if (auto src = rot->getSourcePoint()) t["sourceId"] = src->getID();
          if (auto c = rot->getCenterPoint()) t["centerId"] = c->getID();
          t["angleDeg"] = rot->getAngleDegrees();
          ptJson["transform"] = t;
        } else if (auto tr = std::dynamic_pointer_cast<TranslateVector>(pt)) {
          json t;
          t["type"] = "TranslateVector";
          if (auto src = tr->getSourcePoint()) t["sourceId"] = src->getID();
          if (auto v1 = tr->getVectorStart()) t["vecStartId"] = v1->getID();
          if (auto v2 = tr->getVectorEnd()) t["vecEndId"] = v2->getID();
          ptJson["transform"] = t;
        } else if (auto dil = std::dynamic_pointer_cast<DilatePoint>(pt)) {
          json t;
          t["type"] = "DilatePoint";
          if (auto src = dil->getSourcePoint()) t["sourceId"] = src->getID();
          if (auto c = dil->getCenterPoint()) t["centerId"] = c->getID();
          t["factor"] = dil->getScaleFactor();
          ptJson["transform"] = t;
        }
        pointsArray.push_back(ptJson);
      }
    }
    project["objects"]["points"] = pointsArray;

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
        lnJson["lineType"] = static_cast<int>(ln->getLineType());

        addTransformMetadata(lnJson, ln);

        if (auto startObj = ln->getStartPointObject()) {
          lnJson["startPointID"] = startObj->getID();
        } else {
          Point_2 start = ln->getStartPoint();
          lnJson["startX"] = CGAL::to_double(start.x());
          lnJson["startY"] = CGAL::to_double(start.y());
        }
        if (auto endObj = ln->getEndPointObject()) {
          lnJson["endPointID"] = endObj->getID();
        } else {
          Point_2 end = ln->getEndPoint();
          lnJson["endX"] = CGAL::to_double(end.x());
          lnJson["endY"] = CGAL::to_double(end.y());
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
        rectJson["color"] = colorToHex(rect->getColor());
        rectJson["thickness"] = rect->getThickness();
        rectJson["isRotatable"] = rect->isRotatable();
        rectJson["height"] = rect->getHeight();
        rectJson["width"] = rect->getWidth();

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
        }
        if (auto cir = tangent->getCircle()) {
          tJson["circleID"] = cir->getID();
        }
        tJson["solutionIndex"] = tangent->getSolutionIndex();
        tJson["color"] = colorToHex(tangent->getColor());
        tJson["thickness"] = tangent->getThickness();
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

    // Clear existing scene completely
    editor.clearScene();
    DynamicIntersection::clearAllIntersectionConstraints(editor);

    const json& data = j.contains("objects") ? j["objects"] : j;
    bool axesVisible = j.value("settings", json::object()).value("axesVisible", true);

    // ========================================================================
    // THE MASTER ID MAP (Identity Store)
    // ========================================================================
    std::map<unsigned int, std::shared_ptr<GeometricObject>> masterMap;
    unsigned int maxId = 0;

    auto registerInMap = [&](unsigned int id, const std::shared_ptr<GeometricObject>& obj) {
      if (!obj || id == 0) return;
      masterMap[id] = obj;
      if (id > maxId) maxId = id;
    };

    auto getFromMap = [&](unsigned int id) -> std::shared_ptr<GeometricObject> {
      if (id == 0) return nullptr;
      auto it = masterMap.find(id);
      return (it != masterMap.end()) ? it->second : nullptr;
    };

    auto getPointFromMap = [&](unsigned int id) -> std::shared_ptr<Point> { return std::dynamic_pointer_cast<Point>(getFromMap(id)); };

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
      }
      if (jPt.contains("fixed")) pt->setLocked(jPt.value("fixed", false));
      if (jPt.contains("locked")) pt->setLocked(jPt.value("locked", false));
      if (jPt.contains("visible")) pt->setVisible(jPt.value("visible", true));
    };

    // Helper: Apply transform metadata
    auto applyTransformMetadata = [&](const json& jObj, const std::shared_ptr<GeometricObject>& obj) {
      if (!obj) return;
      if (jObj.contains("isDependent")) obj->setDependent(jObj["isDependent"].get<bool>());
      if (jObj.contains("transformType")) {
        obj->setTransformType(static_cast<TransformationType>(jObj["transformType"].get<int>()));
        obj->setParentSourceID(jObj.value("parentSourceID", 0u));
        obj->setAuxObjectID(jObj.value("auxObjectID", 0u));
        obj->setTransformValue(jObj.value("transformValue", 0.0));
      }
      if (jObj.contains("translationVectorX") && jObj.contains("translationVectorY")) {
        obj->setTranslationVector(Vector_2(jObj["translationVectorX"].get<double>(), jObj["translationVectorY"].get<double>()));
      }
    };

    // ========================================================================
    // PASS 1: Identity Registration (Points & ObjectPoints)
    // ========================================================================
    std::cout << "[3-Pass Engine] Pass 1: Identity Registration..." << std::endl;

    // 1a. Points (Standard, Transformation-derivied, and Intersections)
    if (data.contains("points")) {
      for (const auto& jPt : data["points"]) {
        unsigned int id = jPt.value("id", 0u);
        if (id == 0) continue;

        double x = jPt.value("x", 0.0);
        double y = jPt.value("y", 0.0);
        sf::Color color = colorFromJson(jPt.contains("color") ? jPt["color"] : json(nullptr), sf::Color::Black);

        std::shared_ptr<Point> pt;
        // Instantiate correct subclass based on legacy transform object if present
        if (jPt.contains("transform")) {
          const json& t = jPt["transform"];
          std::string type = t.value("type", "");
          if (type == "ReflectLine")
            pt = std::make_shared<ReflectLine>(nullptr, nullptr, color, id);
          else if (type == "ReflectPoint")
            pt = std::make_shared<ReflectPoint>(nullptr, nullptr, color, id);
          else if (type == "ReflectCircle")
            pt = std::make_shared<ReflectCircle>(nullptr, nullptr, color, id);
          else if (type == "RotatePoint")
            pt = std::make_shared<RotatePoint>(nullptr, nullptr, t.value("angleDeg", 0.0), color, id);
          else if (type == "TranslateVector")
            pt = std::make_shared<TranslateVector>(nullptr, nullptr, nullptr, color, id);
          else if (type == "DilatePoint")
            pt = std::make_shared<DilatePoint>(nullptr, nullptr, t.value("factor", 1.0), color, id);
          else
            pt = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
        } else if (jPt.value("isIntersectionPoint", false)) {
          pt = std::make_shared<IntersectionPoint>(nullptr, nullptr, Point_2(x, y), color, id);
        } else {
          pt = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
        }

        applyCommonPointFields(jPt, pt);
        applyTransformMetadata(jPt, pt);
        editor.points.push_back(pt);
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
        ln->setLineType(static_cast<Line::LineType>(jLn.value("lineType", 1)));
        applyTransformMetadata(jLn, ln);
        editor.lines.push_back(ln);
        registerInMap(id, ln);
      }
    }

    // 2c. Rectangles
    if (data.contains("rectangles")) {
      for (const auto& jRect : data["rectangles"]) {
        unsigned int id = jRect.value("id", 0u);
        auto c1 = getPointFromMap(jRect.value("corner1ID", 0u));
        auto c2 = getPointFromMap(jRect.value("corner2ID", 0u));
        if (!c1 || !c2) continue;

        sf::Color color = colorFromJson(jRect.contains("color") ? jRect["color"] : json(nullptr), sf::Color::White);
        auto rect = std::make_shared<Rectangle>(c1, c2, jRect.value("isRotatable", false), color, id);

        auto cb = getPointFromMap(jRect.value("cornerBID", 0u));
        auto cd = getPointFromMap(jRect.value("cornerDID", 0u));
        if (cb && cd) rect->setDependentCornerPoints(cb, cd);

        rect->setThickness(jRect.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        applyTransformMetadata(jRect, rect);
        editor.rectangles.push_back(rect);
        registerInMap(id, rect);
      }
    }

    // 2d. Polygons, Triangles, RegularPolygons
    if (data.contains("polygons")) {
      for (const auto& jPoly : data["polygons"]) {
        std::vector<std::shared_ptr<Point>> vPts;
        for (const auto& vId : jPoly.value("vertexIds", json::array())) {
          auto v = getPointFromMap(vId.get<unsigned int>());
          if (v) vPts.push_back(v);
        }
        if (vPts.size() < 3) continue;
        auto poly = std::make_shared<Polygon>(vPts, colorFromJson(jPoly["color"], sf::Color::White), jPoly.value("id", 0u));
        applyTransformMetadata(jPoly, poly);
        editor.polygons.push_back(poly);
        registerInMap(poly->getID(), poly);
      }
    }
    if (data.contains("triangles")) {
      for (const auto& jTri : data["triangles"]) {
        auto v0 = getPointFromMap(jTri["vertexIds"][0]);
        auto v1 = getPointFromMap(jTri["vertexIds"][1]);
        auto v2 = getPointFromMap(jTri["vertexIds"][2]);
        if (!v0 || !v1 || !v2) continue;
        auto tri = std::make_shared<Triangle>(v0, v1, v2, colorFromJson(jTri["color"], sf::Color::White), jTri.value("id", 0u));
        applyTransformMetadata(jTri, tri);
        editor.triangles.push_back(tri);
        registerInMap(tri->getID(), tri);
      }
    }
    if (data.contains("regularPolygons")) {
      for (const auto& jRPoly : data["regularPolygons"]) {
        auto center = getPointFromMap(jRPoly.value("centerPointID", 0u));
        auto radPt = getPointFromMap(jRPoly.value("firstVertexPointID", 0u));
        if (!center || !radPt) continue;
        auto rpoly = std::make_shared<RegularPolygon>(center, radPt, jRPoly.value("sides", 3), colorFromJson(jRPoly["color"], sf::Color::White),
                                                      jRPoly.value("id", 0u));
        applyTransformMetadata(jRPoly, rpoly);
        editor.regularPolygons.push_back(rpoly);
        registerInMap(rpoly->getID(), rpoly);
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

    // 3a. Re-link Transformations & Construction Dependencies
    for (auto& [id, obj] : masterMap) {
      if (!obj) continue;
      if (obj->getTransformType() != TransformationType::None || obj->isDependent()) {
        auto parent = getFromMap(obj->getParentSourceID());
        auto aux = getFromMap(obj->getAuxObjectID());
        if (parent) obj->relinkTransformation(parent, aux);
      }
    }

    // 3b. Re-link ObjectPoints host pointers
    if (data.contains("objectPoints")) {
      for (const auto& jOp : data["objectPoints"]) {
        auto op = std::dynamic_pointer_cast<ObjectPoint>(getFromMap(jOp.value("id", 0u)));
        if (!op) continue;
        auto host = getFromMap(jOp.value("hostId", 0u));
        if (host) op->relinkHost(host, jOp.value("t", 0.0), static_cast<ObjectType>(jOp.value("hostType", 0)));
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

                 if (!angle->isReflex()) { if (diff > 3.14159265359) diff -= 2 * 3.14159265359; }
                 else { if (diff < 3.14159265359) diff -= 2 * 3.14159265359; }

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
