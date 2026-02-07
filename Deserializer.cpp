/**
 * Deserializer.cpp
 *
 * Robust 4-Pass Deserialization for 1:1 State Restoration
 *
 * DESIGN:
 * - Pass 1: Load all Points (including intersection points) into objectMap
 * - Pass 2: Load all Shapes (Lines, Circles, Rectangles, Polygons, Triangles, RegularPolygons)
 *           using ONLY references from objectMap (never creating new points)
 * - Pass 3: Restore Dependencies (Transformations, Constraints, TangentLines, ObjectPoints)
 * - Pass 4: Force Update all objects to synchronize visual state
 *
 * CRITICAL: Uses std::map<unsigned int, std::shared_ptr<GeometricObject>> objectMap
 *           to guarantee pointer uniqueness - NO duplicates.
 */

#include "Deserializer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "Angle.h"
#include "Circle.h"
#include "Constants.h"
#include "ConstructionObjects.h"
#include "DynamicIntersection.h"
#include "GeometricObject.h"
#include "GeometryEditor.h"
#include "Line.h"
#include "IntersectionPoint.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "Polygon.h"
#include "Rectangle.h"
#include "RegularPolygon.h"
#include "TransformationObjects.h"
#include "Triangle.h"
#include "json.hpp"

using json = nlohmann::json;

// ============================================================================
// ANONYMOUS NAMESPACE: Helper Functions
// ============================================================================
namespace {

/**
 * Convert hex color string to sf::Color
 */
sf::Color hexToColor(const std::string& hex) {
  if (hex.empty() || hex[0] != '#' || hex.length() < 7) {
    return sf::Color::Blue;
  }

  unsigned int r = 0, g = 0, b = 0;
  std::stringstream ss;
  ss << std::hex << hex.substr(1, 2);
  ss >> r;
  ss.clear();
  ss << std::hex << hex.substr(3, 2);
  ss >> g;
  ss.clear();
  ss << std::hex << hex.substr(5, 2);
  ss >> b;

  return sf::Color(static_cast<sf::Uint8>(r), static_cast<sf::Uint8>(g),
                   static_cast<sf::Uint8>(b));
}

/**
 * Parse color from JSON (supports both hex string and object format)
 */
sf::Color colorFromJson(const json& j, const sf::Color& fallback) {
  if (j.is_string()) {
    return hexToColor(j.get<std::string>());
  }
  if (j.is_object()) {
    return sf::Color(j.value("r", fallback.r), j.value("g", fallback.g),
                     j.value("b", fallback.b), j.value("a", fallback.a));
  }
  return fallback;
}

/**
 * Apply common point attributes from JSON
 */
void applyCommonPointFields(const json& jPoint,
                            const std::shared_ptr<Point>& point) {
  if (jPoint.contains("label")) point->setLabel(jPoint.value("label", ""));
  if (jPoint.contains("showLabel"))
    point->setShowLabel(jPoint.value("showLabel", true));
  if (jPoint.contains("labelOffset") && jPoint["labelOffset"].is_array() &&
      jPoint["labelOffset"].size() >= 2) {
    point->setLabelOffset(
        sf::Vector2f(jPoint["labelOffset"][0], jPoint["labelOffset"][1]));
  }
  if (jPoint.contains("fixed")) point->setLocked(jPoint.value("fixed", false));
  if (jPoint.contains("locked")) point->setLocked(jPoint.value("locked", false));
  if (jPoint.contains("visible"))
    point->setVisible(jPoint.value("visible", true));
}

/**
 * Apply transformation metadata to an object
 */
void applyTransformMetadata(const json& jObj,
                            const std::shared_ptr<GeometricObject>& obj) {
  if (!obj) return;

  if (jObj.contains("isDependent")) {
      obj->setDependent(jObj["isDependent"].get<bool>());
  }

  if (jObj.contains("transformType")) {
    obj->setTransformType(
        static_cast<TransformationType>(jObj["transformType"].get<int>()));
    obj->setParentSourceID(jObj.value("parentSourceID", 0u));
    obj->setAuxObjectID(jObj.value("auxObjectID", 0u));
  }
  if (jObj.contains("transformValue")) {
    obj->setTransformValue(jObj["transformValue"].get<double>());
  }
  
  if (jObj.contains("translationVectorX") && jObj.contains("translationVectorY")) {
      Vector_2 v(jObj["translationVectorX"].get<double>(), jObj["translationVectorY"].get<double>());
      obj->setTranslationVector(v);
  }
}

}  // namespace

// ============================================================================
// MAIN LOAD FUNCTION
// ============================================================================
bool Deserializer::loadProject(GeometryEditor& editor,
                               const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Deserializer::loadProject: Failed to open file: " << filepath
              << std::endl;
    return false;
  }

  try {
    json j;
    file >> j;
    file.close();

    // Clear existing scene completely
    editor.clearScene();
    DynamicIntersection::clearAllIntersectionConstraints(editor);

    // Detect JSON structure (nested "objects" or flat)
    const json& data = j.contains("objects") ? j["objects"] : j;

    // Load settings
    bool axesVisible = true;
    if (j.contains("settings") && j["settings"].contains("axesVisible")) {
      axesVisible = j["settings"]["axesVisible"].get<bool>();
    }

    // ========================================================================
    // CRITICAL: The Object Map - Guarantees Pointer Uniqueness
    // ========================================================================
    std::map<unsigned int, std::shared_ptr<GeometricObject>> objectMap;
    unsigned int maxId = 0;

    // Helper to register objects and track max ID
    auto registerObject = [&](unsigned int id,
                              const std::shared_ptr<GeometricObject>& obj) {
      if (!obj || id == 0) return;
      objectMap[id] = obj;
      if (id > maxId) maxId = id;
    };

    // Helper to get Point from objectMap
    auto getPoint = [&](int id) -> std::shared_ptr<Point> {
      if (id <= 0) return nullptr;
      auto it = objectMap.find(static_cast<unsigned int>(id));
      if (it != objectMap.end()) {
        return std::dynamic_pointer_cast<Point>(it->second);
      }
      return nullptr;
    };

    // Helper to get any GeometricObject from objectMap
    auto getObject =
        [&](int id) -> std::shared_ptr<GeometricObject> {
      if (id <= 0) return nullptr;
      auto it = objectMap.find(static_cast<unsigned int>(id));
      if (it != objectMap.end()) {
        return it->second;
      }
      return nullptr;
    };

    // Helper: find an existing point by exact position (legacy fallback only)
    auto findPointByPosition = [&](const Point_2& pos) -> std::shared_ptr<Point> {
      for (const auto& [objId, obj] : objectMap) {
        auto pt = std::dynamic_pointer_cast<Point>(obj);
        if (!pt) continue;
        Point_2 p = pt->getCGALPosition();
        if (p == pos) return pt;
      }
      return nullptr;
    };

    // ========================================================================
    // PASS 1: LOAD ALL POINTS
    // Constraint: Create points ONLY here. Never create new points in Pass 2.
    // ========================================================================
    std::cout << "[Deserializer] Pass 1: Loading Points..." << std::endl;

    // Deferred transform points (need to be created after base objects)
    std::vector<json> deferredTransformPoints;
    std::vector<json> pendingIntersectionPoints;
    std::vector<json> pendingLines;
    std::vector<json> pendingPolygons;
    std::vector<json> pendingCircles;

    // 1a. Load regular points
    if (data.contains("points")) {
      for (const auto& jPoint : data["points"]) {
        // Check if this is a transformation-derived point (defer to later)
        if (jPoint.contains("transform")) {
          deferredTransformPoints.push_back(jPoint);
          continue;
        }

        int id = jPoint.value("id", -1);
        if (id == -1) continue;

        // Check if already exists (avoid duplicates)
        if (objectMap.count(static_cast<unsigned int>(id))) {
          continue;
        }

        double x = jPoint.value("x", 0.0);
        double y = jPoint.value("y", 0.0);

        sf::Color color = sf::Color::Black;
        if (jPoint.contains("color")) {
          color = colorFromJson(jPoint["color"], color);
        }

        auto newPoint = std::make_shared<Point>(
            Point_2(x, y), Constants::CURRENT_ZOOM, color,
            static_cast<unsigned int>(id));

        applyCommonPointFields(jPoint, newPoint);
        applyTransformMetadata(jPoint, newPoint);
        if (jPoint.contains("isIntersectionPoint") && jPoint["isIntersectionPoint"].get<bool>()) {
          newPoint->setIntersectionPoint(true);
          newPoint->setDependent(true);
          newPoint->setLocked(true);
        }
        if (jPoint.contains("line1Id") || jPoint.contains("line2Id")) {
          pendingIntersectionPoints.push_back(jPoint);
        }

        editor.points.push_back(newPoint);
        registerObject(static_cast<unsigned int>(id), newPoint);
      }
    }

    // 1b. Load intersection points (from intersections array)
    // These are stored within the intersections constraint data
    if (data.contains("intersections")) {
      for (const auto& cJson : data["intersections"]) {
        if (!cJson.contains("points") || !cJson["points"].is_array()) continue;

        for (const auto& pJson : cJson["points"]) {
          int id = pJson.value("id", -1);
          if (id == -1) continue;

          // Check if already exists
          if (objectMap.count(static_cast<unsigned int>(id))) {
            // Already loaded - just mark as intersection point
            auto existing = getPoint(id);
            if (existing) {
              existing->setIntersectionPoint(true);
              existing->setDependent(true);
              existing->setSelected(false);
              existing->lock();
            }
            continue;
          }

          double x = pJson.value("x", 0.0);
          double y = pJson.value("y", 0.0);

          sf::Color color = Constants::INTERSECTION_POINT_COLOR;
          if (pJson.contains("color")) {
            color = colorFromJson(pJson["color"], color);
          }

          auto newPoint = std::make_shared<Point>(
              Point_2(x, y), Constants::CURRENT_ZOOM, color,
              static_cast<unsigned int>(id));

          newPoint->setIntersectionPoint(true);
          newPoint->setDependent(true);
          newPoint->setSelected(false);
          newPoint->lock();

          if (pJson.contains("label")) newPoint->setLabel(pJson.value("label", ""));
          if (pJson.contains("showLabel"))
            newPoint->setShowLabel(pJson.value("showLabel", true));
          if (pJson.contains("labelOffset") && pJson["labelOffset"].is_array() &&
              pJson["labelOffset"].size() >= 2) {
            newPoint->setLabelOffset(
                sf::Vector2f(pJson["labelOffset"][0], pJson["labelOffset"][1]));
          }

          editor.points.push_back(newPoint);
          registerObject(static_cast<unsigned int>(id), newPoint);
        }
      }
    }

    std::cout << "[Deserializer] Pass 1 Complete: " << objectMap.size()
              << " points loaded." << std::endl;

    // ========================================================================
    // PASS 2: LOAD BASE SHAPES (Lines, Circles, Rectangles, Polygons, etc.)
    // Constraint: NEVER create new points. Always look up from objectMap.
    // ========================================================================
    std::cout << "[Deserializer] Pass 2: Loading Shapes..." << std::endl;

    // 2a. Load Lines
    if (data.contains("lines")) {
      for (const auto& jLine : data["lines"]) {
        int id = jLine.value("id", -1);
        int startId = jLine.value("startPointID", jLine.value("startId", -1));
        int endId = jLine.value("endPointID", jLine.value("endId", -1));

        // CRITICAL: Get points from objectMap - never create new ones
        auto startPt = getPoint(startId);
        auto endPt = getPoint(endId);

        if (!startPt || !endPt) {
          pendingLines.push_back(jLine);
          continue;
        }

        bool isSegment = jLine.value("isSegment", false);
        sf::Color color = sf::Color::Black;
        if (jLine.contains("color")) {
          color = colorFromJson(jLine["color"], color);
        }

        auto newLine = std::make_shared<Line>(
            startPt, endPt, isSegment, color,
            static_cast<unsigned int>(std::max(id, 0)));

        newLine->registerWithEndpoints();
        newLine->setThickness(
            jLine.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        newLine->setDecoration(
            static_cast<DecorationType>(jLine.value("decoration", 0)));
        newLine->setLineType(
            static_cast<Line::LineType>(jLine.value("lineType", 1)));

        applyTransformMetadata(jLine, newLine);

        editor.lines.push_back(newLine);
        if (id > 0) registerObject(static_cast<unsigned int>(id), newLine);
      }
    }

    // 2b. Load Circles (including Semicircles)
    if (data.contains("circles")) {
      for (const auto& jCircle : data["circles"]) {
        int id = jCircle.value("id", -1);
        int centerId = jCircle.value("centerPointID", -1);
        int radiusPtId = jCircle.value("radiusPointID", -1);
        double radius = jCircle.value("radiusValue", jCircle.value("radius", 10.0));

        auto centerPt = getPoint(centerId);
        if (!centerPt) {
          pendingCircles.push_back(jCircle);
          continue;
        }

        sf::Color color = sf::Color::Black;
        if (jCircle.contains("color")) {
          color = colorFromJson(jCircle["color"], color);
        }

        std::shared_ptr<Point> radiusPt = getPoint(radiusPtId);

        auto newCircle =
            std::make_shared<Circle>(centerPt.get(), radiusPt, radius, color);
        if (id > 0)
          newCircle->setID(static_cast<unsigned int>(id));

        newCircle->setThickness(
            jCircle.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

        applyTransformMetadata(jCircle, newCircle);

        // Handle Semicircle
        bool isSemi = jCircle.value("isSemicircle", false);
        newCircle->setSemicircle(isSemi);
        if (isSemi) {
          int p1Id = jCircle.value("startArcID", jCircle.value("diameterP1ID", -1));
          int p2Id = jCircle.value("endArcID", jCircle.value("diameterP2ID", -1));
          auto p1 = getPoint(p1Id);
          auto p2 = getPoint(p2Id);
          if (p1 && p2) {
            newCircle->setSemicircleDiameterPoints(p1, p2);
            newCircle->setSemicircleBasis(p1->getCGALPosition(), p2->getCGALPosition());
          }
        }

        editor.circles.push_back(newCircle);
        if (id > 0) registerObject(static_cast<unsigned int>(id), newCircle);
      }
    }

        // Helper to check if any vertex ID is in deferredTransformPoints
        auto isVertexPending = [&](int vId) -> bool {
            for (const auto& dp : deferredTransformPoints) {
                if (dp.value("id", -1) == vId) return true;
            }
            return false;
        };

    // 2c. Load Rectangles
    if (data.contains("rectangles")) {
      for (const auto& rectJson : data["rectangles"]) {
        std::shared_ptr<Rectangle> rect;
        sf::Color color = sf::Color::Black;
        if (rectJson.contains("color")) {
          color = colorFromJson(rectJson["color"], color);
        }

        int id = rectJson.value("id", -1);
        bool isRotatable = rectJson.value("isRotatable", false);
        double height = rectJson.value("height", 0.0);

        // Try loading by Point IDs first (preferred method)
        if (rectJson.contains("corner1ID") && rectJson.contains("corner2ID")) {
          int c1Id = rectJson["corner1ID"].get<int>();
          int c2Id = rectJson["corner2ID"].get<int>();
          
          auto corner1 = getPoint(c1Id);
          auto corner2 = getPoint(c2Id);
          
          if (corner1 && corner2) {
            // Use pointer-based constructor to reuse existing Points
            if (isRotatable && height > 0) {
              rect = std::make_shared<Rectangle>(
                  corner1, corner2, height, color,
                  static_cast<unsigned int>(std::max(id, 0)));
            } else {
              rect = std::make_shared<Rectangle>(
                  corner1, corner2, isRotatable, color,
                  static_cast<unsigned int>(std::max(id, 0)));
            }
            
            // Set dependent corners if they exist
            if (rectJson.contains("cornerBID") && rectJson.contains("cornerDID")) {
              int cbId = rectJson["cornerBID"].get<int>();
              int cdId = rectJson["cornerDID"].get<int>();
              auto cornerB = getPoint(cbId);
              auto cornerD = getPoint(cdId);
              if (cornerB && cornerD) {
                rect->setDependentCornerPoints(cornerB, cornerD);
              }
            }
          }
        }
        
        // Legacy fallback: load by coordinates (creates duplicate Points)
        if (!rect && rectJson.contains("vertices")) {
          auto verticesJson = rectJson["vertices"];
          if (verticesJson.size() >= 2) {
            double x1, y1, x2, y2;
            if (verticesJson[0].is_array()) {
              x1 = verticesJson[0][0].get<double>();
              y1 = verticesJson[0][1].get<double>();
              int idx2 = (verticesJson.size() == 4) ? 2 : 1;
              x2 = verticesJson[idx2][0].get<double>();
              y2 = verticesJson[idx2][1].get<double>();
            } else {
              x1 = verticesJson[0].value("x", 0.0);
              y1 = verticesJson[0].value("y", 0.0);
              int idx2 = (verticesJson.size() == 4) ? 2 : 1;
              x2 = verticesJson[idx2].value("x", 0.0);
              y2 = verticesJson[idx2].value("y", 0.0);
            }

            rect = std::make_shared<Rectangle>(
                Point_2(x1, y1), Point_2(x2, y2), isRotatable, color,
                static_cast<unsigned int>(std::max(id, 0)));

            if (isRotatable && height > 0) {
              rect->setHeight(height);
            }
          }
        }

        if (rect) {
          rect->setThickness(
              rectJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

          applyTransformMetadata(rectJson, rect);

          editor.rectangles.push_back(rect);
          if (id > 0) registerObject(static_cast<unsigned int>(id), rect);
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
            auto vPtr = getPoint(vId);
            if (!vPtr) {
              // vertex missing entirely (yet), so must defer
              deferPoly = true; 
              break;
            }
            vertexPoints.push_back(vPtr);
          }
        } else if (polyJson.contains("vertices")) {
          // Legacy fallback: attempt to match existing points by position
          for (const auto& vJson : polyJson["vertices"]) {
            if (vJson.is_array() && vJson.size() >= 2) {
              Point_2 pos(vJson[0].get<double>(), vJson[1].get<double>());
              auto vPtr = findPointByPosition(pos);
              if (!vPtr) {
                vertexPoints.clear();
                break;
              }
              vertexPoints.push_back(vPtr);
            } else if (vJson.is_object()) {
              Point_2 pos(vJson.value("x", 0.0), vJson.value("y", 0.0));
              auto vPtr = findPointByPosition(pos);
              if (!vPtr) {
                vertexPoints.clear();
                break;
              }
              vertexPoints.push_back(vPtr);
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
        auto polygon = std::make_shared<Polygon>(
            vertexPoints, color, static_cast<unsigned int>(std::max(id, 0)));

        polygon->setThickness(
            polyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

        applyTransformMetadata(polyJson, polygon);

        editor.polygons.push_back(polygon);
        if (id > 0) registerObject(static_cast<unsigned int>(id), polygon);
      }
    }

    // 2e. Load Triangles
    if (data.contains("triangles")) {
      for (const auto& triJson : data["triangles"]) {
        std::shared_ptr<Triangle> triangle;
        sf::Color color = sf::Color::Black;
        if (triJson.contains("color")) {
          color = colorFromJson(triJson["color"], color);
        }

        int id = triJson.value("id", -1);
        
        // Try loading by Point IDs first (preferred method)
        if (triJson.contains("vertexIds") && triJson["vertexIds"].is_array()) {
          auto vIds = triJson["vertexIds"];
          if (vIds.size() >= 3) {
            auto v1 = getPoint(vIds[0].get<int>());
            auto v2 = getPoint(vIds[1].get<int>());
            auto v3 = getPoint(vIds[2].get<int>());
            
            if (v1 && v2 && v3) {
              triangle = std::make_shared<Triangle>(
                  v1, v2, v3, color,
                  static_cast<unsigned int>(std::max(id, 0)));
            }
          }
        }
        
        // Legacy fallback: load by coordinates (creates duplicate Points)
        if (!triangle && triJson.contains("vertices")) {
          auto verticesJson = triJson["vertices"];
          if (verticesJson.size() >= 3) {
            std::vector<Point_2> pts;
            for (int i = 0; i < 3; ++i) {
              if (verticesJson[i].is_array()) {
                pts.emplace_back(verticesJson[i][0].get<double>(),
                                verticesJson[i][1].get<double>());
              } else {
                pts.emplace_back(verticesJson[i].value("x", 0.0),
                                verticesJson[i].value("y", 0.0));
              }
            }

            triangle = std::make_shared<Triangle>(
                pts[0], pts[1], pts[2], color,
                static_cast<unsigned int>(std::max(id, 0)));
          }
        }

        if (triangle) {
          triangle->setThickness(
              triJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

          applyTransformMetadata(triJson, triangle);

          editor.triangles.push_back(triangle);
          if (id > 0) registerObject(static_cast<unsigned int>(id), triangle);
        }
      }
    }

    // 2f. Load Regular Polygons
    if (data.contains("regularPolygons")) {
      for (const auto& rpolyJson : data["regularPolygons"]) {
        std::shared_ptr<RegularPolygon> rpoly;
        int sides = rpolyJson.value("sides", 3);
        int id = rpolyJson.value("id", -1);

        sf::Color color = sf::Color::Black;
        if (rpolyJson.contains("color")) {
          color = colorFromJson(rpolyJson["color"], color);
        }

        // Try loading by Point IDs first (preferred method)
        if (rpolyJson.contains("centerPointID") && rpolyJson.contains("firstVertexPointID")) {
          int centerId = rpolyJson["centerPointID"].get<int>();
          int firstVertId = rpolyJson["firstVertexPointID"].get<int>();
          
          auto centerPt = getPoint(centerId);
          auto firstVertPt = getPoint(firstVertId);
          
          if (centerPt && firstVertPt) {
            rpoly = std::make_shared<RegularPolygon>(
                centerPt, firstVertPt, sides, color,
                static_cast<unsigned int>(std::max(id, 0)));
          }
        }
        
        // Legacy fallback: load by coordinates (creates duplicate Points)
        if (!rpoly && rpolyJson.contains("vertices")) {
          auto verticesJson = rpolyJson["vertices"];
          if (!verticesJson.empty()) {
            // Calculate center and first vertex
            double cx = 0.0, cy = 0.0;
            double fx = 0.0, fy = 0.0;

            if (verticesJson[0].is_array()) {
              fx = verticesJson[0][0].get<double>();
              fy = verticesJson[0][1].get<double>();
            } else {
              fx = verticesJson[0].value("x", 0.0);
              fy = verticesJson[0].value("y", 0.0);
            }

            for (const auto& v : verticesJson) {
              if (v.is_array()) {
                cx += v[0].get<double>();
                cy += v[1].get<double>();
              } else {
                cx += v.value("x", 0.0);
                cy += v.value("y", 0.0);
              }
            }
            cx /= verticesJson.size();
            cy /= verticesJson.size();

            rpoly = std::make_shared<RegularPolygon>(
                Point_2(cx, cy), Point_2(fx, fy), sides, color,
                static_cast<unsigned int>(std::max(id, 0)));
          }
        }

        if (rpoly) {
          rpoly->setThickness(
              rpolyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

          applyTransformMetadata(rpolyJson, rpoly);

          editor.regularPolygons.push_back(rpoly);
          if (id > 0) registerObject(static_cast<unsigned int>(id), rpoly);
        }
      }
    }

    std::cout << "[Deserializer] Pass 2 Complete: " << objectMap.size()
              << " total objects." << std::endl;

    // ========================================================================
    // PASS 2.5: REPLACE INTERSECTION PLACEHOLDERS WITH IntersectionPoint
    // ========================================================================
    for (const auto& jPoint : pendingIntersectionPoints) {
      int id = jPoint.value("id", -1);
      int line1Id = jPoint.value("line1Id", -1);
      int line2Id = jPoint.value("line2Id", -1);

      auto line1 = std::dynamic_pointer_cast<Line>(getObject(line1Id));
      auto line2 = std::dynamic_pointer_cast<Line>(getObject(line2Id));
      if (!line1 || !line2) continue;

      sf::Color color = sf::Color::Black;
      if (jPoint.contains("color")) {
        color = colorFromJson(jPoint["color"], color);
      }

      Point_2 pos(jPoint.value("x", 0.0), jPoint.value("y", 0.0));
      if (auto existing = getPoint(id)) {
        pos = existing->getCGALPosition();
      }

      auto ip = IntersectionPoint::create(line1, line2, pos, color);
      if (id > 0) ip->setID(static_cast<unsigned int>(id));
      ip->setIntersectionPoint(true);
      ip->setDependent(true);
      ip->setLocked(true);
      applyCommonPointFields(jPoint, ip);
      applyTransformMetadata(jPoint, ip);

      auto it = std::find_if(editor.points.begin(), editor.points.end(),
          [&](const std::shared_ptr<Point>& p) { return p && p->getID() == static_cast<unsigned int>(id); });
      if (it != editor.points.end()) {
        *it = ip;
      } else {
        editor.points.push_back(ip);
      }
      if (id > 0) registerObject(static_cast<unsigned int>(id), ip);
    }

    // ========================================================================
    // PASS 3: RESTORE DEPENDENCIES (Transformations, Constraints, etc.)
    // ========================================================================
    std::cout << "[Deserializer] Pass 3: Restoring Dependencies..." << std::endl;

    // 3a. Load Transformation Points (deferred from Pass 1)
    // We use a retry loop to handle chained dependencies (e.g. ReflectPoint -> ReflectPoint)
    std::set<int> processedDeferredPoints;
    bool progress = true;
    while (progress) {
        progress = false;
        
        for (const auto& jPoint : deferredTransformPoints) {
            int id = jPoint.value("id", -1);
            if (id == -1 || processedDeferredPoints.count(id)) continue;

            sf::Color color = sf::Color::Black;
            if (jPoint.contains("color")) {
                color = colorFromJson(jPoint["color"], color);
            }

            if (!jPoint.contains("transform")) continue;
            const auto& t = jPoint["transform"];
            std::string type = t.value("type", "");

            std::shared_ptr<Point> newPoint;

            if (type == "ReflectLine") {
                int srcId = t.value("sourceId", -1);
                int lineId = t.value("lineId", -1);
                auto srcPt = getPoint(srcId);
                auto lineObj = std::dynamic_pointer_cast<Line>(getObject(lineId));
                if (srcPt && lineObj) {
                    newPoint = std::make_shared<ReflectLine>(srcPt, lineObj, color);
                }
            } else if (type == "ReflectPoint") {
                int srcId = t.value("sourceId", -1);
                int centerId = t.value("centerId", -1);
                auto srcPt = getPoint(srcId);
                auto centerPt = getPoint(centerId);
                if (srcPt && centerPt) {
                    newPoint = std::make_shared<ReflectPoint>(srcPt, centerPt, color);
                }
            } else if (type == "ReflectCircle") {
                int srcId = t.value("sourceId", -1);
                int circleId = t.value("circleId", -1);
                auto srcPt = getPoint(srcId);
                auto circleObj = std::dynamic_pointer_cast<Circle>(getObject(circleId));
                if (srcPt && circleObj) {
                    newPoint = std::make_shared<ReflectCircle>(srcPt, circleObj, color);
                }
            } else if (type == "RotatePoint") {
                int srcId = t.value("sourceId", -1);
                int centerId = t.value("centerId", -1);
                double angleDeg = t.value("angleDeg", 0.0);
                auto srcPt = getPoint(srcId);
                auto centerPt = getPoint(centerId);
                if (srcPt && centerPt) {
                    newPoint = std::make_shared<RotatePoint>(srcPt, centerPt, angleDeg, color);
                }
            } else if (type == "TranslateVector") {
                int srcId = t.value("sourceId", -1);
                int vStartId = t.value("vecStartId", -1);
                int vEndId = t.value("vecEndId", -1);
                auto srcPt = getPoint(srcId);
                auto vStart = getPoint(vStartId);
                auto vEnd = getPoint(vEndId);
                if (srcPt && vStart && vEnd) {
                    newPoint = std::make_shared<TranslateVector>(srcPt, vStart, vEnd, color);
                }
            } else if (type == "DilatePoint") {
                int srcId = t.value("sourceId", -1);
                int centerId = t.value("centerId", -1);
                double factor = t.value("factor", 1.0);
                auto srcPt = getPoint(srcId);
                auto centerPt = getPoint(centerId);
                if (srcPt && centerPt) {
                    newPoint = std::make_shared<DilatePoint>(srcPt, centerPt, factor, color);
                }
            }

            if (newPoint) {
                newPoint->setID(static_cast<unsigned int>(id));
                applyCommonPointFields(jPoint, newPoint);
                
                // REPLACEMENT LOGIC:
                auto it = std::find_if(editor.points.begin(), editor.points.end(), 
                    [&](const std::shared_ptr<Point>& p) { return p->getID() == static_cast<unsigned int>(id); });
                    
                if (it != editor.points.end()) {
                    *it = newPoint; // Replace existing base point with ReflectPoint
                } else {
                    editor.points.push_back(newPoint);
                }
                
                registerObject(static_cast<unsigned int>(id), newPoint);
                processedDeferredPoints.insert(id);
                progress = true; // Made progress, might unlock others
            }
        }
        if (!progress) break; // No progress made, stop to avoid infinite loop (if dependencies missing)
    }

    // 3a.1 Resolve pending circles (after transform points exist)
    for (const auto& jCircle : pendingCircles) {
      int id = jCircle.value("id", -1);
      int centerId = jCircle.value("centerPointID", -1);
      int radiusPtId = jCircle.value("radiusPointID", -1);
      double radius = jCircle.value("radiusValue", jCircle.value("radius", 10.0));

      auto centerPt = getPoint(centerId);
      if (!centerPt) continue;

      sf::Color color = sf::Color::Black;
      if (jCircle.contains("color")) {
        color = colorFromJson(jCircle["color"], color);
      }

      std::shared_ptr<Point> radiusPt = getPoint(radiusPtId);

      auto newCircle = std::make_shared<Circle>(centerPt.get(), radiusPt, radius, color);
      if (id > 0) newCircle->setID(static_cast<unsigned int>(id));
      newCircle->setThickness(
          jCircle.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

      applyTransformMetadata(jCircle, newCircle);

      bool isSemi = jCircle.value("isSemicircle", false);
      newCircle->setSemicircle(isSemi);
      if (isSemi) {
        int p1Id = jCircle.value("startArcID", jCircle.value("diameterP1ID", -1));
        int p2Id = jCircle.value("endArcID", jCircle.value("diameterP2ID", -1));
        auto p1 = getPoint(p1Id);
        auto p2 = getPoint(p2Id);
        if (p1 && p2) {
          newCircle->setSemicircleDiameterPoints(p1, p2);
          newCircle->setSemicircleBasis(p1->getCGALPosition(), p2->getCGALPosition());
        }
      }

      editor.circles.push_back(newCircle);
      if (id > 0) registerObject(static_cast<unsigned int>(id), newCircle);
    }

    // 3a.2 Resolve pending lines (after transform points exist)
    for (const auto& jLine : pendingLines) {
      int id = jLine.value("id", -1);
      int startId = jLine.value("startPointID", jLine.value("startId", -1));
      int endId = jLine.value("endPointID", jLine.value("endId", -1));

      auto startPt = getPoint(startId);
      auto endPt = getPoint(endId);
      if (!startPt || !endPt) continue;

      bool isSegment = jLine.value("isSegment", false);
      sf::Color color = sf::Color::Black;
      if (jLine.contains("color")) {
        color = colorFromJson(jLine["color"], color);
      }

      auto newLine = std::make_shared<Line>(startPt, endPt, isSegment, color,
                                            static_cast<unsigned int>(std::max(id, 0)));
      editor.lines.push_back(newLine);
      newLine->registerWithEndpoints();

      newLine->setThickness(jLine.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
      newLine->setDecoration(static_cast<DecorationType>(jLine.value("decoration", 0)));
      newLine->setLineType(static_cast<Line::LineType>(jLine.value("lineType", 1)));

      applyTransformMetadata(jLine, newLine);

      if (id > 0) registerObject(static_cast<unsigned int>(id), newLine);
    }

    // 3a.3 Resolve pending polygons (after transform points exist)
    for (const auto& polyJson : pendingPolygons) {
      std::vector<std::shared_ptr<Point>> vertexPoints;

      if (polyJson.contains("vertexIds") && polyJson["vertexIds"].is_array()) {
        for (const auto& idJson : polyJson["vertexIds"]) {
          int vId = idJson.get<int>();
          auto vPtr = getPoint(vId);
          if (!vPtr) {
            vertexPoints.clear();
            break;
          }
          vertexPoints.push_back(vPtr);
        }
      } else if (polyJson.contains("vertices")) {
        for (const auto& vJson : polyJson["vertices"]) {
          if (vJson.is_array() && vJson.size() >= 2) {
            Point_2 pos(vJson[0].get<double>(), vJson[1].get<double>());
            auto vPtr = findPointByPosition(pos);
            if (!vPtr) {
              vertexPoints.clear();
              break;
            }
            vertexPoints.push_back(vPtr);
          } else if (vJson.is_object()) {
            Point_2 pos(vJson.value("x", 0.0), vJson.value("y", 0.0));
            auto vPtr = findPointByPosition(pos);
            if (!vPtr) {
              vertexPoints.clear();
              break;
            }
            vertexPoints.push_back(vPtr);
          }
        }
      }
        
      if (vertexPoints.size() < 3) continue;

      sf::Color color = sf::Color::Black;
      if (polyJson.contains("color")) {
        color = colorFromJson(polyJson["color"], color);
      }

      int id = polyJson.value("id", -1);
      auto polygon = std::make_shared<Polygon>(
          vertexPoints, color, static_cast<unsigned int>(std::max(id, 0)));
          
      polygon->setThickness(
          polyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

      applyTransformMetadata(polyJson, polygon);

      editor.polygons.push_back(polygon);
      if (id > 0) registerObject(static_cast<unsigned int>(id), polygon);

    }

    // ========================================================================
    // PASS 3b: RESTORE TRANSFORMATIONS (SHAPES & DEPENDENCIES)
    // ========================================================================
    std::cout << "[Deserializer] Pass 3b: Restoring Transformation Dependency Graph..." << std::endl;
    
    // Iterate all objects to re-link dependencies
    for (auto& [id, obj] : objectMap) {
        if (!obj) continue;
        
        // Skip Points (handled in deferred transform points or as base objects)
        // We focus on higher-order shapes that lost their links
        if (obj->getType() == ObjectType::Point) continue;

        // Only process objects with a valid transform type
        // Note: We ignore isDependent() check here because legacy/tool-created shapes
        // might have transformType set but isDependent=false. We want to restore them.
        if (obj->getTransformType() != TransformationType::None) {
            unsigned int parentID = obj->getParentSourceID();
            unsigned int auxID = obj->getAuxObjectID();

            auto parent = getObject(parentID);
            auto aux = getObject(auxID); // Can be null for some transforms

            // CRITICAL: Ensure we don't self-reference or loop
            if (parent && parent->getID() != obj->getID()) {
                // Force dependent flag to true since it has a transform
                obj->setDependent(true);
                
                // Restore the connection
                // This re-registers observers and sets internal pointers
                obj->restoreTransformation(parent, aux, obj->getTransformType());
                
                // Force an update to ensure geometry matches parents
                obj->update(); 
            }
        }
    }


    // 3b. Load ObjectPoints
    if (data.contains("objectPoints")) {
      for (const auto& opJson : data["objectPoints"]) {
        int id = opJson.value("id", -1);
        int hostId = opJson.value("hostId", -1);
        int hostTypeInt =
            opJson.value("hostType", static_cast<int>(ObjectType::None));
        double t = opJson.value("t", 0.0);
        size_t edgeIndex = static_cast<size_t>(opJson.value("edgeIndex", 0));
        double edgeRel = opJson.value("edgeRel", t);
        bool visible = opJson.value("visible", true);

        sf::Color color = sf::Color::Black;
        if (opJson.contains("color")) {
          color = colorFromJson(opJson["color"], color);
        }

        ObjectType hostType = static_cast<ObjectType>(hostTypeInt);
        std::shared_ptr<ObjectPoint> newObjPoint = nullptr;

        auto hostObj = getObject(hostId);
        if (!hostObj) continue;

        if (hostType == ObjectType::Line) {
          auto hostLine = std::dynamic_pointer_cast<Line>(hostObj);
          if (hostLine) newObjPoint = ObjectPoint::create(hostLine, t, color);
        } else if (hostType == ObjectType::Circle) {
          auto hostCircle = std::dynamic_pointer_cast<Circle>(hostObj);
          if (hostCircle)
            newObjPoint = ObjectPoint::create(hostCircle, t, color);
        } else if (hostType == ObjectType::Rectangle ||
                   hostType == ObjectType::Triangle ||
                   hostType == ObjectType::Polygon ||
                   hostType == ObjectType::RegularPolygon) {
          newObjPoint =
              ObjectPoint::createOnShapeEdge(hostObj, edgeIndex, edgeRel, color);
        }

        if (newObjPoint) {
          if (id > 0) newObjPoint->setID(static_cast<unsigned int>(id));
          newObjPoint->setVisible(visible);
          applyTransformMetadata(opJson, newObjPoint);
          if (opJson.contains("label"))
            newObjPoint->setLabel(opJson.value("label", ""));
          if (opJson.contains("showLabel"))
            newObjPoint->setShowLabel(opJson.value("showLabel", true));
          if (opJson.contains("labelOffset") &&
              opJson["labelOffset"].is_array() &&
              opJson["labelOffset"].size() >= 2) {
            newObjPoint->setLabelOffset(sf::Vector2f(opJson["labelOffset"][0],
                                                     opJson["labelOffset"][1]));
          }

          editor.ObjectPoints.push_back(newObjPoint);
          if (id > 0) registerObject(static_cast<unsigned int>(id), newObjPoint);
        }
      }
    }

    // 3c. Load Tangent Lines
    if (data.contains("tangentLines")) {
      for (const auto& tJson : data["tangentLines"]) {
        int id = tJson.value("id", -1);
        int externalId = tJson.value("externalPointID", -1);
        int circleId = tJson.value("circleID", -1);
        int solutionIndex = tJson.value("solutionIndex", 0);

        auto externalPt = getPoint(externalId);
        auto circleObj = std::dynamic_pointer_cast<Circle>(getObject(circleId));

        if (!externalPt || !circleObj) continue;

        sf::Color color = sf::Color::Black;
        if (tJson.contains("color")) {
          color = colorFromJson(tJson["color"], color);
        }

        auto tangent = std::make_shared<TangentLine>(
            externalPt, circleObj, solutionIndex,
            static_cast<unsigned int>(std::max(id, 0)), color);

        tangent->setThickness(static_cast<float>(
            tJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT)));

        editor.lines.push_back(tangent);
        if (id > 0) registerObject(static_cast<unsigned int>(id), tangent);
      }
    }

    // 3d. Load Angles
    if (data.contains("angles")) {
      for (const auto& aJson : data["angles"]) {
        int id = aJson.value("id", -1);
        int aId = aJson.value("pointAID", -1);
        int vId = aJson.value("vertexID", -1);
        int bId = aJson.value("pointBID", -1);
        bool reflex = aJson.value("reflex", false);

        auto pA = getPoint(aId);
        auto pV = getPoint(vId);
        auto pB = getPoint(bId);

        if (!pA || !pV || !pB) continue;

        sf::Color color = sf::Color(255, 200, 0);
        if (aJson.contains("color")) {
          color = colorFromJson(aJson["color"], color);
        }

        auto angle = std::make_shared<Angle>(pA, pV, pB, reflex, color);
        if (id > 0) angle->setID(static_cast<unsigned int>(id));

        if (aJson.contains("radius"))
          angle->setRadius(aJson.value("radius", angle->getRadius()));
        if (aJson.contains("showLabel"))
          angle->setShowLabel(aJson.value("showLabel", true));
        if (aJson.contains("labelOffset") && aJson["labelOffset"].is_array() &&
            aJson["labelOffset"].size() >= 2) {
          angle->setLabelOffset(
              sf::Vector2f(aJson["labelOffset"][0], aJson["labelOffset"][1]));
        }

        editor.angles.push_back(angle);
        if (id > 0) registerObject(static_cast<unsigned int>(id), angle);
      }
    }

    // 3e. Register Intersection Constraints
    if (data.contains("intersections")) {
      for (const auto& cJson : data["intersections"]) {
        int aId = cJson.value("aId", -1);
        int bId = cJson.value("bId", -1);

        auto A = getObject(aId);
        auto B = getObject(bId);

        if (!A || !B) continue;

        std::vector<std::shared_ptr<Point>> createdPoints;
        if (cJson.contains("points") && cJson["points"].is_array()) {
          for (const auto& pJson : cJson["points"]) {
            int ptId = pJson.value("id", -1);
            auto pt = getPoint(ptId);
            if (pt) {
              createdPoints.push_back(pt);
            }
          }
        }

        if (!createdPoints.empty()) {
          DynamicIntersection::registerIntersectionConstraint(A, B,
                                                              createdPoints);
        }
      }
    }

    // 3f. Restore Line Constraints (Parallel/Perpendicular)
    if (data.contains("lines")) {
      for (const auto& jLine : data["lines"]) {
        int id = jLine.value("id", -1);
        if (id <= 0) continue;

        auto lineObj = std::dynamic_pointer_cast<Line>(getObject(id));
        if (!lineObj) continue;

        bool isParallel = jLine.value("isParallel", false);
        bool isPerp = jLine.value("isPerpendicular", false);

        if ((isParallel || isPerp) && jLine.contains("constraintRefId")) {
          int refId = jLine["constraintRefId"];
          int edgeIndex = jLine.value("constraintRefEdgeIndex", -1);

          auto refObj = getObject(refId);
          if (refObj) {
            Point_2 s = lineObj->getStartPoint();
            Point_2 e = lineObj->getEndPoint();
            Vector_2 dir(e.x() - s.x(), e.y() - s.y());

            if (isParallel)
              lineObj->setAsParallelLine(refObj, edgeIndex, dir);
            else
              lineObj->setAsPerpendicularLine(refObj, edgeIndex, dir);
          }
        }
      }
    }

    // 3g. Restore Transformation Links for ALL objects
    auto restoreTransformForJson = [&](const json& objJson) {
      int id = objJson.value("id", -1);
      if (id <= 0) return;
      if (!objJson.contains("transformType")) return;

      int typeInt =
          objJson.value("transformType", static_cast<int>(TransformationType::None));
      if (typeInt == static_cast<int>(TransformationType::None)) return;

      auto obj = getObject(id);
      if (!obj) return;

      unsigned int parentId = objJson.value("parentSourceID", 0u);
      unsigned int auxId = objJson.value("auxObjectID", 0u);

      auto parent = (parentId != 0) ? getObject(static_cast<int>(parentId)) : nullptr;
      auto aux = (auxId != 0) ? getObject(static_cast<int>(auxId)) : nullptr;

      obj->restoreTransformation(parent, aux,
                 static_cast<TransformationType>(typeInt));
      obj->setDependent(true);
      obj->setLocked(true);
    };

    // Apply to all shape types
    if (data.contains("points")) {
      for (const auto& j : data["points"]) restoreTransformForJson(j);
    }
    if (data.contains("lines")) {
      for (const auto& j : data["lines"]) restoreTransformForJson(j);
    }
    if (data.contains("circles")) {
      for (const auto& j : data["circles"]) restoreTransformForJson(j);
    }
    if (data.contains("rectangles")) {
      for (const auto& j : data["rectangles"]) restoreTransformForJson(j);
    }
    if (data.contains("polygons")) {
      for (const auto& j : data["polygons"]) restoreTransformForJson(j);
    }
    if (data.contains("triangles")) {
      for (const auto& j : data["triangles"]) restoreTransformForJson(j);
    }
    if (data.contains("regularPolygons")) {
      for (const auto& j : data["regularPolygons"]) restoreTransformForJson(j);
    }

    std::cout << "[Deserializer] Pass 3 Complete." << std::endl;

    // ========================================================================
    // PASS 4: FORCE UPDATE ALL OBJECTS
    // ========================================================================
    std::cout << "[Deserializer] Pass 4: Updating All Objects..." << std::endl;

    for (auto& [id, obj] : objectMap) {
      if (obj) obj->update();
    }

    // Restore axes visibility
    if (editor.getXAxisShared()) {
      editor.getXAxisShared()->setVisible(axesVisible);
      if (std::find(editor.lines.begin(), editor.lines.end(),
                    editor.getXAxisShared()) == editor.lines.end()) {
        editor.lines.push_back(editor.getXAxisShared());
      }
    }
    if (editor.getYAxisShared()) {
      editor.getYAxisShared()->setVisible(axesVisible);
      if (std::find(editor.lines.begin(), editor.lines.end(),
                    editor.getYAxisShared()) == editor.lines.end()) {
        editor.lines.push_back(editor.getYAxisShared());
      }
    }

    // Final updates
    DynamicIntersection::updateAllIntersections(editor);
    for (auto& ag : editor.angles) {
      if (ag) ag->update();
    }

    editor.objectIdCounter = maxId + 1;
    std::cout << "[Deserializer] Load Complete. ID Counter: "
              << editor.objectIdCounter << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Deserializer::loadProject: Exception: " << e.what()
              << std::endl;
    return false;
  }
}
