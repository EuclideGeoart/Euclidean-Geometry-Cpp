#include "Deserializer.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "Angle.h"
#include "Circle.h"
#include "Constants.h"
#include "ConstructionObjects.h"
#include "DynamicIntersection.h"
#include "GeometricObject.h"
#include "GeometryEditor.h"
#include "Line.h"
#include "ObjectPoint.h"
#include "Point.h"
#include "Polygon.h"
#include "Rectangle.h"
#include "RegularPolygon.h"
#include "Triangle.h"
#include "json.hpp"

using json = nlohmann::json;

namespace {

sf::Color hexToColor(const std::string& hex) {
  if (hex.empty() || hex[0] != '#' || hex.length() < 7) {
    return sf::Color::Blue;
  }

  unsigned int r = 0;
  unsigned int g = 0;
  unsigned int b = 0;
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

sf::Color colorFromJson(const json& j, const sf::Color& fallback) {
  if (j.is_string()) {
    return hexToColor(j.get<std::string>());
  }
  if (j.is_object()) {
    return sf::Color(j.value("r", fallback.r), j.value("g", fallback.g), j.value("b", fallback.b),
                     j.value("a", fallback.a));
  }
  return fallback;
}

void applyCommonPointFields(const json& jPoint, const std::shared_ptr<Point>& point) {
  if (jPoint.contains("label")) point->setLabel(jPoint.value("label", ""));
  if (jPoint.contains("showLabel")) point->setShowLabel(jPoint.value("showLabel", true));
  if (jPoint.contains("labelOffset") && jPoint["labelOffset"].is_array() &&
      jPoint["labelOffset"].size() >= 2) {
    point->setLabelOffset(sf::Vector2f(jPoint["labelOffset"][0], jPoint["labelOffset"][1]));
  }
}

}  // namespace

bool Deserializer::loadProject(GeometryEditor& editor, const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Deserializer::loadProject: Failed to open file: " << filepath << std::endl;
    return false;
  }

  try {
    json j;
    file >> j;
    file.close();

    editor.clearScene();
    DynamicIntersection::clearAllIntersectionConstraints(editor);

    const json& data = j.contains("objects") ? j["objects"] : j;

    bool axesVisible = true;
    if (j.contains("settings") && j["settings"].contains("axesVisible")) {
      axesVisible = j["settings"]["axesVisible"].get<bool>();
    }

    std::map<unsigned int, std::shared_ptr<GeometricObject>> objectMap;
    std::unordered_map<unsigned int, std::shared_ptr<Point>> pointMap;
    unsigned int maxId = 0;

    auto registerObject = [&](unsigned int id, const std::shared_ptr<GeometricObject>& obj) {
      if (!obj) return;
      if (id == 0) return;
      objectMap[id] = obj;
      if (auto pt = std::dynamic_pointer_cast<Point>(obj)) {
        pointMap[id] = pt;
      }
      if (id > maxId) maxId = id;
    };

    auto loadPointFromJson = [&](const json& jPoint, bool isIntersection) -> std::shared_ptr<Point> {
      int id = jPoint.value("id", -1);
      if (id != -1) {
        auto existing = objectMap.find(static_cast<unsigned int>(id));
        if (existing != objectMap.end()) {
          auto existingPoint = std::dynamic_pointer_cast<Point>(existing->second);
          if (existingPoint && isIntersection) {
            existingPoint->setIntersectionPoint(true);
            existingPoint->setDependent(true);
            existingPoint->setSelected(false);
            existingPoint->lock();
          }
          return existingPoint;
        }
      }

      double x = jPoint.value("x", 0.0);
      double y = jPoint.value("y", 0.0);
      sf::Color color = sf::Color::Black;
      if (jPoint.contains("color")) {
        color = colorFromJson(jPoint["color"], color);
      }

      auto newPoint = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color,
                                              static_cast<unsigned int>(std::max(id, 0)));
      applyCommonPointFields(jPoint, newPoint);

      if (jPoint.contains("transformType")) {
        newPoint->setTransformType(
            static_cast<TransformationType>(jPoint["transformType"].get<int>()));
        newPoint->setParentSourceID(jPoint.value("parentSourceID", 0u));
        newPoint->setAuxObjectID(jPoint.value("auxObjectID", 0u));
      }
      if (jPoint.contains("transformValue")) {
        newPoint->setTransformValue(jPoint["transformValue"].get<double>());
      }

      if (isIntersection) {
        newPoint->setIntersectionPoint(true);
        newPoint->setDependent(true);
        newPoint->setSelected(false);
        newPoint->lock();
      }

      editor.points.push_back(newPoint);
      if (id != -1) {
        registerObject(static_cast<unsigned int>(id), newPoint);
      }
      return newPoint;
    };

    // =========================================================
    // PASS 1: LOAD POINTS (and intersection points)
    // =========================================================
    if (data.contains("points")) {
      for (const auto& jPoint : data["points"]) {
        loadPointFromJson(jPoint, false);
      }
    }

    if (data.contains("intersections")) {
      for (const auto& cJson : data["intersections"]) {
        if (cJson.contains("points") && cJson["points"].is_array()) {
          for (const auto& pJson : cJson["points"]) {
            loadPointFromJson(pJson, true);
          }
        }
      }
    }

    // =========================================================
    // PASS 2: LOAD SHAPES (Lines, Circles, Rectangles, etc.)
    // =========================================================
    if (data.contains("lines")) {
      for (const auto& jLine : data["lines"]) {
        int startId = jLine.value("startPointID", -1);
        if (startId == -1) startId = jLine.value("startId", -1);
        int endId = jLine.value("endPointID", -1);
        if (endId == -1) endId = jLine.value("endId", -1);
        int id = jLine.value("id", -1);
        bool isSegment = jLine.value("isSegment", false);

        if (pointMap.count(startId) && pointMap.count(endId)) {
          sf::Color color = sf::Color::Black;
          if (jLine.contains("color")) {
            color = colorFromJson(jLine["color"], color);
          }

          auto newLine = std::make_shared<Line>(pointMap[startId], pointMap[endId], isSegment, color,
                                                static_cast<unsigned int>(std::max(id, 0)));
          editor.lines.push_back(newLine);
          newLine->registerWithEndpoints();

          newLine->setThickness(jLine.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
          newLine->setDecoration(static_cast<DecorationType>(jLine.value("decoration", 0)));
          newLine->setLineType(static_cast<Line::LineType>(jLine.value("lineType", 1)));

          if (jLine.contains("transformType")) {
            newLine->setTransformType(
                static_cast<TransformationType>(jLine["transformType"].get<int>()));
            newLine->setParentSourceID(jLine.value("parentSourceID", 0u));
            newLine->setAuxObjectID(jLine.value("auxObjectID", 0u));
          }
          if (jLine.contains("transformValue")) {
            newLine->setTransformValue(jLine["transformValue"].get<double>());
          }

          if (id != -1) registerObject(static_cast<unsigned int>(id), newLine);
        }
      }
    }

    if (data.contains("circles")) {
      for (const auto& jCircle : data["circles"]) {
        int centerId = jCircle.value("centerPointID", -1);
        int radiusPtId = jCircle.value("radiusPointID", -1);
        double radius = jCircle.value("radiusValue", jCircle.value("radius", 10.0));
        int id = jCircle.value("id", -1);

        if (!pointMap.count(centerId)) continue;

        sf::Color color = sf::Color::Black;
        if (jCircle.contains("color")) {
          color = colorFromJson(jCircle["color"], color);
        }

        std::shared_ptr<Point> radiusPt = nullptr;
        if (radiusPtId != -1 && pointMap.count(radiusPtId)) {
          radiusPt = pointMap[radiusPtId];
        }

        auto newCircle = std::make_shared<Circle>(pointMap[centerId].get(), radiusPt, radius, color);
        if (id != -1) newCircle->setID(static_cast<unsigned int>(id));
        editor.circles.push_back(newCircle);

        newCircle->setThickness(jCircle.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

        if (jCircle.contains("transformType")) {
          newCircle->setTransformType(
              static_cast<TransformationType>(jCircle["transformType"].get<int>()));
          newCircle->setParentSourceID(jCircle.value("parentSourceID", 0u));
          newCircle->setAuxObjectID(jCircle.value("auxObjectID", 0u));
        }
        if (jCircle.contains("transformValue")) {
          newCircle->setTransformValue(jCircle["transformValue"].get<double>());
        }

        bool isSemi = jCircle.value("isSemicircle", false);
        newCircle->setSemicircle(isSemi);
        if (isSemi) {
          int p1Id = jCircle.value("diameterP1ID", -1);
          int p2Id = jCircle.value("diameterP2ID", -1);
          if (pointMap.count(p1Id) && pointMap.count(p2Id)) {
            newCircle->setSemicircleDiameterPoints(pointMap[p1Id], pointMap[p2Id]);
          }
        }

        if (id != -1) registerObject(static_cast<unsigned int>(id), newCircle);
      }
    }

    if (data.contains("rectangles")) {
      for (const auto& rectJson : data["rectangles"]) {
        if (!rectJson.contains("vertices")) continue;
        auto verticesJson = rectJson["vertices"];
        if (verticesJson.size() < 2) continue;

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

        sf::Color color = sf::Color::Black;
        if (rectJson.contains("color")) {
          color = colorFromJson(rectJson["color"], color);
        }

        int id = rectJson.value("id", -1);
        bool isRotatable = rectJson.value("isRotatable", false);
        double height = rectJson.value("height", 0.0);

        auto rect = std::make_shared<Rectangle>(Point_2(x1, y1), Point_2(x2, y2), isRotatable, color,
                                                static_cast<unsigned int>(std::max(id, 0)));
        rect->setThickness(rectJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
        if (isRotatable) {
          rect->setHeight(height);
        }

        if (rectJson.contains("transformType")) {
          rect->setTransformType(
              static_cast<TransformationType>(rectJson["transformType"].get<int>()));
          rect->setParentSourceID(rectJson.value("parentSourceID", 0u));
          rect->setAuxObjectID(rectJson.value("auxObjectID", 0u));
        }
        if (rectJson.contains("transformValue")) {
          rect->setTransformValue(rectJson["transformValue"].get<double>());
        }

        editor.rectangles.push_back(rect);
        if (id != -1) registerObject(static_cast<unsigned int>(id), rect);
      }
    }

    if (data.contains("polygons")) {
      for (const auto& polyJson : data["polygons"]) {
        if (!polyJson.contains("vertices")) continue;
        std::vector<Point_2> vertices;
        for (const auto& vJson : polyJson["vertices"]) {
          if (vJson.is_array() && vJson.size() >= 2) {
            vertices.emplace_back(vJson[0].get<double>(), vJson[1].get<double>());
          } else if (vJson.is_object()) {
            vertices.emplace_back(vJson.value("x", 0.0), vJson.value("y", 0.0));
          }
        }

        sf::Color color = sf::Color::Black;
        if (polyJson.contains("color")) {
          color = colorFromJson(polyJson["color"], color);
        }

        int id = polyJson.value("id", -1);
        auto polygon = std::make_shared<Polygon>(vertices, color,
                                                 static_cast<unsigned int>(std::max(id, 0)));
        polygon->setThickness(polyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

        if (polyJson.contains("transformType")) {
          polygon->setTransformType(
              static_cast<TransformationType>(polyJson["transformType"].get<int>()));
          polygon->setParentSourceID(polyJson.value("parentSourceID", 0u));
          polygon->setAuxObjectID(polyJson.value("auxObjectID", 0u));
        }
        if (polyJson.contains("transformValue")) {
          polygon->setTransformValue(polyJson["transformValue"].get<double>());
        }

        editor.polygons.push_back(polygon);
        if (id != -1) registerObject(static_cast<unsigned int>(id), polygon);
      }
    }

    if (data.contains("triangles")) {
      for (const auto& triJson : data["triangles"]) {
        if (!triJson.contains("vertices")) continue;
        auto verticesJson = triJson["vertices"];
        if (verticesJson.size() < 3) continue;

        std::vector<Point_2> pts;
        for (int i = 0; i < 3; ++i) {
          if (verticesJson[i].is_array()) {
            pts.emplace_back(verticesJson[i][0].get<double>(), verticesJson[i][1].get<double>());
          } else {
            pts.emplace_back(verticesJson[i].value("x", 0.0), verticesJson[i].value("y", 0.0));
          }
        }

        sf::Color color = sf::Color::Black;
        if (triJson.contains("color")) {
          color = colorFromJson(triJson["color"], color);
        }

        int id = triJson.value("id", -1);
        auto triangle = std::make_shared<Triangle>(pts[0], pts[1], pts[2], color,
                                                   static_cast<unsigned int>(std::max(id, 0)));
        triangle->setThickness(triJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

        if (triJson.contains("transformType")) {
          triangle->setTransformType(
              static_cast<TransformationType>(triJson["transformType"].get<int>()));
          triangle->setParentSourceID(triJson.value("parentSourceID", 0u));
          triangle->setAuxObjectID(triJson.value("auxObjectID", 0u));
        }
        if (triJson.contains("transformValue")) {
          triangle->setTransformValue(triJson["transformValue"].get<double>());
        }

        editor.triangles.push_back(triangle);
        if (id != -1) registerObject(static_cast<unsigned int>(id), triangle);
      }
    }

    if (data.contains("regularPolygons")) {
      for (const auto& rpolyJson : data["regularPolygons"]) {
        if (!rpolyJson.contains("vertices")) continue;
        auto verticesJson = rpolyJson["vertices"];
        if (verticesJson.empty()) continue;

        int sides = rpolyJson.value("sides", 3);
        int id = rpolyJson.value("id", -1);

        sf::Color color = sf::Color::Black;
        if (rpolyJson.contains("color")) {
          color = colorFromJson(rpolyJson["color"], color);
        }

        double cx = 0.0;
        double cy = 0.0;
        double fx = 0.0;
        double fy = 0.0;
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

        auto rpoly = std::make_shared<RegularPolygon>(Point_2(cx, cy), Point_2(fx, fy), sides, color,
                                                      static_cast<unsigned int>(std::max(id, 0)));
        rpoly->setThickness(rpolyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

        if (rpolyJson.contains("transformType")) {
          rpoly->setTransformType(
              static_cast<TransformationType>(rpolyJson["transformType"].get<int>()));
          rpoly->setParentSourceID(rpolyJson.value("parentSourceID", 0u));
          rpoly->setAuxObjectID(rpolyJson.value("auxObjectID", 0u));
        }
        if (rpolyJson.contains("transformValue")) {
          rpoly->setTransformValue(rpolyJson["transformValue"].get<double>());
        }

        editor.regularPolygons.push_back(rpoly);
        if (id != -1) registerObject(static_cast<unsigned int>(id), rpoly);
      }
    }

    if (data.contains("objectPoints")) {
      for (const auto& opJson : data["objectPoints"]) {
        int id = opJson.value("id", -1);
        int hostId = opJson.value("hostId", -1);
        int hostTypeInt = opJson.value("hostType", static_cast<int>(ObjectType::None));
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

        if (objectMap.count(static_cast<unsigned int>(hostId))) {
          auto hostShape = objectMap[static_cast<unsigned int>(hostId)];
          if (hostType == ObjectType::Line) {
            auto hostLine = std::dynamic_pointer_cast<Line>(hostShape);
            if (hostLine) newObjPoint = ObjectPoint::create(hostLine, t, color);
          } else if (hostType == ObjectType::Circle) {
            auto hostCircle = std::dynamic_pointer_cast<Circle>(hostShape);
            if (hostCircle) newObjPoint = ObjectPoint::create(hostCircle, t, color);
          } else if (hostType == ObjectType::Rectangle || hostType == ObjectType::Triangle ||
                     hostType == ObjectType::Polygon || hostType == ObjectType::RegularPolygon) {
            newObjPoint = ObjectPoint::createOnShapeEdge(hostShape, edgeIndex, edgeRel, color);
          }
        }

        if (newObjPoint) {
          if (id != -1) newObjPoint->setID(static_cast<unsigned int>(id));
          newObjPoint->setVisible(visible);
          if (opJson.contains("label")) newObjPoint->setLabel(opJson.value("label", ""));
          if (opJson.contains("showLabel")) newObjPoint->setShowLabel(opJson.value("showLabel", true));
          if (opJson.contains("labelOffset") && opJson["labelOffset"].is_array() &&
              opJson["labelOffset"].size() >= 2) {
            newObjPoint->setLabelOffset(sf::Vector2f(opJson["labelOffset"][0], opJson["labelOffset"][1]));
          }

          editor.ObjectPoints.push_back(newObjPoint);
          if (id != -1) registerObject(static_cast<unsigned int>(id), newObjPoint);
        }
      }
    }

    if (data.contains("tangentLines")) {
      for (const auto& tJson : data["tangentLines"]) {
        int id = tJson.value("id", -1);
        int externalId = tJson.value("externalPointID", -1);
        int circleId = tJson.value("circleID", -1);
        int solutionIndex = tJson.value("solutionIndex", 0);

        if (pointMap.count(externalId) && objectMap.count(static_cast<unsigned int>(circleId))) {
          auto circlePtr = std::dynamic_pointer_cast<Circle>(objectMap[static_cast<unsigned int>(circleId)]);
          if (!circlePtr) continue;

          sf::Color color = sf::Color::Black;
          if (tJson.contains("color")) {
            color = colorFromJson(tJson["color"], color);
          }

          auto tangent = std::make_shared<TangentLine>(pointMap[externalId], circlePtr, solutionIndex,
                                                       static_cast<unsigned int>(std::max(id, 0)), color);
          tangent->setThickness(static_cast<float>(tJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT)));
          editor.lines.push_back(tangent);
          if (id != -1) registerObject(static_cast<unsigned int>(id), tangent);
        }
      }
    }

    if (data.contains("angles")) {
      for (const auto& aJson : data["angles"]) {
        int id = aJson.value("id", -1);
        int aId = aJson.value("pointAID", -1);
        int vId = aJson.value("vertexID", -1);
        int bId = aJson.value("pointBID", -1);
        bool reflex = aJson.value("reflex", false);

        if (pointMap.count(aId) && pointMap.count(vId) && pointMap.count(bId)) {
          sf::Color color = sf::Color(255, 200, 0);
          if (aJson.contains("color")) {
            color = colorFromJson(aJson["color"], color);
          }

          auto angle = std::make_shared<Angle>(pointMap[aId], pointMap[vId], pointMap[bId], reflex, color);
          if (id != -1) angle->setID(static_cast<unsigned int>(id));

          if (aJson.contains("radius")) angle->setRadius(aJson.value("radius", angle->getRadius()));
          if (aJson.contains("showLabel")) angle->setShowLabel(aJson.value("showLabel", true));
          if (aJson.contains("labelOffset") && aJson["labelOffset"].is_array() &&
              aJson["labelOffset"].size() >= 2) {
            angle->setLabelOffset(sf::Vector2f(aJson["labelOffset"][0], aJson["labelOffset"][1]));
          }

          editor.angles.push_back(angle);
          if (id != -1) registerObject(static_cast<unsigned int>(id), angle);
        }
      }
    }

    if (data.contains("intersections")) {
      for (const auto& cJson : data["intersections"]) {
        int aId = cJson.value("aId", -1);
        int bId = cJson.value("bId", -1);
        if (!objectMap.count(static_cast<unsigned int>(aId)) ||
            !objectMap.count(static_cast<unsigned int>(bId))) {
          continue;
        }

        auto A = objectMap[static_cast<unsigned int>(aId)];
        auto B = objectMap[static_cast<unsigned int>(bId)];

        std::vector<std::shared_ptr<Point>> createdPoints;
        if (cJson.contains("points") && cJson["points"].is_array()) {
          for (const auto& pJson : cJson["points"]) {
            int id = pJson.value("id", -1);
            if (id != -1 && pointMap.count(id)) {
              createdPoints.push_back(pointMap[id]);
            }
          }
        }

        if (!createdPoints.empty()) {
          DynamicIntersection::registerIntersectionConstraint(A, B, createdPoints);
        }
      }
    }

    if (data.contains("lines")) {
      for (const auto& jLine : data["lines"]) {
        int id = jLine.value("id", -1);
        if (id == -1) continue;

        auto it = objectMap.find(static_cast<unsigned int>(id));
        if (it == objectMap.end()) continue;

        if (it->second->getType() == ObjectType::Line) {
          auto linePtr = std::dynamic_pointer_cast<Line>(it->second);
          bool isParallel = jLine.value("isParallel", false);
          bool isPerp = jLine.value("isPerpendicular", false);

          if ((isParallel || isPerp) && jLine.contains("constraintRefId")) {
            int refId = jLine["constraintRefId"];
            int edgeIndex = jLine.value("constraintRefEdgeIndex", -1);
            auto refIt = objectMap.find(static_cast<unsigned int>(refId));
            if (refIt != objectMap.end()) {
              Point_2 s = linePtr->getStartPoint();
              Point_2 e = linePtr->getEndPoint();
              Vector_2 dir(e.x() - s.x(), e.y() - s.y());
              if (isParallel) linePtr->setAsParallelLine(refIt->second, edgeIndex, dir);
              else linePtr->setAsPerpendicularLine(refIt->second, edgeIndex, dir);
            }
          }
        }
      }
    }

    // =========================================================
    // PASS 3: RESTORE TRANSFORMATIONS (JSON-driven)
    // =========================================================
    auto restoreTransformForJson = [&](const json& objJson) {
      int id = objJson.value("id", -1);
      if (id == -1) return;
      if (!objJson.contains("transformType")) return;
      int typeInt = objJson.value("transformType", static_cast<int>(TransformationType::None));
      if (typeInt == static_cast<int>(TransformationType::None)) return;

      auto it = objectMap.find(static_cast<unsigned int>(id));
      if (it == objectMap.end()) return;

      unsigned int parentId = objJson.value("parentSourceID", 0u);
      unsigned int auxId = objJson.value("auxObjectID", 0u);
      std::shared_ptr<GeometricObject> parent = nullptr;
      std::shared_ptr<GeometricObject> aux = nullptr;
      if (parentId != 0 && objectMap.count(parentId)) parent = objectMap[parentId];
      if (auxId != 0 && objectMap.count(auxId)) aux = objectMap[auxId];

      it->second->restoreTransformation(parent, aux,
                                        static_cast<TransformationType>(typeInt));
    };

    if (data.contains("points")) {
      for (const auto& jPoint : data["points"]) restoreTransformForJson(jPoint);
    }
    if (data.contains("lines")) {
      for (const auto& jLine : data["lines"]) restoreTransformForJson(jLine);
    }
    if (data.contains("circles")) {
      for (const auto& jCircle : data["circles"]) restoreTransformForJson(jCircle);
    }
    if (data.contains("rectangles")) {
      for (const auto& rectJson : data["rectangles"]) restoreTransformForJson(rectJson);
    }
    if (data.contains("polygons")) {
      for (const auto& polyJson : data["polygons"]) restoreTransformForJson(polyJson);
    }
    if (data.contains("triangles")) {
      for (const auto& triJson : data["triangles"]) restoreTransformForJson(triJson);
    }
    if (data.contains("regularPolygons")) {
      for (const auto& rpolyJson : data["regularPolygons"]) restoreTransformForJson(rpolyJson);
    }

    // =========================================================
    // PASS 4: UPDATE ALL OBJECTS
    // =========================================================
    for (auto& [id, obj] : objectMap) {
      if (obj) obj->update();
    }

    // Restore axes visibility and ensure axes are present
    if (editor.getXAxisShared()) {
      editor.getXAxisShared()->setVisible(axesVisible);
      if (std::find(editor.lines.begin(), editor.lines.end(), editor.getXAxisShared()) ==
          editor.lines.end()) {
        editor.lines.push_back(editor.getXAxisShared());
      }
    }
    if (editor.getYAxisShared()) {
      editor.getYAxisShared()->setVisible(axesVisible);
      if (std::find(editor.lines.begin(), editor.lines.end(), editor.getYAxisShared()) ==
          editor.lines.end()) {
        editor.lines.push_back(editor.getYAxisShared());
      }
    }

    DynamicIntersection::updateAllIntersections(editor);

    editor.objectIdCounter = maxId + 1;
    std::cout << "Project loaded. Counter: " << editor.objectIdCounter << std::endl;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "Deserializer::loadProject: Exception: " << e.what() << std::endl;
    return false;
  }
}
