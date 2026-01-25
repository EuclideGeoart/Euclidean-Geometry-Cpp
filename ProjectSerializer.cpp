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
#include "GeometryEditor.h"
#include "Point.h"
#include "Line.h"  
#include "Circle.h"
#include "Rectangle.h"
#include "Polygon.h"
#include "RegularPolygon.h"
#include "Triangle.h"
#include "ObjectPoint.h"
#include "Constants.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

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
    ss << "#" << std::hex << std::setfill('0')
       << std::setw(2) << static_cast<int>(color.r)
       << std::setw(2) << static_cast<int>(color.g)
       << std::setw(2) << static_cast<int>(color.b);
    return ss.str();
}

sf::Color ProjectSerializer::hexToColor(const std::string& hex) {
    if (hex.empty() || hex[0] != '#' || hex.length() < 7) {
        return sf::Color::Blue; // Default
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
    
    return sf::Color(static_cast<sf::Uint8>(r), 
                     static_cast<sf::Uint8>(g), 
                     static_cast<sf::Uint8>(b));
}

void ProjectSerializer::worldToSVG(double worldX, double worldY,
                                    double& svgX, double& svgY,
                                    double viewHeight, double minY) {
    // SVG has Y-axis pointing down, our world has Y-axis pointing up
    // We need to flip Y and offset to make all coordinates positive
    svgX = worldX;
    svgY = viewHeight - (worldY - minY);
}

void ProjectSerializer::calculateBounds(const GeometryEditor& editor,
                                         double& minX, double& minY,
                                         double& maxX, double& maxY) {
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
    
    // Check all rectangles
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
            }
            hasObjects = true;
        }
    }
    
    // Check all polygons
    for (const auto& poly : editor.polygons) {
        if (poly && poly->isValid()) {
            auto vertices = poly->getInteractableVertices();
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
    
    // Default bounds if no objects
    if (!hasObjects) {
        minX = -100; minY = -100;
        maxX = 100; maxY = 100;
    }
    
    // Add 10% padding
    double padX = (maxX - minX) * 0.1;
    double padY = (maxY - minY) * 0.1;
    minX -= padX; minY -= padY;
    maxX += padX; maxY += padY;
}

// ============================================================================
// SAVE PROJECT
// ============================================================================

bool ProjectSerializer::saveProject(const GeometryEditor& editor, const std::string& filepath) {
    try {
        json project;
        project["version"] = "1.0";
        project["objects"] = json::object();

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
                pointsArray.push_back(ptJson);
            }
        }
        project["objects"]["points"] = pointsArray;

        // Save Lines
        json linesArray = json::array();
        for (const auto& ln : editor.lines) {
            if (ln && ln->isValid()) {
                json lnJson;
                lnJson["id"] = ln->getID();
                lnJson["isSegment"] = ln->isSegment();
                lnJson["color"] = colorToHex(ln->getColor());

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
                auto vertices = rect->getInteractableVertices();
                json verticesJson = json::array();
                for (const auto& v : vertices) {
                    verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                }
                rectJson["vertices"] = verticesJson;
                rectJson["color"] = colorToHex(rect->getColor());
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
                auto vertices = poly->getInteractableVertices();
                json verticesJson = json::array();
                for (const auto& v : vertices) {
                    verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                }
                polyJson["vertices"] = verticesJson;
                polyJson["color"] = colorToHex(poly->getColor());
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
                auto vertices = tri->getInteractableVertices();
                json verticesJson = json::array();
                for (const auto& v : vertices) {
                    verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                }
                triJson["vertices"] = verticesJson;
                triJson["color"] = colorToHex(tri->getColor());
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
                auto vertices = rpoly->getInteractableVertices();
                json verticesJson = json::array();
                for (const auto& v : vertices) {
                    verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                }
                rpolyJson["vertices"] = verticesJson;
                rpolyJson["sides"] = rpoly->getNumSides();
                rpolyJson["color"] = colorToHex(rpoly->getColor());
                regularPolygonsArray.push_back(rpolyJson);
            }
        }
        project["objects"]["regularPolygons"] = regularPolygonsArray;

        // Save Shapes (combined for convenience)
        json shapesArray = json::array();
        for (const auto& rect : editor.rectangles) {
            if (rect && rect->isValid()) {
                json shapeJson;
                shapeJson["shapeType"] = "rectangle";
                shapeJson["id"] = rect->getID();
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
        for (const auto& poly : editor.polygons) {
            if (poly && poly->isValid()) {
                json shapeJson;
                shapeJson["shapeType"] = "polygon";
                shapeJson["id"] = poly->getID();
                auto vertices = poly->getInteractableVertices();
                json verticesJson = json::array();
                for (const auto& v : vertices) {
                    verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                }
                shapeJson["vertices"] = verticesJson;
                shapeJson["color"] = colorToHex(poly->getColor());
                shapesArray.push_back(shapeJson);
            }
        }
        for (const auto& tri : editor.triangles) {
            if (tri && tri->isValid()) {
                json shapeJson;
                shapeJson["shapeType"] = "triangle";
                shapeJson["id"] = tri->getID();
                auto vertices = tri->getInteractableVertices();
                json verticesJson = json::array();
                for (const auto& v : vertices) {
                    verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                }
                shapeJson["vertices"] = verticesJson;
                shapeJson["color"] = colorToHex(tri->getColor());
                shapesArray.push_back(shapeJson);
            }
        }
        for (const auto& rpoly : editor.regularPolygons) {
            if (rpoly && rpoly->isValid()) {
                json shapeJson;
                shapeJson["shapeType"] = "regularPolygon";
                shapeJson["id"] = rpoly->getID();
                auto vertices = rpoly->getInteractableVertices();
                json verticesJson = json::array();
                for (const auto& v : vertices) {
                    verticesJson.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                }
                shapeJson["vertices"] = verticesJson;
                shapeJson["sides"] = rpoly->getNumSides();
                shapeJson["color"] = colorToHex(rpoly->getColor());
                shapesArray.push_back(shapeJson);
            }
        }
        project["objects"]["shapes"] = shapesArray;

        // Save ObjectPoints
        json objectPointsArray = json::array();
        for (const auto& op : editor.ObjectPoints) {
            if (op && op->isValid()) {
                json opJson;
                opJson["id"] = op->getID();
                opJson["hostType"] = static_cast<int>(op->getHostType());
                if (op->getHostType() == ObjectType::Circle) {
                    opJson["t"] = op->getAngleOnCircle();
                } else {
                    opJson["t"] = op->getRelativePositionOnLine();
                }
                if (auto host = op->getHostObject()) {
                    opJson["hostId"] = host->getID();
                }
                opJson["color"] = colorToHex(op->getColor());
                objectPointsArray.push_back(opJson);
            }
        }
        project["objects"]["objectPoints"] = objectPointsArray;

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
// LOAD PROJECT
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

        // 1. Clear existing scene
        editor.clearScene();

        // =========================================================
        // STEP 1: DETECT STRUCTURE (Hybrid Fix)
        // =========================================================
        // Points 'data' to either the root (Flat) or 'objects' (Nested)
        const json& data = j.contains("objects") ? j["objects"] : j;
        
        std::cout << "Loading Mode: " << (j.contains("objects") ? "Nested (Old)" : "Flat (New)") << std::endl;

        // Maps for reconstructing relationships (ID -> Object)
        std::unordered_map<int, std::shared_ptr<Point>> pointMap;
        std::unordered_map<int, std::shared_ptr<GeometricObject>> objectMap;
        int maxId = 0;

        // =========================================================
        // PASS 1: LOAD POINTS
        // =========================================================
        if (data.contains("points")) {
            for (const auto& jPoint : data["points"]) {
                double x = jPoint.value("x", 0.0);
                double y = jPoint.value("y", 0.0);
                int id = jPoint.value("id", -1);
                
                sf::Color color = sf::Color::Black;
                if (jPoint.contains("color")) {
                    if (jPoint["color"].is_string()) {
                         // color = hexToColor(jPoint["color"]); // Use if you have hex strings
                    } else {
                        auto c = jPoint["color"];
                        color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                    }
                }

                auto newPoint = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
                editor.points.push_back(newPoint);

                if (id != -1) {
                    pointMap[id] = newPoint;
                    objectMap[id] = newPoint;
                    if (id > maxId) maxId = id;
                }
            }
        }

        // =========================================================
        // PASS 2: LOAD LINES
        // =========================================================
        if (data.contains("lines")) {
            for (const auto& jLine : data["lines"]) {
                int startId = jLine.value("startPointID", -1);
                if (startId == -1) startId = jLine.value("startId", -1); // Fallback
                
                int endId = jLine.value("endPointID", -1);
                if (endId == -1) endId = jLine.value("endId", -1); // Fallback

                int id = jLine.value("id", -1);
                bool isSegment = jLine.value("isSegment", false);
                
                sf::Color color = sf::Color::Black;
                if (jLine.contains("color") && !jLine["color"].is_string()) {
                     auto c = jLine["color"];
                     color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                }

                if (pointMap.count(startId) && pointMap.count(endId)) {
                    auto newLine = std::make_shared<Line>(
                        pointMap[startId], 
                        pointMap[endId], 
                        isSegment, 
                        color, 
                        id
                    );
                    editor.lines.push_back(newLine);
                    newLine->registerWithEndpoints();

                    if (id != -1) {
                        objectMap[id] = newLine;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 3: LOAD CIRCLES
        // =========================================================
        if (data.contains("circles")) {
            for (const auto& jCircle : data["circles"]) {
                int centerId = jCircle.value("centerPointID", -1);
                int radiusPtId = jCircle.value("radiusPointID", -1);
                double radius = jCircle.value("radius", 10.0);
                int id = jCircle.value("id", -1);
                
                sf::Color color = sf::Color::Black; 
                if (jCircle.contains("color") && !jCircle["color"].is_string()) {
                     auto c = jCircle["color"];
                     color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                }

                if (pointMap.count(centerId)) {
                    std::shared_ptr<Point> radiusPt = nullptr;
                    if (radiusPtId != -1 && pointMap.count(radiusPtId)) {
                        radiusPt = pointMap[radiusPtId];
                    }

                    auto newCircle = std::make_shared<Circle>(
                        pointMap[centerId].get(), 
                        radiusPt, 
                        radius, 
                        color
                    );
                    // Manually set ID if constructor doesn't take it
                    // newCircle->setID(id); 

                    editor.circles.push_back(newCircle);
                    if (id != -1) {
                        objectMap[id] = newCircle;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 4: LOAD POLYGONS
        // =========================================================
        if (data.contains("polygons")) {
            for (const auto& polyJson : data["polygons"]) {
                std::vector<Point_2> vertices;
                if (polyJson.contains("vertices")) {
                    for (const auto& vJson : polyJson["vertices"]) {
                        // Handle both [x, y] array and {"x":..., "y":...} object
                        if (vJson.is_array() && vJson.size() >= 2) {
                            vertices.emplace_back(vJson[0].get<double>(), vJson[1].get<double>());
                        } else if (vJson.is_object()) {
                            vertices.emplace_back(vJson.value("x", 0.0), vJson.value("y", 0.0));
                        }
                    }
                }
                
                sf::Color color = sf::Color::Black;
                if (polyJson.contains("color") && !polyJson["color"].is_string()) {
                     auto c = polyJson["color"];
                     color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                }
                
                int id = polyJson.value("id", -1);

                auto polygon = std::make_shared<Polygon>(vertices, color, id);
                editor.polygons.push_back(polygon);
                
                if (id != -1) {
                    objectMap[id] = polygon;
                    if (id > maxId) maxId = id;
                }
            }
        }

        // =========================================================
        // PASS 5: LOAD RECTANGLES
        // =========================================================
        if (data.contains("rectangles")) {
            for (const auto& rectJson : data["rectangles"]) {
                auto verticesJson = rectJson["vertices"];
                // Need at least 2 points (corners) or 4 points (all verts)
                if (verticesJson.size() >= 2) {
                    double x1, y1, x2, y2;
                    
                    // Logic to extract corners based on storage format
                    if (verticesJson[0].is_array()) {
                        x1 = verticesJson[0][0].get<double>();
                        y1 = verticesJson[0][1].get<double>();
                        // If it stores 4 vertices, index 2 is opposite corner
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
                    if (rectJson.contains("color") && !rectJson["color"].is_string()) {
                        auto c = rectJson["color"];
                        color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                    }
                    
                    int id = rectJson.value("id", -1);
                    Point_2 corner1(x1, y1);
                    Point_2 corner2(x2, y2);
                    
                    auto rect = std::make_shared<Rectangle>(corner1, corner2, false, color, id);
                    editor.rectangles.push_back(rect);

                    if (id != -1) {
                        objectMap[id] = rect;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 6: LOAD TRIANGLES (Fixed)
        // =========================================================
        if (data.contains("triangles")) {
            for (const auto& triJson : data["triangles"]) {
                auto verticesJson = triJson["vertices"];
                if (verticesJson.size() >= 3) {
                    std::vector<Point_2> pts;
                    for(int i=0; i<3; ++i) {
                        if (verticesJson[i].is_array()) {
                            // ERROR WAS HERE: Added .get<double>() to force conversion
                            double x = verticesJson[i][0].get<double>();
                            double y = verticesJson[i][1].get<double>();
                            pts.emplace_back(x, y);
                        }
                        else {
                            double x = verticesJson[i].value("x", 0.0);
                            double y = verticesJson[i].value("y", 0.0);
                            pts.emplace_back(x, y);
                        }
                    }

                    sf::Color color = sf::Color::Black;
                    if (triJson.contains("color") && !triJson["color"].is_string()) {
                        auto c = triJson["color"];
                        color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                    }
                    
                    int id = triJson.value("id", -1);
                    
                    auto triangle = std::make_shared<Triangle>(pts[0], pts[1], pts[2], color, id);
                    editor.triangles.push_back(triangle);
                    
                    if (id != -1) {
                        objectMap[id] = triangle;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }
        // =========================================================
        // PASS 7: LOAD REGULAR POLYGONS
        // =========================================================
        if (data.contains("regularPolygons")) {
            for (const auto& rpolyJson : data["regularPolygons"]) {
                auto verticesJson = rpolyJson["vertices"];
                int sides = rpolyJson.value("sides", 3);
                int id = rpolyJson.value("id", -1);
                
                sf::Color color = sf::Color::Black;
                if (rpolyJson.contains("color") && !rpolyJson["color"].is_string()) {
                    auto c = rpolyJson["color"];
                    color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                }

                if (!verticesJson.empty()) {
                    // Reconstruct Center and FirstVertex
                    double cx = 0, cy = 0;
                    double fx=0, fy=0;

                    // Parse first vertex
                   if (verticesJson[0].is_array()) { 
    // Force .get<double>() here too to be safe
    fx = verticesJson[0][0].get<double>(); 
    fy = verticesJson[0][1].get<double>(); 
}

                    // Calculate centroid
                    for (const auto& v : verticesJson) {
                         if (v.is_array()) { cx += v[0].get<double>(); cy += v[1].get<double>(); }
                         else { cx += v.value("x",0.0); cy += v.value("y",0.0); }
                    }
                    cx /= verticesJson.size();
                    cy /= verticesJson.size();

                    Point_2 center(cx, cy);
                    Point_2 firstVertex(fx, fy);
                    
                    auto rpoly = std::make_shared<RegularPolygon>(center, firstVertex, sides, color, id);
                    editor.regularPolygons.push_back(rpoly);

                    if (id != -1) {
                        objectMap[id] = rpoly;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 8: RESTORE LINE CONSTRAINTS
        // =========================================================
        if (data.contains("lines")) {
            for (const auto& jLine : data["lines"]) {
                int id = jLine.value("id", -1);
                
                if (objectMap.count(id) && objectMap[id]->getType() == ObjectType::Line) {
                    auto linePtr = std::dynamic_pointer_cast<Line>(objectMap[id]);
                    
                    bool isParallel = jLine.value("isParallel", false);
                    bool isPerp = jLine.value("isPerpendicular", false);

                    if ((isParallel || isPerp) && jLine.contains("constraintRefId")) {
                        int refId = jLine["constraintRefId"];
                        int edgeIndex = jLine.value("constraintRefEdgeIndex", -1);

                        if (objectMap.count(refId)) {
                            Point_2 s = linePtr->getStartPoint();
                            Point_2 e = linePtr->getEndPoint();
                            Vector_2 dir(e.x() - s.x(), e.y() - s.y());

                            if (isParallel) linePtr->setAsParallelLine(objectMap[refId], edgeIndex, dir);
                            else linePtr->setAsPerpendicularLine(objectMap[refId], edgeIndex, dir);
                        }
                    }
                }
            }
        }

        // Finalize state
        editor.objectIdCounter = maxId + 1;
        std::cout << "Project loaded. Counter: " << editor.objectIdCounter << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Load Error: " << e.what() << std::endl;
        return false;
    }
}

// ============================================================================
// EXPORT SVG
// ============================================================================

bool ProjectSerializer::exportSVG(const GeometryEditor& editor, const std::string& filepath) {
    try {
        sf::Vector2f viewCenter = editor.drawingView.getCenter();
        sf::Vector2f viewSize = editor.drawingView.getSize();

        float left = viewCenter.x - viewSize.x / 2.0f;
        float top = viewCenter.y - viewSize.y / 2.0f;
        float right = left + viewSize.x;
        float bottom = top + viewSize.y;

        std::ofstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "ProjectSerializer::exportSVG: Failed to open file: " << filepath << std::endl;
            return false;
        }

        file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        file << "<svg xmlns=\"http://www.w3.org/2000/svg\" ";
        file << "viewBox=\"" << left << " " << top << " " << viewSize.x << " " << viewSize.y << "\" ";
        file << "width=\"" << viewSize.x << "\" height=\"" << viewSize.y << "\">\n";

        file << "  <rect x=\"" << left << "\" y=\"" << top << "\" ";
        file << "width=\"" << viewSize.x << "\" height=\"" << viewSize.y << "\" fill=\"white\"/>\n";

        float gridSize = Constants::GRID_SIZE;
        for (float x = std::floor(left / gridSize) * gridSize; x <= right; x += gridSize) {
            file << "  <line x1=\"" << x << "\" y1=\"" << top << "\" x2=\"" << x
                 << "\" y2=\"" << bottom << "\" stroke=\"#e0e0e0\" stroke-width=\"1\" />\n";
        }
        for (float y = std::floor(top / gridSize) * gridSize; y <= bottom; y += gridSize) {
            file << "  <line x1=\"" << left << "\" y1=\"" << y << "\" x2=\"" << right
                 << "\" y2=\"" << y << "\" stroke=\"#e0e0e0\" stroke-width=\"1\" />\n";
        }

        auto clipLineToRect = [&](const Point_2& p1, const Point_2& p2,
                                  const sf::FloatRect& rect,
                                  sf::Vector2f& outA, sf::Vector2f& outB) -> bool {
            sf::Vector2f a(static_cast<float>(CGAL::to_double(p1.x())),
                           static_cast<float>(CGAL::to_double(p1.y())));
            sf::Vector2f b(static_cast<float>(CGAL::to_double(p2.x())),
                           static_cast<float>(CGAL::to_double(p2.y())));

            sf::Vector2f r(b.x - a.x, b.y - a.y);
            float eps = 1e-6f;

            auto cross = [](const sf::Vector2f& v1, const sf::Vector2f& v2) {
                return v1.x * v2.y - v1.y * v2.x;
            };

            struct Hit {
                float t;
                sf::Vector2f p;
            };

            std::vector<Hit> hits;

            auto addHit = [&](float t, const sf::Vector2f& p) {
                for (const auto& h : hits) {
                    float dx = h.p.x - p.x;
                    float dy = h.p.y - p.y;
                    if (dx * dx + dy * dy < eps * eps) {
                        return;
                    }
                }
                hits.push_back({t, p});
            };

            sf::Vector2f rectTL(rect.left, rect.top);
            sf::Vector2f rectTR(rect.left + rect.width, rect.top);
            sf::Vector2f rectBL(rect.left, rect.top + rect.height);
            sf::Vector2f rectBR(rect.left + rect.width, rect.top + rect.height);

            std::pair<sf::Vector2f, sf::Vector2f> edges[4] = {
                {rectTL, rectTR},
                {rectTR, rectBR},
                {rectBR, rectBL},
                {rectBL, rectTL}
            };

            for (auto& edge : edges) {
                sf::Vector2f q = edge.first;
                sf::Vector2f s(edge.second.x - edge.first.x, edge.second.y - edge.first.y);
                float denom = cross(r, s);
                if (std::abs(denom) < eps) {
                    continue;
                }
                sf::Vector2f qp(q.x - a.x, q.y - a.y);
                float t = cross(qp, s) / denom;
                float u = cross(qp, r) / denom;
                if (u >= -eps && u <= 1.0f + eps) {
                    sf::Vector2f p(a.x + t * r.x, a.y + t * r.y);
                    addHit(t, p);
                }
            }

            if (hits.size() < 2) {
                return false;
            }

            std::sort(hits.begin(), hits.end(), [](const Hit& h1, const Hit& h2) {
                return h1.t < h2.t;
            });

            outA = hits.front().p;
            outB = hits.back().p;
            return true;
        };

        for (const auto& rect : editor.rectangles) {
            if (rect && rect->isValid() && rect->isVisible()) {
                auto vertices = rect->getInteractableVertices();
                if (vertices.size() >= 4) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < vertices.size(); ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = CGAL::to_double(vertices[i].y());
                        file << x << "," << y;
                        if (i < vertices.size() - 1) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(rect->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }

        for (const auto& poly : editor.polygons) {
            if (poly && poly->isValid() && poly->isVisible()) {
                auto vertices = poly->getInteractableVertices();
                if (!vertices.empty()) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < vertices.size(); ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = CGAL::to_double(vertices[i].y());
                        file << x << "," << y;
                        if (i < vertices.size() - 1) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(poly->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }

        for (const auto& tri : editor.triangles) {
            if (tri && tri->isValid() && tri->isVisible()) {
                auto vertices = tri->getInteractableVertices();
                if (vertices.size() >= 3) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < 3; ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = CGAL::to_double(vertices[i].y());
                        file << x << "," << y;
                        if (i < 2) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(tri->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }

        for (const auto& rpoly : editor.regularPolygons) {
            if (rpoly && rpoly->isValid() && rpoly->isVisible()) {
                auto vertices = rpoly->getInteractableVertices();
                if (!vertices.empty()) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < vertices.size(); ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = CGAL::to_double(vertices[i].y());
                        file << x << "," << y;
                        if (i < vertices.size() - 1) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(rpoly->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }

        for (const auto& ci : editor.circles) {
            if (ci && ci->isValid() && ci->isVisible()) {
                Point_2 center = ci->getCGALPosition();
                double cx = CGAL::to_double(center.x());
                double cy = CGAL::to_double(center.y());
                double r = ci->getRadius();

                file << "  <circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"" << r;
                file << "\" fill=\"none\" stroke=\"" << colorToHex(ci->getColor());
                file << "\" stroke-width=\"2\"/>\n";
            }
        }

        sf::FloatRect viewRect(left, top, viewSize.x, viewSize.y);
        for (const auto& ln : editor.lines) {
            if (ln && ln->isValid() && ln->isVisible()) {
                if (ln->isSegment()) {
                    Point_2 start = ln->getStartPoint();
                    Point_2 end = ln->getEndPoint();

                    double x1 = CGAL::to_double(start.x());
                    double y1 = CGAL::to_double(start.y());
                    double x2 = CGAL::to_double(end.x());
                    double y2 = CGAL::to_double(end.y());

                    file << "  <line x1=\"" << x1 << "\" y1=\"" << y1;
                    file << "\" x2=\"" << x2 << "\" y2=\"" << y2;
                    file << "\" stroke=\"" << colorToHex(ln->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                } else {
                    sf::Vector2f a, b;
                    Point_2 p1 = ln->getStartPoint();
                    Point_2 p2 = ln->getEndPoint();
                    if (clipLineToRect(p1, p2, viewRect, a, b)) {
                        file << "  <line x1=\"" << a.x << "\" y1=\"" << a.y;
                        file << "\" x2=\"" << b.x << "\" y2=\"" << b.y;
                        file << "\" stroke=\"" << colorToHex(ln->getColor());
                        file << "\" stroke-width=\"2\"/>\n";
                    }
                }
            }
        }

        for (const auto& pt : editor.points) {
            if (pt && pt->isValid() && pt->isVisible()) {
                Point_2 pos = pt->getCGALPosition();
                double x = CGAL::to_double(pos.x());
                double y = CGAL::to_double(pos.y());

                file << "  <circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"4";
                file << "\" fill=\"" << colorToHex(pt->getColor()) << "\"/>\n";
            }
        }

        for (const auto& op : editor.ObjectPoints) {
            if (op && op->isValid() && op->isVisible()) {
                Point_2 pos = op->getCGALPosition();
                double x = CGAL::to_double(pos.x());
                double y = CGAL::to_double(pos.y());

                file << "  <circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"4";
                file << "\" fill=\"" << colorToHex(op->getColor()) << "\"/>\n";
            }
        }

        file << "</svg>\n";
        file.close();

        std::cout << "SVG exported successfully to: " << filepath << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "ProjectSerializer::exportSVG: Exception: " << e.what() << std::endl;
        return false;
    }
}
