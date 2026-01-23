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
                
                // Get start/end points
                Point_2 start = ln->getStartPoint();
                Point_2 end = ln->getEndPoint();
                lnJson["startX"] = CGAL::to_double(start.x());
                lnJson["startY"] = CGAL::to_double(start.y());
                lnJson["endX"] = CGAL::to_double(end.x());
                lnJson["endY"] = CGAL::to_double(end.y());
                
                lnJson["isSegment"] = ln->isSegment();
                lnJson["color"] = colorToHex(ln->getColor());
                
                // Save constraint info
                lnJson["isParallel"] = ln->isParallelLine();
                lnJson["isPerpendicular"] = ln->isPerpendicularLine();
                
                // Save constraint reference if exists
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
                Point_2 center = ci->getCGALPosition();
                ciJson["centerX"] = CGAL::to_double(center.x());
                ciJson["centerY"] = CGAL::to_double(center.y());
                ciJson["radius"] = ci->getRadius();
                ciJson["color"] = colorToHex(ci->getColor());
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
        
        // Save ObjectPoints
        json objectPointsArray = json::array();
        for (const auto& op : editor.ObjectPoints) {
            if (op && op->isValid()) {
                json opJson;
                opJson["id"] = op->getID();
                // Serialize t parameter based on host type
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
        
        file << project.dump(2); // Pretty print with 2-space indent
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
    try {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "ProjectSerializer::loadProject: Failed to open file: " << filepath << std::endl;
            return false;
        }
        
        json project;
        file >> project;
        file.close();
        
        // Clear existing scene
        editor.clearScene();
        
        // ID mapping for dependency resolution
        std::map<unsigned int, std::shared_ptr<GeometricObject>> idToObject;
        
        // ========== PASS 1: Create all objects ==========
        
        // Load Points
        if (project["objects"].contains("points")) {
            for (const auto& ptJson : project["objects"]["points"]) {
                double x = ptJson["x"].get<double>();
                double y = ptJson["y"].get<double>();
                sf::Color color = hexToColor(ptJson["color"].get<std::string>());
                
                Point_2 pos(x, y);
                // Point constructor: Point(Point_2, float zoomFactor, sf::Color fillColor, sf::Color outlineColor)
                auto point = std::make_shared<Point>(pos, 1.0f, color, color);
                
                // Store with original ID for reference
                unsigned int originalId = ptJson["id"].get<unsigned int>();
                idToObject[originalId] = point;
                
                editor.points.push_back(point);
            }
        }
        
        // Load Lines (without constraints first)
        if (project["objects"].contains("lines")) {
            for (const auto& lnJson : project["objects"]["lines"]) {
                double startX = lnJson["startX"].get<double>();
                double startY = lnJson["startY"].get<double>();
                double endX = lnJson["endX"].get<double>();
                double endY = lnJson["endY"].get<double>();
                bool isSegment = lnJson["isSegment"].get<bool>();
                sf::Color color = hexToColor(lnJson["color"].get<std::string>());
                
                Point_2 start(startX, startY);
                Point_2 end(endX, endY);
                
                auto line = std::make_shared<Line>(start, end, isSegment, color);
                
                unsigned int originalId = lnJson["id"].get<unsigned int>();
                idToObject[originalId] = line;
                
                editor.lines.push_back(line);
            }
        }
        
        // Load Circles - Circle requires a Point* for center, so we create an internal Point first
        if (project["objects"].contains("circles")) {
            for (const auto& ciJson : project["objects"]["circles"]) {
                double cx = ciJson["centerX"].get<double>();
                double cy = ciJson["centerY"].get<double>();
                double radius = ciJson["radius"].get<double>();
                sf::Color color = hexToColor(ciJson["color"].get<std::string>());
                
                // Create a center point first
                Point_2 centerPos(cx, cy);
                auto centerPoint = std::make_shared<Point>(centerPos, 1.0f, color, color);
                editor.points.push_back(centerPoint);
                
                // Create circle with raw pointer to the center point
                auto circle = Circle::create(centerPoint.get(), radius, color);
                
                unsigned int originalId = ciJson["id"].get<unsigned int>();
                idToObject[originalId] = circle;
                
                editor.circles.push_back(circle);
            }
        }
        
        // Load Rectangles
        if (project["objects"].contains("rectangles")) {
            for (const auto& rectJson : project["objects"]["rectangles"]) {
                auto verticesJson = rectJson["vertices"];
                if (verticesJson.size() >= 2) {
                    double x1 = verticesJson[0][0].get<double>();
                    double y1 = verticesJson[0][1].get<double>();
                    double x2 = verticesJson[2][0].get<double>(); // Opposite corner
                    double y2 = verticesJson[2][1].get<double>();
                    sf::Color color = hexToColor(rectJson["color"].get<std::string>());
                    
                    Point_2 corner1(x1, y1);
                    Point_2 corner2(x2, y2);
                    auto rect = std::make_shared<Rectangle>(corner1, corner2, false, color);
                    
                    unsigned int originalId = rectJson["id"].get<unsigned int>();
                    idToObject[originalId] = rect;
                    
                    editor.rectangles.push_back(rect);
                }
            }
        }
        
        // Load Polygons
        if (project["objects"].contains("polygons")) {
            for (const auto& polyJson : project["objects"]["polygons"]) {
                std::vector<Point_2> vertices;
                for (const auto& vJson : polyJson["vertices"]) {
                    vertices.emplace_back(vJson[0].get<double>(), vJson[1].get<double>());
                }
                sf::Color color = hexToColor(polyJson["color"].get<std::string>());
                
                auto polygon = std::make_shared<Polygon>(vertices, color);
                
                unsigned int originalId = polyJson["id"].get<unsigned int>();
                idToObject[originalId] = polygon;
                
                editor.polygons.push_back(polygon);
            }
        }
        
        // Load Triangles
        if (project["objects"].contains("triangles")) {
            for (const auto& triJson : project["objects"]["triangles"]) {
                auto verticesJson = triJson["vertices"];
                if (verticesJson.size() >= 3) {
                    Point_2 p1(verticesJson[0][0].get<double>(), verticesJson[0][1].get<double>());
                    Point_2 p2(verticesJson[1][0].get<double>(), verticesJson[1][1].get<double>());
                    Point_2 p3(verticesJson[2][0].get<double>(), verticesJson[2][1].get<double>());
                    sf::Color color = hexToColor(triJson["color"].get<std::string>());
                    
                    auto triangle = std::make_shared<Triangle>(p1, p2, p3, color);
                    
                    unsigned int originalId = triJson["id"].get<unsigned int>();
                    idToObject[originalId] = triangle;
                    
                    editor.triangles.push_back(triangle);
                }
            }
        }
        
        // Load Regular Polygons
        if (project["objects"].contains("regularPolygons")) {
            for (const auto& rpolyJson : project["objects"]["regularPolygons"]) {
                auto verticesJson = rpolyJson["vertices"];
                int sides = rpolyJson["sides"].get<int>();
                sf::Color color = hexToColor(rpolyJson["color"].get<std::string>());
                
                // Calculate center from vertices
                if (!verticesJson.empty()) {
                    double cx = 0, cy = 0;
                    for (const auto& v : verticesJson) {
                        cx += v[0].get<double>();
                        cy += v[1].get<double>();
                    }
                    cx /= verticesJson.size();
                    cy /= verticesJson.size();
                    
                    // Get the first vertex for the constructor
                    double firstX = verticesJson[0][0].get<double>();
                    double firstY = verticesJson[0][1].get<double>();
                    
                    Point_2 center(cx, cy);
                    Point_2 firstVertex(firstX, firstY);
                    
                    // RegularPolygon constructor: (center, firstVertex, numSides, color)
                    auto rpoly = std::make_shared<RegularPolygon>(center, firstVertex, sides, color);
                    
                    unsigned int originalId = rpolyJson["id"].get<unsigned int>();
                    idToObject[originalId] = rpoly;
                    
                    editor.regularPolygons.push_back(rpoly);
                }
            }
        }
        
        // ========== PASS 2: Re-link dependencies ==========
        
        // Re-link Line constraints
        if (project["objects"].contains("lines")) {
            size_t lineIndex = 0;
            for (const auto& lnJson : project["objects"]["lines"]) {
                if (lineIndex < editor.lines.size()) {
                    auto& line = editor.lines[lineIndex];
                    
                    bool isParallel = lnJson.value("isParallel", false);
                    bool isPerpendicular = lnJson.value("isPerpendicular", false);
                    
                    if ((isParallel || isPerpendicular) && lnJson.contains("constraintRefId")) {
                        unsigned int refId = lnJson["constraintRefId"].get<unsigned int>();
                        int edgeIndex = lnJson.value("constraintRefEdgeIndex", -1);
                        
                        auto it = idToObject.find(refId);
                        if (it != idToObject.end()) {
                            // Get current line direction for constraint
                            Point_2 start = line->getStartPoint();
                            Point_2 end = line->getEndPoint();
                            Vector_2 dir(end.x() - start.x(), end.y() - start.y());
                            
                            if (isParallel) {
                                line->setAsParallelLine(it->second, edgeIndex, dir);
                            } else if (isPerpendicular) {
                                line->setAsPerpendicularLine(it->second, edgeIndex, dir);
                            }
                        }
                    }
                }
                lineIndex++;
            }
        }
        
        // ObjectPoints would need host re-linking here
        // For now, skip ObjectPoints during load (they're dependent objects)
        
        std::cout << "Project loaded successfully from: " << filepath << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "ProjectSerializer::loadProject: Exception: " << e.what() << std::endl;
        return false;
    }
}

// ============================================================================
// EXPORT SVG
// ============================================================================

bool ProjectSerializer::exportSVG(const GeometryEditor& editor, const std::string& filepath) {
    try {
        // Calculate bounds
        double minX, minY, maxX, maxY;
        calculateBounds(editor, minX, minY, maxX, maxY);
        
        double width = maxX - minX;
        double height = maxY - minY;
        
        std::ofstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "ProjectSerializer::exportSVG: Failed to open file: " << filepath << std::endl;
            return false;
        }
        
        // SVG Header
        file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        file << "<svg xmlns=\"http://www.w3.org/2000/svg\" ";
        file << "viewBox=\"" << minX << " " << -maxY << " " << width << " " << height << "\" ";
        file << "width=\"" << width << "\" height=\"" << height << "\">\n";
        
        // Background
        file << "  <rect x=\"" << minX << "\" y=\"" << -maxY << "\" ";
        file << "width=\"" << width << "\" height=\"" << height << "\" fill=\"white\"/>\n";
        
        // Draw Rectangles
        for (const auto& rect : editor.rectangles) {
            if (rect && rect->isValid()) {
                auto vertices = rect->getInteractableVertices();
                if (vertices.size() >= 4) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < vertices.size(); ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = -CGAL::to_double(vertices[i].y()); // Flip Y for SVG
                        file << x << "," << y;
                        if (i < vertices.size() - 1) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(rect->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }
        
        // Draw Polygons
        for (const auto& poly : editor.polygons) {
            if (poly && poly->isValid()) {
                auto vertices = poly->getInteractableVertices();
                if (!vertices.empty()) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < vertices.size(); ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = -CGAL::to_double(vertices[i].y());
                        file << x << "," << y;
                        if (i < vertices.size() - 1) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(poly->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }
        
        // Draw Triangles
        for (const auto& tri : editor.triangles) {
            if (tri && tri->isValid()) {
                auto vertices = tri->getInteractableVertices();
                if (vertices.size() >= 3) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < 3; ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = -CGAL::to_double(vertices[i].y());
                        file << x << "," << y;
                        if (i < 2) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(tri->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }
        
        // Draw Regular Polygons
        for (const auto& rpoly : editor.regularPolygons) {
            if (rpoly && rpoly->isValid()) {
                auto vertices = rpoly->getInteractableVertices();
                if (!vertices.empty()) {
                    file << "  <polygon points=\"";
                    for (size_t i = 0; i < vertices.size(); ++i) {
                        double x = CGAL::to_double(vertices[i].x());
                        double y = -CGAL::to_double(vertices[i].y());
                        file << x << "," << y;
                        if (i < vertices.size() - 1) file << " ";
                    }
                    file << "\" fill=\"none\" stroke=\"" << colorToHex(rpoly->getColor());
                    file << "\" stroke-width=\"2\"/>\n";
                }
            }
        }
        
        // Draw Circles
        for (const auto& ci : editor.circles) {
            if (ci && ci->isValid()) {
                Point_2 center = ci->getCGALPosition();
                double cx = CGAL::to_double(center.x());
                double cy = -CGAL::to_double(center.y()); // Flip Y
                double r = ci->getRadius();
                
                file << "  <circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"" << r;
                file << "\" fill=\"none\" stroke=\"" << colorToHex(ci->getColor());
                file << "\" stroke-width=\"2\"/>\n";
            }
        }
        
        // Draw Lines
        for (const auto& ln : editor.lines) {
            if (ln && ln->isValid()) {
                Point_2 start = ln->getStartPoint();
                Point_2 end = ln->getEndPoint();
                
                double x1 = CGAL::to_double(start.x());
                double y1 = -CGAL::to_double(start.y()); // Flip Y
                double x2 = CGAL::to_double(end.x());
                double y2 = -CGAL::to_double(end.y());
                
                file << "  <line x1=\"" << x1 << "\" y1=\"" << y1;
                file << "\" x2=\"" << x2 << "\" y2=\"" << y2;
                file << "\" stroke=\"" << colorToHex(ln->getColor());
                file << "\" stroke-width=\"2\"/>\n";
            }
        }
        
        // Draw Points
        for (const auto& pt : editor.points) {
            if (pt && pt->isValid()) {
                Point_2 pos = pt->getCGALPosition();
                double x = CGAL::to_double(pos.x());
                double y = -CGAL::to_double(pos.y()); // Flip Y
                
                file << "  <circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"4";
                file << "\" fill=\"" << colorToHex(pt->getColor()) << "\"/>\n";
            }
        }
        
        // SVG Footer
        file << "</svg>\n";
        file.close();
        
        std::cout << "SVG exported successfully to: " << filepath << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "ProjectSerializer::exportSVG: Exception: " << e.what() << std::endl;
        return false;
    }
}
