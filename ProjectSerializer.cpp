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
#include "Deserializer.h"
#include "GeometryEditor.h"
#include "Point.h"
#include "Line.h"  
#include "Circle.h"
#include "Rectangle.h"
#include "Polygon.h"
#include "RegularPolygon.h"
#include "Triangle.h"
#include "Angle.h"
#include "ConstructionObjects.h"
#include "TransformationObjects.h"
#include "ObjectPoint.h"
#include "Constants.h"
#include "Intersection.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "SVGWriter.h"
#include <cmath>
#include <algorithm>
#include <limits>

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
                sf::Vector2f offset = pt->getLabelOffset();
                ptJson["labelOffset"] = { offset.x, offset.y };

                // --- Transformation Metadata ---
                ptJson["transformType"] = static_cast<int>(pt->getTransformType());
                if (pt->getTransformType() != TransformationType::None) {
                    ptJson["parentSourceID"] = pt->getParentSourceID();
                    ptJson["auxObjectID"] = pt->getAuxObjectID();
                }

                // Transformation-derived points
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

                // --- Transformation Metadata ---
                lnJson["transformType"] = static_cast<int>(ln->getTransformType());
                if (ln->getTransformType() != TransformationType::None) {
                    lnJson["parentSourceID"] = ln->getParentSourceID();
                    lnJson["auxObjectID"] = ln->getAuxObjectID();
                    lnJson["transformValue"] = ln->getTransformValue();
                }

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

                // --- Transformation Metadata ---
                ciJson["transformType"] = static_cast<int>(ci->getTransformType());
                if (ci->getTransformType() != TransformationType::None) {
                    ciJson["parentSourceID"] = ci->getParentSourceID();
                    ciJson["auxObjectID"] = ci->getAuxObjectID();
                    ciJson["transformValue"] = ci->getTransformValue();
                }

                ciJson["isSemicircle"] = ci->isSemicircle();
                if (ci->isSemicircle()) {
                    if (auto p1 = ci->getDiameterP1()) ciJson["diameterP1ID"] = p1->getID();
                    if (auto p2 = ci->getDiameterP2()) ciJson["diameterP2ID"] = p2->getID();
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

                // --- Transformation Metadata ---
                rectJson["transformType"] = static_cast<int>(rect->getTransformType());
                if (rect->getTransformType() != TransformationType::None) {
                    rectJson["parentSourceID"] = rect->getParentSourceID();
                    rectJson["auxObjectID"] = rect->getAuxObjectID();
                    rectJson["transformValue"] = rect->getTransformValue();
                }

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
                
                // --- Transformation Metadata ---
                polyJson["transformType"] = static_cast<int>(poly->getTransformType());
                if (poly->getTransformType() != TransformationType::None) {
                    polyJson["parentSourceID"] = poly->getParentSourceID();
                    polyJson["auxObjectID"] = poly->getAuxObjectID();
                    polyJson["transformValue"] = poly->getTransformValue();
                }

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
                
                // --- Transformation Metadata ---
                triJson["transformType"] = static_cast<int>(tri->getTransformType());
                if (tri->getTransformType() != TransformationType::None) {
                    triJson["parentSourceID"] = tri->getParentSourceID();
                    triJson["auxObjectID"] = tri->getAuxObjectID();
                    triJson["transformValue"] = tri->getTransformValue();
                }

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
                
                // --- Transformation Metadata ---
                rpolyJson["transformType"] = static_cast<int>(rpoly->getTransformType());
                if (rpoly->getTransformType() != TransformationType::None) {
                    rpolyJson["parentSourceID"] = rpoly->getParentSourceID();
                    rpolyJson["auxObjectID"] = rpoly->getAuxObjectID();
                    rpolyJson["transformValue"] = rpoly->getTransformValue();
                }

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
                opJson["labelOffset"] = { offset.x, offset.y };
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
            aJson["labelOffset"] = { aOffset.x, aOffset.y };
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
                    pJson["labelOffset"] = { offset.x, offset.y };
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
    // Delegate to the robust Deserializer implementation
    return Deserializer::loadProject(editor, filepath);
}

// ============================================================================
// LEGACY LOAD PROJECT - Kept for reference, but NOT USED
// ============================================================================
#if 0  // DISABLED - Using Deserializer::loadProject instead

bool ProjectSerializer::loadProject_LEGACY(GeometryEditor& editor, const std::string& filepath) {
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
        DynamicIntersection::clearAllIntersectionConstraints(editor);

        // =========================================================
        // STEP 1: DETECT STRUCTURE (Hybrid Fix)
        // =========================================================
        // Points 'data' to either the root (Flat) or 'objects' (Nested)
        const json& data = j.contains("objects") ? j["objects"] : j;
        
        std::cout << "Loading Mode: " << (j.contains("objects") ? "Nested (Old)" : "Flat (New)") << std::endl;

        bool axesVisible = true;
        if (j.contains("settings") && j["settings"].contains("axesVisible")) {
            axesVisible = j["settings"]["axesVisible"].get<bool>();
        }

        // Maps for reconstructing relationships (ID -> Object)
        std::unordered_map<int, std::shared_ptr<Point>> pointMap;
        std::unordered_map<int, std::shared_ptr<GeometricObject>> objectMap;
        int maxId = 0;
        std::vector<json> deferredTransformPoints;

        // =========================================================
        // PASS 1: LOAD POINTS
        // =========================================================
        if (data.contains("points")) {
            for (const auto& jPoint : data["points"]) {
                if (jPoint.contains("transform")) {
                    deferredTransformPoints.push_back(jPoint);
                    continue;
                }
                double x = jPoint.value("x", 0.0);
                double y = jPoint.value("y", 0.0);
                int id = jPoint.value("id", -1);
                
                sf::Color color = sf::Color::Black;
                if (jPoint.contains("color")) {
                    if (jPoint["color"].is_string()) {
                         color = hexToColor(jPoint["color"]);
                    } else {
                        auto c = jPoint["color"];
                        color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                    }
                }

                auto newPoint = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM, color, id);
                if (jPoint.contains("label")) newPoint->setLabel(jPoint.value("label", ""));
                if (jPoint.contains("showLabel")) newPoint->setShowLabel(jPoint.value("showLabel", true));
                if (jPoint.contains("labelOffset") && jPoint["labelOffset"].is_array() && jPoint["labelOffset"].size() >= 2) {
                    newPoint->setLabelOffset(sf::Vector2f(jPoint["labelOffset"][0], jPoint["labelOffset"][1]));
                }

                // Read Transformation Metadata
                if (jPoint.contains("transformType")) {
                    newPoint->setTransformType(static_cast<TransformationType>(jPoint["transformType"].get<int>()));
                    newPoint->setParentSourceID(jPoint.value("parentSourceID", 0u));
                    newPoint->setAuxObjectID(jPoint.value("auxObjectID", 0u));
                }

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
                if (jLine.contains("color")) {
                    if (jLine["color"].is_string()) {
                        color = hexToColor(jLine["color"]);
                    } else {
                        auto c = jLine["color"];
                        color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                    }
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
                    
                    newLine->setThickness(jLine.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
                    newLine->setDecoration(static_cast<DecorationType>(jLine.value("decoration", 0)));
                    newLine->setLineType(static_cast<Line::LineType>(jLine.value("lineType", 1))); // Default to Segment if missing

                    // Read Transformation Metadata
                    if (jLine.contains("transformType")) {
                        newLine->setTransformType(static_cast<TransformationType>(jLine["transformType"].get<int>()));
                        newLine->setParentSourceID(jLine.value("parentSourceID", 0u));
                        newLine->setAuxObjectID(jLine.value("auxObjectID", 0u));
                    }
                    if (jLine.contains("transformValue")) {
                        newLine->setTransformValue(jLine["transformValue"].get<double>());
                    }

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
                if (jCircle.contains("color")) {
                     if (jCircle["color"].is_string()) {
                         color = hexToColor(jCircle["color"]);
                     } else {
                         auto c = jCircle["color"];
                         color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                     }
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
                    if (id != -1) {
                        newCircle->setID(id);
                    }

                    editor.circles.push_back(newCircle);
                    
                    newCircle->setThickness(jCircle.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

                    // Read Transformation Metadata
                    if (jCircle.contains("transformType")) {
                        newCircle->setTransformType(static_cast<TransformationType>(jCircle["transformType"].get<int>()));
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
                if (polyJson.contains("color")) {
                     if (polyJson["color"].is_string()) {
                         color = hexToColor(polyJson["color"]);
                     } else {
                         auto c = polyJson["color"];
                         color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                     }
                }
                
                int id = polyJson.value("id", -1);

                auto polygon = std::make_shared<Polygon>(vertices, color, id);
                polygon->setThickness(polyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

                // Read Transformation Metadata
                if (polyJson.contains("transformType")) {
                    polygon->setTransformType(static_cast<TransformationType>(polyJson["transformType"].get<int>()));
                    polygon->setParentSourceID(polyJson.value("parentSourceID", 0u));
                    polygon->setAuxObjectID(polyJson.value("auxObjectID", 0u));
                }

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
                    if (rectJson.contains("color")) {
                        if (rectJson["color"].is_string()) {
                            color = hexToColor(rectJson["color"]);
                        } else {
                            auto c = rectJson["color"];
                            color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                        }
                    }
                    
                    int id = rectJson.value("id", -1);
                    Point_2 corner1(x1, y1);
                    Point_2 corner2(x2, y2);
                    
                    bool isRotatable = rectJson.value("isRotatable", false);
                    double height = rectJson.value("height", 0.0);
                    auto rect = std::make_shared<Rectangle>(corner1, corner2, isRotatable, color, id);
                    rect->setThickness(rectJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
                    if (isRotatable) {
                        rect->setHeight(height);
                    }

                    // Read Transformation Metadata
                    if (rectJson.contains("transformType")) {
                        rect->setTransformType(static_cast<TransformationType>(rectJson["transformType"].get<int>()));
                        rect->setParentSourceID(rectJson.value("parentSourceID", 0u));
                        rect->setAuxObjectID(rectJson.value("auxObjectID", 0u));
                    }
                    if (rectJson.contains("transformValue")) {
                        rect->setTransformValue(rectJson["transformValue"].get<double>());
                    }
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


        // =========================================================
        // PASS 5: LOAD POLYGONS
        // =========================================================
        if (data.contains("polygons")) {
            for (const auto& polyJson : data["polygons"]) {
                auto verticesJson = polyJson["vertices"];
                if (verticesJson.size() >= 3) {
                    std::vector<Point_2> pts;
                    for(const auto& v : verticesJson) {
                         if (v.is_array()) { 
                            double x = v[0].get<double>();
                            double y = v[1].get<double>();
                            pts.emplace_back(x, y);
                         } else {
                            pts.emplace_back(v.value("x", 0.0), v.value("y", 0.0));
                         }
                    }

                    sf::Color color = sf::Color::Black;
                    if (polyJson.contains("color")) {
                        if (polyJson["color"].is_string()) {
                            color = hexToColor(polyJson["color"]);
                        } else {
                            auto c = polyJson["color"];
                            color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                        }
                    }
                    
                    int id = polyJson.value("id", -1);
                    
                    auto poly = std::make_shared<Polygon>(pts, color, id);
                    
                    // Read Transformation Metadata
                    if (polyJson.contains("transformType")) {
                        poly->setTransformType(static_cast<TransformationType>(polyJson["transformType"].get<int>()));
                        poly->setParentSourceID(polyJson.value("parentSourceID", 0u));
                        poly->setAuxObjectID(polyJson.value("auxObjectID", 0u));
                    }
                    if (polyJson.contains("transformValue")) {
                        poly->setTransformValue(polyJson["transformValue"].get<double>());
                    }
                    
                    editor.polygons.push_back(poly);

                    if (id != -1) {
                        objectMap[id] = poly;
                        if (id > maxId) maxId = id;
                    }
                } 
            }
        } 
        
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
                    if (triJson.contains("color")) {
                        if (triJson["color"].is_string()) {
                            color = hexToColor(triJson["color"]);
                        } else {
                            auto c = triJson["color"];
                            color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                        }
                    }
                    
                    int id = triJson.value("id", -1);
                    
                    auto triangle = std::make_shared<Triangle>(pts[0], pts[1], pts[2], color, id);
                    triangle->setThickness(triJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));

                    // Read Transformation Metadata
                    if (triJson.contains("transformType")) {
                        triangle->setTransformType(static_cast<TransformationType>(triJson["transformType"].get<int>()));
                        triangle->setParentSourceID(triJson.value("parentSourceID", 0u));
                        triangle->setAuxObjectID(triJson.value("auxObjectID", 0u));
                    }
                    if (triJson.contains("transformValue")) {
                        triangle->setTransformValue(triJson["transformValue"].get<double>());
                    }

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
                if (rpolyJson.contains("color")) {
                    if (rpolyJson["color"].is_string()) {
                        color = hexToColor(rpolyJson["color"]);
                    } else {
                        auto c = rpolyJson["color"];
                        color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                    }
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
                    rpoly->setThickness(rpolyJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT));
                    
                    // Read Transformation Metadata
                    if (rpolyJson.contains("transformType")) {
                        rpoly->setTransformType(static_cast<TransformationType>(rpolyJson["transformType"].get<int>()));
                        rpoly->setParentSourceID(rpolyJson.value("parentSourceID", 0u));
                        rpoly->setAuxObjectID(rpolyJson.value("auxObjectID", 0u));
                    }
                    if (rpolyJson.contains("transformValue")) {
                        rpoly->setTransformValue(rpolyJson["transformValue"].get<double>());
                    }

                    editor.regularPolygons.push_back(rpoly);

                    if (id != -1) {
                        objectMap[id] = rpoly;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 7.5: LOAD TRANSFORMATION POINTS
        // =========================================================
        for (const auto& jPoint : deferredTransformPoints) {
            int id = jPoint.value("id", -1);
            sf::Color color = sf::Color::Black;
            if (jPoint.contains("color")) {
                if (jPoint["color"].is_string()) {
                    color = hexToColor(jPoint["color"]);
                } else {
                    auto c = jPoint["color"];
                    color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                }
            }

            if (!jPoint.contains("transform")) continue;
            const auto& t = jPoint["transform"];
            std::string type = t.value("type", "");

            std::shared_ptr<Point> newPoint;
            if (type == "ReflectLine") {
                int srcId = t.value("sourceId", -1);
                int lineId = t.value("lineId", -1);
                if (pointMap.count(srcId) && objectMap.count(lineId)) {
                    auto linePtr = std::dynamic_pointer_cast<Line>(objectMap[lineId]);
                    if (linePtr) newPoint = std::make_shared<ReflectLine>(pointMap[srcId], linePtr, color);
                }
            } else if (type == "ReflectPoint") {
                int srcId = t.value("sourceId", -1);
                int centerId = t.value("centerId", -1);
                if (pointMap.count(srcId) && pointMap.count(centerId)) {
                    newPoint = std::make_shared<ReflectPoint>(pointMap[srcId], pointMap[centerId], color);
                }
            } else if (type == "ReflectCircle") {
                int srcId = t.value("sourceId", -1);
                int circleId = t.value("circleId", -1);
                if (pointMap.count(srcId) && objectMap.count(circleId)) {
                    auto circlePtr = std::dynamic_pointer_cast<Circle>(objectMap[circleId]);
                    if (circlePtr) newPoint = std::make_shared<ReflectCircle>(pointMap[srcId], circlePtr, color);
                }
            } else if (type == "RotatePoint") {
                int srcId = t.value("sourceId", -1);
                int centerId = t.value("centerId", -1);
                double angleDeg = t.value("angleDeg", 0.0);
                if (pointMap.count(srcId) && pointMap.count(centerId)) {
                    newPoint = std::make_shared<RotatePoint>(pointMap[srcId], pointMap[centerId], angleDeg, color);
                }
            } else if (type == "TranslateVector") {
                int srcId = t.value("sourceId", -1);
                int vStartId = t.value("vecStartId", -1);
                int vEndId = t.value("vecEndId", -1);
                if (pointMap.count(srcId) && pointMap.count(vStartId) && pointMap.count(vEndId)) {
                    newPoint = std::make_shared<TranslateVector>(pointMap[srcId], pointMap[vStartId], pointMap[vEndId], color);
                }
            } else if (type == "DilatePoint") {
                int srcId = t.value("sourceId", -1);
                int centerId = t.value("centerId", -1);
                double factor = t.value("factor", 1.0);
                if (pointMap.count(srcId) && pointMap.count(centerId)) {
                    newPoint = std::make_shared<DilatePoint>(pointMap[srcId], pointMap[centerId], factor, color);
                }
            }

            if (newPoint) {
                if (id != -1) newPoint->setID(id);
                if (jPoint.contains("label")) newPoint->setLabel(jPoint.value("label", ""));
                if (jPoint.contains("showLabel")) newPoint->setShowLabel(jPoint.value("showLabel", true));
                if (jPoint.contains("labelOffset") && jPoint["labelOffset"].is_array() && jPoint["labelOffset"].size() >= 2) {
                    newPoint->setLabelOffset(sf::Vector2f(jPoint["labelOffset"][0], jPoint["labelOffset"][1]));
                }

                editor.points.push_back(newPoint);
                if (id != -1) {
                    pointMap[id] = newPoint;
                    objectMap[id] = newPoint;
                    if (id > maxId) maxId = id;
                }
            }
        }

        // =========================================================
        // PASS 8: LOAD OBJECTPOINTS
        // =========================================================
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
                    if (opJson["color"].is_string()) {
                        color = hexToColor(opJson["color"]);
                    } else {
                        auto c = opJson["color"];
                        color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                    }
                }

                ObjectType hostType = static_cast<ObjectType>(hostTypeInt);
                std::shared_ptr<ObjectPoint> newObjPoint = nullptr;

                if (hostType == ObjectType::Line && objectMap.count(hostId)) {
                    auto hostLine = std::dynamic_pointer_cast<Line>(objectMap[hostId]);
                    if (hostLine) {
                        newObjPoint = ObjectPoint::create(hostLine, t, color);
                    }
                } else if (hostType == ObjectType::Circle && objectMap.count(hostId)) {
                    auto hostCircle = std::dynamic_pointer_cast<Circle>(objectMap[hostId]);
                    if (hostCircle) {
                        newObjPoint = ObjectPoint::create(hostCircle, t, color);
                    }
                } else if ((hostType == ObjectType::Rectangle || hostType == ObjectType::Triangle ||
                            hostType == ObjectType::Polygon || hostType == ObjectType::RegularPolygon) &&
                           objectMap.count(hostId)) {
                    auto hostShape = objectMap[hostId];
                    if (hostShape) {
                        newObjPoint = ObjectPoint::createOnShapeEdge(hostShape, edgeIndex, edgeRel, color);
                    }
                }

                if (newObjPoint) {
                    if (id != -1) newObjPoint->setID(id);
                    newObjPoint->setVisible(visible);
                    if (opJson.contains("label")) newObjPoint->setLabel(opJson.value("label", ""));
                    if (opJson.contains("showLabel")) newObjPoint->setShowLabel(opJson.value("showLabel", true));
                    if (opJson.contains("labelOffset") && opJson["labelOffset"].is_array() && opJson["labelOffset"].size() >= 2) {
                        newObjPoint->setLabelOffset(sf::Vector2f(opJson["labelOffset"][0], opJson["labelOffset"][1]));
                    }

                    editor.ObjectPoints.push_back(newObjPoint);
                    if (id != -1) {
                        pointMap[id] = newObjPoint;
                        objectMap[id] = newObjPoint;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 9: LOAD TANGENT LINES
        // =========================================================
        if (data.contains("tangentLines")) {
            for (const auto& tJson : data["tangentLines"]) {
                int id = tJson.value("id", -1);
                int externalId = tJson.value("externalPointID", -1);
                int circleId = tJson.value("circleID", -1);
                int solutionIndex = tJson.value("solutionIndex", 0);

                if (pointMap.count(externalId) && objectMap.count(circleId)) {
                    auto circlePtr = std::dynamic_pointer_cast<Circle>(objectMap[circleId]);
                    if (!circlePtr) continue;

                    sf::Color color = sf::Color::Black;
                    if (tJson.contains("color")) {
                        if (tJson["color"].is_string()) {
                            color = hexToColor(tJson["color"]);
                        } else {
                            auto c = tJson["color"];
                            color = sf::Color(c.value("r", 0), c.value("g", 0), c.value("b", 0), c.value("a", 255));
                        }
                    }

                    auto tangent = std::make_shared<TangentLine>(pointMap[externalId], circlePtr, solutionIndex, id, color);
                    tangent->setThickness(static_cast<float>(tJson.value("thickness", Constants::LINE_THICKNESS_DEFAULT)));
                    editor.lines.push_back(tangent);

                    if (id != -1) {
                        objectMap[id] = tangent;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 10: LOAD ANGLES
        // =========================================================
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
                        if (aJson["color"].is_string()) {
                            color = hexToColor(aJson["color"]);
                        } else {
                            auto c = aJson["color"];
                            color = sf::Color(c.value("r", 255), c.value("g", 200), c.value("b", 0), c.value("a", 255));
                        }
                    }

                    auto angle = std::make_shared<Angle>(pointMap[aId], pointMap[vId], pointMap[bId], reflex, color);
                    if (id != -1) angle->setID(id);

                    if (aJson.contains("radius")) angle->setRadius(aJson.value("radius", angle->getRadius()));
                    if (aJson.contains("showLabel")) angle->setShowLabel(aJson.value("showLabel", true));
                    if (aJson.contains("labelOffset") && aJson["labelOffset"].is_array() && aJson["labelOffset"].size() >= 2) {
                        angle->setLabelOffset(sf::Vector2f(aJson["labelOffset"][0], aJson["labelOffset"][1]));
                    }

                    editor.angles.push_back(angle);
                    if (id != -1) {
                        objectMap[id] = angle;
                        if (id > maxId) maxId = id;
                    }
                }
            }
        }

        // =========================================================
        // PASS 11: LOAD INTERSECTION CONSTRAINTS
        // =========================================================
        if (data.contains("intersections")) {
            for (const auto& cJson : data["intersections"]) {
                int aId = cJson.value("aId", -1);
                int bId = cJson.value("bId", -1);
                if (!objectMap.count(aId) || !objectMap.count(bId)) continue;

                auto A = objectMap[aId];
                auto B = objectMap[bId];

                std::vector<std::shared_ptr<Point>> createdPoints;
                if (cJson.contains("points") && cJson["points"].is_array()) {
                    for (const auto& pJson : cJson["points"]) {
                        double x = pJson.value("x", 0.0);
                        double y = pJson.value("y", 0.0);
                        int id = pJson.value("id", -1);

                        auto pt = std::make_shared<Point>(Point_2(x, y), Constants::CURRENT_ZOOM,
                                                          Constants::INTERSECTION_POINT_COLOR, id);
                        pt->setIntersectionPoint(true);
                        pt->setDependent(true);
                        pt->setSelected(false);
                        pt->lock();

                        if (pJson.contains("label")) pt->setLabel(pJson.value("label", ""));
                        if (pJson.contains("showLabel")) pt->setShowLabel(pJson.value("showLabel", true));
                        if (pJson.contains("labelOffset") && pJson["labelOffset"].is_array() && pJson["labelOffset"].size() >= 2) {
                            pt->setLabelOffset(sf::Vector2f(pJson["labelOffset"][0], pJson["labelOffset"][1]));
                        }

                        editor.points.push_back(pt);
                        if (id != -1) {
                            pointMap[id] = pt;
                            objectMap[id] = pt;
                            if (id > maxId) maxId = id;
                        }
                        createdPoints.push_back(pt);
                    }
                }

                if (!createdPoints.empty()) {
                    DynamicIntersection::registerIntersectionConstraint(A, B, createdPoints);
                }
            }
        }

        // =========================================================
        // PASS 12: RESTORE LINE CONSTRAINTS
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

        // =========================================================
        // PASS 13: RESTITCH TRANSFORMATION LOGIC
        // =========================================================
        // We iterate through all objects and re-establish transformation links
        for (auto& pair : objectMap) {
            auto obj = pair.second;
            if (obj && obj->getTransformType() != TransformationType::None) {
                unsigned int parentId = obj->getParentSourceID();
                unsigned int auxId = obj->getAuxObjectID();
                
                std::shared_ptr<GeometricObject> parent = (parentId != 0 && objectMap.count(parentId)) ? objectMap[parentId] : nullptr;
                std::shared_ptr<GeometricObject> aux = (auxId != 0 && objectMap.count(auxId)) ? objectMap[auxId] : nullptr;
                
                obj->restoreTransformation(parent, aux, obj->getTransformType());
                obj->update(); // Ensure initial position is correct after re-linking
            }
        }

        // Restore axes visibility and ensure axes are present
        if (editor.getXAxisShared()) {
            editor.getXAxisShared()->setVisible(axesVisible);
            if (std::find(editor.lines.begin(), editor.lines.end(), editor.getXAxisShared()) == editor.lines.end()) {
                editor.lines.push_back(editor.getXAxisShared());
            }
        }
        if (editor.getYAxisShared()) {
            editor.getYAxisShared()->setVisible(axesVisible);
            if (std::find(editor.lines.begin(), editor.lines.end(), editor.getYAxisShared()) == editor.lines.end()) {
                editor.lines.push_back(editor.getYAxisShared());
            }
        }

        // Finalize state
        DynamicIntersection::updateAllIntersections(editor);
        for (auto &ag : editor.angles) {
            if (ag) ag->update();
        }
        editor.objectIdCounter = maxId + 1;
        std::cout << "Project loaded. Counter: " << editor.objectIdCounter << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Load Error: " << e.what() << std::endl;
        return false;
    }
}

#endif  // DISABLED LEGACY LOAD

// ============================================================================
// EXPORT SVG
// ============================================================================

bool ProjectSerializer::exportSVG(const GeometryEditor& editor, const std::string& filepath) {
    try {
        // STEP 1: Use current view bounds (WYSIWYG)
        const sf::View& view = editor.getDrawingView();
        sf::Vector2f center = view.getCenter();
        sf::Vector2f size = view.getSize();

        double minX = center.x - size.x / 2.0;
        double minY = center.y - size.y / 2.0;
        double width = size.x;
        double height = size.y;
        double maxX = minX + width;
        double maxY = minY + height;

        // Initialize SVG Writer
        SVGWriter svg(width, height);
        // Use world-space viewBox and flip Y with a translate+scale group
        svg.setViewBox(minX, minY, width, height);

        // ========================================================================
        // STEP 4: Write SVG header with viewBox
        // ========================================================================
        // Ensure aspect ratio of the output image matches the view bounds
        double outputWidth = 800.0;
        double outputHeight = 800.0;
        
        if (width > 0 && height > 0) {
            double aspectRatio = width / height;
            if (aspectRatio > 1.0) {
                 outputWidth = 800.0;
                 outputHeight = 800.0 / aspectRatio;
            } else {
                 outputHeight = 800.0;
                 outputWidth = 800.0 * aspectRatio;
            }
        }
        svg.setOutputSize(outputWidth, outputHeight);

        // We use the View bounds directly (WYSIWYG).
        // This part of the code is not directly used in the current SVGWriter implementation,
        // as SVGWriter handles the header. However, the outputWidth/Height and scaling logic
        // below are relevant.
        // file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        // file << "<svg xmlns=\"http://www.w3.org/2000/svg\" ";
        // file << "viewBox=\"" << minX << " " << minY << " " << width << " " << height << "\" ";
        // file << "width=\"" << outputWidth << "\" height=\"" << outputHeight << "\">\n";
        // file << "  <g>\n";

        // Background
        SVGWriter::Style bgStyle;
        bgStyle.fill = "white";
        bgStyle.stroke = "none";
        // Background in world-space viewBox
        svg.drawRect(minX, minY, width, height, bgStyle);

        // ========================================================================
        // SCALING FIX:
        // Calculate stroke/radius in World Units corresponding to Screen Pixels.
        // Derived from: desired_pixels * (world_width / output_width)
        // This ensures lineweights look "normal" (e.g. 2px) regardless of zoom level.
        // ========================================================================
        double pixelToWorldScale = width / outputWidth;
        
        double strokeWidth = 2.0 * pixelToWorldScale;       // ~2 pixels thick
        double pointRadius = 4.0 * pixelToWorldScale;       // ~4 pixels radius
        double gridStrokeWidth = 1.0 * pixelToWorldScale;   // ~1 pixel grid

        // Grid
        // Adaptive grid size to prevent massive file size on large coordinates
        double adaptiveGridStep = Constants::GRID_SIZE;
        double maxDimension = std::max(width, height);
        
        // If view is very large, increase grid step to keep line count reasonable (~200 lines max)
        if (maxDimension / adaptiveGridStep > 200.0) {
            adaptiveGridStep = maxDimension / 200.0;
        }

        // Begin flipped group for all world-space geometry (Y-up)
        svg.beginGroup("translate(0," + std::to_string(minY + maxY) + ") scale(1,-1)");

        if (editor.isGridVisible()) {
            SVGWriter::Style gridStyle;
            gridStyle.stroke = "#e0e0e0";
            gridStyle.strokeWidth = gridStrokeWidth; // Use the calculated adaptive grid stroke width

            for (double y = std::floor(minY / adaptiveGridStep) * adaptiveGridStep; y <= maxY; y += adaptiveGridStep) {
                svg.drawLine(minX, y, maxX, y, gridStyle);
            }
            for (double x = std::floor(minX / adaptiveGridStep) * adaptiveGridStep; x <= maxX; x += adaptiveGridStep) {
                svg.drawLine(x, minY, x, maxY, gridStyle);
            }
        }

        // Axes
        if (editor.areAxesVisible()) {
            SVGWriter::Style axesStyle;
            axesStyle.stroke = "#000000";
            axesStyle.strokeWidth = strokeWidth;
            svg.drawLine(minX, 0.0, maxX, 0.0, axesStyle);
            svg.drawLine(0.0, minY, 0.0, maxY, axesStyle);
        }

        // Removed redundant definitions


        // STEP 4: Shapes
        // Rectangles
        for (const auto& rect : editor.rectangles) {
            if (rect && rect->isValid() && rect->isVisible()) {
                auto vertices = rect->getInteractableVertices();
                std::vector<std::pair<double, double>> points;
                for(const auto& v : vertices) points.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                
                SVGWriter::Style style;
                style.stroke = colorToHex(rect->getColor());
                style.strokeWidth = rect->getThickness() * pixelToWorldScale;
                svg.drawPolygon(points, style);
            }
        }
        
        // Lines
        for (const auto& ln : editor.lines) {
            if (ln && ln->isValid() && ln->isVisible()) {
                Point_2 start = ln->getStartPoint();
                Point_2 end = ln->getEndPoint();
                
                SVGWriter::Style style;
                style.stroke = colorToHex(ln->getColor());
                style.strokeWidth = ln->getThickness() * pixelToWorldScale;
                
                // Handle infinite lines / rays by clipping to view bounds
                // Reuse logic from Line::draw but in world space
                Point_2 p1 = start;
                Point_2 p2 = end;

                if (!ln->isSegment()) {
                    Vector_2 dir = end - start;
                    Direction_2 direction(dir);
                    double viewDiag = std::sqrt(width * width + height * height);
                    double extension = viewDiag * 1.5;

                    Point_2 viewCenter(minX + width / 2.0, minY + height / 2.0);
                    // Projections in world space
                    Vector_2 toCenter = viewCenter - start;
                    double proj = CGAL::to_double(toCenter.x() * dir.x() + toCenter.y() * dir.y()) / CGAL::to_double(dir.squared_length());
                    Point_2 mid = start + dir * proj;

                    if (ln->getLineType() == Line::LineType::Ray) {
                        p1 = start;
                        p2 = mid + direction.to_vector() * extension;
                    } else {
                        p1 = mid - direction.to_vector() * extension;
                        p2 = mid + direction.to_vector() * extension;
                    }
                }

                svg.drawLine(CGAL::to_double(p1.x()), CGAL::to_double(p1.y()),
                             CGAL::to_double(p2.x()), CGAL::to_double(p2.y()), style);

                // --- Vector Arrowhead ---
                if (ln->getLineType() == Line::LineType::Vector || ln->getType() == ObjectType::Vector) {
                    double x1 = CGAL::to_double(p1.x());
                    double y1 = CGAL::to_double(p1.y());
                    double x2 = CGAL::to_double(p2.x());
                    double y2 = CGAL::to_double(p2.y());

                    double dx = x2 - x1;
                    double dy = y2 - y1;
                    double l = std::sqrt(dx*dx + dy*dy);
                    if (l > 1e-6) {
                        double ux = dx / l;
                        double uy = dy / l;
                        double nx = -uy;
                        double ny = ux;

                        double arrowLen = 15.0 * pixelToWorldScale;
                        double arrowWidth = 10.0 * pixelToWorldScale;

                        double tipX = x2;
                        double tipY = y2;
                        double baseX = tipX - ux * arrowLen;
                        double baseY = tipY - uy * arrowLen;

                        double leftX = baseX + nx * (arrowWidth * 0.5);
                        double leftY = baseY + ny * (arrowWidth * 0.5);
                        double rightX = baseX - nx * (arrowWidth * 0.5);
                        double rightY = baseY - ny * (arrowWidth * 0.5);

                        std::vector<std::pair<double, double>> arrowPoints = {
                            {tipX, tipY}, {leftX, leftY}, {rightX, rightY}
                        };
                        SVGWriter::Style arrowStyle = style;
                        arrowStyle.fill = style.stroke; // Fill with stroke color
                        svg.drawPolygon(arrowPoints, arrowStyle);
                    }
                }

                // Export Decorations
                if (ln->getDecoration() != DecorationType::None) {
                    double x1 = CGAL::to_double(p1.x());
                    double y1 = CGAL::to_double(p1.y());
                    double x2 = CGAL::to_double(p2.x());
                    double y2 = CGAL::to_double(p2.y());

                    double dx = x2 - x1;
                    double dy = y2 - y1;
                    double l = std::sqrt(dx*dx + dy*dy);
                    if (l > 1e-6) {
                        double ux = dx / l;
                        double uy = dy / l;
                        double nx = -uy;
                        double ny = ux;
                        double cx = (x1 + x2) * 0.5;
                        double cy = (y1 + y2) * 0.5;
                        double size = 12.0 * pixelToWorldScale;
                        double spacing = 6.0 * pixelToWorldScale;

                        auto exportTick = [&](double px, double py) {
                            svg.drawLine(px + nx * size * 0.5, py + ny * size * 0.5,
                                         px - nx * size * 0.5, py - ny * size * 0.5, style);
                        };

                        auto exportArrow = [&](double px, double py) {
                            double tipX = px + ux * size * 0.5;
                            double tipY = py + uy * size * 0.5;
                            double baseX = px - ux * size * 0.5;
                            double baseY = py - uy * size * 0.5;
                            svg.drawLine(baseX + nx * size * 0.4, baseY + ny * size * 0.4, tipX, tipY, style);
                            svg.drawLine(baseX - nx * size * 0.4, baseY - ny * size * 0.4, tipX, tipY, style);
                        };

                        switch (ln->getDecoration()) {
                            case DecorationType::Tick1: exportTick(cx, cy); break;
                            case DecorationType::Tick2:
                                exportTick(cx - ux * spacing * 0.5, cy - uy * spacing * 0.5);
                                exportTick(cx + ux * spacing * 0.5, cy + uy * spacing * 0.5);
                                break;
                            case DecorationType::Tick3:
                                exportTick(cx, cy);
                                exportTick(cx - ux * spacing, cy - uy * spacing);
                                exportTick(cx + ux * spacing, cy + uy * spacing);
                                break;
                            case DecorationType::Arrow1: exportArrow(cx, cy); break;
                            case DecorationType::Arrow2:
                                exportArrow(cx - ux * spacing * 0.7, cy - uy * spacing * 0.7);
                                exportArrow(cx + ux * spacing * 0.7, cy + uy * spacing * 0.7);
                                break;
                            case DecorationType::Arrow3:
                                exportArrow(cx, cy);
                                exportArrow(cx - ux * spacing * 1.2, cy - uy * spacing * 1.2);
                                exportArrow(cx + ux * spacing * 1.2, cy + uy * spacing * 1.2);
                                break;
                            case DecorationType::None:
                                break;
                        }
                    }
                }
            }
        }

        // Circles
        for (const auto& ci : editor.circles) {
             if (ci && ci->isValid() && ci->isVisible()) {
                SVGWriter::Style style;
                style.stroke = colorToHex(ci->getColor());
                style.strokeWidth = ci->getThickness() * pixelToWorldScale;

                if (ci->isSemicircle()) {
                    // Semicircle export using SVG Arc
                    Point_2 center = ci->getCenterPoint();
                    double r = ci->getRadius();
                    auto p1 = ci->getDiameterP1();
                    auto p2 = ci->getDiameterP2();
                    
                    if (p1 && p2) {
                        double x1 = CGAL::to_double(p1->getCGALPosition().x());
                        double y1 = CGAL::to_double(p1->getCGALPosition().y());
                        double x2 = CGAL::to_double(p2->getCGALPosition().x());
                        double y2 = CGAL::to_double(p2->getCGALPosition().y());

                        // SVG path: M x1 y1 A r r 0 0 1 x2 y2
                        // We need to determine the arc sweep flag. 
                        // Our semicircle is always the CCW arc from P1 to P2 
                        // in our Cartesian system.
                        std::stringstream ss;
                        ss << "M " << x1 << " " << y1 << " A " << r << " " << r << " 0 0 1 " << x2 << " " << y2;
                        svg.drawPath(ss.str(), style);
                    }
                } else {
                    Point_2 center = ci->getCGALPosition();
                    svg.drawCircle(CGAL::to_double(center.x()), CGAL::to_double(center.y()), ci->getRadius(), style);
                }
             }
        }
        
        // Polygons
        for (const auto& poly : editor.polygons) {
            if (poly && poly->isValid() && poly->isVisible()) {
                auto vertices = poly->getInteractableVertices();
                std::vector<std::pair<double, double>> points;
                for(const auto& v : vertices) points.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                
                SVGWriter::Style style;
                style.stroke = colorToHex(poly->getColor());
                style.strokeWidth = poly->getThickness() * pixelToWorldScale;
                svg.drawPolygon(points, style);
            }
        }

        // Triangles
         for (const auto& tri : editor.triangles) {
            if (tri && tri->isValid() && tri->isVisible()) {
                auto vertices = tri->getInteractableVertices();
                std::vector<std::pair<double, double>> points;
                for(const auto& v : vertices) points.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                
                SVGWriter::Style style;
                style.stroke = colorToHex(tri->getColor());
                style.strokeWidth = tri->getThickness() * pixelToWorldScale;
                svg.drawPolygon(points, style);
            }
        }

        // Regular Polygons
         for (const auto& rp : editor.regularPolygons) {
            if (rp && rp->isValid() && rp->isVisible()) {
                auto vertices = rp->getInteractableVertices();
                std::vector<std::pair<double, double>> points;
                for(const auto& v : vertices) points.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
                
                SVGWriter::Style style;
                style.stroke = colorToHex(rp->getColor());
                style.strokeWidth = rp->getThickness() * pixelToWorldScale;
                svg.drawPolygon(points, style);
            }
        }

        // Points (Draw on top)
        for (const auto& pt : editor.points) {
            if (pt && pt->isValid() && pt->isVisible()) {
                Point_2 pos = pt->getCGALPosition();
                SVGWriter::Style style;
                style.fill = colorToHex(pt->getColor());
                style.stroke = "none";
                svg.drawCircle(CGAL::to_double(pos.x()), CGAL::to_double(pos.y()), pointRadius, style);
            }
        }
        
        for (const auto& pt : editor.ObjectPoints) {
            if (pt && pt->isValid() && pt->isVisible()) {
                Point_2 pos = pt->getCGALPosition();
                SVGWriter::Style style;
                style.fill = colorToHex(pt->getColor());
                style.stroke = "none";
                svg.drawCircle(CGAL::to_double(pos.x()), CGAL::to_double(pos.y()), pointRadius, style);
            }
        }

        // End flipped group
        svg.endGroup();

        // Save file
        if (svg.save(filepath)) {
            std::cout << "SVG exported successfully (Professional Writer): " << filepath << std::endl;
            return true;
        } else {
             std::cerr << "ProjectSerializer::exportSVG: Failed to save file." << std::endl;
             return false;
        }

    } catch (const std::exception& e) {
        std::cerr << "ProjectSerializer::exportSVG: Exception: " << e.what() << std::endl;
        return false;
    }
}
// End exportSVG
