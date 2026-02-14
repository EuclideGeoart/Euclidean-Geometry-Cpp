#pragma once
#ifndef PROJECT_SERIALIZER_H
#define PROJECT_SERIALIZER_H

#include <string>
#include <SFML/Graphics/Color.hpp>  // For sf::Color

// Forward declarations
class GeometryEditor;
class GeometricObject;
class Point;
class Line;
class Circle;
class Rectangle;
class Polygon;
class RegularPolygon;
class Triangle;
class ObjectPoint;

/**
 * @brief Handles project serialization (save/load) and SVG export.
 * 
 * JSON Format:
 * - Uses nlohmann/json library (single-header)
 * - Two-pass loading for dependency resolution
 * 
 * SVG Export:
 * - Uses standard C++ file streams
 * - Transforms world coordinates to SVG screen coordinates
 */
class ProjectSerializer {
public:
    /**
     * @brief Save the current project to a JSON file.
     * @param editor The GeometryEditor containing all objects
     * @param filepath Path to save the file (e.g., "project.json")
     * @return true if save was successful
     */
    static bool saveProject(const GeometryEditor& editor, const std::string& filepath);
    
    /**
     * @brief Load a project from a JSON file.
     * @param editor The GeometryEditor to populate with loaded objects
     * @param filepath Path to the JSON file
     * @return true if load was successful
     */
    static bool loadProject(GeometryEditor& editor, const std::string& filepath);
    
    /**
     * @brief Export the current scene to an SVG file.
     * @param editor The GeometryEditor containing all objects
     * @param filepath Path to save the SVG file (e.g., "export.svg")
     * @return true if export was successful
     */
    static bool exportSVG(const GeometryEditor& editor, const std::string& filepath);

private:
    // Helper to convert sf::Color to hex string
    static std::string colorToHex(const sf::Color& color);
    
    // Helper to convert hex string to sf::Color
    static sf::Color hexToColor(const std::string& hex);
    
    // Helper to convert CGAL coordinates to SVG coordinates
    static void worldToSVG(double worldX, double worldY, 
                           double& svgX, double& svgY,
                           double viewHeight, double minY);
    
    // Calculate bounding box of all objects
    static void calculateBounds(const GeometryEditor& editor,
                                double& minX, double& minY,
                                double& maxX, double& maxY);
};

#endif // PROJECT_SERIALIZER_H
