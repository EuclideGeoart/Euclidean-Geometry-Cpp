#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include <algorithm>
#include <variant>
#include "Constants.h"
#include "GUI.h"
#include "Grid.h"
#include "Types.h"
#include "CommandManager.h"
#include "GenericDeleteCommand.h"
#include "Point.h"
#include "ObjectPoint.h"
#include "ObjectType.h"
#include "GeometricObject.h"
#include "Intersection.h"
#include "ProjectionUtils.h"
#include "GeometryFactory.h"
#include "HandleEvents.h"
#include "command.h"
#include "Line.h"
#include "Circle.h"

enum class DragMode {
    None,
    MoveEndpoint,
    TranslateSegment,
    MoveControlPoint
};

enum class EndpointSelection {
    None,
    Start,
    End
};

// Global state variables for dragging line segments:
extern DragMode segmentDragMode;
extern EndpointSelection selectedEndpoint;
extern sf::Vector2f dragStartPos;

// used when translating the whole segment

class GeometryEditor {
public:
    GeometryEditor();
    void run();

    void handleEvents(GeometryEditor& editor);
    sf::RenderWindow window;
    sf::View drawingView;
    GUI gui;
    Grid grid;
    CommandManager commandManager;
    sf::View guiView;

    std::vector<std::unique_ptr<Point>> points;
    std::vector<std::unique_ptr<ObjectPoint>> ObjectPoints;
    std::vector<std::unique_ptr<Line>> lines;
    std::vector<std::unique_ptr<Circle>> circles;
    std::vector<sf::Vector2f> linePoints;
    sf::Vector2f lastMousePos;
    sf::Vector2f dragOffset;
    Point* lineCreationPoint = nullptr;

    // Panning state
    bool isPanning = false;
    sf::Vector2f panningVelocity;
    sf::Clock panClock;
    float panSpeed = 1.0f;
    const float dampingFactor = 0.85f;
    sf::Vector2f lastValidMousePos;
    bool isFirstPanFrame = true;

    // Selection and dragging state
    Point* selectedPoint = nullptr;
    Point* selectedSecondPoint = nullptr;
    Line* selectedLine = nullptr;
    Line* selectedLineSegment = nullptr;
    bool isDragging = false;
    bool isDraggingEndpoint = false;
    int selectedEndpointIndex = -1;
    int selectedControlPointIndex = -1;
    bool isDraggingControlPoint = false;
    DragMode dragMode = DragMode::None;

    // Selection and dragging state
    Circle* selectedCircle = nullptr;
    bool isDraggingCircle = false;
    bool isResizingCircle = false;

    // ObjectPoint
    bool isDraggingObjectPoint = false;
    ObjectPoint* selectedObjectPoint = nullptr;
    //Utils
    sf::Vector2f toSFMLVector(const Point_2& point);
    Point_2 toCGALPoint(const sf::Vector2f& vector);

    float getScaledTolerance(const sf::View& view);
    void handleGeometryCreation(const sf::Vector2f& worldPos, const sf::Vector2i& pos);
    void calculateIntersections(const sf::Vector2f& clickPos);
    float length(const sf::Vector2f& vec);
    Circle* findNearbyCircle(const sf::Vector2f& worldPos);
private:
    void render();
    void update(sf::Time deltaTime);
    
    //void handleObjectPointDrag(sf::Vector2f mousePos);
    void handleDragging(const sf::Vector2f& worldPos);
    
    Point* findNearbyPoint(const sf::Vector2f& worldPos);
    void handlePointMovement(Point* point, const sf::Vector2f& newPos);
    std::optional<Point_2> findIntersection(const Line_2& line1, const Line_2& line2);
};
