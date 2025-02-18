//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/squared_distance_2.h>
//#include <SFML/Graphics.hpp>
//#include <CGAL/intersections.h>
//#include <algorithm>
//#include <variant>
//#include "Types.h"
//#include "Constants.h"
//#include "CommandManager.h"
//#include "GenericDeleteCommand.h"
//#include "GUI.h"
//#include "Grid.h"
//#include "Point.h"
//#include "ObjectPoint.h"
//#include "ObjectType.h"
//#include "GeometricObject.h"
//#include "Line.h"
//#include "Circle.h"
//#include "Intersection.h"
//#include "command.h"
//#include <memory>
//#include <vector>
//#include <iostream>
//#include "ProjectionUtils.h"
//#include "GeometryFactory.h"
//#include "HandleEvents.h"
#include "GeometryEditor.h"
// Replace the problematic line with the following:




//class GeometryEditor {
//private:
//	sf::RenderWindow window;
//	sf::View drawingView;
//	GUI gui;
//	Grid grid;
//	CommandManager commandManager;
//	sf::View guiView;
//	std::vector<std::unique_ptr<Point>> points;
//	std::vector<std::unique_ptr<ObjectPoint>>ObjectPoints;
//	std::vector<std::unique_ptr<Line>> lines;
//	std::vector<std::unique_ptr<Circle>> circles;
//	std::vector<sf::Vector2f> linePoints;
//	sf::Vector2f lastMousePos;
//	sf::Vector2f dragOffset;
//	Point* lineCreationPoint = nullptr;
//
//	//Panning state
//	bool isPanning = false;
//	sf::Vector2f panningVelocity;
//	sf::Clock panClock;
//	float panSpeed = 5.0f;
//	const float dampingFactor = 0.85f;
//	sf::Vector2f lastValidMousePos;
//	bool isFirstPanFrame = true;     // Track first frame of panning
//
//	// Selection and dragging state
//	Point* selectedPoint = nullptr;
//	Point* selectedSecondPoint = nullptr;
//	Line* selectedLine = nullptr;
//	Line* selectedLineSegment = nullptr;
//	bool isDragging = false;
//	bool isDraggingEndpoint = false;
//	int selectedEndpointIndex = -1;
//	int selectedControlPointIndex = -1;
//	bool isDraggingControlPoint = false;
//	DragMode dragMode = DragMode::None; // ADDED: To track the drag mode
//
//	// Selection and dragging state
//	Circle* selectedCircle = nullptr;
//	bool isDraggingCircle = false;
//	bool isResizingCircle = false;
//
//	// ObjectPoint
//	bool isDraggingObjectPoint = false;
//	ObjectPoint* selectedObjectPoint = nullptr; // ADDED: To track the selectedObjectPoint


	
	/*Point_2 projectPointOntoLine(const Point_2& p, const Line* lineObj, bool isSegment) {
		return ProjectionUtils::projectPointOntoLine(p, lineObj, isSegment);
	}

	Point_2 projectPointOntoCircle(const Point_2& p, const Point_2& center, double radius, double tolerance) {
		return ProjectionUtils::projectPointOntoCircle(p, center, radius, tolerance);
	}*/
	// Otherwise translate the whole segment


	/*void handlePanning(const sf::Event::MouseMoveEvent& mouseEvent) {
		if (isPanning) {
			sf::Vector2f currentMousePos = window.mapPixelToCoords(sf::Vector2i(mouseEvent.x, mouseEvent.y), drawingView);
			sf::Vector2f delta = lastMousePos - currentMousePos;
			drawingView.move(delta);
			window.setView(drawingView);
			lastMousePos = currentMousePos;
		}
	}*/
	//    if (isPanning) {
	//        sf::Vector2f currentMousePos = window.mapPixelToCoords(
	//            sf::Vector2i(mouseEvent.x, mouseEvent.y), drawingView);

	//        if (isFirstPanFrame) {
	//            lastMousePos = currentMousePos;  // Initialize lastMousePos correctly
	//            isFirstPanFrame = false;
	//            return;
	//        }

	//        sf::Vector2f delta = lastMousePos - currentMousePos;

	//        // Clamp maximum movement to avoid jumps
	//        const float maxDelta = 30.0f;
	//        delta.x = std::clamp(delta.x, -maxDelta, maxDelta);
	//        delta.y = std::clamp(delta.y, -maxDelta, maxDelta);

	//        // Apply smooth panning with damping
	//        drawingView.move(delta);
	//        window.setView(drawingView);

	//        lastMousePos = currentMousePos;  // Update last position properly
	//    }
	//    else {
	//        isFirstPanFrame = true;
	//    }
	//}

int main() {
	try {
		GeometryEditor editor;
		editor.run();
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}

