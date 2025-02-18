#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/squared_distance_2.h>
#include <SFML/Graphics.hpp>
#include <CGAL/intersections.h>
#include <algorithm>
#include <variant>
#include "Types.h"
#include "Constants.h"
#include "CommandManager.h"
#include "GenericDeleteCommand.h"
#include "GUI.h"
#include "Grid.h"
#include "Point.h"
#include "ObjectPoint.h"
#include "ObjectType.h"
#include "GeometricObject.h"
#include "Line.h"
#include "Circle.h"
#include "Intersection.h"
#include "command.h"
#include <memory>
#include <vector>
#include <iostream>
#include "ProjectionUtils.h"
#include "GeometryFactory.h"
#include "HandleEvents.h"

DragMode segmentDragMode = DragMode::None;
EndpointSelection selectedEndpoint = EndpointSelection::None;
sf::Vector2f dragStartPos;

GeometryEditor::GeometryEditor()
	: window(sf::VideoMode(1920, 1080), "Geometry Editor"),
	drawingView(sf::FloatRect(0, 0, 1920, 1080)),
	guiView(sf::FloatRect(0, 0, 1920, 1080)),
	lineCreationPoint(nullptr),
	selectedPoint(nullptr),
	selectedSecondPoint(nullptr),
	selectedLine(nullptr),
	selectedLineSegment(nullptr),
	selectedCircle(nullptr),
	selectedObjectPoint(nullptr) {

	drawingView = window.getDefaultView();
	window.setFramerateLimit(60);
	guiView = sf::View(sf::FloatRect(0, 0, window.getSize().x, window.getSize().y));
	gui.setView(guiView);
	grid = Grid();
}

void GeometryEditor::run() {
	while (window.isOpen()) {
		handleEvents(*this);
		render();
	}
}

void GeometryEditor::handleEvents(GeometryEditor& editor) {
	sf::Event event;
	while (editor.window.pollEvent(event)) {
		switch (event.type) {
		case sf::Event::KeyPressed:
			// Handle key press events
			if (event.key.code == sf::Keyboard::Delete) {
				if (editor.selectedPoint) {
					int pointIndex = -1;
					for (size_t i = 0; i < editor.points.size(); ++i) {
						if (editor.points[i].get() == editor.selectedPoint) {
							pointIndex = static_cast<int>(i);
							break;
						}
					}
					if (pointIndex != -1) {
						auto deleteCmd = std::make_unique<GenericDeleteCommand<Point>>(editor.points, pointIndex);
						editor.commandManager.executeCommand(std::move(deleteCmd));
						editor.selectedPoint = nullptr;
					}
				}
				if (editor.selectedLine) {
					int lineIndex = -1;
					for (size_t i = 0; i < editor.lines.size(); ++i) {
						if (editor.lines[i].get() == editor.selectedLine) {
							lineIndex = static_cast<int>(i);
							break;
						}
					}
					if (lineIndex != -1) {
						auto deleteCmd = std::make_unique<GenericDeleteCommand<Line>>(editor.lines, lineIndex);
						editor.commandManager.executeCommand(std::move(deleteCmd));
						editor.selectedLine = nullptr;
					}
				}
				if (editor.selectedLineSegment) {
					int segmentIndex = -1;
					for (size_t i = 0; i < editor.lines.size(); ++i) {
						if (editor.lines[i].get() == editor.selectedLineSegment) {
							segmentIndex = static_cast<int>(i);
							break;
						}
					}
					if (segmentIndex != -1) {
						auto deleteCmd = std::make_unique<GenericDeleteCommand<Line>>(editor.lines, segmentIndex);
						editor.commandManager.executeCommand(std::move(deleteCmd));
						editor.selectedLineSegment = nullptr;
					}
				}
			}
			// Undo: Ctrl+Z
			if (event.key.code == sf::Keyboard::Z && sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
				editor.commandManager.undo();
			}
			// Redo: Ctrl+Y
			if (event.key.code == sf::Keyboard::Y && sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
				editor.commandManager.redo();
			}
			break;
		case sf::Event::MouseButtonPressed:
			if (event.mouseButton.button == sf::Mouse::Left && (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))) {
				editor.isPanning = true;
				editor.lastMousePos = editor.window.mapPixelToCoords(sf::Mouse::getPosition(editor.window));
			}
			else {
				handleMousePress(editor, event.mouseButton);
			}
			break;
		case sf::Event::MouseButtonReleased:
			if (event.mouseButton.button == sf::Mouse::Left && editor.isPanning) {
				editor.isPanning = false;
			}
			else {
				handleMouseRelease(editor, event.mouseButton);
			}
			break;
		case sf::Event::MouseMoved:
			if (editor.isPanning) {
				handlePanning(editor, event.mouseMove);
			}
			else {
				handleMouseMove(editor, event.mouseMove);
			}
			break;
		case sf::Event::MouseWheelScrolled:
			handleZoom(editor, event.mouseWheelScroll.delta, sf::Vector2i(event.mouseWheelScroll.x, event.mouseWheelScroll.y));
			break;
		case sf::Event::Resized:
			handleResize(editor, event.size.width, event.size.height);
			break;
		case sf::Event::Closed:
			editor.window.close();
			break;
		}
	}
}
sf::Vector2f GeometryEditor::toSFMLVector(const Point_2& point) {
	return sf::Vector2f(
		static_cast<float>(CGAL::to_double(point.x())),
		static_cast<float>(CGAL::to_double(point.y()))
	);
}

Point_2 GeometryEditor::toCGALPoint(const sf::Vector2f& vector) {
	return Point_2(vector.x, vector.y);
}

float GeometryEditor::length(const sf::Vector2f& vec) {
	return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

float GeometryEditor::getScaledTolerance(const sf::View& view) {
	// Get current zoom level from view
	float viewWidth = view.getSize().x;
	float windowWidth = window.getSize().x;
	float zoomFactor = viewWidth / windowWidth;

	// Scale base tolerance with zoom
	float baseTolerance = 8.0f;  // Adjust this base value as needed
	return baseTolerance * zoomFactor;
}
// Helper function to find the nearest point to a given position
Circle* GeometryEditor::findNearbyCircle(const sf::Vector2f& worldPos) {
	for (const auto& circle : circles) {
		if (circle->containsCircumference(worldPos, getScaledTolerance(drawingView))) {
			return circle.get();
		}
	}
	return nullptr;
}

void GeometryEditor::handleGeometryCreation(const sf::Vector2f& worldPos, const sf::Vector2i& pos) {
	// Point creation
	// Check if clicking near existing point
	if (gui.isPointActive()) {
		Point* existingPoint = findNearbyPoint(worldPos); // Use findNearbyPoint
		if (existingPoint) {
			lineCreationPoint = existingPoint;
		}
		else {
			auto newPoint = std::make_unique<Point>(toCGALPoint(worldPos)); // Ensure toCGALPoint is used
			points.push_back(std::move(newPoint));
		}
		return;
	}
	// Line or Line Segment creation
	if (gui.isLineActive() || gui.isLineSegmentActive()) {
		std::cout << "Adding point for line creation at: (" << pos.x << ", " << pos.y << ")" << std::endl;
		Point* nearbyPoint = findNearbyPoint(worldPos);

		if (nearbyPoint) {
			lineCreationPoint = nearbyPoint;
		}
		else {
			auto newPoint = std::make_unique<Point>(toCGALPoint(worldPos));
			lineCreationPoint = newPoint.get();
			points.push_back(std::move(newPoint));
		}

		linePoints.push_back(worldPos);

		if (linePoints.size() == 2) {
			Point_2 start = toCGALPoint(linePoints[0]);
			Point_2 end = toCGALPoint(linePoints[1]);
			bool isSegment = gui.isLineSegmentActive();
			std::cout << "Creating line from: (" << CGAL::to_double(start.x()) << ","
				<< CGAL::to_double(start.y()) << ") to ("
				<< CGAL::to_double(end.x()) << ","
				<< CGAL::to_double(end.y()) << ")" << std::endl;

			/*sf::Vector2f sfStart(static_cast<float>(CGAL::to_double(start.x())), static_cast<float>(CGAL::to_double(start.y())));
			sf::Vector2f sfEnd(static_cast<float>(CGAL::to_double(end.x())), static_cast<float>(CGAL::to_double(end.y())));*/

			//Create new line or segment
			auto newLine = std::make_unique<Line>(toSFMLVector(start), toSFMLVector(end), isSegment);
			// Connect the line to its points
			Point* startPoint = findNearbyPoint(toSFMLVector(start));
			Point* endPoint = findNearbyPoint(toSFMLVector(end));

			if (startPoint) startPoint->addConnectedLine(newLine.get());
			if (endPoint) endPoint->addConnectedLine(newLine.get());
			lines.push_back(std::move(newLine));

			std::cout << "Created " << (isSegment ? "line segment" : "infinite line")
				<< " from (" << CGAL::to_double(start.x()) << ", " << CGAL::to_double(start.y())
				<< ") to (" << CGAL::to_double(end.x()) << ", " << CGAL::to_double(end.y()) << ")" << std::endl;

			// Check for intersections with existing lines
			Line_2 newCGALLine(start, end);
			for (const auto& existingLine : lines) {
				if (existingLine->isSegment()) continue; // Skip segments for intersection testing

				Line_2 existingCGALLine(existingLine->getStartPoint(), existingLine->getEndPoint());
				auto result = CGAL::intersection(newCGALLine, existingCGALLine);

				if (result) {
					if (auto intersectionPoint = findIntersection(newCGALLine, existingCGALLine)) {
						points.push_back(std::make_unique<Point>(*intersectionPoint, Constants::INTERSECTION_POINT_COLOR, true));
						std::cout << "Intersection point created" << std::endl;
					}
				}
			}

			linePoints.clear();
			std::cout << "Total lines in container: " << lines.size() << std::endl;
		}
	}
	if (gui.isCircleActive()) {
		Circle* nearbyCircle = findNearbyCircle(worldPos);
		if (nearbyCircle) {
			sf::Vector2f circleCenter = nearbyCircle->getCenter();
			float circleRadius = nearbyCircle->getRadius();
			sf::Vector2f direction = worldPos - circleCenter;
			float distance = length(direction);
			if (distance <= circleRadius + getScaledTolerance(drawingView)) {
				sf::Vector2f snappedPosition = circleCenter + (direction / distance) * circleRadius;
				auto newObjectPoint = std::make_unique<ObjectPoint>(snappedPosition);
				newObjectPoint->attachToCircle(nearbyCircle);
				ObjectPoints.push_back(std::move(newObjectPoint));
				return;
			}
		}
	}
}
void GeometryEditor::handleDragging(const sf::Vector2f& worldPos) {
	if (isDragging) {
		if (selectedPoint) {
			Point_2 cgalWorldPos = toCGALPoint(worldPos);
			selectedPoint->setPosition(cgalWorldPos);
			selectedPoint->updateConnectedGeometry(worldPos);
		}
		else if (selectedLineSegment || selectedLine) {
			sf::Vector2f offset = worldPos - lastMousePos;
			Vector_2 cgalOffset(offset.x, offset.y); // Convert sf::Vector2f to CGAL::Vector_2
			// Use the stored selectedEndpoint value instead of testing lastMousePos
			if (selectedEndpoint == EndpointSelection::Start) {
				// Drag the start endpoint
				selectedLineSegment->moveEndpointToStart(toCGALPoint(worldPos));
			}
			else if (selectedEndpoint == EndpointSelection::End) {
				// Drag the end endpoint
				selectedLineSegment->moveEndpointToEnd(toCGALPoint(worldPos));
			}
			else {
				// Otherwise translate the whole segment
				if (selectedLineSegment) {
					selectedLineSegment->translate(cgalOffset);
				}
				else if (selectedLine) {
					selectedLine->translate(cgalOffset);
				}
			}
			lastMousePos = worldPos;
		}
		if (selectedCircle) {
			sf::Vector2f offset = worldPos - lastMousePos;
			selectedCircle->move(offset);
			lastMousePos = worldPos;
		}
	}
}

// Implement other methods...
void GeometryEditor::calculateIntersections(const sf::Vector2f& clickPos) {
	std::vector<Line*> nearbyLines;
	for (const auto& line : lines) {
		if (line->containsPoint(clickPos)) {
			nearbyLines.push_back(line.get());
		}
	}

	if (nearbyLines.size() >= 2) {
		for (size_t i = 0; i < nearbyLines.size() - 1; ++i) {
			for (size_t j = i + 1; j < nearbyLines.size(); ++j) {
				sf::Vector2f intersectionPoint;
				if (Line::calculateIntersection(nearbyLines[i], nearbyLines[j], intersectionPoint)) {
					Point_2 cgalIntersectionPoint = toCGALPoint(intersectionPoint);
					auto newPoint = std::make_unique<Point>(cgalIntersectionPoint, Constants::INTERSECTION_POINT_COLOR, true);
					points.push_back(std::move(newPoint));
				}
			}
		}
	}
}
std::optional<Point_2> GeometryEditor::findIntersection(const Line_2& line1, const Line_2& line2) {
	auto result = CGAL::intersection(line1, line2);

	if (result) {
		// Use std::visit to handle the variant
		return std::visit([](auto&& arg) -> std::optional<Point_2> {
			using T = std::decay_t<decltype(arg)>;
			if constexpr (std::is_same_v<T, Point_2>) {
				return arg; // Return the intersection point
			}
			else if constexpr (std::is_same_v<T, Segment_2>) {
				// If the intersection is a segment, return its midpoint
				return CGAL::midpoint(arg.source(), arg.target());
			}
			else {
				return std::nullopt; // No intersection
			}
			}, *result);
	}
	return std::nullopt;
}
// Define the findNearbyPoint method
Point* GeometryEditor::findNearbyPoint(const sf::Vector2f& worldPos) {
	// Implement the logic to find a nearby point
	for (const auto& point : points) {
		if (point->contains(worldPos, getScaledTolerance(drawingView))) {
			return point.get();
		}
	}
	return nullptr;
}
void GeometryEditor::handlePointMovement(Point* point, const sf::Vector2f& newPos) {
	if (point == nullptr) {
		return; // or handle the error appropriately
	}
	point->setPosition(toCGALPoint(newPos)); // Ensure toCGALPoint is used herenewPos);
	point->updateConnectedGeometry(newPos);
}

void GeometryEditor::render() {
	window.clear(sf::Color::White);
	window.setView(drawingView);

	if (gui.isGridActive()) {
		grid.draw(window, drawingView);
	}

	// Draw geometry
	for (const auto& line : lines) {
		if (line) {
			line->draw(window, drawingView);
		}
	}

	for (const auto& pt : points) {
		pt->draw(window);
	}
	for (const auto& circle : circles) {
		circle->draw(window);
	}
	for (const auto& objPoint : ObjectPoints) {
		objPoint->draw(window);
	}
	// Draw GUI last
	window.setView(guiView);
	gui.draw(window);

	window.display();
}
void GeometryEditor::update(sf::Time deltaTime) {
	// Update panning
	if (isPanning) {
		sf::Vector2f panOffset = panningVelocity * deltaTime.asSeconds();
		drawingView.move(panOffset);
		panningVelocity *= dampingFactor;
		if (length(panningVelocity) < 0.1f) {
			isPanning = false;
		}
	}

	// Update points
	for (auto& point : points) {
		point->updatePosition();
	}

	// Update lines
	for (auto& line : lines) {
		line->update();
	}

	// Update circles
	for (auto& circle : circles) {
		circle->update();
	}

	// Update object points
	for (auto& objPoint : ObjectPoints) {
		objPoint->update();
	}
}