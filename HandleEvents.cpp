#include "HandleEvents.h"
#include "GeometryEditor.h"
#include "ProjectionUtils.h"
#include "GeometryFactory.h"
#include "Line.h"
#include "Circle.h"
#include "ObjectPoint.h"
#include "Types.h"
#include "Constants.h"
#include "GUI.h"
#include "Point.h"
#include <iostream>

void handleMousePress(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
	// Convert click position into GUI and world (drawing) coordinates.
	sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
	sf::Vector2f guiPos = editor.window.mapPixelToCoords(pixelPos, editor.guiView);
	sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);

	// 1. GUI region: forward event and return.
	if (guiPos.y <= Constants::BUTTON_SIZE.y + 0.7f) {
		sf::Event event;
		event.type = sf::Event::MouseButtonPressed;
		event.mouseButton = mouseEvent;
		editor.gui.handleEvent(editor.window, event);
		return;
	}
	if (editor.gui.isObjPointActive()) {
		const float tolerance = editor.getScaledTolerance(editor.drawingView);
		// Check for intersection with lines
		for (const auto& line : editor.lines) {
			// If the world position is near the line, we want to create an ObjectPoint snapped to the line.
			if (line->containsPoint(worldPos)) {
				Point_2 originalPoint = editor.toCGALPoint(worldPos);
				bool isSegment = line->isSegment();
				Point_2 snappedPoint = ProjectionUtils::projectPointOntoLine(originalPoint, line.get(), isSegment);

				auto newObjPoint = std::make_unique<ObjectPoint>(snappedPoint, line.get());
				line->addObjectPoint(newObjPoint.get()); // Ensure the line keeps a reference
				editor.ObjectPoints.push_back(std::move(newObjPoint));
				return;
			}
		}

		// Check for intersection with circles
		for (const auto& circle : editor.circles) {
			if (circle->containsCircumference(worldPos, editor.getScaledTolerance(editor.drawingView))) {
				// Convert the click worldPos to a CGAL point.
				Point_2 originalPoint = editor.toCGALPoint(worldPos);
				// Use the circle instance for projection
				Point_2 snappedPoint = ProjectionUtils::projectPointOntoCircle(originalPoint, circle->getCenterPoint(), circle->getRadius(), tolerance);
				auto newPoint = std::make_unique<ObjectPoint>(snappedPoint, circle.get(), Constants::POINT_COLOR, true);
				editor.ObjectPoints.push_back(std::move(newPoint));
				return;
			}
		}
	}

	// 2. Creation mode for lines/segments: only one branch for free point creation.
	if (editor.gui.isLineActive() || editor.gui.isLineSegmentActive()) {
		// Check if an existing point exists beneath the cursor.
		Point* clickedFreePoint = nullptr;
		for (auto& point : editor.points) {
			float tolerance = editor.getScaledTolerance(editor.drawingView);
			if (point->contains(worldPos, tolerance)) {  // Do not filter out locked ones!
				clickedFreePoint = point.get();
				break;
			}
		}

		// If no point exists, create a new free point.
		if (!clickedFreePoint) {
			Point_2 newPos = editor.toCGALPoint(worldPos);
			auto newPoint = std::make_unique<Point>(newPos, Constants::POINT_COLOR);
			clickedFreePoint = newPoint.get();
			editor.points.push_back(std::move(newPoint));
		}

		// Use the clicked free point to form a line.
		if (editor.lineCreationPoint == nullptr) {
			// Selecting the first point
			editor.lineCreationPoint = clickedFreePoint;
			editor.lineCreationPoint->setSelected(true);
			std::cout << "Line creation start point selected!\n";
		}
		else if (editor.lineCreationPoint != clickedFreePoint) {
			// Finalize line creation
			auto newLine = std::make_unique<Line>(editor.lineCreationPoint, clickedFreePoint, editor.gui.isLineSegmentActive());
			// Connect the new line to both points
			editor.lineCreationPoint->addConnectedLine(newLine.get());
			clickedFreePoint->addConnectedLine(newLine.get());

			editor.lines.push_back(std::move(newLine));

			// Deselect the first point and select the second point
			editor.lineCreationPoint->setSelected(false);
			clickedFreePoint->setSelected(true); // **Select the second point**
			editor.selectedSecondPoint = clickedFreePoint; // **Track the second point**
			std::cout << "Line creation end point selected!\n";

			// Reset line creation state
			editor.lineCreationPoint = nullptr;
			editor.linePoints.clear();
		}
		// Return from creation mode.
		return;
	}
	if (editor.gui.isCircleActive()) {
		const float tolerance = editor.getScaledTolerance(editor.drawingView);

		// First: try to use an existing free point to serve as center.
		Point* clickedPoint = nullptr;
		for (auto& pt : editor.points) {
			if (pt->contains(worldPos, tolerance)) {
				clickedPoint = pt.get();
				break;
			}
		}
		if (clickedPoint) {
			// If a free point is found, use its position as circle center.
			if (!editor.selectedCircle) {
				auto newCircle = std::make_unique<Circle>(clickedPoint->getPosition());
				editor.selectedCircle = newCircle.get();
				editor.selectedCircle->setSelected(true);
				editor.circles.push_back(std::move(newCircle));
				std::cout << "Circle center set from existing free point.\n";
				return;
			}
			else if (!editor.selectedCircle->isCreated()) {
				// In creation phase: if user clicks using a free point that's away from center,
				// treat it as the radius control.
				if (!editor.selectedCircle->containsCenter(clickedPoint->getPosition(), tolerance)) {
					std::cout << "Create Radius\n";
					editor.selectedCircle->updateRadius(clickedPoint->getPosition());
					editor.selectedCircle->finishCreation();
					// Create a line segment from center to the clicked free point.
					auto newLine = std::make_unique<Line>(editor.selectedCircle->getCenter(), clickedPoint->getPosition(), true);
					editor.lines.push_back(std::move(newLine));
					std::cout << "Circle radius set from free point.\n";
					return;
				}
			}
		}

		// Otherwise, if no free point was clicked, use the current mouse position.
		if (!editor.selectedCircle) {
			// Create a new circle with initial center at the current mouse position.
			auto newCircle = std::make_unique<Circle>(worldPos);
			editor.selectedCircle = newCircle.get();
			editor.selectedCircle->setSelected(true);
			editor.circles.push_back(std::move(newCircle));
			std::cout << "Circle center created at mouse click.\n";
		}
		else {
			// A circle exists. Decide between updating the radius or moving/resizing it.
			if (!editor.selectedCircle->isCreated()) {
				// Check if the mouse click is sufficiently away from the center.
				if (!editor.selectedCircle->containsCenter(worldPos, tolerance)) {
					std::cout << "Create Radius\n";
					editor.selectedCircle->updateRadius(worldPos);
					editor.selectedCircle->finishCreation();
					// Create the radius line – a new line segment from center to the current mouse position.
					auto newLine = std::make_unique<Line>(editor.selectedCircle->getCenter(), worldPos, true);
					editor.lines.push_back(std::move(newLine));
					return;
				}
				else {
					std::cout << "Click near center detected; please click away from the center to set radius.\n";
					return;
				}
			}
			else {
				// Circle is finalized; allow move/resize.
				if (editor.selectedCircle->containsCircumference(worldPos, tolerance)) {
					editor.isResizingCircle = true;
					editor.selectedCircle->setSelected(true);
					std::cout << "Circle selected for resizing.\n";
				}
				else if (editor.selectedCircle->containsCenter(worldPos, tolerance)) {
					editor.isDraggingCircle = true;
					editor.selectedCircle->setSelected(true);
					std::cout << "Circle selected for moving.\n";
				}
			}
		}
		// Return from circle creation mode.
		return;
	}


	if (editor.gui.isLineSegmentActive()) {
		// Check if the user is trying to draw a radius for a circle
		if (editor.selectedCircle && !editor.selectedCircle->hasRadius()) {
			float tolerance = editor.getScaledTolerance(editor.drawingView);
			if (editor.selectedCircle->containsCenter(worldPos, tolerance)) {
				editor.lineCreationPoint = new Point(editor.toCGALPoint(worldPos), sf::Color::Blue);
				editor.points.push_back(std::unique_ptr<Point>(editor.lineCreationPoint));
				std::cout << "Circle center selected for radius drawing.\n";
				editor.gui.displayMessage("Draw Radius");
				return;
			}
		}
	}
	if (editor.gui.isLineSegmentActive() && editor.lineCreationPoint) {
		float tolerance = editor.getScaledTolerance(editor.drawingView);
		if (editor.selectedCircle && editor.selectedCircle->containsCircumference(worldPos, tolerance)) {
			// Finalize the radius line
			editor.selectedCircle->setRadiusPoint(worldPos);
			std::cout << "Radius line finalized.\n";
		}
		editor.lineCreationPoint = nullptr;
	}
	if (editor.gui.isLineSegmentActive() && editor.lineCreationPoint && editor.selectedCircle) {
		float tolerance = editor.getScaledTolerance(editor.drawingView);
		if (editor.selectedCircle->isNearCircumference(worldPos, tolerance)) {
			GeometricObject* parentObject = editor.selectedCircle;
			if (parentObject) {
				auto radiusPoint = std::make_unique<ObjectPoint>(editor.toCGALPoint(worldPos), parentObject, sf::Color::Blue);
				editor.ObjectPoints.push_back(std::move(radiusPoint));
				editor.gui.clearMessage();
				std::cout << "Radius point created and linked to the circle.\n";
			}
		}
	}
	// 3. Allow circle manipulation when no tool is active or when the Move tool is active
	if ((editor.gui.isButtonActive("Move") && mouseEvent.button == sf::Mouse::Left) ||
		(!editor.gui.isAnyToolActive() && mouseEvent.button == sf::Mouse::Left)) {
		float tolerance = editor.getScaledTolerance(editor.drawingView);
		// Look for line endpoints / control points.
		for (auto& line : editor.lines) {
			if (line->isSegment()) {
				sf::Vector2f startPt = editor.toSFMLVector(line->getStartPoint());
				sf::Vector2f endPt = editor.toSFMLVector(line->getEndPoint());
				if (editor.length(worldPos - startPt) <= tolerance) {
					editor.selectedLineSegment = line.get();
					editor.selectedEndpointIndex = 0;
					editor.isDraggingEndpoint = true;
					editor.dragMode = DragMode::MoveEndpoint;
					line->setSelected(true);
					std::cout << "Line segment start endpoint selected for dragging.\n";
					return;
				}
				if (editor.length(worldPos - endPt) <= tolerance) {
					editor.selectedLineSegment = line.get();
					editor.selectedEndpointIndex = 1;
					editor.isDraggingEndpoint = true;
					editor.dragMode = DragMode::MoveEndpoint;
					line->setSelected(true);
					std::cout << "Line segment end endpoint selected for dragging.\n";
					return;
				}
				if (line->containsPoint(worldPos)) {
					editor.selectedLineSegment = line.get();
					// Begin translation mode.
					editor.isDragging = true;
					editor.dragMode = DragMode::TranslateSegment;
					dragStartPos = worldPos; // record initial drag position
					line->setSelected(true);
					std::cout << "Line selected for translation.\n";
					return;
				}
			}
			else { // plain infinite line: check control points.

				sf::Vector2f startPt = editor.toSFMLVector(line->getStartPoint());
				sf::Vector2f endPt = editor.toSFMLVector(line->getEndPoint());
				if (editor.length(worldPos - startPt) <= tolerance) {
					editor.selectedLine = line.get();
					editor.selectedControlPointIndex = 0;
					editor.isDraggingControlPoint = true;
					editor.dragMode = DragMode::MoveControlPoint;
					line->setSelected(true);
					std::cout << "Line start control point selected for dragging.\n";
					return;
				}
				if (editor.length(worldPos - endPt) <= tolerance) {
					editor.selectedLine = line.get();
					editor.selectedControlPointIndex = 1;
					editor.isDraggingControlPoint = true;
					editor.dragMode = DragMode::MoveControlPoint;
					line->setSelected(true);
					std::cout << "Line end control point selected for dragging.\n";
					return;
				}
				// If not near control points, check if click is in the body for translation.
				if (line->containsPoint(worldPos)) {
					editor.selectedLine = line.get();
					editor.isDragging = true;
					editor.dragMode = DragMode::TranslateSegment;
					dragStartPos = worldPos;
					line->setSelected(true);
					std::cout << "Line selected for translation.\n";
					return;
				}
			}
		}
	}
	// Check free points for moving.
	for (auto& point : editor.points) {
		if (point->contains(worldPos)) {  // do not filter out locked ones here either if you want to reuse them
			editor.selectedPoint = point.get();
			editor.isDragging = true;
			point->setSelected(true);
			std::cout << "Free point selected for moving.\n";
			return;
		}
	}
	// Check circles for moving or resizing.
	for (auto& circle : editor.circles) {
		float tolerance = editor.getScaledTolerance(editor.drawingView);
		if (circle->containsCenter(worldPos, tolerance)) {
			editor.selectedCircle = circle.get();
			editor.isDraggingCircle = true;
			editor.selectedCircle->setSelected(true);
			std::cout << "Circle selected for moving.\n";
			return;
		}
		if (circle->containsCircumference(worldPos, tolerance)) {
			editor.selectedCircle = circle.get();
			editor.isResizingCircle = true;
			editor.selectedCircle->setSelected(true);
			std::cout << "Circle selected for resizing.\n";
			return;
		}
	}
	if (editor.gui.isObjPointActive()) {
		bool objectFound = false;
		float tolerance = editor.getScaledTolerance(editor.drawingView); // Define tolerance here
		// Try to attach the new point to a line first.
		for (auto& line : editor.lines) {
			// Here we use the line's containsPoint method, or you could implement a
			// distance check. Adjust the method as needed:
			if (line->containsPoint(worldPos)) {
				// Create an ObjectPoint attached to this line.
				auto newObjPoint = std::make_unique<ObjectPoint>(editor.toCGALPoint(worldPos), line.get());
				// You might keep these in a dedicated container; if not, simply push into points.
				editor.points.push_back(std::move(newObjPoint));
				std::cout << "Created ObjectPoint on line.\n";
				objectFound = true;
				break;
			}
		}
		// If not attached to a line, check circles.
		if (!objectFound) {
			for (auto& circle : editor.circles) {
				// Here we check if the click is near the circumference.
				if (circle->containsCircumference(worldPos, tolerance)) {
					auto newObjPoint = std::make_unique<ObjectPoint>(worldPos, circle.get());
					editor.points.push_back(std::move(newObjPoint));
					std::cout << "Created ObjectPoint on circle.\n";
					objectFound = true;
					break;
				}
			}
		}
		if (!objectFound) {
			std::cout << "No object within tolerance to attach ObjectPoint.\n";
		}
		return;
	}
	// 4. Other creation modes (point or intersection creation).
	if (editor.gui.isPointActive() || editor.gui.isIntersectionActive()) {
		editor.handleGeometryCreation(worldPos, pixelPos);
		return;
	}

	// 5. Default selection: if no creation or move tool is active.
	for (auto& line : editor.lines) {
		if (line->containsPoint(worldPos)) {
			editor.selectedLineSegment = line.get();
			editor.selectedLineSegment->setSelected(true);
			editor.selectedLine = line.get();
			editor.selectedLine->setSelected(true);
			editor.isDragging = true;
			std::cout << (line->isSegment() ? "Line segment" : "Infinite line")
				<< " selected!\n";
			return;
		}
	}
	for (auto& point : editor.points) {
		if (point->contains(worldPos, editor.getScaledTolerance(editor.drawingView))) {
			editor.selectedPoint = point.get();
			editor.selectedPoint->setSelected(true);
			editor.isDragging = true;
			std::cout << "Free point selected.\n";
			return;
		}
	}
	for (auto& objPoint : editor.ObjectPoints) {
		if (objPoint->contains(worldPos, editor.getScaledTolerance(editor.drawingView))) {
			editor.selectedObjectPoint = objPoint.get();
			editor.isDraggingObjectPoint = true;
			std::cout << "ObjectPoint selected for moving.\n";
			return;
		}
	}
	// Fallback for intersection mode.
	if (editor.gui.isIntersectionActive()) {
		editor.calculateIntersections(worldPos);
	}
}

void handleMouseMove(GeometryEditor& editor, const sf::Event::MouseMoveEvent& moveEvent) {
	sf::Vector2i pixelPos(moveEvent.x, moveEvent.y);
	sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
	sf::Vector2f mousePos = editor.window.mapPixelToCoords(sf::Mouse::getPosition(editor.window));

	if (editor.selectedPoint != nullptr) {
		if (editor.gui.isMoveActive() || !editor.gui.isAnyToolActive()) {
			editor.selectedPoint->setPosition(editor.toCGALPoint(worldPos));
		}
	}

	if (editor.isDraggingObjectPoint && editor.selectedObjectPoint) {
		Point_2 newPos = editor.toCGALPoint(worldPos);

		if (editor.selectedObjectPoint->getAttachedType() == ObjectType::Line) {
			Line* parentLine = editor.selectedObjectPoint->getAttachedLine();
			Point_2 projectedPos = ProjectionUtils::projectPointOntoLine(
				newPos,
				parentLine,
				parentLine->isSegment()
			);
			editor.selectedObjectPoint->setPosition(projectedPos);
		}
		else if (editor.selectedObjectPoint->getAttachedType() == ObjectType::Circle) {
			Circle* parentCircle = editor.selectedObjectPoint->getAttachedCircle();
			Point_2 projectedPos = ProjectionUtils::projectPointOntoCircle(
				newPos,
				parentCircle->getCenterPoint(),
				parentCircle->getRadius(),
				editor.getScaledTolerance(editor.drawingView)
			);
			editor.selectedObjectPoint->setPosition(projectedPos);
		}
		std::cout << "ObjectPoint moved to: (" << worldPos.x << ", " << worldPos.y << ")\n";
	}
	if (editor.isDraggingEndpoint && editor.selectedLineSegment && editor.dragMode == DragMode::MoveEndpoint) {
		if (editor.selectedEndpointIndex == 0) {
			editor.selectedLineSegment->moveEndpointToStart(editor.toCGALPoint(worldPos));
			if (editor.selectedLineSegment->getStartPointPtr()) {
				editor.selectedLineSegment->getStartPointPtr()->setPosition(editor.toCGALPoint(worldPos));
			}
		}
		else {
			editor.selectedLineSegment->moveEndpointToEnd(editor.toCGALPoint(worldPos));
			if (editor.selectedLineSegment->getEndPointPtr()) {
				editor.selectedLineSegment->getEndPointPtr()->setPosition(editor.toCGALPoint(worldPos));
			}
		}
	}
	else if (editor.isDraggingControlPoint && editor.selectedLine && editor.dragMode == DragMode::MoveControlPoint) {
		if (editor.selectedControlPointIndex == 0) {
			editor.selectedLine->moveEndpointToStart(editor.toCGALPoint(worldPos));
			if (editor.selectedLine->getStartPointPtr()) {
				editor.selectedLine->getStartPointPtr()->setPosition(editor.toCGALPoint(worldPos));
			}
		}
		else {
			editor.selectedLine->moveEndpointToEnd(editor.toCGALPoint(worldPos));
			if (editor.selectedLine->getEndPointPtr()) {
				editor.selectedLine->getEndPointPtr()->setPosition(editor.toCGALPoint(worldPos));
			}
		}
	}
	else if (editor.isDragging && editor.dragMode == DragMode::TranslateSegment) {
		// Compute the delta since the last event
		sf::Vector2f delta = worldPos - dragStartPos;
		Vector_2 cgalDelta(delta.x, delta.y); // Convert sf::Vector2f to CGAL::Vector_2

		if (editor.selectedLineSegment) {
			/*selectedLineSegment->applyTranslationOffset();
			selectedLineSegment->setTranslationOffset(delta);*/
			editor.selectedLineSegment->translate(cgalDelta);
			for (auto& objPoint : editor.ObjectPoints) {
				if (objPoint->getAttachedLine() == editor.selectedLineSegment) {
					objPoint->moveOnObject(editor.selectedLineSegment);
				}
			}
		}
		else if (editor.selectedLine) {
			/*selectedLine->applyTranslationOffset();
			selectedLine->setTranslationOffset(delta);*/
			editor.selectedLine->translate(cgalDelta);
			// Update attached ObjectPoints
			for (auto& objPoint : editor.ObjectPoints) {
				if (objPoint->getAttachedLine() == editor.selectedLine) {
					objPoint->moveOnObject(editor.selectedLine);
				}
			}
		}
		dragStartPos = worldPos; // update this for the next frame
	}
	// In handleMouseMove:
	else if (editor.gui.isCircleActive() && editor.selectedCircle) {
		// If the circle is still in creation phase, update the radius as the mouse moves.
		if (!editor.selectedCircle->isCreated()) {
			editor.selectedCircle->updateRadius(worldPos);
			std::cout << "Updating circle radius via drag.\n";
		}
		else if (editor.isResizingCircle) {
			// Update the circle's radius based on mouse distance from its center.
			float newRadius = std::sqrt(std::pow(worldPos.x - editor.selectedCircle->getCenter().x, 2) +
				std::pow(worldPos.y - editor.selectedCircle->getCenter().y, 2));
			editor.selectedCircle->setRadius(newRadius);
			std::cout << "Resizing circle to radius: " << newRadius << "\n";
		}
		else if (editor.isDraggingCircle) {
			// Move the circle; compute offset relative to the last mouse position.
			sf::Vector2f offset = worldPos - editor.lastMousePos;
			editor.selectedCircle->move(offset);
			std::cout << "Moving circle; new center: (" << editor.selectedCircle->getCenter().x
				<< ", " << editor.selectedCircle->getCenter().y << ")\n";
		}
	}
	// Handle circle dragging or resizing
	if (editor.isDraggingCircle && editor.selectedCircle) {
		sf::Vector2f offset = worldPos - editor.lastMousePos;
		editor.selectedCircle->move(offset);
		// Update attached ObjectPoints
		for (auto& objPoint : editor.ObjectPoints) {
			if (objPoint->getAttachedCircle() == editor.selectedCircle) {
				// Use the GeometricObject* version of moveOnObject
				GeometricObject* circleObj = editor.selectedCircle;
				objPoint->moveOnObject(circleObj);
			}
		}
		editor.lastMousePos = worldPos;
	}
	else if (editor.isResizingCircle && editor.selectedCircle) {
		float newRadius = std::sqrt(std::pow(worldPos.x - editor.selectedCircle->getCenter().x, 2) +
			std::pow(worldPos.y - editor.selectedCircle->getCenter().y, 2));
		editor.selectedCircle->setRadius(newRadius);
		std::cout << "Resizing circle to radius: " << newRadius << "\n";
	}

	if (editor.gui.isLineSegmentActive() && editor.lineCreationPoint && editor.selectedCircle) {
		float tolerance = editor.getScaledTolerance(editor.drawingView); // Declare tolerance here
		if (editor.selectedCircle->isNearCircumference(worldPos, tolerance)) {
			// Ensure tolerance is declared
			// (Already declared above)
			// Additional logic if needed
		}
		else {
			editor.gui.clearMessage();
		}
	}

	editor.lastMousePos = worldPos;
}

void handleMouseRelease(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent) {
	sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
	sf::Vector2f worldPos = editor.window.mapPixelToCoords(pixelPos, editor.drawingView);
	sf::Vector2f mousePos = editor.window.mapPixelToCoords(sf::Mouse::getPosition(editor.window));
	sf::Vector2f delta = worldPos - dragStartPos;
	Vector_2 cgalDelta(delta.x, delta.y);
	if (editor.isDraggingEndpoint && editor.selectedLineSegment) {
		editor.selectedLineSegment->setSelected(false);
		editor.selectedLineSegment = nullptr;
		editor.isDraggingEndpoint = false;
		editor.dragMode = DragMode::None;
		editor.selectedEndpointIndex = -1;
		std::cout << "Line segment endpoint dragging finished.\n";
	}

	if (editor.isDraggingControlPoint && editor.selectedLine) {
		editor.selectedLine->setSelected(false);
		editor.selectedLine = nullptr;
		editor.isDraggingControlPoint = false;
		editor.dragMode = DragMode::None;
		editor.selectedControlPointIndex = -1;
		std::cout << "Line control point dragging finished.\n";
	}

	if (editor.selectedSecondPoint) {
		editor.selectedSecondPoint->setSelected(false);
		std::cout << "Second point deselected after line creation.\n";
		editor.selectedSecondPoint = nullptr;
	}

	if (editor.dragMode == DragMode::TranslateSegment && editor.selectedLineSegment) {
		//editor.selectedLineSegment->applyTranslationOffset();
		//editor.selectedLineSegment->setTranslationOffset(sf::Vector2f(0.f, 0.f)); // Ensure translation is finalized
		editor.selectedLineSegment->translate(cgalDelta);
		editor.selectedLineSegment->setSelected(false);
		editor.selectedLineSegment = nullptr;
	}

	if (editor.dragMode == DragMode::TranslateSegment && editor.selectedLine) {
		/*editor.selectedLine->applyTranslationOffset();
		editor.selectedLine->setTranslationOffset(sf::Vector2f(0.f, 0.f));*/ // Ensure translation is finalized
		editor.selectedLine->translate(cgalDelta);
		editor.selectedLine->setSelected(false);
		editor.selectedLine = nullptr;
	}

	if (editor.isDragging && editor.selectedPoint) {
		editor.selectedPoint->setSelected(false);
		editor.selectedPoint = nullptr;
		editor.isDragging = false;
		std::cout << "Free point dragging finished.\n";
	}

	if (editor.gui.isCircleActive() && editor.selectedCircle) {
		// If the circle has not been finalized (i.e. creation phase), finish its creation:
		if (!editor.selectedCircle->isCreated()) {
			editor.selectedCircle->finishCreation();
			std::cout << "Circle creation finished.\n";
		}
		if (editor.isDraggingCircle || editor.isResizingCircle) {
			editor.selectedCircle->setSelected(false);
			editor.isDraggingCircle = false;
			editor.isResizingCircle = false;
			editor.selectedCircle = nullptr; // Optionally reset selection; or allow further editing
			std::cout << "Circle editing finished.\n";
		}
	}

	if (editor.isDraggingCircle || editor.isResizingCircle) {
		editor.selectedCircle->setSelected(false);
		editor.isDraggingCircle = false;
		editor.isResizingCircle = false;
		editor.selectedCircle = nullptr;
		std::cout << "Circle editing finished.\n";
	}

	if (editor.isDraggingObjectPoint && editor.selectedObjectPoint) {
		editor.selectedObjectPoint->setSelected(false);
		editor.selectedObjectPoint = nullptr;
		editor.isDraggingObjectPoint = false;
		std::cout << "ObjectPoint movement finished.\n";
	}

	// **Deselect the second point after line creation**
	// Ensure that the second point is deselected here
	// If you have a reference to the second point, ensure it's deselected
	// Example:
	// if (clickedFreePoint) {
	//     clickedFreePoint->setSelected(false);
	// }

	// However, since `clickedFreePoint` is local to `handleMousePress`,
	// you can manage deselection based on other tracking variables if necessary.
}
void handlePanning(GeometryEditor& editor, const sf::Event::MouseMoveEvent& mouseEvent) {
	if (editor.isPanning) {
		sf::Time deltaTime = editor.panClock.restart();
		float smoothFactor = deltaTime.asSeconds() * 60.0f; // Normalize to 60 FPS

		sf::Vector2f currentMousePos = editor.window.mapPixelToCoords(
			sf::Vector2i(mouseEvent.x, mouseEvent.y), editor.drawingView);
		if (editor.isFirstPanFrame) {
			editor.lastMousePos = currentMousePos;
			editor.isFirstPanFrame = false;
			return;
		}

		sf::Vector2f delta = (editor.lastMousePos - currentMousePos) * editor.panSpeed * smoothFactor;
		const float maxDelta = 100.0f;
		delta.x = std::clamp(delta.x, -maxDelta, maxDelta);
		delta.y = std::clamp(delta.y, -maxDelta, maxDelta);
		editor.drawingView.move(delta);
		editor.window.setView(editor.drawingView);

		editor.lastMousePos = editor.window.mapPixelToCoords(
			sf::Vector2i(mouseEvent.x, mouseEvent.y),
			editor.drawingView
		);
	}
}

void handleZoom(GeometryEditor& editor, float delta, const sf::Vector2i& mousePos) {
	sf::Vector2f beforeZoom = editor.window.mapPixelToCoords(mousePos, editor.drawingView);
	float factor = (delta < 0) ? Constants::ZOOM_FACTOR : 1.0f / Constants::ZOOM_FACTOR;
	editor.drawingView.zoom(factor);
	editor.window.setView(editor.drawingView);
	sf::Vector2f afterZoom = editor.window.mapPixelToCoords(mousePos, editor.drawingView);
	sf::Vector2f offset = beforeZoom - afterZoom;
	editor.drawingView.move(offset);
	editor.window.setView(editor.drawingView);
	editor.grid.draw(editor.window, editor.drawingView);
}

void handleResize(GeometryEditor& editor, unsigned int width, unsigned int height) {
	// Update the window's viewport to the new size
	sf::FloatRect visibleArea(0, 0, width, height);
	editor.window.setView(sf::View(visibleArea));

	// Adjust the drawingView to maintain the aspect ratio and prevent stretching
	sf::Vector2f center = editor.drawingView.getCenter();
	sf::Vector2f size = editor.drawingView.getSize();
	float aspectRatio = static_cast<float>(width) / height;

	if (aspectRatio > size.x / size.y) {
		// Window is wider than the view, adjust height
		size.y = size.x / aspectRatio;
	}
	else {
		// Window is taller than the view, adjust width
		size.x = size.y * aspectRatio;
	}

	editor.drawingView.setSize(size);
	editor.drawingView.setCenter(center);
	editor.window.setView(editor.drawingView);

	// Adjust the guiView to match the new window size
	editor.guiView.setSize(width, height);
	editor.guiView.setCenter(width / 2.0f, height / 2.0f);
}

void handleEvents(GeometryEditor& editor) {
	sf::Event event;
	while (editor.window.pollEvent(event)) {
		switch (event.type) {
		case sf::Event::KeyPressed:
			// Handle key press events
			break;
		case sf::Event::MouseButtonPressed:
			handleMousePress(editor, event.mouseButton);
			break;
		case sf::Event::MouseButtonReleased:
			handleMouseRelease(editor, event.mouseButton);
			break;
		case sf::Event::MouseMoved:
			handleMouseMove(editor, event.mouseMove);
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
