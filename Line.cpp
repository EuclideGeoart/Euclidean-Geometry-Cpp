#include "Line.h"
#include "Types.h"
#include "Constants.h"
#include <SFML/Graphics.hpp>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>
#include "ObjectPoint.h"
#include <iostream>
#include <cmath>
#include <vector>
#include "Point.h"
#define _USE_MATH_DEFINES
#include <math.h>

//Line::Line(const sf::Vector2f& start, const sf::Vector2f& end, bool inf)
//    : start(start), end(end), infinite(inf)
//{
//    Point_2 cgalStart(start.x, start.y);
//    Point_2 cgalEnd(end.x, end.y);
//    cgalLine = Line_2(cgalStart, cgalEnd);
//
//    if (infinite) {
//        sf::Vector2f dir = end - start;
//        float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
//        if (length != 0) {
//            dir /= length;
//            drawnStart = start - dir * 10000.f;
//            drawnEnd = start + dir * 10000.f;
//        }
//    }
//    else {
//        drawnStart = start;
//        drawnEnd = end;
//    }
//}

// Constructor for line segment (default)
//Line::Line(const Point_2& start, const Point_2& end)
//	: startPoint(start), endPoint(end), isSegment(true), isSelected(false)
//{
//	updateSFMLVertices();
//}
// Constructor with explicit isSegment flag (true for segment, false for infinite line)
Line::Line(const sf::Vector2f& start, const sf::Vector2f& end, bool isSegment)
	: start(sf::Vector2f(start.x, start.y)),
	end(sf::Vector2f(end.x, end.y)),
	segmentFlag(isSegment),
	startPoint(Point_2(start.x, start.y)),
	endPoint(Point_2(end.x, end.y))
{
	if (segmentFlag) {
		cgalSegment = Segment_2(startPoint, endPoint);
	}
	else {
		cgalLine = Line_2(startPoint, endPoint);
	}
	updateSFMLVertices();
}
Line::Line(Point* start, Point* end, bool isSegment)
	: startPointPtr(start),
	endPointPtr(end),
	segmentFlag(isSegment),
	startPoint(start->getCGALPoint()),
	endPoint(end->getCGALPoint())
{
	if (segmentFlag) {
		cgalSegment = Segment_2(startPoint, endPoint);
	}
	else {
		cgalLine = Line_2(startPoint, endPoint);
	}
	std::cout << "Line created: ("
		<< CGAL::to_double(startPoint.x()) << ", "
		<< CGAL::to_double(startPoint.y()) << ") -> ("
		<< CGAL::to_double(endPoint.x()) << ", "
		<< CGAL::to_double(endPoint.y()) << ")  isSegment=" << segmentFlag << "\n";

	updateSFMLVertices();
}
Line::Line(const Point_2& start, const Point_2& end, bool isSegment)
	: cgalLine(start, end), segmentFlag(isSegment) {
	// Convert Point_2 to sf::Vector2f for SFML rendering
	this->start = sf::Vector2f(CGAL::to_double(start.x()), CGAL::to_double(start.y()));
	this->end = sf::Vector2f(CGAL::to_double(end.x()), CGAL::to_double(end.y()));
	updateSFMLVertices();
}
sf::Vector2f toSFMLVector(const Point_2& point) {
	return sf::Vector2f(
		static_cast<float>(CGAL::to_double(point.x())),
		static_cast<float>(CGAL::to_double(point.y()))
	);
}
void Line::updateSFMLVertices() {
	// Example: convert CGAL points to sf::Vector2f and then add the translationOffset.
	sf::Vector2f sfStart(
		static_cast<float>(CGAL::to_double(startPoint.x())),
		static_cast<float>(CGAL::to_double(startPoint.y()))
	);
	sf::Vector2f sfEnd(
		static_cast<float>(CGAL::to_double(endPoint.x())),
		static_cast<float>(CGAL::to_double(endPoint.y()))
	);
	// Apply the translation offset.
	sfStart += translationOffset;
	sfEnd += translationOffset;

	sf::VertexArray lineVertices(sf::Lines, 2);
	lineVertices[0].position = sfStart;
	lineVertices[1].position = sfEnd;
}
Point_2 toCGALPoint(const sf::Vector2f& vector) {
	return Point_2(vector.x, vector.y);
}
// Add ObjectPoint to the Line
void Line::addObjectPoint(ObjectPoint* objPoint) {
	attachedObjectPoints.push_back(objPoint);
}

// Remove ObjectPoint from the Line
void Line::removeObjectPoint(ObjectPoint* objPoint) {
	attachedObjectPoints.erase(
		std::remove(attachedObjectPoints.begin(), attachedObjectPoints.end(), objPoint),
		attachedObjectPoints.end()
	);
}

// Notify all attached ObjectPoints to update their positions
//void Line::notifyObjectPoints() {
//	for (auto& objPoint : attachedObjectPoints) {
//		objPoint->moveOnObject(toSFMLVector(startPoint)); // Fix for E0415
//	}
//}

void Line::notifyObjectPoints() {
	for (ObjectPoint* objPoint : attachedObjectPoints) {
		if (objPoint) {
			objPoint->moveOnObject(this);
		}
	}
}
void Line::updatePosition(Point* movedPoint, const sf::Vector2f& newPos) {
	Point_2 newCGALPoint(newPos.x, newPos.y);
	if (movedPoint == startPointPtr) {
		startPoint = newCGALPoint;
		if (segmentFlag) {
			cgalSegment = Segment_2(newCGALPoint, endPoint);
		}
		else {
			cgalLine = Line_2(newCGALPoint, endPoint);
		}
	}
	else if (movedPoint == endPointPtr) {
		endPoint = newCGALPoint;
		if (segmentFlag) {
			cgalSegment = Segment_2(startPoint, newCGALPoint);
		}
		else {
			cgalLine = Line_2(startPoint, newCGALPoint);
		}
	}
	updateSFMLVertices();
	notifyObjectPoints();
}


// Remove:  No longer needed, we use CGAL types directly in the constructors
// const Line_2& Line::getLine() const {
//     return cgalLine;
// }
// 
// / Remove: updateGeometry is not needed with the CGAL-based approach
// void Line::updateGeometry() { ... }
// --- Helper function to update SFML vertices (defined here) ---

// --- Helper function to update SFML vertices (Keep) ---

void Line::updateGeometry() {
	// Get the current positions from the connected free points
	startPoint = startPointPtr->getCGALPoint();
	endPoint = endPointPtr->getCGALPoint();

	// (Re)compute the CGAL objects using the free points.
	if (segmentFlag) {
		cgalSegment = Segment_2(startPoint, endPoint);
	}
	else {
		cgalLine = Line_2(startPoint, endPoint);
	}
	// Then update the drawn vertices by applying the translation offset.
	updateSFMLVertices();
	notifyObjectPoints();
}
// override draw function to draw the line
void Line::draw(sf::RenderWindow& window) const {
	sf::VertexArray vertices(sf::Lines, 2);
	vertices[0].position = start;
	vertices[0].color = isSelected ? sf::Color::Red : sf::Color::Black;
	vertices[1].position = end;
	vertices[1].color = isSelected ? sf::Color::Red : sf::Color::Black;

	window.draw(vertices);

	// Optionally, draw endpoints or other decorations
	// ...
}

void Line::draw(sf::RenderWindow& window, const sf::View& view) const {
	// Convert CGAL points to SFML coordinates.
	double sx = CGAL::to_double(startPoint.x());
	double sy = CGAL::to_double(startPoint.y());
	double ex = CGAL::to_double(endPoint.x());
	double ey = CGAL::to_double(endPoint.y());

	sf::Vector2f sfStart(static_cast<float>(sx), static_cast<float>(sy));
	sf::Vector2f sfEnd(static_cast<float>(ex), static_cast<float>(ey));

	// Use a visible color.
	sf::Color drawColor = isSelected ? sf::Color::Red : sf::Color::Black;

	// If this is a line segment, draw it directly.
	if (segmentFlag) {
		// Check for degenerate segment.
		if (std::hypot(sfEnd.x - sfStart.x, sfEnd.y - sfStart.y) < 0.001f) {
			std::cerr << "Line segment degenerate: not drawing.\n";
			return;
		}
		sf::Vertex vertices[2] = {
			sf::Vertex(sfStart, drawColor),
			sf::Vertex(sfEnd, drawColor)
		};
		window.draw(vertices, 2, sf::Lines);
		const sf::View& view = window.getView();
		float scale = static_cast<float>(window.getSize().x) / view.getSize().x;
		float radius = Constants::POINT_SIZE / scale;
		sf::CircleShape circle(radius);
		circle.setOrigin(radius, radius);
		circle.setFillColor(sf::Color::Green);
		circle.setPosition(sfStart);
		window.draw(circle);
		circle.setPosition(sfEnd);
		window.draw(circle);
		if (isSelected) {
			float radius = Constants::POINT_SIZE + 1.f / scale;
			sf::CircleShape circle(radius);
			circle.setOrigin(radius, radius);
			circle.setFillColor(sf::Color(255, 165, 0, 128));
			circle.setPosition(sfStart);
			window.draw(circle);
			circle.setPosition(sfEnd);
			window.draw(circle);
		}
	}
	else {
		// Draw infinite line.
		// Compute direction vector.
		sf::Vector2f direction = sfEnd - sfStart;
		float len = std::sqrt(direction.x * direction.x + direction.y * direction.y);
		if (len < 1e-8f) {
			std::cerr << "Infinite line degenerate: not drawing.\n";
			return;
		}
		direction /= len;
		// Extend well beyond view.
		sf::Vector2f viewSize = view.getSize();
		float extension = std::max(viewSize.x, viewSize.y) * 5.0f;
		sf::Vector2f p1 = sfStart - direction * extension;
		sf::Vector2f p2 = sfStart + direction * extension;

		// Draw the infinite line
		sf::Vertex vertices[2] = {
			sf::Vertex(p1, drawColor),
			sf::Vertex(p2, drawColor)
		};
		window.draw(vertices, 2, sf::Lines);
		// Calculate control points within view bounds
		sf::Vector2f viewCenter = view.getCenter();
		// Draw control points
		const sf::View& view = window.getView();
		float scale = static_cast<float>(window.getSize().x) / view.getSize().x;
		float radius = Constants::POINT_SIZE / scale;
		sf::CircleShape controlCircle(radius);
		controlCircle.setOrigin(radius, radius);
		controlCircle.setFillColor(sf::Color::Magenta);

		controlCircle.setPosition(sfStart);
		window.draw(controlCircle);
		controlCircle.setPosition(sfEnd);
		window.draw(controlCircle);

		window.draw(vertices, 2, sf::Lines);
		if (isSelected) {
			float radius = Constants::POINT_SIZE + 1.f / scale;
			sf::CircleShape circle(radius);
			circle.setOrigin(radius, radius);
			circle.setFillColor(sf::Color(255, 165, 0, 128));
			circle.setPosition(sfStart);
			window.draw(circle);
			circle.setPosition(sfEnd);
			window.draw(circle);
		}
		// Calculate control points within view bounds

	}
}
// Implement isMouseOver method
bool Line::isMouseOver(const sf::Vector2f& pos) const {
	// Calculate distance from point to the line
	float distance = 0.0f;
	float dx = end.x - start.x;
	float dy = end.y - start.y;
	if (dx == 0 && dy == 0) {
		distance = std::sqrt((pos.x - start.x) * (pos.x - start.x) + (pos.y - start.y) * (pos.y - start.y));
	}
	else {
		float t = ((pos.x - start.x) * dx + (pos.y - start.y) * dy) / (dx * dx + dy * dy);
		t = std::max(0.0f, std::min(1.0f, t));
		float projX = start.x + t * dx;
		float projY = start.y + t * dy;
		distance = std::sqrt((pos.x - projX) * (pos.x - projX) + (pos.y - projY) * (pos.y - projY));
	}
	return distance <= Constants::MOUSE_OVER_TOLERANCE;
}

// Implement getAttachedType method
ObjectType Line::getAttachedType() const {
	return ObjectType::Line;
}

// Implement getStatus method
std::string Line::getStatus() const {
	return "Line from (" + std::to_string(start.x) + ", " + std::to_string(start.y) + ") to (" +
		std::to_string(end.x) + ", " + std::to_string(end.y) + ")";
}

// Implement update method
void Line::update() {
	// Update geometry or other properties as needed
	// For example, recalculate CGAL objects based on current points
	if (segmentFlag) {
		cgalSegment = Segment_2(startPoint, endPoint);
	}
	else {
		cgalLine = Line_2(startPoint, endPoint);
	}

	// Notify associated ObjectPoints
	notifyObjectPoints();
}
// Methods to manage ObjectPoints



//    // Get CGAL line components
//    Vector_2 cgalDirection = cgalLine.to_vector();
//    Point_2 cgalPoint = cgalLine.point();

//    // Normalize CGAL direction vector using exact arithmetic
//    double dirLength = std::sqrt(CGAL::to_double(cgalDirection.squared_length()));
//    Vector_2 normalizedDir = cgalDirection / dirLength;

//    // Calculate view extension in CGAL coordinates
//    sf::Vector2f viewSize = view.getSize();
//    double viewDiagonal = std::sqrt(viewSize.x * viewSize.x + viewSize.y * viewSize.y);
//    double extensionLength = viewDiagonal * 3.0;

//    // Calculate extended points using CGAL
//    Point_2 cgalExtendedStart = cgalPoint + (-extensionLength * normalizedDir);
//    Point_2 cgalExtendedEnd = cgalPoint + (extensionLength * normalizedDir);

//    // Convert to SFML for rendering
//    sf::Vector2f extendedStart(
//        static_cast<float>(CGAL::to_double(cgalExtendedStart.x())),
//        static_cast<float>(CGAL::to_double(cgalExtendedStart.y()))
//    );
//    sf::Vector2f extendedEnd(
//        static_cast<float>(CGAL::to_double(cgalExtendedEnd.x())),
//        static_cast<float>(CGAL::to_double(cgalExtendedEnd.y()))
//    );

//    // Draw highlight when selected
//    if (isSelected) {
//        std::cout << "Line selected!" << std::endl;
//        sf::Vertex glowLine[] = {
//            sf::Vertex(extendedStart, sf::Color(255, 165, 0, 128)),
//            sf::Vertex(extendedEnd, sf::Color(255, 165, 0, 128))
//        };
//        window.draw(glowLine, 2, sf::Lines);
//        std::cout << "Infinite line highlighted!" << std::endl;
//    }

//    // Draw main line with color based on selection
//    sf::Color lineColor = isSelected ? sf::Color::Red : Constants::LINE_COLOR;
//    sf::Vertex mainLine[] = {
//        sf::Vertex(extendedStart, lineColor),
//        sf::Vertex(extendedEnd, lineColor)
//    };
//    window.draw(mainLine, 2, sf::Lines);
//}

bool Line::containsPoint(const sf::Vector2f& point) const {
	Point_2 cgalPoint(point.x, point.y);
	if (segmentFlag) {
		Segment_2 segment(startPoint, endPoint);
		return CGAL::squared_distance(segment, cgalPoint) < 10.0; // 5 pixel tolerance squared
	}
	else {
		Line_2 line(startPoint, endPoint);
		return CGAL::squared_distance(line, cgalPoint) < 10.0;
	}
}


void Line::moveEndpointToStart(const Point_2& newStart) {
	startPoint = newStart;
	if (startPointPtr) {
		startPointPtr->setPosition(newStart);
	}
	updateSFMLVertices();
	notifyObjectPoints();
}

void Line::moveEndpointToEnd(const Point_2& newEnd) {
	endPoint = newEnd;
	if (endPointPtr) {
		endPointPtr->setPosition(newEnd);
	}
	updateSFMLVertices();
	notifyObjectPoints();
}

//void Line::translate(const Vector_2& offset) {
//	// Update the line's CGAL geometry.
//	startPoint = startPoint + offset;
//	endPoint = endPoint + offset;
//
//	// Update the associated free points.
//	// We assume these free-point methods update both internal state and any cached positioning.
//	if (startPointPtr) {
//		startPointPtr->setPosition(startPoint);
//		// Optionally, if updateConnectedGeometry is causing conflicts because of shared points,
//		// consider either calling it conditionally or ensuring it uses the current, already-updated position.
//		startPointPtr->updateConnectedGeometry(toSFMLVector(startPoint));
//	}
//	if (endPointPtr) {
//		endPointPtr->setPosition(endPoint);
//		endPointPtr->updateConnectedGeometry(toSFMLVector(endPoint));
//	}
//
//	// Update the vertices used for SFML drawing.
//	updateSFMLVertices();
//}
void Line::translate(const Vector_2& offset) {
	// Update the line's CGAL points
	startPoint = startPoint + offset;
	endPoint = endPoint + offset;

	// Update the associated free points
	if (startPointPtr) {
		startPointPtr->setPosition(startPoint);
	}
	if (endPointPtr) {
		endPointPtr->setPosition(endPoint);
	}

	// Update the SFML vertices
	updateSFMLVertices();
	// Notify ObjectPoints of the translation
	notifyObjectPoints();
	/* for (auto* point : attachedObjectPoints) {
        point->onParentTransformed(); */
}

void Line::setTranslationOffset(const sf::Vector2f& delta) {
	// Accumulate the delta into the translation offset.
	translationOffset += delta;
	updateSFMLVertices();
	notifyObjectPoints();
}
void Line::applyTranslationOffset() {
	// Update the underlying points by adding the offset.
	// This requires that your Point class supports a method to translate its position.
	// For example, assume Point has a translate(const sf::Vector2f&) method.
	if (startPointPtr) {
		// Assuming Point::translate works as expected.
		startPointPtr->translate(translationOffset);
	}
	if (endPointPtr) {
		endPointPtr->translate(translationOffset);
	}
	// Now refresh the geometry from the updated free points.
	updateGeometry();  // Or updateFromPoints() 
	// Reset internal offset.
	translationOffset = sf::Vector2f(0.f, 0.f);
	notifyObjectPoints();
}
void Line::finalizeTranslation() {
	// Apply the accumulated translation offset to the connected points.
	if (startPointPtr)
		startPointPtr->translate(translationOffset);
	if (endPointPtr)
		endPointPtr->translate(translationOffset);

	// Reset the translation offset.
	translationOffset = sf::Vector2f(0.f, 0.f);
	// Finally, re-read geometry from the free points.
	updateGeometry();
	notifyObjectPoints();
}
void Line::updateEndpoints(const Point_2& newEndPoint) {
	endPoint = newEndPoint;
	updateSFMLVertices();
}

/*void Line::updateCGAL() {
	Point_2 cgalStart(start.x, start.y);
	Point_2 cgalEnd(end.x, end.y);
	cgalLine = Line_2(cgalStart, cgalEnd);
}*/


bool Line::containsEndpoint(const sf::Vector2f& point, float threshold) const { // Keep sf::Vector2f for input
	// Convert SFML point to CGAL point for calculations
	Point_2 cgalPoint(point.x, point.y);
	// Use CGAL's squared_distance for efficiency and consistency
	return (CGAL::squared_distance(startPoint, cgalPoint) <= threshold * threshold) ||
		(CGAL::squared_distance(endPoint, cgalPoint) <= threshold * threshold);
}

/*float Line::dotProduct(sf::Vector2f v1, sf::Vector2f v2) const {
	return v1.x * v2.x + v1.y * v2.y;
}

float Line::distance(sf::Vector2f p1, sf::Vector2f p2) const {
	sf::Vector2f diff = p1 - p2;
	return std::sqrt(diff.x * diff.x + diff.y * diff.y);
}*/
/*sf::Vector2f Line::normalize(sf::Vector2f vector) const {
	float length = std::sqrt(vector.x * vector.x + vector.y * vector.y);
	if (length != 0) {
		return sf::Vector2f(vector.x / length, vector.y / length);
	}
	return vector;
}*/

bool Line::calculateIntersection(const Line* l1, const Line* l2, sf::Vector2f& intersection) {
	auto result = CGAL::intersection(Line_2(l1->startPoint, l1->endPoint),
		Line_2(l2->startPoint, l2->endPoint));
	if (result) {
		std::visit([&](auto&& arg) {
			using T = std::decay_t<decltype(arg)>;
			if constexpr (std::is_same_v<T, Point_2>) {
				intersection = sf::Vector2f(static_cast<float>(CGAL::to_double(arg.x())),
					static_cast<float>(CGAL::to_double(arg.y())));
				return true;
			}
			else if constexpr (std::is_same_v<T, Segment_2>) {
				Point_2 midpoint = CGAL::midpoint(arg.source(), arg.target());
				intersection = sf::Vector2f(static_cast<float>(CGAL::to_double(midpoint.x())),
					static_cast<float>(CGAL::to_double(midpoint.y())));
				return true;
			}
			else {
				return false;
			}
			}, *result);
	}
	return false;
}

void Line::setEndpointOnCircle(const sf::Vector2f& center, float radius, bool isEndPoint) {
	sf::Vector2f direction = isEndPoint ? (end - center) : (start - center);
	float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
	if (length != 0) {
		if (isEndPoint) {
			end = center + (direction / length) * radius;
			line[1].position = end;
		}
		else {
			start = center + (direction / length) * radius;
			line[0].position = start;
		}
	}
}
