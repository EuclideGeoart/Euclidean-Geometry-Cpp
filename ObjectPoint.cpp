// ObjectPoint.cpp
#include "ObjectPoint.h"
#include "Line.h"
#include "Circle.h"
#include "Point.h"
#include <cmath>
#include "Types.h"
#include "ProjectionUtils.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Line_2.h>


// Constructor for attaching to a Line
ObjectPoint::ObjectPoint(const Point_2& pos, Line* lineObj)
	: Point(pos, Constants::POINT_COLOR, true),
	Selected(false),
	attachedType(ObjectType::Line),
	attachedLine(lineObj),
	attachedCircle(nullptr),
	parentLine(lineObj),
	parentCircle(nullptr),
	parentObject(nullptr)
{
	if (parentLine) {
		parentLine->addObjectPoint(this);
		
		// Compute the relative position along the line
		Point_2 A = parentLine->getStartPoint();
		Point_2 B = parentLine->getEndPoint();
		double Ax = CGAL::to_double(A.x());
		double Ay = CGAL::to_double(A.y());
		double Bx = CGAL::to_double(B.x());
		double By = CGAL::to_double(B.y());
		double Px = CGAL::to_double(pos.x());
		double Py = CGAL::to_double(pos.y());
		double ABx = Bx - Ax, ABy = By - Ay;
		double APx = Px - Ax, APy = Py - Ay;
		double lenSq = ABx * ABx + ABy * ABy;
		relativePosition = (lenSq == 0.0) ? 0.0 : (ABx * APx + ABy * APy) / lenSq;
		relativePosition = std::clamp(relativePosition, 0.0, 1.0);
	}
}

// Constructor for attaching to a Circle
// Constructor for attaching to a Circle
ObjectPoint::ObjectPoint(const Point_2& pos, Circle* circleObj, sf::Color color)
	: Point(pos, color, true),
	Selected(false),
	attachedType(ObjectType::Circle),
	attachedLine(nullptr),
	attachedCircle(circleObj),
	parentLine(nullptr),
	parentCircle(circleObj),
	parentObject(nullptr)
{
	if (attachedCircle) {
		// Compute the angle between the point and the circle's center.
		Point_2 c = attachedCircle->getCenterPoint();
		double dx = CGAL::to_double(pos.x()) - CGAL::to_double(c.x());
		double dy = CGAL::to_double(pos.y()) - CGAL::to_double(c.y());
		circleAngle = std::atan2(dy, dx);
		attachedCircle->addObjectPoint(this);
	}
}

// Constructor without attachments
ObjectPoint::ObjectPoint(const Point_2& position, const sf::Color& color, bool isLocked)
	: Point(position, color, isLocked),
	Selected(false),
	attachedType(ObjectType::None),
	attachedLine(nullptr),
	attachedCircle(nullptr),
	parentLine(nullptr),
	parentCircle(nullptr),
	parentObject(nullptr)
{
	// Initialization code...
}

// Constructor with parent GeometricObject
ObjectPoint::ObjectPoint(const Point_2& position, GeometricObject* parentObj, const sf::Color& color, bool isOnCircle)
	: Point(position, color, true), parentObject(parentObj), isOnCircle(isOnCircle) {
	Selected = false;
	attachedType = ObjectType::None;
	attachedLine = nullptr;
	attachedCircle = nullptr;
	parentLine = nullptr;
	parentCircle = nullptr;

	shape.setRadius(5.f);
	shape.setFillColor(color);
	shape.setOrigin(5.f, 5.f);
	shape.setPosition(sf::Vector2f(CGAL::to_double(position.x()), CGAL::to_double(position.y())));
}


// Use the functions from ProjectionUtils
Point_2 ObjectPoint::projectPointOntoLine(const Point_2& p, const Line_2& line, bool isSegment) {
	return ProjectionUtils::projectPointOntoLine(p, line, isSegment);
}

Point_2 ObjectPoint::projectPointOntoCircle(const Point_2& p, const Point_2& center, double radius, double tolerance) {
	return ProjectionUtils::projectPointOntoCircle(p, center, radius, tolerance);
}

// Methode to draw the point
void ObjectPoint::draw(sf::RenderWindow& window) const {
	const sf::View& view = window.getView();
	float scale = static_cast<float>(window.getSize().x) / view.getSize().x;
	// For a fixed size in screen pixels, divide constant by scale:
	float radius = Constants::POINT_SIZE / scale;
	// Example: draw a small circle at the point location.
	sf::CircleShape pointShape(radius);
	pointShape.setOrigin(radius, radius);


	// Use the position member from the Point base class
	pointShape.setPosition(static_cast<float>(CGAL::to_double(getPositionCGAL().x())) - pointShape.getRadius(),
		static_cast<float>(CGAL::to_double(getPositionCGAL().y())) - pointShape.getRadius());
	window.draw(pointShape);
	pointShape.setFillColor(sf::Color::Red);
	window.draw(pointShape);
}

bool ObjectPoint::isMouseOver(const sf::Vector2f& pos) const {
	sf::Vector2f pointPos(static_cast<float>(CGAL::to_double(getPosition().x())),
		static_cast<float>(CGAL::to_double(getPosition().y())));
	float distance = std::sqrt(std::pow(pos.x - pointPos.x, 2) + std::pow(pos.y - pointPos.y, 2));
	return distance <= 30.0f; // Example tolerance
}
// Implement getAttachedType()
ObjectType ObjectPoint::getAttachedType() const {
	return attachedType;
}

// Implement other member functions...

// Helper: Compute dot product of two vectors.
static float dot(const sf::Vector2f& a, const sf::Vector2f& b) {
	return a.x * b.x + a.y * b.y;
}

// Helper: Subtract two vectors.
static sf::Vector2f subtract(const sf::Vector2f& a, const sf::Vector2f& b) {
	return sf::Vector2f(a.x - b.x, a.y - b.y);
}

// Helper: Multiply vector by scalar.
static sf::Vector2f multiply(const sf::Vector2f& v, float scalar) {
	return sf::Vector2f(v.x * scalar, v.y * scalar);
}

// Helper: Add two vectors.
static sf::Vector2f add(const sf::Vector2f& a, const sf::Vector2f& b) {
	return sf::Vector2f(a.x + b.x, a.y + b.y);
}
//void ObjectPoint::update() {
//	if (attachedType == ObjectType::Circle && attachedCircle) {
//		// GeoGebra logic: Recompute the point position based on the circle's center, radius, and the stored angle.
//		Point_2 center = attachedCircle->getCenterPoint();
//		double r = attachedCircle->getRadius();
//		double newX = CGAL::to_double(center.x()) + r * std::cos(circleAngle);
//		double newY = CGAL::to_double(center.y()) + r * std::sin(circleAngle);
//		setPosition(Point_2(newX, newY));
//	}
//	else if (attachedType == ObjectType::Circle && attachedCircle) {
//		moveOnObjectCircleSf(attachedCircle->getCenter());
//	}
//
//}

//bool ObjectPoint::contains(const sf::Vector2f& pos, float tolerance) const {
//	sf::Vector2f pointPos(static_cast<float>(CGAL::to_double(getPosition().x())),
//		static_cast<float>(CGAL::to_double(getPosition().y())));
//	float distance = std::sqrt(std::pow(pos.x - pointPos.x, 2) + std::pow(pos.y - pointPos.y, 2));
//	return distance <= tolerance;
//}

bool ObjectPoint::contains(const sf::Vector2f& pos, float tolerance) const {
	sf::Vector2f shapePos = shape.getPosition();
	float dx = pos.x - shapePos.x;
	float dy = pos.y - shapePos.y;
	return (dx * dx + dy * dy) <= 2 * (tolerance * tolerance);
}
// Correctly retrieve the position from the base class
	// Add a conversion function from sf::Vector2f to Point_2

// Modify the return statement to use the conversion function
Point_2 ObjectPoint::getPosition() const {
	return toCGALPoint(this->Point::getPosition());
}
void ObjectPoint::attachToCircle(Circle* circle) {
	attachedCircle = circle;
	if (circle) {
		// Convert CGAL Point_2 to SFML Vector2f for subtraction
		sf::Vector2f posSFML = CGALToSFML(position);
		sf::Vector2f centerSFML = (circle->getCenter());
		offsetFromCenter = posSFML - centerSFML;
		circle->addObjectPoint(this);
	}
}

void ObjectPoint::setPosition(const sf::Vector2f& newPos) {
	Point_2 cgalPos(newPos.x, newPos.y);
	setPosition(cgalPos);
}

void ObjectPoint::setPosition(const Point_2& newPos) {
	this->Point::setPosition(newPos);
	shape.setPosition(sf::Vector2f(CGAL::to_double(newPos.x()), CGAL::to_double(newPos.y())));
}


sf::Vector2f ObjectPoint::getOffsetFromCenter() const {
	return offsetFromCenter;
}
void ObjectPoint::setSelected(bool selected) {
	this->selected = selected;
}
void ObjectPoint::updateConnectedGeometry(const sf::Vector2f& newPos) {
	// Update connected lines or other geometry based on the new position
	if (attachedLine) {
		attachedLine->updateEndpoints(getPosition());
	}
	if (attachedCircle) {
		attachedCircle->update();
	}
	if (parentObject) {
		parentObject->update();
	}
}
std::string ObjectPoint::getStatus() const {
	switch (attachedType) {
	case ObjectType::Line:
		return "Attached to Line";
	case ObjectType::Circle:
		return "Attached to Circle";
	default:
		return "Standalone Point";
	}
}
// Helper: Convert a CGAL Point_2 to an SFML Vector2f.
//static sf::Vector2f toSFMLVector(const Point_2& point) {
//    return sf::Vector2f(static_cast<float>(CGAL::to_double(point.x())), static_cast<float>(CGAL::to_double(point.y())));
//}


 //Implementation for Line
 void ObjectPoint::moveOnObject(Line* parentLine) {
	if (!parentLine) {
		return;
	}
	Point_2 A = parentLine->getStartPoint();
	Point_2 B = parentLine->getEndPoint();
	double Ax = CGAL::to_double(A.x());
	double Ay = CGAL::to_double(A.y());
	double Bx = CGAL::to_double(B.x());
	double By = CGAL::to_double(B.y());

	if (parentLine->isSegment()) {
		// When attached to a line segment, update based on stored relative position.
		double newX = Ax + relativePosition * (Bx - Ax);
		double newY = Ay + relativePosition * (By - Ay);
		Point_2 newPos(newX, newY);
		setPosition(newPos);
	}
	else {
		// For infinite lines, calculate the projection of the current position onto the line.
		double ABx = Bx - Ax, ABy = By - Ay;
		double len = std::sqrt(ABx * ABx + ABy * ABy);
		if (len == 0.0) {
			return; // Avoid division by zero if the line is degenerate.
		}
		ABx /= len;
		ABy /= len;

		// Calculate the new position based on the relative position along the infinite line.
		double newX = Ax + relativePosition * ABx * len;
		double newY = Ay + relativePosition * ABy * len;
		Point_2 newPos(newX, newY);
		setPosition(newPos);
	}
}

 /* void ObjectPoint::updateRelativePosition() {
    if (!parentLine) return;
    
    Point_2 A = parentLine->getStartPoint();
    Point_2 B = parentLine->getEndPoint();
    Point_2 P = getPosition();
    
    double Ax = CGAL::to_double(A.x());
    double Ay = CGAL::to_double(A.y());
    double Bx = CGAL::to_double(B.x());
    double By = CGAL::to_double(B.y());
    double Px = CGAL::to_double(P.x());
    double Py = CGAL::to_double(P.y());
    
    double ABx = Bx - Ax;
    double ABy = By - Ay;
    double APx = Px - Ax;
    double APy = Py - Ay;
    
    double lenSq = ABx * ABx + ABy * ABy;
    if (lenSq > 0) {
        relativePosition = (ABx * APx + ABy * APy) / lenSq;
        if (parentLine->isSegment()) {
            relativePosition = std::clamp(relativePosition, 0.0, 1.0);
        }
    }
} */

//void ObjectPoint::moveOnObject(Line* parentLine) {
//	if (!parentLine) return;
//	Point_2 A = parentLine->getStartPoint();
//	Point_2 B = parentLine->getEndPoint();
//	Line_2 line(A, B);
//	Point_2 newPos = ProjectionUtils::projectPointOntoLine(position, line, parentLine->isSegment());
//	setPosition(newPos);
//}

void ObjectPoint::moveOnObject(Circle* parentCircle) {
	if (parentCircle) {
		Point_2 center = parentCircle->getCenterPoint();
		float radius = parentCircle->getRadius();
		Point_2 currentPos = getPosition();
		Point_2 newPos = ProjectionUtils::projectPointOntoCircle(currentPos, center, radius, 1.0f); // Adjust tolerance as needed
		setPosition(newPos);
	}
}
//Move ObjectPoint to a specific position
void ObjectPoint::moveOnObject(const sf::Vector2f& newPos) {
	// Convert SFML vector to CGAL point
	Point_2 cgalPos(newPos.x, newPos.y);

	// Update the position
	this->setPosition(cgalPos);
}

void ObjectPoint::moveOnObject(GeometricObject* parentObj) {
	if (auto* line = dynamic_cast<Line*>(parentObj)) {
		moveOnObject(line);
	}
	else if (auto* circle = dynamic_cast<Circle*>(parentObj)) {
		moveOnObject(circle);
	}
	else {
		// Handle other types or throw an error
	}
}

void ObjectPoint::moveOnObjectCircleSf(const sf::Vector2f& circleCenter) { // Renamed function
	if (attachedCircle) {
		// Use the stored circleAngle to recompute the position along the circumference.
		float radius = attachedCircle->getRadius();
		sf::Vector2f newPosition = circleCenter + sf::Vector2f(
			std::cos(circleAngle) * radius,
			std::sin(circleAngle) * radius);
		setPosition(newPosition);
	}
}
Point_2 ObjectPoint::toCGALPoint(const sf::Vector2f& vector) const {
	return Point_2(vector.x, vector.y);
}
// Helper function to convert an SFML Vector2f to a CGAL Point_2.
Point_2 ObjectPoint::toCGALPoint(const sf::Vector2f& vector) {
	return Point_2(vector.x, vector.y);
}
// Helper function to convert a CGAL Point_2 to an SFML Vector2f.
sf::Vector2f ObjectPoint::CGALToSFML(const Point_2& point) const {
	return sf::Vector2f(
		static_cast<float>(CGAL::to_double(point.x())),
		static_cast<float>(CGAL::to_double(point.y()))
	);
}
ObjectPoint::~ObjectPoint() {
	if (parentLine) {
		parentLine->removeObjectPoint(this);
	}
	if (parentCircle) {
		parentCircle->removeObjectPoint(this);
	}
}

// Helper function to convert an SFML Vector2f to a CGAL Point_2.


void ObjectPoint::update() {
	if (attachedType == ObjectType::Circle && attachedCircle) {
		// Recompute the point position based on the circle's center, radius, and the stored angle.
		Point_2 center = attachedCircle->getCenterPoint();
		double r = attachedCircle->getRadius();
		double newX = CGAL::to_double(center.x()) + r * std::cos(circleAngle);
		double newY = CGAL::to_double(center.y()) + r * std::sin(circleAngle);
		setPosition(Point_2(newX, newY));
	}
	else if (attachedType == ObjectType::Line && attachedLine) {
		moveOnObject(attachedLine);
	}
}