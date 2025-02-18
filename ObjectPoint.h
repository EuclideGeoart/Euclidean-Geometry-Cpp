#pragma once
#ifndef OBJECT_POINT_H
#define OBJECT_POINT_H

#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>
#include "Point.h"
#include "Line.h"
#include "Circle.h"  // Ensure Circle is included
#include "GeometricObject.h"  // Needed since ObjectPoint might interact with children of GeometricObject.
#include "Constants.h"
#include "Types.h"  
#include "ObjectType.h"

class Line;
class Circle;

class ObjectPoint : public GeometricObject, public Point {
public:
	// Existing constructors  
	ObjectPoint(const Point_2& pos, Line* lineObj);
	ObjectPoint(const Point_2& pos, Circle* circleObj, sf::Color color);
	ObjectPoint(const Point_2& position, const sf::Color& color, bool isLocked = false);
	ObjectPoint(const Point_2& position, GeometricObject* parentObj, const sf::Color& color, bool isOnCircle = false);
	ObjectPoint(const sf::Vector2f& pt, Circle* parent = nullptr)
		: ObjectPoint(Point_2(pt.x, pt.y), parent, sf::Color::Black) {
	}
	~ObjectPoint();
	// Method to update position based on parent
	void moveOnObject(Line* parentLine);
	void moveOnObject(Circle* parentCircle);
	void moveOnObject(const sf::Vector2f& newPos);
	void moveOnObjectCircleSf(const sf::Vector2f& circleCenter); // Renamed function
	void updateRelativePosition ();
	// Add a method to set the parent object
	GeometricObject* getParentObject() const {
		return parentObject;
	}
	// Override virtual functions from GeometricObject
	void draw(sf::RenderWindow& window) const override;
	/*void draw(sf::RenderWindow& window, const sf::View& view) const override;*/
	bool isMouseOver(const sf::Vector2f& pos) const override;
	ObjectType getAttachedType() const override;
	std::string getStatus() const override;
	void update() override;
	
	// Add methods for attaching to different types
	double relativePosition = 0.5;

	bool contains(const sf::Vector2f& pos, float tolerance) const;
	void setPosition(const Point_2& pos);
	Point_2 getPosition() const;
	void setSelected(bool selected);
	bool isSelected() const {
		return selected;
	}
	       
	void moveOnObject(GeometricObject* parentObj);

	//void setParentObject(GeometricObject* parent);  
	void updateConnectedGeometry(const sf::Vector2f& newPos);
	//Point_2 projectPointOntoLine(const Point_2& p, const Line_2& line, bool isSegment);
	Point_2 projectPointOntoLine(const Point_2& p, const Line_2& line, bool isSegment);
	Point_2 projectPointOntoCircle(const Point_2& p, const Point_2& center, double radius, double tolerance = 1e-6);
	Line* getAttachedLine() const {
		if (attachedType == ObjectType::Line) {
			return attachedLine;
		}
		return nullptr;
	}
	Circle* getAttachedCircle() const {
		if (attachedType == ObjectType::Circle) {
			return attachedCircle;
		}
		return nullptr;
	}
	//void moveOnObject(Line* parentLine);
	void attachToCircle(Circle* circle);
	void setPosition(const sf::Vector2f& position);
	sf::Vector2f getOffsetFromCenter() const;
	void processPoints() const {
		for (auto* point : attachedObjectPoints) {
			ObjectPoint* objPoint = dynamic_cast<ObjectPoint*>(point);
			if (objPoint) {
				Point_2 projectedPoint; // Assume this is calculated somewhere  
				objPoint->setPosition(projectedPoint);
			}
		}
	}
private:
	bool Selected;
	ObjectType attachedType;
	Line* attachedLine;
	Circle* attachedCircle = nullptr;
	Line* parentLine;
	Circle* parentCircle;

	sf::CircleShape shape;
	Point_2 position;
	bool isOnCircle;

	sf::Vector2f offsetFromCenter;
	GeometricObject* parentObject = nullptr;
	std::vector<Line*> connectedLines; // Added this line
	//sf::Vector2f position;
	double circleAngle = 0.0; //
	sf::Vector2f CGALToSFML(const Point_2& point) const;
	Point_2 toCGALPoint(const sf::Vector2f& vector);
	Point_2 toCGALPoint(const sf::Vector2f& vector) const;
	std::vector<ObjectPoint*> attachedObjectPoints;
};


#endif // OBJECT_POINT_H
