// Circle.h
#pragma once
#ifndef CIRCLE_H
#define CIRCLE_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include "GeometricObject.h"
#include "ObjectPoint.h"
#include "Types.h" // For Point_2 type
#include "Constants.h"


class Circle : public GeometricObject {
public:
	Circle(const sf::Vector2f& centerPos, const sf::Color& outlineCol = sf::Color::Blue);
	Point_2 getCenterPoint() const {
		return toCGALPoint(center);
	}
	Circle(const Point_2& center, double radius);
	void updateRadius(const sf::Vector2f& currentPos);
	sf::Vector2f constrainToCircumference(const sf::Vector2f& point) const;
	void finishCreation();
	void draw(sf::RenderWindow& window) const override;
	bool containsCenter(const sf::Vector2f& pos, float tolerance) const;
	bool containsCircumference(const sf::Vector2f& pos, float tolerance) const;
	bool isNearCircumference(const sf::Vector2f& point, float tolerance) const;
	void move(const sf::Vector2f& offset);
	void setRadius(float newRadius);
	sf::Vector2f getCenter() const;
	float getRadius() const;
	std::string getStatus() const override;
	bool hasRadius() const;
	bool isCreated() const;
	void setSelected(bool s);
	void setRadiusPoint(const sf::Vector2f& point);
	bool isMouseOver(const sf::Vector2f& pos) const override; // Implement isMouseOver
	void addChildPoint(ObjectPoint* child);
	void update() override; // Implement update
	void resize(float newRadius);
	void updateObjectPoints();
	void updateConnectedPoints();
	virtual ObjectType getAttachedType() const override; // Implement getAttachedType
	void addObjectPoint(ObjectPoint* objPoint);
	void removeObjectPoint(ObjectPoint* objPoint);
	void notifyObjectPoints();
	void moveCenter(const sf::Vector2f& newCenter);
	Point_2 toCGALPoint(const sf::Vector2f& vector) const {
		return Point_2(vector.x, vector.y);
	}
	
	sf::Vector2f toSFMLVector(const Point_2& point) {
		return sf::Vector2f(
			static_cast<float>(CGAL::to_double(point.x())),
			static_cast<float>(CGAL::to_double(point.y()))
		);
	}
private:
	sf::Vector2f center;
	float radius;
	bool selected;
	bool creationFinished;
	sf::Color outlineColor;
	bool created = false;
	sf::Vector2f radiusPoint;
	std::vector<ObjectPoint*> childPoints;
	sf::CircleShape shape;
	sf::Vector2f CGALToSFML(const Point_2& point) const;
	float distance(const sf::Vector2f& a, const sf::Vector2f& b) const;
	float length(const sf::Vector2f& point) const;
	std::vector<ObjectPoint*> attachedObjectPoints;
};

#endif // CIRCLE_H