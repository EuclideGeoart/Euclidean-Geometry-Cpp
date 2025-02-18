#pragma once
#include <vector>
#include <string>
#include <SFML/Graphics.hpp>
#include "ObjectType.h"

class ObjectPoint; // Forward declaration

class GeometricObject {
protected:
	bool selected = false;

public:

	virtual ~GeometricObject() = default;
	// Pure virtual functions
	virtual void draw(sf::RenderWindow& window) const = 0;
	virtual bool isMouseOver(const sf::Vector2f& pos) const = 0; // Corrected signature
	virtual std::string getStatus() const = 0; // Added virtual method
	virtual ObjectType getAttachedType() const = 0; // Added virtual method
	virtual void update() = 0;


	bool isSelected() const { return selected; }

	// Add method to handle child points
   // virtual void addChildPoint(ObjectPoint* child) = 0;

private:
	std::vector<ObjectPoint*> childPoints;
};