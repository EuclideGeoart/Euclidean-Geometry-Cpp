#ifndef POINT_H  
#define POINT_H  

#include <SFML/Graphics.hpp>  
#include <vector>  
#include "Types.h"  
#include "Constants.h"  

// Forward declaration for Line to avoid circular dependency.  
class Line;

class Point {
public:
	Point(const Point_2& position, const sf::Color& color = Constants::POINT_COLOR, bool isLocked = false);
	virtual ~Point() = default;

	// Position management
	void setPosition(const Point_2& newPos);
	// Override draw if necessary
	void draw(sf::RenderWindow& window) const;
	void setSelected(bool selected);
	bool getSelected() const { return isSelected; }
	void updatePosition();
	bool contains(const sf::Vector2f& worldPos, float tolerance = 5.0f) const;
	void setLocked(bool locked) { this->locked = locked; }
	void setHovered(bool hovered);
	bool isIntersectionPoint() const { return isIntersection; }
	void setIntersectionPoint(bool isInter) { isIntersection = isInter; }
	const sf::Vector2f& getPosition() const { return position; }
	void translate(const sf::Vector2f& offset);
	void updateGeometry();
	void finalizeTranslation();
	// Functions for connecting this point with lines.  
	void addConnectedLine(Line* line);
	void removeConnectedLine(Line* line);
	void updateConnectedGeometry(const sf::Vector2f& newPos);
	const sf::Color& getColor() const { return color; }

	// Returns the CGAL Point constructed from SFML position.  
	Point_2 getCGALPoint() const { return Point_2(position.x, position.y); }
	Point_2 getPositionCGAL() const { return getCGALPoint(); }
	const sf::Vector2f& getSFMLPosition() const { return position; }

private:
	std::vector<Line*> connectedLines;
	sf::Vector2f position;
	sf::Color color;
	sf::CircleShape shape;
	bool locked;
	bool isHovered;
	bool isSelected = false;
	bool isIntersection = false;
	bool isLocked;  // Add this line
	float selectionRadius = 5.0f;
	sf::Vector2f translationOffset;
	sf::Vector2f sfToSFML(const Point_2& point) const;
};
#endif