#ifndef LINE_H
#define LINE_H

#include <SFML/Graphics.hpp>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Point_2.h>
#include "Types.h"
#include "Point.h"
#include "GeometricObject.h"
#include "ObjectPoint.h"

class ObjectPoint;

class Line : public GeometricObject {
public:

	//Line(const Point_2& start, const Point_2& end);
	  // (You can also keep an overload that takes sf::Vector2f if needed.)
	Line(const sf::Vector2f& start, const sf::Vector2f& end, bool isSegment);
	Line(Point* start, Point* end, bool isSegment = false);
	Line(const Point_2& start, const Point_2& end, bool isSegment);

	void updateFromPoints() {
		if (startPointPtr && endPointPtr) {
			startPoint = startPointPtr->getPositionCGAL();
			endPoint = endPointPtr->getPositionCGAL();
			updateSFMLVertices();
		}
	}
	bool isSegment() const { return segmentFlag; }  // Renamed member to avoid conflict
	void draw(sf::RenderWindow& window, const sf::View& view) const;
	void setSelected(bool selected) { isSelected = selected; }

	Point_2 getStartPoint() const { return startPoint; }
	Point_2 getEndPoint() const { return endPoint; }
	// Methods now take CGAL Point_2 and Vector_2
	void moveEndpointToStart(const Point_2& newStart);
	void moveEndpointToEnd(const Point_2& newEnd);
	void translate(const Vector_2& offset);
	void updatePosition(Point* movedPoint, const sf::Vector2f& newPos);
	Point* getStartPointPtr() const { return startPointPtr; }
	Point* getEndPointPtr() const { return endPointPtr; }
	// Accessors return CGAL Point_2

	// Keep sf::Vector2f for input, but use CGAL internally
	void updateGeometry();
	void setTranslationOffset(const sf::Vector2f& offset);
	void applyTranslationOffset();
	void finalizeTranslation();
	bool containsPoint(const sf::Vector2f& Point) const;
	bool containsEndpoint(const sf::Vector2f& point, float threshold) const;

	// Use CGAL for intersection, but return sf::Vector2f for convenience
	static bool calculateIntersection(const Line* l1, const Line* l2, sf::Vector2f& intersection);
	// Add the missing setStartPoint method
	void setStartPoint(const Point_2& newStart) {
		startPoint = newStart;
		cgalLine = Line_2(startPoint, endPoint);
		notifyObjectPoints();
	}
	void setEndPoint(const Point_2& newEnd) {
		endPoint = newEnd;
		cgalLine = Line_2(startPoint, endPoint);
		notifyObjectPoints();
	}

	
	const Line_2& getCgalLine() const { return cgalLine; }
	
	// Add public getter for startPointPtr

	// Ensures the endpoint lies on the circle's circumference
	void setEndpointOnCircle(const sf::Vector2f& center, float radius, bool isEndPoint);
	void updateEndpoints(const Point_2& newEndPoint);
	// New methods for managing ObjectPoints
	void addObjectPoint(ObjectPoint* objPoint);
	void removeObjectPoint(ObjectPoint* objPoint);
	void notifyObjectPoints();
	// Override pure virtual functions
	void draw(sf::RenderWindow& window) const override;
	bool isMouseOver(const sf::Vector2f& pos) const override;
	ObjectType getAttachedType() const override;
	std::string getStatus() const override;
	void update() override;

	void updateSFMLVertices();

private:
	// Store CGAL points
	Line_2 cgalLine;
	Point_2 startPoint;
	Point_2 endPoint;
	Segment_2 cgalSegment;
	bool segmentFlag;
	/*bool isSegment;*/
	bool isSelected = false;
	// Store pointers to connected ObjectPoints
	sf::Vector2f start;
	sf::Vector2f end; // Keep for drawing
	Point* startPointPtr;
	Point* endPointPtr;
	// SFML vertices for drawing the line (kept for rendering)
	sf::Vertex line[2];
	float thickness = 1.0f;
	// Add a temporary offset that is applied during line-drag.
	sf::Vector2f translationOffset = sf::Vector2f(0.f, 0.f);
	sf::Vector2f currentOffset = sf::Vector2f(0.f, 0.f);
	// New member variable to store associated ObjectPoints
	std::vector<ObjectPoint*> attachedObjectPoints;
	// Helper function to update SFML vertices
	//void updateSFMLVertices();
};
#endif