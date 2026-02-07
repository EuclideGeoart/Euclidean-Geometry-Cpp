#include "IntersectionPoint.h"
#include "Constants.h"
#include <iostream>
#include <CGAL/intersections.h>

IntersectionPoint::IntersectionPoint(std::shared_ptr<Line> line1, std::shared_ptr<Line> line2, const Point_2& pos,
                                   const sf::Color& color)
    : Point(pos, 1.0f, color), m_line1(line1), m_line2(line2) {
    // We don't automatically register because we might be in creation phase
    // User/Factory should ensure registration if needed, but here we just observe
    // Actually, we need to know when lines move. IntersectionPoint should arguably observe them.
    // For now, we rely on the generic update loop or the lines notifying us if we are added as a child.
    // But Point isn't a child of Line.
    // The Tool will manage updates or we assume lines trigger global updates.
    // Better: Lines should know about this intersection point? 
    // No, standard dependency is usually Point <- Line. Here it's Line -> Point.
    // We'll assume the editor calls update() or we implement specific observers later.
    // For the "Dynamic Intersection" task, we simply recalculate in update().
    m_isIntersectionPoint = true;
}

std::shared_ptr<IntersectionPoint> IntersectionPoint::create(std::shared_ptr<Line> line1, std::shared_ptr<Line> line2, 
                                                           const Point_2& pos, const sf::Color& color) {
    return std::make_shared<IntersectionPoint>(line1, line2, pos, color);
}

bool IntersectionPoint::areParentsValid() const {
    return !m_line1.expired() && !m_line2.expired();
}

void IntersectionPoint::update() {
    recalculateIntersection();
    Point::update(); // Update visuals
}

void IntersectionPoint::recalculateIntersection() {
    if (!areParentsValid()) {
        m_isValid = false; 
        return;
    }

    auto l1 = m_line1.lock();
    auto l2 = m_line2.lock();

    if (!l1 || !l2) return;

    // Get CGAL lines
    Line_2 cgalLine1 = l1->getCGALLine();
    Line_2 cgalLine2 = l2->getCGALLine();

    // Intersect
    CGAL::Object result = CGAL::intersection(cgalLine1, cgalLine2);

    if (const Point_2* ip = CGAL::object_cast<Point_2>(&result)) {
        setCGALPosition(*ip);
        m_isValid = true;
    } else {
        // Parallel or coincident?
        m_isValid = false;
    }
}
