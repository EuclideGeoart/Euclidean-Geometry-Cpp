#pragma once
#ifndef INTERSECTION_POINT_H
#define INTERSECTION_POINT_H

#include "Point.h"
#include "Line.h"
#include <memory>

class IntersectionPoint : public Point {
public:
    IntersectionPoint(std::shared_ptr<Line> line1 = nullptr, std::shared_ptr<Line> line2 = nullptr, 
                     const Point_2& pos = Point_2(0,0),
                     const sf::Color& color = Constants::POINT_INTERSECTION_COLOR, 
                     unsigned int id = 0);

    // Factory method
    static std::shared_ptr<IntersectionPoint> create(std::shared_ptr<Line> line1, std::shared_ptr<Line> line2, 
                                                   const Point_2& pos,
                                                   const sf::Color& color = Constants::POINT_INTERSECTION_COLOR);

    void update() override;
    
    // Check if parents are still valid
    bool areParentsValid() const;
    
    // Recalculate position based on current parent lines
    void recalculateIntersection();
    
    ObjectType getType() const override { return ObjectType::IntersectionPoint; }

    std::shared_ptr<Line> getLine1() const { return m_line1.lock(); }
    std::shared_ptr<Line> getLine2() const { return m_line2.lock(); }

private:
    std::weak_ptr<Line> m_line1;
    std::weak_ptr<Line> m_line2;
};

#endif // INTERSECTION_POINT_H
