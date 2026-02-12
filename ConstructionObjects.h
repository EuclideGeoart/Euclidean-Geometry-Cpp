#pragma once

#include <cmath>
#include <memory>

#include "Line.h"
#include "Circle.h"
#include "Constants.h"
#include "VariantUtils.h"

// Dynamic construction helpers that recompute geometry from parents.
class PerpendicularBisector : public Line {
 public:
  PerpendicularBisector(std::shared_ptr<Point> first,
                        std::shared_ptr<Point> second,
                        unsigned int id,
                        const sf::Color &color = Constants::CONSTRUCTION_LINE_COLOR)
      : Line(makePlaceholderPoint(), makePlaceholderPoint(), false, color, id),
        p1(std::move(first)),
        p2(std::move(second)) {
    setAsConstructionLine();
    updateSFMLShape();
  }

  void updateSFMLShape() override {
    if (!p1 || !p2 || !p1->isValid() || !p2->isValid() || !p1->isVisible() || !p2->isVisible()) {
        if (isVisible()) setVisible(false);
        return;
    }

    try {
      Point_2 pos1 = p1->getCGALPosition();
      Point_2 pos2 = p2->getCGALPosition();
      Vector_2 v = pos2 - pos1;
      double len = std::sqrt(CGAL::to_double(v.squared_length()));
      if (len < 1e-9) {
          if (isVisible()) setVisible(false);
          return;
      }

      Point_2 mid((pos1.x() + pos2.x()) * FT(0.5), (pos1.y() + pos2.y()) * FT(0.5));
      Vector_2 perp(-v.y(), v.x());
      double perpLen = std::sqrt(CGAL::to_double(perp.squared_length()));
      if (perpLen < 1e-9) {
          if (isVisible()) setVisible(false);
          return;
      }
      Vector_2 perpUnit = perp / FT(perpLen);
      FT span = FT(10000.0);
      Point_2 start = mid + perpUnit * span;
      Point_2 end = mid - perpUnit * span;

      if (auto sp = getStartPointObjectShared()) sp->setCGALPosition(start);
      if (auto ep = getEndPointObjectShared()) ep->setCGALPosition(end);

      Line::updateCGALLine();
      Line::updateSFMLShape();
      if (!isVisible()) setVisible(true);
    } catch (...) {
      if (isVisible()) setVisible(false);
    }
  }

  ObjectType getType() const override { return ObjectType::PerpendicularBisector; }
  std::shared_ptr<Point> getFirstParentPoint() const { return p1; }
  std::shared_ptr<Point> getSecondParentPoint() const { return p2; }

 private:
  static std::shared_ptr<Point> makePlaceholderPoint() {
    return std::make_shared<Point>(Point_2(0, 0), 1.0f, Constants::POINT_DEFAULT_COLOR,
                                   Constants::POINT_DEFAULT_COLOR);
  }

  std::shared_ptr<Point> p1;
  std::shared_ptr<Point> p2;
};

class AngleBisector : public Line {
 public:
  AngleBisector(std::shared_ptr<Point> vertexPoint,
                std::shared_ptr<Point> armPoint1,
                std::shared_ptr<Point> armPoint2,
                unsigned int id,
                bool isExternal = false,
                const sf::Color &color = Constants::CONSTRUCTION_LINE_COLOR)
      : Line(makePlaceholderPoint(), makePlaceholderPoint(), false, color, id),
        vertex(std::move(vertexPoint)),
        arm1(std::move(armPoint1)),
        arm2(std::move(armPoint2)),
        usesLines(false),
        m_isExternal(isExternal) {
    setAsConstructionLine();
    updateSFMLShape();
  }

  AngleBisector(std::shared_ptr<Line> l1,
                std::shared_ptr<Line> l2,
                unsigned int id,
                bool isExternal = false,
                const sf::Color &color = Constants::CONSTRUCTION_LINE_COLOR)
      : Line(makePlaceholderPoint(), makePlaceholderPoint(), false, color, id),
        line1(std::move(l1)),
        line2(std::move(l2)),
        usesLines(true),
        m_isExternal(isExternal) {
    setAsConstructionLine();
    updateSFMLShape();
  }

  void updateSFMLShape() override {
    try {
      Point_2 origin;
      Vector_2 dir;

      if (usesLines) {
        auto l1Locked = line1.lock();
        auto l2Locked = line2.lock();
        if (!l1Locked || !l2Locked || !l1Locked->isValid() || !l2Locked->isValid() || !l1Locked->isVisible() || !l2Locked->isVisible()) {
             if (isVisible()) setVisible(false);
             return;
        }

        auto result = CGAL::intersection(l1Locked->getCGALLine(), l2Locked->getCGALLine());
        if (!result) {
            if (isVisible()) setVisible(false);
            return;
        }
        const Point_2 *I = safe_get_point<Point_2>(&(*result));
        if (!I) {
            if (isVisible()) setVisible(false);
            return;
        }
        origin = *I;

        Vector_2 d1 = l1Locked->getCGALLine().direction().to_vector();
        Vector_2 d2 = l2Locked->getCGALLine().direction().to_vector();
        double l1Len = std::sqrt(CGAL::to_double(d1.squared_length()));
        double l2Len = std::sqrt(CGAL::to_double(d2.squared_length()));
        if (l1Len < 1e-9 || l2Len < 1e-9) {
            if (isVisible()) setVisible(false);
            return;
        }
        uN = d1 / FT(l1Len);
        vN = d2 / FT(l2Len);
        dir = uN + vN;
        if (CGAL::to_double(dir.squared_length()) < 1e-12) {
            // Parallel/Anti-parallel lines, bisector undefined or mid-line?
            if (isVisible()) setVisible(false);
            return;
        }
      } else {
        if (!vertex || !arm1 || !arm2 || !vertex->isValid() || !arm1->isValid() || !arm2->isValid() || !vertex->isVisible() || !arm1->isVisible() || !arm2->isVisible()) {
            if (isVisible()) setVisible(false);
            return;
        }
        Point_2 vtx = vertex->getCGALPosition();
        Point_2 a = arm1->getCGALPosition();
        Point_2 c = arm2->getCGALPosition();
        Vector_2 u = a - vtx;
        Vector_2 v = c - vtx;
        double lu = std::sqrt(CGAL::to_double(u.squared_length()));
        double lv = std::sqrt(CGAL::to_double(v.squared_length()));
        if (lu < 1e-9 || lv < 1e-9) {
            if (isVisible()) setVisible(false);
            return;
        }
        uN = u / FT(lu);
        vN = v / FT(lv);
        dir = uN + vN;
        if (CGAL::to_double(dir.squared_length()) < 1e-12) {
            if (isVisible()) setVisible(false);
            return;
        }
        origin = vtx;
      }

      if (m_isExternal) {
          // The external bisector is perpendicular to the internal one
          // We can get it by subtracting the unit vectors instead of adding them, 
          // or by taking the perpendicular of the internal bisector. 
          // uN - vN gives the direction of the external bisector.
          dir = uN - vN;
          if (CGAL::to_double(dir.squared_length()) < 1e-12) {
              if (isVisible()) setVisible(false);
              return;
          }
      }

      double dLen = std::sqrt(CGAL::to_double(dir.squared_length()));
      if (dLen < 1e-9) {
          if (isVisible()) setVisible(false);
          return;
      }
      Vector_2 unit = dir / FT(dLen);
      FT span = FT(10000.0);
      Point_2 start = origin;
      Point_2 end = origin + unit * span;

      if (auto sp = getStartPointObjectShared()) sp->setCGALPosition(start);
      if (auto ep = getEndPointObjectShared()) ep->setCGALPosition(end);

      Line::updateCGALLine();
      Line::updateSFMLShape();
      if (!isVisible()) setVisible(true);
    } catch (...) {
      if (isVisible()) setVisible(false);
    }
  }

  ObjectType getType() const override { return ObjectType::AngleBisector; }
  bool usesLineParents() const { return usesLines; }
  bool isExternalBisector() const { return m_isExternal; }
  std::shared_ptr<Line> getFirstParentLine() const { return line1.lock(); }
  std::shared_ptr<Line> getSecondParentLine() const { return line2.lock(); }
  std::shared_ptr<Point> getVertexParentPoint() const { return vertex; }
  std::shared_ptr<Point> getFirstArmParentPoint() const { return arm1; }
  std::shared_ptr<Point> getSecondArmParentPoint() const { return arm2; }

 private:
  static std::shared_ptr<Point> makePlaceholderPoint() {
    return std::make_shared<Point>(Point_2(0, 0), 1.0f, Constants::POINT_DEFAULT_COLOR,
                                   Constants::POINT_DEFAULT_COLOR);
  }

  std::shared_ptr<Point> vertex;
  std::shared_ptr<Point> arm1;
  std::shared_ptr<Point> arm2;
  std::weak_ptr<Line> line1;
  std::weak_ptr<Line> line2;
  bool usesLines;
  bool m_isExternal = false;
  Vector_2 uN;
  Vector_2 vN;
};

class TangentLine : public Line {
 public:
  TangentLine(std::shared_ptr<Point> external,
              std::shared_ptr<Circle> circlePtr,
              int solution,
              unsigned int id,
              const sf::Color &color = Constants::CONSTRUCTION_LINE_COLOR)
      : Line(makePlaceholderPoint(), makePlaceholderPoint(), false, color, id),
        externalPoint(std::move(external)),
        circle(std::move(circlePtr)),
        solutionIndex(solution) {
    setAsConstructionLine();
    updateSFMLShape();
  }

  void updateSFMLShape() override {
    if (!externalPoint || !circle || !externalPoint->isValid() || !circle->isValid() || !externalPoint->isVisible() || !circle->isVisible()) {
      if (isVisible()) setVisible(false);
      return;
    }

    try {
      Point_2 P = externalPoint->getCGALPosition();
      Point_2 O = circle->getCenterPoint();
      double r = circle->getRadius();
      Vector_2 OP = P - O;
      double d = std::sqrt(CGAL::to_double(OP.squared_length()));
      
      // Check if point is inside circle (no tangent possible)
      if (d < r - 1e-6) {
          if (isVisible()) setVisible(false);
          return;
      }

      if (d < 1e-9) { 
          if (isVisible()) setVisible(false);
          return; 
      }
      
      Vector_2 OP_unit = OP / FT(d);

      Point_2 tangentPoint;
      if (std::abs(d - r) < 1e-6) {
        tangentPoint = P;
        Vector_2 perp(-OP_unit.y(), OP_unit.x());
        Point_2 end = tangentPoint + perp * FT(100.0);
        applyEndpoints(tangentPoint, end);
        if (!isVisible()) setVisible(true);
        return;
      }

      double proj = r * r / d;
      double h = std::sqrt(std::max(0.0, r * r - proj * proj));
      Point_2 base = O + OP_unit * FT(proj);
      Vector_2 perp(-OP_unit.y(), OP_unit.x());
      Point_2 t1 = base + perp * FT(h);
      Point_2 t2 = base - perp * FT(h);
      tangentPoint = (solutionIndex == 0) ? t1 : t2;

      Point_2 end = tangentPoint + (tangentPoint - P) * FT(100.0);
      applyEndpoints(P, end);
      if (!isVisible()) setVisible(true);
    } catch (...) {
      if (isVisible()) setVisible(false);
    }
  }

  std::shared_ptr<Point> getExternalPoint() const { return externalPoint; }
  std::shared_ptr<Circle> getCircle() const { return circle; }
  int getSolutionIndex() const { return solutionIndex; }

 private:
  static std::shared_ptr<Point> makePlaceholderPoint() {
    return std::make_shared<Point>(Point_2(0, 0), 1.0f, Constants::POINT_DEFAULT_COLOR,
                                   Constants::POINT_DEFAULT_COLOR);
  }

  void applyEndpoints(const Point_2 &start, const Point_2 &end) {
    if (auto sp = getStartPointObjectShared()) sp->setCGALPosition(start);
    if (auto ep = getEndPointObjectShared()) ep->setCGALPosition(end);
    Line::updateCGALLine();
    Line::updateSFMLShape();
  }

  std::shared_ptr<Point> externalPoint;
  std::shared_ptr<Circle> circle;
  int solutionIndex;
};

// --- New Dynamic Construction Classes ---

class Midpoint : public Point {
public:
    // Constructor 1: From two points
    Midpoint(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2, const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
        : Point(CGAL::midpoint(p1->getCGALPosition(), p2->getCGALPosition()), 1.0f, color),
          parent1(p1), parent2(p2) {
        setDependent(true);
        setLabel("M");
    }

    // Constructor 2: From line (or segment)
    Midpoint(std::shared_ptr<Line> linePtr, const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
        : Point(CGAL::midpoint(linePtr->getStartPoint(), linePtr->getEndPoint()), 1.0f, color),
          parentLine(linePtr) {
        setDependent(true);
        setLabel("M");
    }
    
  void update() override {
        bool valid = false;
        Point_2 newPos;

        if (parentLine) {
            if (parentLine->isValid() && parentLine->isVisible()) {
                newPos = CGAL::midpoint(parentLine->getStartPoint(), parentLine->getEndPoint());
                valid = true;
            }
        } else if (parent1 && parent2) {
            if (parent1->isValid() && parent2->isValid() && parent1->isVisible() && parent2->isVisible()) {
                newPos = CGAL::midpoint(parent1->getCGALPosition(), parent2->getCGALPosition());
                valid = true;
            }
        }

        if (valid) {
            setCGALPosition(newPos);
            setVisible(true);
            Point::update(); 
        } else {
            setVisible(false);
        }
    }

    void relinkTransformation(std::shared_ptr<GeometricObject> p1, std::shared_ptr<GeometricObject> p2, std::shared_ptr<GeometricObject> aux2 = nullptr) override {
        (void)aux2;
        parent1 = std::dynamic_pointer_cast<Point>(p1);
        parent2 = std::dynamic_pointer_cast<Point>(p2);
        parentLine = std::dynamic_pointer_cast<Line>(p1); // Handle Line-based midpoint via p1
        if (parent1) parent1->addDependent(m_selfHandle);
        if (parent2) parent2->addDependent(m_selfHandle);
        if (parentLine) parentLine->addDependent(m_selfHandle);
        update();
    }

private:
    std::shared_ptr<Point> parent1;
    std::shared_ptr<Point> parent2;
    std::shared_ptr<Line> parentLine;
};

class CompassCircle : public Circle {
public:
    // Constructor: Center + 2 Points for Radius
    CompassCircle(std::shared_ptr<Point> center, std::shared_ptr<Point> radP1, std::shared_ptr<Point> radP2, unsigned int id, const sf::Color& color)
      : Circle(center ? center.get() : nullptr, nullptr, 
                 std::sqrt(CGAL::to_double(CGAL::squared_distance(radP1->getCGALPosition(), radP2->getCGALPosition()))),
                 color),
          centerPt(center), radiusP1(radP1), radiusP2(radP2) {
          setDependent(true);
    }

    // Constructor: Center + Line/Segment for Radius
    CompassCircle(std::shared_ptr<Point> center, std::shared_ptr<Line> radLine, unsigned int id, const sf::Color& color)
        : Circle(center ? center.get() : nullptr, nullptr, 
                 std::sqrt(CGAL::to_double(CGAL::squared_distance(radLine->getStartPoint(), radLine->getEndPoint()))),
                 color),
          centerPt(center), radiusLine(radLine) {
          setDependent(true);
    }

    // Override this to tell the Editor "Don't try to resize me"
    virtual bool isResizable() const override { return false; }

    void setRadius(double r) override {
        // 1. Check if we are a "Live" Compass (bound to parents)
        bool hasParents = (radiusP1 && radiusP2) || (radiusLine);

        if (hasParents) {
            // IGNORE the resize request. 
            // Our radius is determined strictly by our parents.
            return; 
        }

        // 2. If we are a "broken" compass (static), behave like a normal circle
        Circle::setRadius(r);
    }

    void update() override {
    if (!centerPt || !centerPt->isValid() || !centerPt->isVisible()) {
        setVisible(false);
        return;
    }

    // --- FIX 1: FLATTEN THE CENTER POINT (Prevents Stack Overflow) ---
    // We get the raw position, convert to double, and make a NEW point.
    // This strips away the massive "history tree" that causes the crash.
    Point_2 rawPos = centerPt->getCGALPosition();
    Point_2 flatCenter(CGAL::to_double(rawPos.x()), CGAL::to_double(rawPos.y()));

    // Use the Base Class setter to avoid any overrides
    Circle::setCenter(flatCenter); 
    // ---------------------------------------------------------------

    double newRadius = 0.0;
    bool radValid = false;

    // Calculate Radius (The math here converts to double, so it is already "flat")
    if (radiusLine) {
        if (radiusLine->isValid() && radiusLine->isVisible()) {
            // Optimization: Use getSquaredLength() if available on your Line class
            newRadius = std::sqrt(CGAL::to_double(CGAL::squared_distance(radiusLine->getStartPoint(), radiusLine->getEndPoint())));
            radValid = true;
        }
    } else if (radiusP1 && radiusP2) {
        if (radiusP1->isValid() && radiusP2->isValid()) {
            newRadius = std::sqrt(CGAL::to_double(CGAL::squared_distance(radiusP1->getCGALPosition(), radiusP2->getCGALPosition())));
            radValid = true;
        }
    }

    if (radValid && newRadius > Constants::EPSILON_LENGTH_CONSTRUCTION) {
        
        // --- FIX 2: APPLY RADIUS DIRECTLY ---
        // We call the Base Class setRadius to bypass the "Block" we added to CompassCircle.
        Circle::setRadius(newRadius);
        
        setVisible(true);
        
        // Update the visual representation (SFML shape)
        // If Circle::update() calls updateSFMLShape(), this is fine.
        // Otherwise, call updateSFMLShape() directly.
        Circle::updateSFMLShape(); 
    } else {
        setVisible(false);
    }
}

void relinkTransformation(std::shared_ptr<GeometricObject> c, std::shared_ptr<GeometricObject> p1, std::shared_ptr<GeometricObject> p2 = nullptr) override {
    centerPt = std::dynamic_pointer_cast<Point>(c);
    if (auto ln = std::dynamic_pointer_cast<Line>(p1)) {
        radiusLine = ln;
        radiusP1.reset();
        radiusP2.reset();
    } else {
        radiusP1 = std::dynamic_pointer_cast<Point>(p1);
        radiusP2 = std::dynamic_pointer_cast<Point>(p2);
        radiusLine.reset();
    }
    if (centerPt) centerPt->addDependent(m_selfHandle);
    if (radiusP1) radiusP1->addDependent(m_selfHandle);
    if (radiusP2) radiusP2->addDependent(m_selfHandle);
    if (auto rl = radiusLine) rl->addDependent(m_selfHandle);
    update();
}

private:
    std::shared_ptr<Point> centerPt;
    std::shared_ptr<Point> radiusP1;
    std::shared_ptr<Point> radiusP2;
    std::shared_ptr<Line> radiusLine;
};
