#pragma once

#include <cmath>
#include <memory>

#include "Line.h"
#include "Circle.h"
#include "Constants.h"

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
    if (!p1 || !p2) return;

    try {
      Point_2 pos1 = p1->getCGALPosition();
      Point_2 pos2 = p2->getCGALPosition();
      Vector_2 v = pos2 - pos1;
      double len = std::sqrt(CGAL::to_double(v.squared_length()));
      if (len < 1e-9) return;

      Point_2 mid((pos1.x() + pos2.x()) * FT(0.5), (pos1.y() + pos2.y()) * FT(0.5));
      Vector_2 perp(-v.y(), v.x());
      double perpLen = std::sqrt(CGAL::to_double(perp.squared_length()));
      if (perpLen < 1e-9) return;
      Vector_2 perpUnit = perp / FT(perpLen);
      FT span = FT(10000.0);
      Point_2 start = mid + perpUnit * span;
      Point_2 end = mid - perpUnit * span;

      if (auto sp = getStartPointObjectShared()) sp->setCGALPosition(start);
      if (auto ep = getEndPointObjectShared()) ep->setCGALPosition(end);

      Line::updateCGALLine();
      Line::updateSFMLShape();
    } catch (...) {
      // Silently ignore; construction lines are best-effort
    }
  }

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
                const sf::Color &color = Constants::CONSTRUCTION_LINE_COLOR)
      : Line(makePlaceholderPoint(), makePlaceholderPoint(), false, color, id),
        vertex(std::move(vertexPoint)),
        arm1(std::move(armPoint1)),
        arm2(std::move(armPoint2)),
        usesLines(false) {
    setAsConstructionLine();
    updateSFMLShape();
  }

  AngleBisector(std::shared_ptr<Line> l1,
                std::shared_ptr<Line> l2,
                unsigned int id,
                const sf::Color &color = Constants::CONSTRUCTION_LINE_COLOR)
      : Line(makePlaceholderPoint(), makePlaceholderPoint(), false, color, id),
        line1(std::move(l1)),
        line2(std::move(l2)),
        usesLines(true) {
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
        if (!l1Locked || !l2Locked) return;

        auto result = CGAL::intersection(l1Locked->getCGALLine(), l2Locked->getCGALLine());
        if (!result) return;
        const Point_2 *I = std::get_if<Point_2>(&(*result));
        if (!I) return;
        origin = *I;

        Vector_2 d1 = l1Locked->getCGALLine().direction().to_vector();
        Vector_2 d2 = l2Locked->getCGALLine().direction().to_vector();
        double l1Len = std::sqrt(CGAL::to_double(d1.squared_length()));
        double l2Len = std::sqrt(CGAL::to_double(d2.squared_length()));
        if (l1Len < 1e-9 || l2Len < 1e-9) return;
        Vector_2 u = d1 / FT(l1Len);
        Vector_2 v = d2 / FT(l2Len);
        dir = u + v;
        if (CGAL::to_double(dir.squared_length()) < 1e-12) return;
      } else {
        if (!vertex || !arm1 || !arm2) return;
        Point_2 vtx = vertex->getCGALPosition();
        Point_2 a = arm1->getCGALPosition();
        Point_2 c = arm2->getCGALPosition();
        Vector_2 u = a - vtx;
        Vector_2 v = c - vtx;
        double lu = std::sqrt(CGAL::to_double(u.squared_length()));
        double lv = std::sqrt(CGAL::to_double(v.squared_length()));
        if (lu < 1e-9 || lv < 1e-9) return;
        u = u / FT(lu);
        v = v / FT(lv);
        dir = u + v;
        if (CGAL::to_double(dir.squared_length()) < 1e-12) return;
        origin = vtx;
      }

      double dLen = std::sqrt(CGAL::to_double(dir.squared_length()));
      if (dLen < 1e-9) return;
      Vector_2 unit = dir / FT(dLen);
      FT span = FT(10000.0);
      Point_2 start = origin;
      Point_2 end = origin + unit * span;

      if (auto sp = getStartPointObjectShared()) sp->setCGALPosition(start);
      if (auto ep = getEndPointObjectShared()) ep->setCGALPosition(end);

      Line::updateCGALLine();
      Line::updateSFMLShape();
    } catch (...) {
      // Ignore errors to avoid cascading failures during interactive updates
    }
  }

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
    if (!externalPoint || !circle) return;

    try {
      Point_2 P = externalPoint->getCGALPosition();
      Point_2 O = circle->getCenterPoint();
      double r = circle->getRadius();
      Vector_2 OP = P - O;
      double d = std::sqrt(CGAL::to_double(OP.squared_length()));
      if (d < 1e-9) return;
      Vector_2 OP_unit = OP / FT(d);

      Point_2 tangentPoint;
      if (std::abs(d - r) < 1e-6) {
        tangentPoint = P;
        Vector_2 perp(-OP_unit.y(), OP_unit.x());
        Point_2 end = tangentPoint + perp * FT(100.0);
        applyEndpoints(tangentPoint, end);
        return;
      }

      if (d < r - 1e-6) return; // Inside circle, no tangent

      double proj = r * r / d;
      double h = std::sqrt(std::max(0.0, r * r - proj * proj));
      Point_2 base = O + OP_unit * FT(proj);
      Vector_2 perp(-OP_unit.y(), OP_unit.x());
      Point_2 t1 = base + perp * FT(h);
      Point_2 t2 = base - perp * FT(h);
      tangentPoint = (solutionIndex == 0) ? t1 : t2;

      Point_2 end = tangentPoint + (tangentPoint - P) * FT(100.0);
      applyEndpoints(P, end);
    } catch (...) {
      // Skip update on error
    }
  }

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
