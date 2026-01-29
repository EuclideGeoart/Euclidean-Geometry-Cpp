#pragma once

#include <cmath>
#include <memory>

#include "Circle.h"
#include "Constants.h"
#include "Line.h"
#include "Point.h"
#include "Types.h"

class ReflectLine : public Point {
 public:
  ReflectLine(std::shared_ptr<Point> source,
              std::shared_ptr<Line> line,
              const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
      : Point(Point_2(0, 0), 1.0f, color), sourcePoint(std::move(source)),
        reflectLine(std::move(line)) {
    setDependent(true);
    update();
  }

  void update() override {
    if (!sourcePoint || !reflectLine || !sourcePoint->isValid() || !reflectLine->isValid() ||
        !sourcePoint->isVisible() || !reflectLine->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 a = reflectLine->getStartPoint();
    Point_2 b = reflectLine->getEndPoint();

    Vector_2 ab = b - a;
    double abLenSq = CGAL::to_double(ab.squared_length());
    if (abLenSq < 1e-12) {
      setVisible(false);
      return;
    }

    Vector_2 ap = p - a;
    FT t = (ap * ab) / ab.squared_length();
    Point_2 h = a + ab * t;
    Point_2 pPrime = p + (h - p) * FT(2.0);

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Line> reflectLine;
};

class ReflectPoint : public Point {
 public:
  ReflectPoint(std::shared_ptr<Point> source,
               std::shared_ptr<Point> center,
               const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
      : Point(Point_2(0, 0), 1.0f, color), sourcePoint(std::move(source)),
        centerPoint(std::move(center)) {
    setDependent(true);
    update();
  }

  void update() override {
    if (!sourcePoint || !centerPoint || !sourcePoint->isValid() || !centerPoint->isValid() ||
        !sourcePoint->isVisible() || !centerPoint->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 c = centerPoint->getCGALPosition();
    Point_2 pPrime = c + (c - p);

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> centerPoint;
};

class ReflectCircle : public Point {
 public:
  ReflectCircle(std::shared_ptr<Point> source,
                std::shared_ptr<Circle> circle,
                const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
      : Point(Point_2(0, 0), 1.0f, color), sourcePoint(std::move(source)),
        reflectCircle(std::move(circle)) {
    setDependent(true);
    update();
  }

  void update() override {
    if (!sourcePoint || !reflectCircle || !sourcePoint->isValid() || !reflectCircle->isValid() ||
        !sourcePoint->isVisible() || !reflectCircle->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 o = reflectCircle->getCenterPoint();
    double r = reflectCircle->getRadius();

    Vector_2 op = p - o;
    double opLenSq = CGAL::to_double(op.squared_length());
    if (opLenSq < 1e-12) {
      setVisible(false);
      return;
    }

    double scale = (r * r) / opLenSq;
    Point_2 pPrime = o + op * FT(scale);

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Circle> reflectCircle;
};

class RotatePoint : public Point {
 public:
  RotatePoint(std::shared_ptr<Point> source,
              std::shared_ptr<Point> center,
              double angleDegrees,
              const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
      : Point(Point_2(0, 0), 1.0f, color), sourcePoint(std::move(source)),
        centerPoint(std::move(center)), angleDeg(angleDegrees) {
    setDependent(true);
    update();
  }

  void update() override {
    if (!sourcePoint || !centerPoint || !sourcePoint->isValid() || !centerPoint->isValid() ||
        !sourcePoint->isVisible() || !centerPoint->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 c = centerPoint->getCGALPosition();

    double px = CGAL::to_double(p.x()) - CGAL::to_double(c.x());
    double py = CGAL::to_double(p.y()) - CGAL::to_double(c.y());

    double rad = angleDeg * (3.14159265358979323846 / 180.0);
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);

    double rx = px * cosA - py * sinA;
    double ry = px * sinA + py * cosA;

    Point_2 pPrime(CGAL::to_double(c.x()) + rx, CGAL::to_double(c.y()) + ry);

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> centerPoint;
  double angleDeg;
};

class TranslateVector : public Point {
 public:
  TranslateVector(std::shared_ptr<Point> source,
                  std::shared_ptr<Point> vecStart,
                  std::shared_ptr<Point> vecEnd,
                  const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
      : Point(Point_2(0, 0), 1.0f, color), sourcePoint(std::move(source)),
        vectorStart(std::move(vecStart)), vectorEnd(std::move(vecEnd)) {
    setDependent(true);
    update();
  }

  void update() override {
    if (!sourcePoint || !vectorStart || !vectorEnd || !sourcePoint->isValid() || !vectorStart->isValid() ||
        !vectorEnd->isValid() || !sourcePoint->isVisible() || !vectorStart->isVisible() || !vectorEnd->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 v1 = vectorStart->getCGALPosition();
    Point_2 v2 = vectorEnd->getCGALPosition();

    Vector_2 delta = v2 - v1;
    Point_2 pPrime = p + delta;

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> vectorStart;
  std::shared_ptr<Point> vectorEnd;
};

class DilatePoint : public Point {
 public:
  DilatePoint(std::shared_ptr<Point> source,
              std::shared_ptr<Point> center,
              double factor,
              const sf::Color& color = Constants::POINT_DEFAULT_COLOR)
      : Point(Point_2(0, 0), 1.0f, color), sourcePoint(std::move(source)),
        centerPoint(std::move(center)), scaleFactor(factor) {
    setDependent(true);
    update();
  }

  void update() override {
    if (!sourcePoint || !centerPoint || !sourcePoint->isValid() || !centerPoint->isValid() ||
        !sourcePoint->isVisible() || !centerPoint->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 c = centerPoint->getCGALPosition();
    Vector_2 cp = p - c;
    Point_2 pPrime = c + cp * FT(scaleFactor);

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> centerPoint;
  double scaleFactor;
};
