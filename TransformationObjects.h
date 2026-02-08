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
  ReflectLine(std::shared_ptr<Point> source = nullptr,
              std::shared_ptr<Line> line = nullptr,
              const sf::Color& color = Constants::POINT_DEFAULT_COLOR,
              unsigned int id = 0)
      : Point(Point_2(0, 0), 1.0f, color, id), sourcePoint(std::move(source)),
        reflectLine(std::move(line)) {
    setDependent(true);
    update();
  }

  void restoreTransformation(std::shared_ptr<GeometricObject> parent,
                            std::shared_ptr<GeometricObject> aux,
                            TransformationType type) override {
    GeometricObject::restoreTransformation(parent, aux, type);
    sourcePoint = std::dynamic_pointer_cast<Point>(parent);
    reflectLine = std::dynamic_pointer_cast<Line>(aux);
    setTransformType(type);
    if (sourcePoint) {
        sourcePoint->addDependent(shared_from_this());
        setParentSourceID(sourcePoint->getID());
    }
    if (reflectLine) {
        reflectLine->addDependent(shared_from_this());
        setAuxObjectID(reflectLine->getID());
    }
    update();
  }

  void update() override {
    if (!sourcePoint || !reflectLine || !sourcePoint->isValid() || !reflectLine->isValid()) {
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
    // Project point p onto line ab to find foot of perpendicular h
    Vector_2 ap = p - a;
    FT t = (ap * ab) / ab.squared_length();
    // t is the parameter along ab where the projection falls
    Point_2 h = a + ab * t;
    Point_2 pPrime = p + (h - p) * FT(2.0);

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

  std::shared_ptr<Point> getSourcePoint() const { return sourcePoint; }
  std::shared_ptr<Line> getReflectLine() const { return reflectLine; }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Line> reflectLine;
};

class ReflectPoint : public Point {
 public:
  ReflectPoint(std::shared_ptr<Point> source = nullptr,
               std::shared_ptr<Point> center = nullptr,
               const sf::Color& color = Constants::POINT_DEFAULT_COLOR,
               unsigned int id = 0)
      : Point(Point_2(0, 0), 1.0f, color, id), sourcePoint(std::move(source)),
        centerPoint(std::move(center)) {
    setDependent(true);
    update();
  }

  void restoreTransformation(std::shared_ptr<GeometricObject> parent,
                            std::shared_ptr<GeometricObject> aux,
                            TransformationType type) override {
    GeometricObject::restoreTransformation(parent, aux, type);
    sourcePoint = std::dynamic_pointer_cast<Point>(parent);
    centerPoint = std::dynamic_pointer_cast<Point>(aux);
    setTransformType(type);
    if (sourcePoint) {
        sourcePoint->addDependent(shared_from_this());
        setParentSourceID(sourcePoint->getID());
    }
    if (centerPoint) {
        centerPoint->addDependent(shared_from_this());
        setAuxObjectID(centerPoint->getID());
    }
    update();
  }

  void update() override {
    if (!sourcePoint || !centerPoint || !sourcePoint->isValid() || !centerPoint->isValid() ||
        !centerPoint->isVisible()) {
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

  std::shared_ptr<Point> getSourcePoint() const { return sourcePoint; }
  std::shared_ptr<Point> getCenterPoint() const { return centerPoint; }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> centerPoint;
};

class ReflectCircle : public Point {
 public:
  ReflectCircle(std::shared_ptr<Point> source = nullptr,
                std::shared_ptr<Circle> circle = nullptr,
                const sf::Color& color = Constants::POINT_DEFAULT_COLOR,
                unsigned int id = 0)
      : Point(Point_2(0, 0), 1.0f, color, id), sourcePoint(std::move(source)),
        reflectCircle(std::move(circle)) {
    setDependent(true);
    update();
  }

  void restoreTransformation(std::shared_ptr<GeometricObject> parent,
                            std::shared_ptr<GeometricObject> aux,
                            TransformationType type) override {
    GeometricObject::restoreTransformation(parent, aux, type);
    sourcePoint = std::dynamic_pointer_cast<Point>(parent);
    reflectCircle = std::dynamic_pointer_cast<Circle>(aux);
    setTransformType(type);
    if (sourcePoint) {
        sourcePoint->addDependent(shared_from_this());
        setParentSourceID(sourcePoint->getID());
    }
    if (reflectCircle) {
        reflectCircle->addDependent(shared_from_this());
        setAuxObjectID(reflectCircle->getID());
    }
    update();
  }

  void update() override {
    if (!sourcePoint || !reflectCircle || !sourcePoint->isValid() || !reflectCircle->isValid() ||
        !reflectCircle->isVisible()) {
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

  std::shared_ptr<Point> getSourcePoint() const { return sourcePoint; }
  std::shared_ptr<Circle> getCircle() const { return reflectCircle; }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Circle> reflectCircle;
};

class RotatePoint : public Point {
 public:
  RotatePoint(std::shared_ptr<Point> source = nullptr,
              std::shared_ptr<Point> center = nullptr,
              double angleDegrees = 0.0,
              const sf::Color& color = Constants::POINT_DEFAULT_COLOR,
              unsigned int id = 0)
      : Point(Point_2(0, 0), 1.0f, color, id), sourcePoint(std::move(source)),
        centerPoint(std::move(center)) {
    setTransformValue(angleDegrees);
    setDependent(true);
    update();
  }

  void restoreTransformation(std::shared_ptr<GeometricObject> parent,
                            std::shared_ptr<GeometricObject> aux,
                            TransformationType type) override {
    GeometricObject::restoreTransformation(parent, aux, type);
    sourcePoint = std::dynamic_pointer_cast<Point>(parent);
    centerPoint = std::dynamic_pointer_cast<Point>(aux);
    setTransformType(type);
    if (sourcePoint) {
        sourcePoint->addDependent(shared_from_this());
        setParentSourceID(sourcePoint->getID());
    }
    if (centerPoint) {
        centerPoint->addDependent(shared_from_this());
        setAuxObjectID(centerPoint->getID());
    }
    update();
  }

  void update() override {
    if (!sourcePoint || !centerPoint || !sourcePoint->isValid() || !centerPoint->isValid() ||
        !centerPoint->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 c = centerPoint->getCGALPosition();

    double px = CGAL::to_double(p.x()) - CGAL::to_double(c.x());
    double py = CGAL::to_double(p.y()) - CGAL::to_double(c.y());

    double rad = getTransformValue() * (3.14159265358979323846 / 180.0);
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);

    double rx = px * cosA - py * sinA;
    double ry = px * sinA + py * cosA;

    Point_2 pPrime(CGAL::to_double(c.x()) + rx, CGAL::to_double(c.y()) + ry);

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

  std::shared_ptr<Point> getSourcePoint() const { return sourcePoint; }
  std::shared_ptr<Point> getCenterPoint() const { return centerPoint; }
  double getAngleDegrees() const { return getTransformValue(); }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> centerPoint;
};

class TranslateVector : public Point {
 public:
  TranslateVector(std::shared_ptr<Point> source = nullptr,
                  std::shared_ptr<Point> vecStart = nullptr,
                  std::shared_ptr<Point> vecEnd = nullptr,
                  const sf::Color& color = Constants::POINT_DEFAULT_COLOR,
                  unsigned int id = 0)
      : Point(Point_2(0, 0), 1.0f, color, id), sourcePoint(std::move(source)),
        vectorStart(std::move(vecStart)), vectorEnd(std::move(vecEnd)) {
    setDependent(true);
    update();
  }

  void restoreTransformation(std::shared_ptr<GeometricObject> parent,
                            std::shared_ptr<GeometricObject> aux,
                            TransformationType type) override {
    GeometricObject::restoreTransformation(parent, aux, type);
    sourcePoint = std::dynamic_pointer_cast<Point>(parent);
    vectorEnd = std::dynamic_pointer_cast<Point>(aux);
    // vectorStart restoration is currently limited by metadata having only 1 aux ID.
    // However, vectorEnd is usually the primary anchor for the vector.
    setTransformType(type);
    if (sourcePoint) {
        sourcePoint->addDependent(shared_from_this());
        setParentSourceID(sourcePoint->getID());
    }
    if (vectorEnd) {
        vectorEnd->addDependent(shared_from_this());
        setAuxObjectID(vectorEnd->getID());
    }
    update();
  }

  void update() override {
    if (!sourcePoint || !vectorStart || !vectorEnd || !sourcePoint->isValid() || !vectorStart->isValid() ||
        !vectorEnd->isValid() || !vectorStart->isVisible() || !vectorEnd->isVisible()) {
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

  std::shared_ptr<Point> getSourcePoint() const { return sourcePoint; }
  std::shared_ptr<Point> getVectorStart() const { return vectorStart; }
  std::shared_ptr<Point> getVectorEnd() const { return vectorEnd; }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> vectorStart;
  std::shared_ptr<Point> vectorEnd;
};

class DilatePoint : public Point {
 public:
  DilatePoint(std::shared_ptr<Point> source = nullptr,
              std::shared_ptr<Point> center = nullptr,
              double factor = 1.0,
              const sf::Color& color = Constants::POINT_DEFAULT_COLOR,
              unsigned int id = 0)
      : Point(Point_2(0, 0), 1.0f, color, id), sourcePoint(std::move(source)),
        centerPoint(std::move(center)) {
    setTransformValue(factor);
    setDependent(true);
    update();
  }

  void restoreTransformation(std::shared_ptr<GeometricObject> parent,
                            std::shared_ptr<GeometricObject> aux,
                            TransformationType type) override {
    GeometricObject::restoreTransformation(parent, aux, type);
    sourcePoint = std::dynamic_pointer_cast<Point>(parent);
    centerPoint = std::dynamic_pointer_cast<Point>(aux);
    setTransformType(type);
    if (sourcePoint) {
        sourcePoint->addDependent(shared_from_this());
        setParentSourceID(sourcePoint->getID());
    }
    if (centerPoint) {
        centerPoint->addDependent(shared_from_this());
        setAuxObjectID(centerPoint->getID());
    }
    update();
  }

  void update() override {
    if (!sourcePoint || !centerPoint || !sourcePoint->isValid() || !centerPoint->isValid() ||
        !centerPoint->isVisible()) {
      setVisible(false);
      return;
    }

    Point_2 p = sourcePoint->getCGALPosition();
    Point_2 c = centerPoint->getCGALPosition();
    Vector_2 cp = p - c;
    Point_2 pPrime = c + cp * FT(getTransformValue());

    setCGALPosition(pPrime);
    setVisible(true);
    Point::update();
  }

  std::shared_ptr<Point> getSourcePoint() const { return sourcePoint; }
  std::shared_ptr<Point> getCenterPoint() const { return centerPoint; }
  double getScaleFactor() const { return getTransformValue(); }

 private:
  std::shared_ptr<Point> sourcePoint;
  std::shared_ptr<Point> centerPoint;
};
