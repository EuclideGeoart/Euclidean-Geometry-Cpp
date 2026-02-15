#ifdef CGAL_USE_SSE2
#undef CGAL_USE_SSE2
#endif

#include "IntersectionSystem.h"
#include "Circle.h"
#include "GeometricObject.h"
#include "Intersection.h"
#include "Line.h"
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>
#include <algorithm>

namespace {
constexpr double kEpsilon = 1e-8;
constexpr double kEpsilonSq = kEpsilon * kEpsilon;

void addUnique(std::vector<Point_2> &out, const Point_2 &p) {
  for (const auto &existing : out) {
    if (CGAL::to_double(CGAL::squared_distance(existing, p)) <= kEpsilonSq) {
      return;
    }
  }
  out.push_back(p);
}

void addIntersectionObject(std::vector<Point_2> &out, const CGAL::Object &obj) {
  if (const auto *p = CGAL::object_cast<Point_2>(&obj)) {
    addUnique(out, *p);
    return;
  }
  if (const auto *s = CGAL::object_cast<Segment_2>(&obj)) {
    addUnique(out, s->source());
    addUnique(out, s->target());
  }
}

bool isPointOnSegmentApprox(const Segment_2 &seg, const Point_2 &p,
                            double tolerance = 1.0) {
  double sx = CGAL::to_double(seg.source().x());
  double sy = CGAL::to_double(seg.source().y());
  double tx = CGAL::to_double(seg.target().x());
  double ty = CGAL::to_double(seg.target().y());

  double minX = std::min(sx, tx) - tolerance;
  double maxX = std::max(sx, tx) + tolerance;
  double minY = std::min(sy, ty) - tolerance;
  double maxY = std::max(sy, ty) + tolerance;

  double px = CGAL::to_double(p.x());
  double py = CGAL::to_double(p.y());

  if (px < minX || px > maxX || py < minY || py > maxY) return false;

  return CGAL::to_double(CGAL::squared_distance(seg, p)) < (tolerance * tolerance);
}

struct GeometryView {
  const Line *line = nullptr;
  const Circle *circle = nullptr;
  std::vector<Segment_2> segments;
  bool isLineSegment = false;
};

bool tryGetValidLineEndpoints(const Line *line, Point_2 &start, Point_2 &end) {
  if (!line || !line->getStartPointPtr() || !line->getEndPointPtr()) {
    return false;
  }

  try {
    start = line->getStartPoint();
    end = line->getEndPoint();
  } catch (...) {
    return false;
  }

  return CGAL::to_double(CGAL::squared_distance(start, end)) > kEpsilonSq;
}

GeometryView buildGeometryView(const GeometricObject &obj) {
  GeometryView view;
  if (auto line = dynamic_cast<const Line *>(&obj)) {
    view.line = line;
    view.isLineSegment = line->isSegment();
    if (view.isLineSegment) {
      try {
        view.segments.emplace_back(line->getStartPoint(), line->getEndPoint());
      } catch (...) {
      }
    }
    return view;
  }

  if (auto circle = dynamic_cast<const Circle *>(&obj)) {
    view.circle = circle;
    return view;
  }

  view.segments = obj.getBoundarySegments();
  return view;
}

void intersectLineLine(const GeometryView &a, const GeometryView &b,
                       std::vector<Point_2> &out) {
  if (!a.line || !b.line) {
    return;
  }

  Point_2 aStart, aEnd, bStart, bEnd;
  if (!tryGetValidLineEndpoints(a.line, aStart, aEnd) ||
      !tryGetValidLineEndpoints(b.line, bStart, bEnd)) {
    return;
  }

  if (a.isLineSegment && b.isLineSegment) {
    Segment_2 sa(aStart, aEnd);
    Segment_2 sb(bStart, bEnd);
    addIntersectionObject(out, CGAL::intersection(sa, sb));
    return;
  }

  if (a.isLineSegment && !b.isLineSegment) {
    Segment_2 sa(aStart, aEnd);
    Line_2 lb(bStart, bEnd);
    addIntersectionObject(out, CGAL::intersection(lb, sa));
    return;
  }

  if (!a.isLineSegment && b.isLineSegment) {
    Segment_2 sb(bStart, bEnd);
    Line_2 la(aStart, aEnd);
    addIntersectionObject(out, CGAL::intersection(la, sb));
    return;
  }

  Line_2 la(aStart, aEnd);
  Line_2 lb(bStart, bEnd);
  CGAL::Object result = CGAL::intersection(la, lb);
  addIntersectionObject(out, result);
}

void intersectLineCircle(const GeometryView &lineView, const GeometryView &circleView,
                          std::vector<Point_2> &out) {
  if (!lineView.line || !circleView.circle) return;

  Point_2 start, end;
  if (!tryGetValidLineEndpoints(lineView.line, start, end)) {
    return;
  }

  Line_2 line(start, end);
  auto points = findIntersection(line, circleView.circle->getCGALCircle());

  if (!lineView.isLineSegment) {
    for (const auto &p : points) addUnique(out, p);
    return;
  }

  Segment_2 seg(start, end);
  for (const auto &p : points) {
    if (isPointOnSegmentApprox(seg, p, 1.0)) {
      addUnique(out, p);
    }
  }
}

void intersectCircleCircle(const GeometryView &a, const GeometryView &b,
                           std::vector<Point_2> &out) {
  if (!a.circle || !b.circle) {
    return;
  }

  auto points = findIntersection(a.circle->getCGALCircle(), b.circle->getCGALCircle());
  for (const auto &p : points) {
    Point_2 projA = a.circle->projectOntoCircumference(p);
    Point_2 projB = b.circle->projectOntoCircumference(p);
    Point_2 merged((projA.x() + projB.x()) / 2, (projA.y() + projB.y()) / 2);
    addUnique(out, merged);
  }
}

void intersectLineSegments(const GeometryView &lineView,
                           const std::vector<Segment_2> &segments,
                           std::vector<Point_2> &out) {
  if (!lineView.line) {
    return;
  }

  Point_2 start, end;
  if (!tryGetValidLineEndpoints(lineView.line, start, end)) {
    return;
  }

  Line_2 line(start, end);
  Segment_2 lineSegment(start, end);
  for (const auto &seg : segments) {
    if (lineView.isLineSegment) {
      addIntersectionObject(out, CGAL::intersection(seg, lineSegment));
    } else {
      addIntersectionObject(out, CGAL::intersection(line, seg));
    }
  }
}

void intersectCircleSegments(const GeometryView &circleView,
                             const std::vector<Segment_2> &segments,
                             std::vector<Point_2> &out) {
  if (!circleView.circle) {
    return;
  }

  const Circle_2 circle = circleView.circle->getCGALCircle();
  for (const auto &seg : segments) {
    Line_2 support = seg.supporting_line();
    auto points = findIntersection(support, circle);
    for (const auto &p : points) {
      if (isPointOnSegmentApprox(seg, p)) {
        addUnique(out, p);
      }
    }
  }
}

void intersectSegments(const std::vector<Segment_2> &a,
                       const std::vector<Segment_2> &b,
                       std::vector<Point_2> &out) {
  for (const auto &sa : a) {
    for (const auto &sb : b) {
      addIntersectionObject(out, CGAL::intersection(sa, sb));
    }
  }
}
} // namespace

namespace IntersectionSystem {
std::vector<Point_2> computeIntersections(const GeometricObject &a,
                                          const GeometricObject &b) {
  std::vector<Point_2> result;

  const auto *lineA = dynamic_cast<const Line *>(&a);
  const auto *lineB = dynamic_cast<const Line *>(&b);
  const auto *circleA = dynamic_cast<const Circle *>(&a);
  const auto *circleB = dynamic_cast<const Circle *>(&b);

  if (lineA && lineB) {
    GeometryView ga = buildGeometryView(*lineA);
    GeometryView gb = buildGeometryView(*lineB);
    intersectLineLine(ga, gb, result);
    return result;
  }

  if (lineA && circleB) {
    GeometryView ga = buildGeometryView(*lineA);
    GeometryView gb = buildGeometryView(*circleB);
    intersectLineCircle(ga, gb, result);
    return result;
  }

  if (circleA && lineB) {
    GeometryView ga = buildGeometryView(*lineB);
    GeometryView gb = buildGeometryView(*circleA);
    intersectLineCircle(ga, gb, result);
    return result;
  }

  if (circleA && circleB) {
    GeometryView ga = buildGeometryView(*circleA);
    GeometryView gb = buildGeometryView(*circleB);
    intersectCircleCircle(ga, gb, result);
    return result;
  }

  GeometryView ga = buildGeometryView(a);
  GeometryView gb = buildGeometryView(b);

  if (ga.line) {
    intersectLineSegments(ga, gb.segments, result);
    return result;
  }

  if (gb.line) {
    intersectLineSegments(gb, ga.segments, result);
    return result;
  }

  if (ga.circle) {
    intersectCircleSegments(ga, gb.segments, result);
    return result;
  }

  if (gb.circle) {
    intersectCircleSegments(gb, ga.segments, result);
    return result;
  }

  intersectSegments(ga.segments, gb.segments, result);
  return result;
}
} // namespace IntersectionSystem
