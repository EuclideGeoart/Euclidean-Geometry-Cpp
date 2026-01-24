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
                            double tolerance = 1e-4) {
  return CGAL::to_double(CGAL::squared_distance(seg, p)) <= tolerance;
}

struct GeometryView {
  const Line *line = nullptr;
  const Circle *circle = nullptr;
  std::vector<Segment_2> segments;
  bool isLineSegment = false;
};

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

  if (a.isLineSegment && b.isLineSegment) {
    Segment_2 sa(a.line->getStartPoint(), a.line->getEndPoint());
    Segment_2 sb(b.line->getStartPoint(), b.line->getEndPoint());
    addIntersectionObject(out, CGAL::intersection(sa, sb));
    return;
  }

  if (a.isLineSegment && !b.isLineSegment) {
    Segment_2 sa(a.line->getStartPoint(), a.line->getEndPoint());
    addIntersectionObject(out, CGAL::intersection(b.line->getCGALLine(), sa));
    return;
  }

  if (!a.isLineSegment && b.isLineSegment) {
    Segment_2 sb(b.line->getStartPoint(), b.line->getEndPoint());
    addIntersectionObject(out, CGAL::intersection(a.line->getCGALLine(), sb));
    return;
  }

  CGAL::Object result = CGAL::intersection(a.line->getCGALLine(), b.line->getCGALLine());
  addIntersectionObject(out, result);
}

void intersectLineCircle(const GeometryView &lineView, const GeometryView &circleView,
                          std::vector<Point_2> &out) {
  if (!lineView.line || !circleView.circle) return;

  Line_2 line = lineView.line->getCGALLine();
  auto points = findIntersection(line, circleView.circle->getCGALCircle());

  if (!lineView.isLineSegment) {
    for (const auto &p : points) addUnique(out, p);
    return;
  }

  Segment_2 seg(lineView.line->getStartPoint(), lineView.line->getEndPoint());
  for (const auto &p : points) {
    if (CGAL::to_double(CGAL::squared_distance(seg, p)) < 1.0) {
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

  Line_2 line = lineView.line->getCGALLine();
  for (const auto &seg : segments) {
    if (lineView.isLineSegment) {
      addIntersectionObject(out, CGAL::intersection(seg, Segment_2(lineView.line->getStartPoint(),
                                                                    lineView.line->getEndPoint())));
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
