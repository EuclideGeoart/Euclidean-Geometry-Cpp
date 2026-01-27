#include "GeometricObject.h"
#include "ObjectPoint.h"
#include <CGAL/squared_distance_2.h>
#include <algorithm>
#include <iostream>

// Constructor for generic shapes
GeometricObject::GeometricObject(ObjectType type, const sf::Color &color, unsigned int id)
    : m_type(type), m_color(color), m_id(id), m_selected(false), m_hovered(false), m_isValid(true) {}

// Constructor for shapes with initial position (point-based)
GeometricObject::GeometricObject(ObjectType type, const sf::Color &color, const Point_2 &cgal_pos,
                                 unsigned int id)
    : m_type(type), m_color(color), m_id(id), m_selected(false), m_hovered(false), m_isValid(true) {
  // m_cgalPosition would be set by derived class or virtual calls
}

void GeometricObject::setSelected(bool selected) { m_selected = selected; }

bool GeometricObject::isSelected() const { return m_selected; }

void GeometricObject::setHovered(bool hovered) { m_hovered = hovered; }

bool GeometricObject::isHovered() const { return m_hovered; }

void GeometricObject::setColor(const sf::Color &color) { m_color = color; }

void GeometricObject::setLocked(bool locked) { m_locked = locked; }

bool GeometricObject::isLocked() const { return m_locked; }

// --- Hosted ObjectPoint Management ---

void GeometricObject::addChildPoint(std::shared_ptr<ObjectPoint> point) {
  if (!point) return;
  
  // Check if already exists to avoid duplicates
  for (const auto& wp : m_hostedObjectPoints) {
    if (auto sp = wp.lock()) {
      if (sp == point) return;
    }
  }
  
  m_hostedObjectPoints.push_back(point);
  // std::cout << "GeometricObject::addChildPoint: Added point to shape " << m_id << std::endl;
}

void GeometricObject::removeChildPoint(ObjectPoint* point) {
  if (!point) return;
  
  auto newEnd = std::remove_if(m_hostedObjectPoints.begin(), m_hostedObjectPoints.end(),
                               [point](const std::weak_ptr<ObjectPoint>& wp) {
                                 if (auto sp = wp.lock()) {
                                   return sp.get() == point;
                                 }
                                 return true; // Remove expired pointers too
                               });
                               
  if (newEnd != m_hostedObjectPoints.end()) {
    m_hostedObjectPoints.erase(newEnd, m_hostedObjectPoints.end());
    // std::cout << "GeometricObject::removeChildPoint: Removed point from shape " << m_id << std::endl;
  }
}

void GeometricObject::updateHostedPoints() {
  // Notify all hosted ObjectPoints to update their positions
  std::vector<std::weak_ptr<ObjectPoint>> validPoints;
  validPoints.reserve(m_hostedObjectPoints.size());
  
  for (auto& wp : m_hostedObjectPoints) {
    if (auto sp = wp.lock()) {
      sp->updatePositionFromHost();
      validPoints.push_back(wp);
    }
  }
  
  if (validPoints.size() != m_hostedObjectPoints.size()) {
    m_hostedObjectPoints = std::move(validPoints);
  }
}

namespace {
Point_2 projectPointOntoSegmentCgal(const Point_2 &point, const Segment_2 &segment) {
  const Point_2 a = segment.source();
  const Point_2 b = segment.target();
  const Vector_2 ab = b - a;
  const FT ab2 = ab.squared_length();
  if (ab2 == FT(0)) {
    return a;
  }
  const Vector_2 ap = point - a;
  FT t = (ab * ap) / ab2;
  if (t < FT(0)) {
    t = FT(0);
  } else if (t > FT(1)) {
    t = FT(1);
  }
  return a + ab * t;
}
}

bool GeometricObject::getClosestPointOnPerimeter(const Point_2 &query, Point_2 &outPoint) const {
  auto segments = getBoundarySegments();
  if (segments.empty()) {
    return false;
  }

  bool hasBest = false;
  FT bestDist = FT(0);
  for (const auto &seg : segments) {
    Point_2 candidate = projectPointOntoSegmentCgal(query, seg);
    FT dist = CGAL::squared_distance(query, candidate);
    if (!hasBest || dist < bestDist) {
      bestDist = dist;
      outPoint = candidate;
      hasBest = true;
    }
  }

  return hasBest;
}
