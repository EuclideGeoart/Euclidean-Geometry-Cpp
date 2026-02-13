#include "Rectangle.h"

#include <SFML/Graphics/Color.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>

#include "Circle.h"
#include "Constants.h"
#include "Line.h"
#include "Point.h"
#include "PointUtils.h"
#include "Types.h"


Rectangle::Rectangle(const Point_2& corner1, const Point_2& corner2, bool isRotatable, const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
      m_corner1(std::make_shared<Point>(corner1, 1.0f)),
      m_corner2(std::make_shared<Point>(corner2, 1.0f)),
      m_isRotatable(isRotatable),
      m_width(0),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  // Initialize B and D
  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  updateDimensionsFromCorners();
  update();
  updateSFMLShape();
  setShowLabel(true);
}

Rectangle::Rectangle(const std::shared_ptr<Point>& corner1,
                     const std::shared_ptr<Point>& corner2,
                     bool isRotatable,
                     const sf::Color& color,
                     unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
      m_corner1(corner1 ? corner1 : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_corner2(corner2 ? corner2 : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_isRotatable(isRotatable),
      m_width(0),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  // Register as dependent of corner points so we update when they move
  if (m_corner1) m_corner1->addDependent(m_selfHandle);
  if (m_corner2) m_corner2->addDependent(m_selfHandle);

  updateDimensionsFromCorners();
  update();
  updateSFMLShape();
  setShowLabel(true);
}

Rectangle::Rectangle(const Point_2& corner, const Point_2& adjacentPoint, double width, const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::RectangleRotatable, color, id),
      m_corner1(std::make_shared<Point>(corner, 1.0f)),
      m_corner2(std::make_shared<Point>(adjacentPoint, 1.0f)),
      m_isRotatable(true),
      m_width(0),
      m_height(width),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  syncRotatableFromAnchors();
  update();
  updateSFMLShape();
  setShowLabel(true);
}

Rectangle::Rectangle(const std::shared_ptr<Point>& corner,
                     const std::shared_ptr<Point>& adjacentPoint,
                     double width,
                     const sf::Color& color,
                     unsigned int id)
    : GeometricObject(ObjectType::RectangleRotatable, color, id),
      m_corner1(corner ? corner : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_corner2(adjacentPoint ? adjacentPoint : std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f)),
      m_isRotatable(true),
      m_width(0),
      m_height(width),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Default transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));

  m_cornerB = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);
  m_cornerD = std::make_shared<Point>(Point_2(FT(0), FT(0)), 1.0f);

  // Register as dependent of corner points so we update when they move
  if (m_corner1) m_corner1->addDependent(m_selfHandle);
  if (m_corner2) m_corner2->addDependent(m_selfHandle);

  syncRotatableFromAnchors();
  updateSFMLShape();
  setShowLabel(true);
}
// new constructor that takes 4 points for full connectivity (used by deserializer)
Rectangle::Rectangle(std::shared_ptr<Point> p1,
                     std::shared_ptr<Point> p2,
                     std::shared_ptr<Point> pb,
                     std::shared_ptr<Point> pd,
                     bool isRotatable,
                     const sf::Color& color,
                     unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id),
      m_corner1(p1),
      m_corner2(p2),
      m_cornerB(pb),
      m_cornerD(pd),
      m_isRotatable(isRotatable),
      m_width(0),
      m_height(0),
      m_rotationAngle(0),
      m_center(FT(0), FT(0)) {
  m_color.a = 0;  // Transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));
  m_useExplicitVertices = true;
  
  // CRITICAL FIX: Robustly enforce perimeter winding using spatial sort.
  // We cannot rely on the input order (p1, p2, pb, pd) because transformations
  // like Reflection can invert winding (CCW -> CW) or scramble indices.
  // We must ensure the internal mapping corner1->corner2->cornerB->cornerD 
  // forms a valid CCW convex loop.
  
  if (m_isRotatable && p1 && p2 && pb && pd) {
      std::vector<std::shared_ptr<Point>> pts = {p1, p2, pb, pd};
      
      // 1. Calculate Centroid
      double cx = 0, cy = 0;
      for (const auto& p : pts) {
          auto pos = p->getCGALPosition();
          cx += CGAL::to_double(pos.x());
          cy += CGAL::to_double(pos.y());
      }
      cx *= 0.25;
      cy *= 0.25;

      // 2. Sort by Angle (CCW)
      std::sort(pts.begin(), pts.end(), [cx, cy](const std::shared_ptr<Point>& a, const std::shared_ptr<Point>& b) {
          auto posA = a->getCGALPosition();
          auto posB = b->getCGALPosition();
          double angA = std::atan2(CGAL::to_double(posA.y()) - cy, CGAL::to_double(posA.x()) - cx);
          double angB = std::atan2(CGAL::to_double(posB.y()) - cy, CGAL::to_double(posB.x()) - cx);
          return angA < angB;
      });

      // 3. Align Start Point
      // We want to keep 'p1' as m_corner1 if possible to preserve the "Anchor".
      // Find p1 in the sorted list and rotate so it's first.
      auto it = std::find(pts.begin(), pts.end(), p1);
      if (it != pts.end()) {
          std::rotate(pts.begin(), it, pts.end());
      }

      // 4. Re-assign strictly
      m_corner1 = pts[0]; // A (p1)
      m_corner2 = pts[1]; // B (CCW neighbor)
      m_cornerB = pts[2]; // C (Opposite)
      m_cornerD = pts[3]; // D (CW neighbor) 
  }
  
  // 1. Force Dependencies (Like Polygon)
  // This ensures that if any of these 4 points move, the Rectangle updates.
  if (m_corner1) m_corner1->addDependent(m_selfHandle);
  if (m_corner2) m_corner2->addDependent(m_selfHandle);
  if (m_cornerB) m_cornerB->addDependent(m_selfHandle);
  if (m_cornerD) m_cornerD->addDependent(m_selfHandle);

  // 2. Initial Geometry Calculation
  // We trust the points are already in the correct place.
  syncRotatableFromAnchors();
  update();
  updateSFMLShape();
  setShowLabel(true);
}
// 1. THE POLYGON-STYLE CONSTRUCTOR
Rectangle::Rectangle(const std::vector<std::shared_ptr<Point>>& vertices, bool isRotatable, const sf::Color& color, unsigned int id)
    : GeometricObject(ObjectType::Rectangle, color, id) {
  m_color.a = 0;  // Transparent fill
  m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));
  m_useExplicitVertices = true;

  if (vertices.size() == 4) {
    // 1. Assign Pointers
    m_corner1 = vertices[0];  // Bottom-Left (A)
    m_cornerB = vertices[1];  // Bottom-Right (B)
    m_corner2 = vertices[2];  // Top-Right (C)
    m_cornerD = vertices[3];  // Top-Left (D)

    // 2. Reverse-Engineer Math from Points
    Point_2 p1 = m_corner1->getCGALPosition();
    Point_2 pb = m_cornerB->getCGALPosition();
    Point_2 p2 = m_corner2->getCGALPosition();

    // Width = Distance(A -> B)
    double dx_w = CGAL::to_double(pb.x() - p1.x());
    double dy_w = CGAL::to_double(pb.y() - p1.y());
    m_width = std::sqrt(dx_w * dx_w + dy_w * dy_w);

    // Height = Distance(B -> C)
    double dx_h = CGAL::to_double(p2.x() - pb.x());
    double dy_h = CGAL::to_double(p2.y() - pb.y());
    m_height = std::sqrt(dx_h * dx_h + dy_h * dy_h);

    // Rotation = Angle of Bottom Edge (A -> B)
    m_rotationAngle = std::atan2(dy_w, dx_w);

    // Center = Midpoint of Diagonal (A -> C)
    m_center = Point_2((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);

    m_isRotatable = isRotatable;

    // VISIBILITY LOGIC:
    // If Rotatable, we need all corners visible for dragging/shearing.
    // If Standard, we usually hide B and D and manage them via A and C.
    if (m_isRotatable) {
      if (m_cornerB) m_cornerB->setVisible(true);
      if (m_cornerD) m_cornerD->setVisible(true);
    } else {
      if (m_cornerB) m_cornerB->setVisible(false);
      if (m_cornerD) m_cornerD->setVisible(false);
    }

    // 3. Link Dependencies
    for (auto& v : vertices) {
      if (v) v->addDependent(m_selfHandle);
    }
  }
  syncRotatableFromAnchors();
  update();
  updateSFMLShape();
  setShowLabel(true);
}

Point_2 Rectangle::getCorner1() const { return getCorner1Position(); }

Point_2 Rectangle::getCorner2() const { return getCorner2Position(); }

Point_2 Rectangle::getCorner1Position() const {
  if (m_corner1) {
    return m_corner1->getCGALPosition();
  }
  return Point_2(FT(0), FT(0));
}

Point_2 Rectangle::getCorner2Position() const {
  if (m_corner2) {
    return m_corner2->getCGALPosition();
  }
  return Point_2(FT(0), FT(0));
}

void Rectangle::setCorner1Position(const Point_2& pos, bool triggerUpdate) {
  if (m_corner1) {
    m_corner1->setCGALPosition(pos);
  } else {
    m_corner1 = std::make_shared<Point>(pos, 1.0f);
  }
  if (triggerUpdate) {
    updateSFMLShape();
    updateHostedPoints();  // This calls notifyDependents()
  }
}

void Rectangle::setCorner2Position(const Point_2& pos, bool triggerUpdate) {
  if (m_corner2) {
    m_corner2->setCGALPosition(pos);
  } else {
    m_corner2 = std::make_shared<Point>(pos, 1.0f);
  }
  if (triggerUpdate) {
    updateSFMLShape();
    updateHostedPoints();  // This calls notifyDependents()
  }
}

void Rectangle::syncRotatableFromAnchors() {
  if (!m_isRotatable) return;

  Point_2 corner = getCorner1Position();
  Point_2 adjacent = getCorner2Position();

  double dx = CGAL::to_double(adjacent.x()) - CGAL::to_double(corner.x());
  double dy = CGAL::to_double(adjacent.y()) - CGAL::to_double(corner.y());
  m_rotationAngle = std::atan2(dy, dx);
  double baseLen = std::sqrt(dx * dx + dy * dy);
  m_width = baseLen;

  if (baseLen < 1e-9) return;

  double nx = -dy / baseLen;
  double ny = dx / baseLen;

  double midX = CGAL::to_double(corner.x()) + dx * 0.5;
  double midY = CGAL::to_double(corner.y()) + dy * 0.5;

  double cx = midX + nx * (m_height * 0.5);
  double cy = midY + ny * (m_height * 0.5);
  m_center = Point_2(FT(cx), FT(cy));
}

void Rectangle::updateDimensionsFromCorners() {
  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  double x1 = CGAL::to_double(c1.x());
  double y1 = CGAL::to_double(c1.y());
  double x2 = CGAL::to_double(c2.x());
  double y2 = CGAL::to_double(c2.y());

  if (m_isRotatable) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double baseLen = std::sqrt(dx * dx + dy * dy);
    m_width = baseLen;
    m_rotationAngle = std::atan2(dy, dx);

    const double minExtent = 1e-3;
    double heightSign = (m_height >= 0.0) ? 1.0 : -1.0;
    double heightAbs = std::fabs(m_height);
    if (heightAbs < minExtent) {
      heightAbs = minExtent;
    }
    m_height = heightAbs * heightSign;

    if (baseLen < minExtent) {
      return;
    }

    double nx = -dy / baseLen;
    double ny = dx / baseLen;
    double midX = x1 + dx * 0.5;
    double midY = y1 + dy * 0.5;

    double cx = midX + nx * (m_height * 0.5);
    double cy = midY + ny * (m_height * 0.5);
    m_center = Point_2(FT(cx), FT(cy));
    return;
  }

  m_width = std::fabs(x2 - x1);
  m_height = std::fabs(y2 - y1);
  m_rotationAngle = 0;  // Axis-aligned rectangles have no rotation
  m_center = Point_2(FT((x1 + x2) * 0.5), FT((y1 + y2) * 0.5));
}
// ---------------------------------------------------------
// VISUAL RENDERER (Correct Winding Order)
// ---------------------------------------------------------
void Rectangle::updateSFMLShape() {
  if (!m_useExplicitVertices) {
    if (m_isRotatable) {
      syncRotatableFromAnchors();
    } else {
      updateDimensionsFromCorners();
    }
    syncDependentCorners();
  }

  m_sfmlShape.setPointCount(4);
  m_sfmlShape.setFillColor(m_color);
  m_sfmlShape.setOutlineThickness(m_thickness);
  m_sfmlShape.setOutlineColor(sf::Color::Black);

  auto setPt = [&](int idx, std::shared_ptr<Point> p) {
    if (p) {
      Point_2 pos = p->getCGALPosition();
      m_sfmlShape.setPoint(idx, sf::Vector2f((float)CGAL::to_double(pos.x()), (float)CGAL::to_double(pos.y())));
    }
  };

  // STRICT PERIMETER ORDER: A -> B -> C -> D
  // Rotatable internal mapping: corner1=A, corner2=B, cornerB=C, cornerD=D
  // Axis-aligned internal mapping: corner1=A, cornerB=B, corner2=C, cornerD=D
  if (m_corner1 && m_cornerB && m_corner2 && m_cornerD) {
    if (m_isRotatable) {
      setPt(0, m_corner1);  // A
      setPt(1, m_corner2);  // B
      setPt(2, m_cornerB);  // C
      setPt(3, m_cornerD);  // D
    } else {
      setPt(0, m_corner1);  // A
      setPt(1, m_cornerB);  // B
      setPt(2, m_corner2);  // C
      setPt(3, m_cornerD);  // D
    }
  }
}

// ---------------------------------------------------------
// SAFE UPDATE METHOD (Breaks Infinite Recursion)
// ---------------------------------------------------------
void Rectangle::update() {
  // 1. RECURSION GUARD: Stop if we are already updating
  if (m_isUpdating) return;
  m_isUpdating = true;  // Lock

  try {
    // 2. CONSTRAINT LOGIC: Enforce 90-degree corners
    // Only run if we have the defining points (A, B, C)
    if (m_corner1 && m_cornerB && m_corner2) {
      Point_2 p1 = m_corner1->getCGALPosition();
      Point_2 pb = m_isRotatable ? m_corner2->getCGALPosition() : m_cornerB->getCGALPosition();
      Point_2 p2 = m_isRotatable ? m_cornerB->getCGALPosition() : m_corner2->getCGALPosition();

      // CREATION GUARD: Check if geometry is complete before enforcing constraints
      // During interactive creation, edges may be degenerate (zero-length or tiny)
      // Skip enforcement if base edge (p1→pb) or height edge (pb→p2) is incomplete
      double baseEdgeSqDist = CGAL::to_double(CGAL::squared_distance(p1, pb));
      double heightEdgeSqDist = CGAL::to_double(CGAL::squared_distance(pb, p2));
      double minThresholdSq = Constants::MIN_CIRCLE_RADIUS * Constants::MIN_CIRCLE_RADIUS;

      if (baseEdgeSqDist < minThresholdSq || heightEdgeSqDist < minThresholdSq) {
        // Rectangle is incomplete (being created or loaded with degenerate edges)
        // Skip constraint enforcement to avoid snapping D prematurely
        GeometricObject::update();
        updateSFMLShape();
        m_isUpdating = false;
        return;
      }

      // Recalculate dimensions based on current points
      double dx_w = CGAL::to_double(pb.x() - p1.x());
      double dy_w = CGAL::to_double(pb.y() - p1.y());
      m_width = std::sqrt(dx_w * dx_w + dy_w * dy_w);

      double dx_h = CGAL::to_double(p2.x() - pb.x());
      double dy_h = CGAL::to_double(p2.y() - pb.y());
      m_height = std::sqrt(dx_h * dx_h + dy_h * dy_h);

      m_rotationAngle = std::atan2(dy_w, dx_w);
      m_center = Point_2((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);

      // 3. CONSTRAINT ENFORCEMENT (Base Edge Dominance)
      // For Rotatable Rectangles, we enforce that ABCD forms a rectangle
      // where AB is the "Base" and Height depends on C/D's projection.
      if (m_isRotatable) {
          // CORNER MAPPING FOR ROTATABLE RECTANGLES:
          // m_corner1 = A (Base Start)
          // m_corner2 = B (Base End)  <-- THIS WAS THE CONFUSION (Used to be Diagonal)
          // m_cornerB = C (Top Right)
          // m_cornerD = D (Top Left)

          // 1. Get current positions
          Point_2 pA = m_corner1->getCGALPosition();
          Point_2 pB = m_corner2->getCGALPosition(); // Base End
          Point_2 pC = m_cornerB->getCGALPosition(); // Top Right
          Point_2 pD = m_cornerD->getCGALPosition(); // Top Left

          // 2. Base Vector U = B - A
          double ux = CGAL::to_double(pB.x() - pA.x());
          double uy = CGAL::to_double(pB.y() - pA.y());
          double baseLenSq = ux*ux + uy*uy;
          
          if (baseLenSq > 1e-9) {
              double baseLen = std::sqrt(baseLenSq);
              ux /= baseLen;
              uy /= baseLen;

              // 3. Normal Vector V (Rotated 90 deg CCW: -y, x)
              double vx = -uy;
              double vy = ux;

              // 4. Project C and D onto Normal to find Height
              // Height = Dot(C-B, V)
              // Note: C is connected to B. D is connected to A.
              double h1 = (CGAL::to_double(pC.x() - pB.x()) * vx + CGAL::to_double(pC.y() - pB.y()) * vy);
              double h2 = (CGAL::to_double(pD.x() - pA.x()) * vx + CGAL::to_double(pD.y() - pA.y()) * vy);
              
              // Average height for stability
              double newHeight = (h1 + h2) / 2.0;
              
              m_height = newHeight; 
              m_width = baseLen;

              // 5. Reconstruct Perfect C and D
              // C = B + V * h
              // D = A + V * h
              double cx = CGAL::to_double(pB.x()) + vx * newHeight;
              double cy = CGAL::to_double(pB.y()) + vy * newHeight;
              Point_2 idealC = Point_2(cx, cy);

              double dx = CGAL::to_double(pA.x()) + vx * newHeight;
              double dy = CGAL::to_double(pA.y()) + vy * newHeight;
              Point_2 idealD = Point_2(dx, dy);

              // Update C (cornerB) if needed
              if (CGAL::squared_distance(pC, idealC) > 1e-6) {
                   m_cornerB->setCGALPosition(idealC);
              }
              // Update D (cornerD) if needed
              if (CGAL::squared_distance(pD, idealD) > 1e-6) {
                   m_cornerD->setCGALPosition(idealD);
              }
          }
      } else {
        // Axis Aligned (A=BL, B=BR, C=TR, D=TL)
        // ... (Existing Axis Aligned Logic if needed, but usually simpler)
        // For now, only Rotatable was broken. 
        // Snap D based on A, B, C logic for Axis Aligned?
        // Axis aligned is usually driven by A and C (diagonal).
        // Let's leave strict update for Rotatable mainly.
        // But we DO need to update cornerD for AA rects too!
        if (m_cornerD) {
             Point_2 idealD(p1.x(), p2.y()); // TL = (x1, y2)
             Point_2 idealB(p2.x(), p1.y()); // BR = (x2, y1)
             
             if (CGAL::to_double(CGAL::squared_distance(m_cornerD->getCGALPosition(), idealD)) > 1e-6) {
                m_cornerD->setCGALPosition(idealD);
             }
             if (CGAL::to_double(CGAL::squared_distance(m_cornerB->getCGALPosition(), idealB)) > 1e-6) {
                m_cornerB->setCGALPosition(idealB);
             }
        }
      }

    }

    // 4. Update Base & Visuals
    GeometricObject::update();
    updateSFMLShape();

  } catch (...) {
    m_isUpdating = false;  // Ensure unlock on error
    throw;
  }

  m_isUpdating = false;  // Unlock
}
// this stays commented out, do not delete now
// void Rectangle::updateSFMLShape() {
//   if (m_isRotatable) {
//     syncRotatableFromAnchors();
//   } else {
//     updateDimensionsFromCorners();
//   }

//   if (m_isRotatable) {
//     double heightAbs = std::abs(m_height);
//     m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(m_width), static_cast<float>(heightAbs)));

//     // --- FIX FOR ROTATED RECTANGLE DRAG FLIP ---
//     // If height is negative, we flip the shape along the Y axis.
//     // This ensures that Vertex 2 (Bottom Right in local coords) physically moves to the
//     // "negative" side relative to the base line, matching the logical coordinates.
//     if (m_height < 0) {
//       m_sfmlShape.setScale(1.0f, -1.0f);
//     } else {
//       m_sfmlShape.setScale(1.0f, 1.0f);
//     }
//     // -------------------------------------------
//   } else {
//     m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(m_width), static_cast<float>(m_height)));
//     m_sfmlShape.setScale(1.0f, 1.0f);  // Reset scale for non-rotatable
//   }

//   m_sfmlShape.setFillColor(m_color);
//   m_sfmlShape.setOutlineThickness(m_thickness);
//   m_sfmlShape.setOutlineColor(sf::Color::Black);

//   if (m_isRotatable) {
//     double cx = CGAL::to_double(m_center.x());
//     double cy = CGAL::to_double(m_center.y());
//     double heightAbs = std::abs(m_height);

//     // Origin is at the center of the rectangle
//     m_sfmlShape.setOrigin(static_cast<float>(m_width * 0.5), static_cast<float>(heightAbs * 0.5));
//     m_sfmlShape.setPosition(static_cast<float>(cx), static_cast<float>(cy));
//     m_sfmlShape.setRotation(static_cast<float>(m_rotationAngle * 180.0 / 3.14159265359));
//   } else {
//     Point_2 c1 = getCorner1Position();
//     Point_2 c2 = getCorner2Position();
//     double x1 = CGAL::to_double(c1.x());
//     double y1 = CGAL::to_double(c1.y());
//     double x2 = CGAL::to_double(c2.x());
//     double y2 = CGAL::to_double(c2.y());

//     double minX = std::min(x1, x2);
//     double minY = std::min(y1, y2);
//     double width = std::abs(x2 - x1);
//     double height = std::abs(y2 - y1);

//     m_sfmlShape.setSize(sf::Vector2f(static_cast<float>(width), static_cast<float>(height)));
//     m_sfmlShape.setOrigin(0.0f, 0.0f);
//     m_sfmlShape.setPosition(sf::Vector2f(static_cast<float>(minX), static_cast<float>(minY)));
//     m_sfmlShape.setRotation(0.0f);
//   }

//   syncDependentCorners();
// }

void Rectangle::setDependentCornerPoints(const std::shared_ptr<Point>& b, const std::shared_ptr<Point>& d) {
  m_cornerB = b;
  m_cornerD = d;

  // Register Rectangle as dependent on B and D corners so we get notified when they change
  // The recursion guard in update() prevents infinite loops
  if (m_cornerB && m_selfHandle) m_cornerB->addDependent(m_selfHandle);
  if (m_cornerD && m_selfHandle) m_cornerD->addDependent(m_selfHandle);

  bool deferB = false;
  bool deferD = false;
  if (m_cornerB) {
    deferB = m_cornerB->isDeferringConstraintUpdates();
    m_cornerB->setDeferConstraintUpdates(true);
  }
  if (m_cornerD) {
    deferD = m_cornerD->isDeferringConstraintUpdates();
    m_cornerD->setDeferConstraintUpdates(true);
  }

  syncDependentCorners();

  if (m_cornerB) m_cornerB->setDeferConstraintUpdates(deferB);
  if (m_cornerD) m_cornerD->setDeferConstraintUpdates(deferD);
}

void Rectangle::syncDependentCorners() {
  if (!m_cornerB && !m_cornerD) return;

  bool deferB = false;
  bool deferD = false;
  if (m_cornerB) {
    deferB = m_cornerB->isDeferringConstraintUpdates();
    m_cornerB->setDeferConstraintUpdates(true);
  }
  if (m_cornerD) {
    deferD = m_cornerD->isDeferringConstraintUpdates();
    m_cornerD->setDeferConstraintUpdates(true);
  }

  if (!m_isRotatable) {
    Point_2 c1 = getCorner1Position();
    Point_2 c2 = getCorner2Position();
    double x1 = CGAL::to_double(c1.x());
    double y1 = CGAL::to_double(c1.y());
    double x2 = CGAL::to_double(c2.x());
    double y2 = CGAL::to_double(c2.y());

    if (m_cornerB) {
      m_cornerB->setCGALPosition(Point_2(FT(x2), FT(y1)));
    }
    if (m_cornerD) {
      m_cornerD->setCGALPosition(Point_2(FT(x1), FT(y2)));
    }

    if (m_cornerB) m_cornerB->setDeferConstraintUpdates(deferB);
    if (m_cornerD) m_cornerD->setDeferConstraintUpdates(deferD);
    return;
  }

  Point_2 a = getCorner1Position();
  Point_2 b = getCorner2Position();
  double dx = CGAL::to_double(b.x()) - CGAL::to_double(a.x());
  double dy = CGAL::to_double(b.y()) - CGAL::to_double(a.y());
  double baseLen = std::sqrt(dx * dx + dy * dy);
  if (baseLen < 1e-9) return;

  double ux = -dy / baseLen;
  double uy = dx / baseLen;

  double heightAbs = std::abs(m_height);
  double heightSign = (m_height >= 0.0) ? 1.0 : -1.0;

  Point_2 c(FT(CGAL::to_double(b.x()) + ux * heightAbs * heightSign), FT(CGAL::to_double(b.y()) + uy * heightAbs * heightSign));
  Point_2 d(FT(CGAL::to_double(a.x()) + ux * heightAbs * heightSign), FT(CGAL::to_double(a.y()) + uy * heightAbs * heightSign));

  if (m_cornerB) m_cornerB->setCGALPosition(c);
  if (m_cornerD) m_cornerD->setCGALPosition(d);

  if (m_cornerB) m_cornerB->setDeferConstraintUpdates(deferB);
}

void Rectangle::draw(sf::RenderWindow& window, float scale, bool forceVisible) const {
  if (!m_visible && !forceVisible) return;

  // Scale the main shape's outline
  sf::ConvexShape scaledShape = m_sfmlShape;
  
  // If styled (dashed/dotted), we disable the SFML outline and draw manually
  bool isStyled = (m_lineStyle != LineStyle::Solid);
  if (isStyled) {
      scaledShape.setOutlineThickness(0);
  } else {
      scaledShape.setOutlineThickness(m_sfmlShape.getOutlineThickness() * scale);
  }

  // GHOST MODE: Apply transparency if hidden but forced visible
  if (!m_visible && forceVisible) {
    sf::Color ghostFill = scaledShape.getFillColor();
    ghostFill.a = 50;  // Faint alpha
    scaledShape.setFillColor(ghostFill);

    sf::Color ghostOutline = scaledShape.getOutlineColor();
    ghostOutline.a = 50;
    scaledShape.setOutlineColor(ghostOutline);
  }

  window.draw(scaledShape);

  // --- Styled Outline Rendering ---
  if (isStyled) {
      auto vertices = getVerticesSFML();
      if (vertices.size() >= 4) {
          sf::Color drawOutlineColor = scaledShape.getOutlineColor();
          float baseThickness = m_thickness;
          if (isSelected()) baseThickness += 2.0f;
          else if (isHovered()) baseThickness += 1.0f;
          float pixelThickness = std::round(baseThickness);
          if (pixelThickness < 1.0f) pixelThickness = 1.0f;

          for (size_t i = 0; i < vertices.size(); ++i) {
              sf::Vector2f p1 = vertices[i];
              sf::Vector2f p2 = vertices[(i + 1) % vertices.size()];
              GeometricObject::drawStyledLine(window, p1, p2, m_lineStyle, pixelThickness, drawOutlineColor);
          }
      }
  }

  // Draw selection highlight if selected
  if (isSelected()) {
    sf::ConvexShape highlight = m_sfmlShape;
    highlight.setFillColor(sf::Color::Transparent);
    highlight.setOutlineThickness(3.0f * scale);
    highlight.setOutlineColor(Constants::SELECTION_COLOR);
    window.draw(highlight);
  } else if (isHovered()) {

    // If specific edge hovered, highlight ONLY that edge, OR highlight shape + edge
    // User requested "highlighted separately from whole shape".
    // We'll draw the whole shape with thinner/alpha cyan, and the edge with thick distinct color.

    sf::ConvexShape highlight = m_sfmlShape;
    highlight.setFillColor(sf::Color::Transparent);
    highlight.setOutlineThickness(1.0f * scale);
    highlight.setOutlineColor(sf::Color(0, 255, 255, 100));  // Dim Cyan
    if (getHoveredEdge() == -1) {
      highlight.setOutlineThickness(2.0f * scale);
      highlight.setOutlineColor(sf::Color::Cyan);  // Full Cyan if generic hover
    }
    window.draw(highlight);

    if (getHoveredEdge() >= 0) {
      auto verts = getVerticesSFML();
      int idx = getHoveredEdge();
      if (idx >= 0 && idx < static_cast<int>(verts.size())) {
        sf::Vector2f p1 = verts[idx];
        sf::Vector2f p2 = verts[(idx + 1) % verts.size()];

        sf::Vector2f dir = p2 - p1;
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len > 0.1f) {
          dir /= len;
          sf::Vector2f perp(-dir.y, dir.x);
          float thickness = 4.0f * scale;

          sf::ConvexShape thickLine;
          thickLine.setPointCount(4);
          thickLine.setPoint(0, p1 + perp * thickness * 0.5f);
          thickLine.setPoint(1, p2 + perp * thickness * 0.5f);
          thickLine.setPoint(2, p2 - perp * thickness * 0.5f);
          thickLine.setPoint(3, p1 - perp * thickness * 0.5f);
          thickLine.setFillColor(sf::Color(255, 100, 50, 200));  // Red-Orange with alpha
          window.draw(thickLine);
        }
      }
    }
  }

  // Delegate drawing to the constituent points
  if (m_corner1) m_corner1->draw(window, scale, forceVisible);
  if (m_cornerB) m_cornerB->draw(window, scale, forceVisible);
  if (m_corner2) m_corner2->draw(window, scale, forceVisible);
  if (m_cornerD) m_cornerD->draw(window, scale, forceVisible);

  drawVertexHandles(window, scale);
}

void Rectangle::updateDependentShape() {
  if (m_corner1) m_corner1->setDeferConstraintUpdates(true);
  if (m_corner2) m_corner2->setDeferConstraintUpdates(true);
  if (m_cornerB) m_cornerB->setDeferConstraintUpdates(true);
  if (m_cornerD) m_cornerD->setDeferConstraintUpdates(true);

  auto parent = m_parentSource.lock();
  if (!parent || !parent->isValid()) {
    updateSFMLShape();
    updateHostedPoints();
    if (m_corner1) m_corner1->forceConstraintUpdate();
    if (m_corner2) m_corner2->forceConstraintUpdate();
    if (m_cornerB) m_cornerB->forceConstraintUpdate();
    if (m_cornerD) m_cornerD->forceConstraintUpdate();
    return;
  }

  auto sourceRect = std::dynamic_pointer_cast<Rectangle>(parent);
  if (!sourceRect || !sourceRect->isValid()) {
    updateSFMLShape();
    updateHostedPoints();
    if (m_corner1) m_corner1->forceConstraintUpdate();
    if (m_corner2) m_corner2->forceConstraintUpdate();
    if (m_cornerB) m_cornerB->forceConstraintUpdate();
    if (m_cornerD) m_cornerD->forceConstraintUpdate();
    return;
  }

  auto aux = m_auxObject.lock();

  auto reflectAcrossLine = [](const Point_2& p, const std::shared_ptr<Line>& line) -> Point_2 {
    Point_2 a = line->getStartPoint();
    Point_2 b = line->getEndPoint();

    Vector_2 ab = b - a;
    double abLenSq = CGAL::to_double(ab.squared_length());
    if (abLenSq < 1e-12) {
      return p;
    }

    Vector_2 ap = p - a;
    FT t = (ap * ab) / ab.squared_length();
    Point_2 h = a + ab * t;
    return p + (h - p) * FT(2.0);
  };

  auto transformPoint = [&](const Point_2& p) -> std::optional<Point_2> {
    switch (m_transformType) {
      case TransformationType::Reflect: {
        auto line = std::dynamic_pointer_cast<Line>(aux);
        if (!line || !line->isValid()) return std::nullopt;
        return reflectAcrossLine(p, line);
      }
      case TransformationType::ReflectPoint: {
        auto center = std::dynamic_pointer_cast<Point>(aux);
        if (!center || !center->isValid()) return std::nullopt;
        Point_2 c = center->getCGALPosition();
        return c + (c - p);
      }
      case TransformationType::ReflectCircle: {
        auto circle = std::dynamic_pointer_cast<Circle>(aux);
        if (!circle || !circle->isValid()) return std::nullopt;
        Point_2 o = circle->getCenterPoint();
        double r = circle->getRadius();
        Vector_2 op = p - o;
        double opLenSq = CGAL::to_double(op.squared_length());
        if (opLenSq < 1e-12) return std::nullopt;
        double scale = (r * r) / opLenSq;
        return o + op * FT(scale);
      }
      case TransformationType::Translate: {
        Vector_2 delta = m_translationVector;
        auto line = std::dynamic_pointer_cast<Line>(aux);
        if (line && line->isValid()) {
          delta = line->getEndPoint() - line->getStartPoint();
        }
        return p + delta;
      }
      case TransformationType::Rotate: {
        auto center = std::dynamic_pointer_cast<Point>(aux);
        if (!center || !center->isValid()) return std::nullopt;
        Point_2 c = center->getCGALPosition();
        // Convert degrees to radians
        double rad = m_transformValue * 3.14159265358979323846 / 180.0;
        double s = std::sin(rad);
        double c_val = std::cos(rad);
        double dx = CGAL::to_double(p.x() - c.x());
        double dy = CGAL::to_double(p.y() - c.y());
        return Point_2(c.x() + FT(dx * c_val - dy * s), c.y() + FT(dx * s + dy * c_val));
      }
      case TransformationType::Dilate: {
        auto center = std::dynamic_pointer_cast<Point>(aux);
        if (!center || !center->isValid()) return std::nullopt;
        Point_2 c = center->getCGALPosition();
        double scale = m_transformValue;
        return c + (p - c) * FT(scale);
      }
      default:
        return p;
    }
  };

  auto verts = sourceRect->getVertices();
  if (verts.size() < 4) {
    setVisible(false);
    return;
  }

  if (m_useExplicitVertices) {
    std::array<std::optional<Point_2>, 4> t;
    for (size_t i = 0; i < 4; ++i) {
      t[i] = transformPoint(verts[i]);
      if (!t[i]) {
        setVisible(false);
        return;
      }
    }

    if (m_isRotatable) {
      // Rotatable explicit order is A(corner1), B(corner2), C(cornerB), D(cornerD)
      if (m_corner1) m_corner1->setCGALPosition(flattenPoint(*t[0]));
      if (m_corner2) m_corner2->setCGALPosition(flattenPoint(*t[1]));
      if (m_cornerB) m_cornerB->setCGALPosition(flattenPoint(*t[2]));
      if (m_cornerD) m_cornerD->setCGALPosition(flattenPoint(*t[3]));
    } else {
      // Axis-aligned explicit order is A(corner1), B(cornerB), C(corner2), D(cornerD)
      if (m_corner1) m_corner1->setCGALPosition(flattenPoint(*t[0]));
      if (m_cornerB) m_cornerB->setCGALPosition(flattenPoint(*t[1]));
      if (m_corner2) m_corner2->setCGALPosition(flattenPoint(*t[2]));
      if (m_cornerD) m_cornerD->setCGALPosition(flattenPoint(*t[3]));
    }

    updateSFMLShape();
    updateHostedPoints();
    if (!hasVisibilityUserOverride()) {
      setVisible(sourceRect->isVisible());
    }
    if (m_corner1) m_corner1->forceConstraintUpdate();
    if (m_corner2) m_corner2->forceConstraintUpdate();
    if (m_cornerB) m_cornerB->forceConstraintUpdate();
    if (m_cornerD) m_cornerD->forceConstraintUpdate();
    return;
  }

  if (!isRotatable()) {
    auto p0 = transformPoint(verts[0]);
    auto p2 = transformPoint(verts[2]);
    if (!p0 || !p2) {
      setVisible(false);
      return;
    }
    setCorner1Position(flattenPoint(*p0), false);  // Don't auto-update
    setCorner2Position(flattenPoint(*p2), false);  // Don't auto-update
    updateSFMLShape();
    updateHostedPoints();  // Notifies our dependents
    setVisible(sourceRect->isVisible());
    if (m_corner1) m_corner1->forceConstraintUpdate();
    if (m_corner2) m_corner2->forceConstraintUpdate();
    if (m_cornerB) m_cornerB->forceConstraintUpdate();
    if (m_cornerD) m_cornerD->forceConstraintUpdate();
    return;
  }

  auto p0 = transformPoint(verts[0]);
  auto p1 = transformPoint(verts[1]);
  auto p3 = transformPoint(verts[3]);
  if (!p0 || !p1 || !p3) {
    setVisible(false);
    return;
  }

  setCorner1Position(flattenPoint(*p0), false);  // Don't auto-update
  setCorner2Position(flattenPoint(*p1), false);  // Don't auto-update

  double dx = CGAL::to_double(p1->x() - p0->x());
  double dy = CGAL::to_double(p1->y() - p0->y());
  double hx = CGAL::to_double(p3->x() - p0->x());
  double hy = CGAL::to_double(p3->y() - p0->y());
  double det = dx * hy - dy * hx;
  double dist = std::sqrt(CGAL::to_double(CGAL::squared_distance(*p0, *p3)));
  double h = (det >= 0) ? dist : -dist;

  setHeight(h);
  if (!hasVisibilityUserOverride()) {
    setVisible(sourceRect->isVisible());
  }
  if (m_corner1) m_corner1->forceConstraintUpdate();
  if (m_corner2) m_corner2->forceConstraintUpdate();
  if (m_cornerB) m_cornerB->forceConstraintUpdate();
  if (m_cornerD) m_cornerD->forceConstraintUpdate();
}

void Rectangle::setColor(const sf::Color& color) {
  m_color = color;
  m_sfmlShape.setFillColor(color);
}

Point_2 Rectangle::getCenter() const { return m_center; }

bool Rectangle::contains(const sf::Vector2f& screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += 2 * tolerance;
  bounds.height += 2 * tolerance;
  return bounds.contains(screenPos);
}

bool Rectangle::isWithinDistance(const sf::Vector2f& screenPos, float tolerance) const {
  sf::FloatRect bounds = m_sfmlShape.getGlobalBounds();
  bounds.left -= tolerance;
  bounds.top -= tolerance;
  bounds.width += tolerance * 2;
  bounds.height += tolerance * 2;
  return bounds.contains(screenPos);
}

void Rectangle::translate(const Vector_2& translation) {
  if (m_useExplicitVertices) {
        // Defer constraint updates on all corners
        bool defer1 = m_corner1 ? m_corner1->isDeferringConstraintUpdates() : false;
        bool deferB = m_cornerB ? m_cornerB->isDeferringConstraintUpdates() : false;
        bool defer2 = m_corner2 ? m_corner2->isDeferringConstraintUpdates() : false;
        bool deferD = m_cornerD ? m_cornerD->isDeferringConstraintUpdates() : false;
        if (m_corner1) m_corner1->setDeferConstraintUpdates(true);
        if (m_cornerB) m_cornerB->setDeferConstraintUpdates(true);
        if (m_corner2) m_corner2->setDeferConstraintUpdates(true);
        if (m_cornerD) m_cornerD->setDeferConstraintUpdates(true);

        // Move all corners
        if (m_corner1) {
            Point_2 p = m_corner1->getCGALPosition();
            m_corner1->setCGALPosition(flattenPoint(Point_2(p.x() + translation.x(), p.y() + translation.y())));
        }
        if (m_cornerB) {
            Point_2 p = m_cornerB->getCGALPosition();
            m_cornerB->setCGALPosition(flattenPoint(Point_2(p.x() + translation.x(), p.y() + translation.y())));
        }
        if (m_corner2) {
            Point_2 p = m_corner2->getCGALPosition();
            m_corner2->setCGALPosition(flattenPoint(Point_2(p.x() + translation.x(), p.y() + translation.y())));
        }
        if (m_cornerD) {
            Point_2 p = m_cornerD->getCGALPosition();
            m_cornerD->setCGALPosition(flattenPoint(Point_2(p.x() + translation.x(), p.y() + translation.y())));
        }
        m_center = flattenPoint(Point_2(m_center.x() + translation.x(), m_center.y() + translation.y()));

        // Restore constraint update flags
        if (m_corner1) m_corner1->setDeferConstraintUpdates(defer1);
        if (m_cornerB) m_cornerB->setDeferConstraintUpdates(deferB);
        if (m_corner2) m_corner2->setDeferConstraintUpdates(defer2);
        if (m_cornerD) m_cornerD->setDeferConstraintUpdates(deferD);

        // Now update once
        updateSFMLShape();
        updateHostedPoints();
        return;
    }

  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  setCorner1Position(flattenPoint(Point_2(c1.x() + translation.x(), c1.y() + translation.y())), false);
  setCorner2Position(flattenPoint(Point_2(c2.x() + translation.x(), c2.y() + translation.y())), false);
  m_center = flattenPoint(Point_2(m_center.x() + translation.x(), m_center.y() + translation.y()));
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
}

void Rectangle::rotateCCW(const Point_2& center, double angleRadians) {
  if (!m_isRotatable) return;

  m_rotationAngle += angleRadians;

  // Rotate corners around the center point
  double centerX = CGAL::to_double(center.x());
  double centerY = CGAL::to_double(center.y());
  double cos_a = std::cos(angleRadians);
  double sin_a = std::sin(angleRadians);

  auto rotatePoint = [centerX, centerY, cos_a, sin_a](const Point_2& p) {
    double x = CGAL::to_double(p.x()) - centerX;
    double y = CGAL::to_double(p.y()) - centerY;
    double newX = x * cos_a - y * sin_a + centerX;
    double newY = x * sin_a + y * cos_a + centerY;
    return Point_2(FT(newX), FT(newY));
  };

  setCorner1Position(rotatePoint(getCorner1Position()), false);
  setCorner2Position(rotatePoint(getCorner2Position()), false);
  m_center = rotatePoint(m_center);
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
}

void Rectangle::setCorners(const Point_2& corner1, const Point_2& corner2) {
  setCorner1Position(corner1, false);
  setCorner2Position(corner2, false);
  updateDimensionsFromCorners();
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
}

std::vector<Point_2> Rectangle::getVertices() const {
  std::vector<Point_2> verts;
  verts.reserve(4);

  if (m_useExplicitVertices && m_corner1 && m_cornerB && m_corner2 && m_cornerD) {
    // Match updateSFMLShape() ordering
    if (m_isRotatable) {
      // Rotatable: A(corner1) → B(corner2) → C(cornerB) → D(cornerD)
      verts.push_back(m_corner1->getCGALPosition());
      verts.push_back(m_corner2->getCGALPosition());
      verts.push_back(m_cornerB->getCGALPosition());
      verts.push_back(m_cornerD->getCGALPosition());
    } else {
      // Axis-aligned: A(corner1) → B(cornerB) → C(corner2) → D(cornerD)
      verts.push_back(m_corner1->getCGALPosition());
      verts.push_back(m_cornerB->getCGALPosition());
      verts.push_back(m_corner2->getCGALPosition());
      verts.push_back(m_cornerD->getCGALPosition());
    }
    return verts;
  }

  if (!m_isRotatable) {
    Point_2 c1 = getCorner1Position();
    Point_2 c2 = getCorner2Position();
    double x1 = CGAL::to_double(c1.x());
    double y1 = CGAL::to_double(c1.y());
    double x2 = CGAL::to_double(c2.x());
    double y2 = CGAL::to_double(c2.y());

    verts.emplace_back(FT(x1), FT(y1));
    verts.emplace_back(FT(x2), FT(y1));
    verts.emplace_back(FT(x2), FT(y2));
    verts.emplace_back(FT(x1), FT(y2));
    return verts;
  }

  // Use SFML points directly
  for (unsigned int i = 0; i < m_sfmlShape.getPointCount(); ++i) {
    sf::Vector2f p = m_sfmlShape.getPoint(i);
    verts.emplace_back(FT(p.x), FT(p.y));
  }
  return verts;
}

std::vector<sf::Vector2f> Rectangle::getVerticesSFML() const {
  std::vector<sf::Vector2f> verts;
  verts.reserve(4);
  auto cgalVerts = getVertices();
  for (const auto& v : cgalVerts) {
    verts.emplace_back(static_cast<float>(CGAL::to_double(v.x())), static_cast<float>(CGAL::to_double(v.y())));
  }
  return verts;
}

void Rectangle::setVertexPosition(size_t index, const Point_2& value) {
  if (index >= 4) return;

  if (m_useExplicitVertices) {
    // Match getVertices() and updateSFMLShape() ordering
    if (m_isRotatable) {
      // Rotatable: 0=corner1(A), 1=corner2(B), 2=cornerB(C), 3=cornerD(D)
      switch (index) {
        case 0:
          if (m_corner1) m_corner1->setCGALPosition(flattenPoint(value));
          break;
        case 1:
          if (m_corner2) m_corner2->setCGALPosition(flattenPoint(value));
          break;
        case 2:
          if (m_cornerB) m_cornerB->setCGALPosition(flattenPoint(value));
          break;
        case 3:
          if (m_cornerD) m_cornerD->setCGALPosition(flattenPoint(value));
          break;
      }
    } else {
      // Axis-aligned: 0=corner1(A), 1=cornerB(B), 2=corner2(C), 3=cornerD(D)
      switch (index) {
        case 0:
          if (m_corner1) m_corner1->setCGALPosition(flattenPoint(value));
          break;
        case 1:
          if (m_cornerB) m_cornerB->setCGALPosition(flattenPoint(value));
          break;
        case 2:
          if (m_corner2) m_corner2->setCGALPosition(flattenPoint(value));
          break;
        case 3:
          if (m_cornerD) m_cornerD->setCGALPosition(flattenPoint(value));
          break;
      }
    }
    updateSFMLShape();
    updateHostedPoints();
    return;
  }

  if (m_isRotatable) {
    Point_2 a = getCorner1Position();
    Point_2 b = getCorner2Position();

    if (index == 0) {
      setCorner1Position(value, false);  // Don't auto-update, we'll do it below
      updateSFMLShape();
      updateHostedPoints();  // Notifies dependents
      return;
    }
    if (index == 1) {
      setCorner2Position(value, false);  // Don't auto-update, we'll do it below
      updateSFMLShape();
      updateHostedPoints();  // Notifies dependents
      return;
    }

    // For vertices 2 (C) and 3 (D), we need to adjust the height
    // The base edge is A->B (corner1->corner2)
    double dx = CGAL::to_double(b.x()) - CGAL::to_double(a.x());
    double dy = CGAL::to_double(b.y()) - CGAL::to_double(a.y());
    double length = std::sqrt(dx * dx + dy * dy);
    const double minExtent = 1e-3;
    if (length < minExtent) return;

    // Normal vector perpendicular to the base (A->B), pointing "up"
    double nx = -dy / length;
    double ny = dx / length;

    // For index 2 (vertex C): measure from B
    // For index 3 (vertex D): measure from A
    double vx, vy;
    if (index == 2) {
      // Vertex C: calculate vector from B to the dragged position
      vx = CGAL::to_double(value.x()) - CGAL::to_double(b.x());
      vy = CGAL::to_double(value.y()) - CGAL::to_double(b.y());
    } else {
      // Vertex D: calculate vector from A to the dragged position
      vx = CGAL::to_double(value.x()) - CGAL::to_double(a.x());
      vy = CGAL::to_double(value.y()) - CGAL::to_double(a.y());
    }

    // Project the vector onto the normal to get signed distance
    double signedDist = vx * nx + vy * ny;

    // Allow crossing the center line (changing sign), but prevent exact zero/degenerate height
    if (std::abs(signedDist) < minExtent) {
      signedDist = (signedDist >= 0.0 ? 1.0 : -1.0) * minExtent;
    }

    // Delegate height/center logic to setHeight (which calls updateSFMLShape and updateHostedPoints)
    setHeight(signedDist);
    // Note: setHeight() already calls updateSFMLShape() and updateHostedPoints()
    // No need to call updateSFMLShape() again here
    return;
  }

  double newX = CGAL::to_double(value.x());
  double newY = CGAL::to_double(value.y());

  switch (index) {
    case 0:  // Dragging Corner 1
      setCorner1Position(value, false);
      break;
    case 1:  // Dragging Corner (C2.x, C1.y)
      setCorner2Position(Point_2(FT(newX), getCorner2Position().y()), false);
      setCorner1Position(Point_2(getCorner1Position().x(), FT(newY)), false);
      break;
    case 2:  // Dragging Corner 2
      setCorner2Position(value, false);
      break;
    case 3:  // Dragging Corner (C1.x, C2.y)
      setCorner1Position(Point_2(FT(newX), getCorner1Position().y()), false);
      setCorner2Position(Point_2(getCorner2Position().x(), FT(newY)), false);
      break;
  }

  Point_2 finalC1 = getCorner1Position();
  Point_2 finalC2 = getCorner2Position();
  double finalX1 = CGAL::to_double(finalC1.x());
  double finalY1 = CGAL::to_double(finalC1.y());
  double finalX2 = CGAL::to_double(finalC2.x());
  double finalY2 = CGAL::to_double(finalC2.y());

  m_center = Point_2(FT((finalX1 + finalX2) * 0.5), FT((finalY1 + finalY2) * 0.5));

  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setRotation(double angleRadians) {
  m_rotationAngle = angleRadians;
  updateSFMLShape();
  updateHostedPoints();
}

void Rectangle::setHeight(double height) {
  if (m_isRotatable) {
    // if (!std::isfinite(height)) return;
    if (!(height > -1e100 && height < 1e100)) return;  // Robust NaN/Inf check
    const double minExtent = 1e-3;
    if (std::abs(height) < minExtent) {
      height = (height >= 0.0 ? 1.0 : -1.0) * minExtent;
    }
    m_height = height;

    // Force update of C and D positions for explicit vertices
    if (m_corner1 && m_corner2 && m_cornerB && m_cornerD) {
      Point_2 pA = m_corner1->getCGALPosition();
      Point_2 pB = m_corner2->getCGALPosition(); // Base End (B)
      // Note: m_cornerB is C, m_cornerD is D
      
      double dx = CGAL::to_double(pB.x() - pA.x());
      double dy = CGAL::to_double(pB.y() - pA.y());
      double len = std::sqrt(dx*dx + dy*dy);
      
      if (len > 1e-9) {
         double invLen = 1.0/len;
         // Normal vector (Rotated 90 deg CCW: -y, x)
         // ux = dx/len, uy = dy/len
         // vx = -uy, vy = ux
         double nx = -dy * invLen;
         double ny = dx * invLen;
         
         // C = B + Normal * height
         double cx = CGAL::to_double(pB.x()) + nx * height;
         double cy = CGAL::to_double(pB.y()) + ny * height;
         Point_2 newC = Point_2(cx, cy);
         
         // D = A + Normal * height
         double dx = CGAL::to_double(pA.x()) + nx * height;
         double dy = CGAL::to_double(pA.y()) + ny * height;
         Point_2 newD = Point_2(dx, dy);
         
         m_cornerB->setCGALPosition(newC); // C
         m_cornerD->setCGALPosition(newD); // D
      }
    }

  } else {
    // if (!std::isfinite(height)) return;
    if (!(height > -1e100 && height < 1e100)) return;  // Robust NaN/Inf check
    m_height = std::abs(height);
    
    // Update D and B for Axis Aligned
    if (m_cornerD && m_corner1) {
        // D is (x1, y1 + height)? No, (x1, y2).
        // A is (x1, y1). D is (x1, y1 + h)?
        // m_cornerB is (x1+w, y1).
        // Let's assume BL is A.
        Point_2 p1 = m_corner1->getCGALPosition();
    }
  }
  updateSFMLShape();
  updateHostedPoints();
}
sf::FloatRect Rectangle::getGlobalBounds() const { return m_sfmlShape.getGlobalBounds(); }

Point_2 Rectangle::getCGALPosition() const { return m_center; }

void Rectangle::setCGALPosition(const Point_2& newPos) {
  Vector_2 translation = newPos - getCGALPosition();
  Point_2 c1 = getCorner1Position();
  Point_2 c2 = getCorner2Position();
  setCorner1Position(flattenPoint(Point_2(c1.x() + translation.x(), c1.y() + translation.y())), false);
  setCorner2Position(flattenPoint(Point_2(c2.x() + translation.x(), c2.y() + translation.y())), false);
  m_center = flattenPoint(Point_2(m_center.x() + translation.x(), m_center.y() + translation.y()));
  updateSFMLShape();
  updateHostedPoints();  // Notifies dependents
}

void Rectangle::setPosition(const sf::Vector2f& newSfmlPos) {
  // Get current position
  sf::Vector2f currentPos = m_sfmlShape.getPosition();

  // Calculate translation
  sf::Vector2f translation = newSfmlPos - currentPos;

  // Translate the shape
  Vector_2 delta(translation.x, translation.y);
  translate(delta);
}

void Rectangle::drawVertexHandles(sf::RenderWindow& window, float scale) const {
  const float handleRadius = m_vertexHandleSize * scale;
  auto verts = getVertices();

  for (size_t i = 0; i < verts.size(); ++i) {
    sf::CircleShape handle(handleRadius);
    handle.setOrigin(handleRadius, handleRadius);
    float x = static_cast<float>(CGAL::to_double(verts[i].x()));
    float y = static_cast<float>(CGAL::to_double(verts[i].y()));
    handle.setPosition(x, y);

    sf::Color base = sf::Color(180, 180, 180);
    if (static_cast<int>(i) == m_activeVertex) {
      base = sf::Color(255, 140, 0);
    } else if (static_cast<int>(i) == m_hoveredVertex) {
      base = sf::Color::Yellow;
    }
    handle.setFillColor(base);
    handle.setOutlineThickness(1.0f * scale);
    handle.setOutlineColor(sf::Color::Black);
    window.draw(handle);
  }
}

void Rectangle::drawLabel(sf::RenderWindow& window, const sf::View& worldView) const {
  if (!isVisible() || getLabelMode() == LabelMode::Hidden || !Point::commonFont) return;

  // 1. Delegate label drawing to the constituent corner points
  if (m_corner1) m_corner1->drawLabelExplicit(window, worldView);
  if (m_cornerB) m_cornerB->drawLabelExplicit(window, worldView);
  if (m_corner2) m_corner2->drawLabelExplicit(window, worldView);
  if (m_cornerD) m_cornerD->drawLabelExplicit(window, worldView);

  // 2. Draw the rectangle's own label (e.g., name or area) at center
  std::string labelStr = "";
  switch (getLabelMode()) {
    case LabelMode::Name:
      labelStr = getLabel();
      break;
    case LabelMode::Value: {
      double area = m_width * std::abs(m_height);
      labelStr = std::to_string(static_cast<int>(std::round(area)));
      break;
    }
    case LabelMode::NameAndValue: {
      labelStr = getLabel();
      double area = m_width * std::abs(m_height);
      labelStr += (labelStr.empty() ? "" : " = ") + std::to_string(static_cast<int>(std::round(area)));
      break;
    }
    case LabelMode::Caption:
      labelStr = getCaption();
      break;
    default:
      break;
  }

  if (labelStr.empty()) return;

  sf::Vector2i screenPos = window.mapCoordsToPixel(Point::cgalToSFML(m_center), worldView);

  sf::Text text;
  text.setFont(*Point::commonFont);
  text.setString(sf::String::fromUtf8(labelStr.begin(), labelStr.end()));
  text.setCharacterSize(LabelManager::instance().getFontSize());

  sf::Color textColor = m_color;
  if (textColor.r > 200 && textColor.g > 200 && textColor.b > 200) textColor = sf::Color::Black;
  text.setFillColor(textColor);

  text.setPosition(static_cast<float>(screenPos.x), static_cast<float>(screenPos.y));
  sf::FloatRect bounds = text.getLocalBounds();
  text.setOrigin(bounds.width / 2.0f, bounds.height / 2.0f);

  window.draw(text);
}

std::vector<Point_2> Rectangle::getInteractableVertices() const {
  if (!m_isRotatable) {
    std::vector<Point_2> verts;
    verts.reserve(4);
    Point_2 c1 = getCorner1Position();
    Point_2 c2 = getCorner2Position();
    verts.emplace_back(c1);
    verts.emplace_back(Point_2(c2.x(), c1.y()));
    verts.emplace_back(c2);
    verts.emplace_back(Point_2(c1.x(), c2.y()));
    return verts;
  }
  return getVertices();
}

std::vector<Segment_2> Rectangle::getEdges() const {
  auto verts = getVertices();
  std::vector<Segment_2> edges;
  edges.reserve(4);

  if (verts.size() == 4) {
    // Create 4 edges connecting consecutive vertices
    edges.emplace_back(verts[0], verts[1]);
    edges.emplace_back(verts[1], verts[2]);
    edges.emplace_back(verts[2], verts[3]);
    edges.emplace_back(verts[3], verts[0]);
  }

  return edges;
}

void Rectangle::setVertexLabelOffset(size_t vertexIndex, const sf::Vector2f& offset) {
  // Ensure vector is sized appropriately
  if (m_vertexLabelOffsets.size() < 4) {
    m_vertexLabelOffsets.resize(4, sf::Vector2f(0.f, 0.f));
  }
  if (vertexIndex < 4) {
    m_vertexLabelOffsets[vertexIndex] = offset;
    // Sync to Point if exists
    if (vertexIndex == 0 && m_corner1) m_corner1->setLabelOffset(offset);
    if (vertexIndex == 1 && m_cornerB) m_cornerB->setLabelOffset(offset);
    if (vertexIndex == 2 && m_corner2) m_corner2->setLabelOffset(offset);
    if (vertexIndex == 3 && m_cornerD) m_cornerD->setLabelOffset(offset);
  }
}

sf::Vector2f Rectangle::getVertexLabelOffset(size_t vertexIndex) const {
  if (vertexIndex < m_vertexLabelOffsets.size()) {
    return m_vertexLabelOffsets[vertexIndex];
  }
  return sf::Vector2f(0.f, 0.f);
}
