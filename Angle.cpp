#include "Angle.h"
#include <algorithm>
#include <cmath>
#include <iostream>

sf::Font Angle::s_font;
bool Angle::s_fontLoaded = false;

Angle::Angle(const std::shared_ptr<Point> &a, const std::shared_ptr<Point> &vertex,
             const std::shared_ptr<Point> &b, bool reflex, const sf::Color &color)
    : GeometricObject(ObjectType::Angle, color),
      m_pointA(a),
      m_vertex(vertex),
      m_pointB(b),
      m_isReflex(reflex) {
  // Init colors
  m_fillColor = color;
  m_fillColor.a = 50;
  m_outlineColor = color;
  m_outlineColor.a = 255;
      
  if (!s_fontLoaded) {
    s_fontLoaded = s_font.loadFromFile(Constants::DEFAULT_FONT_PATH);
    if (!s_fontLoaded) {
      std::cerr << "Angle: Failed to load font from " << Constants::DEFAULT_FONT_PATH << std::endl;
    }
  }
  if (s_fontLoaded) {
    m_text.setFont(s_font);
  }
  m_text.setCharacterSize(Constants::BUTTON_TEXT_SIZE);
  m_text.setFillColor(sf::Color::Black);
  updateSFMLShape();
}


void Angle::setReflex(bool reflex) {
  if (m_isReflex != reflex) {
    m_isReflex = reflex;
    updateSFMLShape();
  }
}

bool Angle::resolvePoints(Point_2 &a, Point_2 &v, Point_2 &b) const {
  auto spA = m_pointA.lock();
  auto spV = m_vertex.lock();
  auto spB = m_pointB.lock();
  if (!spA || !spV || !spB) return false;
  if (!spA->isValid() || !spV->isValid() || !spB->isValid()) return false;
  if (!spA->isVisible() || !spV->isVisible() || !spB->isVisible()) return false;
  a = spA->getCGALPosition();
  v = spV->getCGALPosition();
  b = spB->getCGALPosition();
  return true;
}

double Angle::normalizeSignedPi(double angleRad) {
  constexpr double kPi = 3.14159265358979323846;
  while (angleRad <= -kPi) angleRad += 2.0 * kPi;
  while (angleRad > kPi) angleRad -= 2.0 * kPi;
  return angleRad;
}

double Angle::normalizePositive(double angleRad) {
  constexpr double kPi = 3.14159265358979323846;
  while (angleRad < 0.0) angleRad += 2.0 * kPi;
  while (angleRad >= 2.0 * kPi) angleRad -= 2.0 * kPi;
  return angleRad;
}

void Angle::updateSFMLShape() {
  if (!isVisible()) return;

  Point_2 a, v, b;
  if (!resolvePoints(a, v, b)) {
    m_arc.clear();
    m_fillFan.clear();
    m_currentDegrees = 0.0f;
    m_isValid = false;
    return;
  }

  m_isValid = true;

  m_vertexPoint = v;

  double ax = CGAL::to_double(a.x());
  double ay = CGAL::to_double(a.y());
  double vx = CGAL::to_double(v.x());
  double vy = CGAL::to_double(v.y());
  double bx = CGAL::to_double(b.x());
  double by = CGAL::to_double(b.y());

  double vax = ax - vx;
  double vay = ay - vy;
  double vbx = bx - vx;
  double vby = by - vy;

  double lenA = std::sqrt(vax * vax + vay * vay);
  double lenB = std::sqrt(vbx * vbx + vby * vby);
  if (lenA < 1e-6 || lenB < 1e-6) {
    m_arc.clear();
    m_fillFan.clear();
    m_currentDegrees = 0.0f;
    m_isValid = false;
    return;
  }

  double angleA = std::atan2(vay, vax);
  double angleB = std::atan2(vby, vbx);

  double delta = angleB - angleA;
  double sweepInner = normalizeSignedPi(delta);
  constexpr double kPi = 3.14159265358979323846;
  double sweepReflex = (sweepInner >= 0.0) ? (sweepInner - 2.0 * kPi) : (sweepInner + 2.0 * kPi);

  m_startAngle = angleA;
  m_sweepAngle = m_isReflex ? sweepReflex : sweepInner;

  double sweepAbs = std::abs(m_sweepAngle);
  m_currentDegrees = static_cast<float>(sweepAbs * 180.0 / kPi);

  // Resize: Use custom radius if set, else auto-calculate
  if (m_customRadius > 0) {
      m_arcRadius = m_customRadius;
  } else {
      m_arcRadius = std::max(15.0, std::min(lenA, lenB) * 0.3);
  }

  // --- VISUALIZATION SETUP ---
  bool hasFill = (m_color.a > 0); // Only generate fill geometry if alpha > 0
  double cx = vx;
  double cy = vy;

  // --- RIGHT ANGLE VISUALIZATION ---
  bool isRightAngle = std::abs(m_currentDegrees - 90.0f) < 0.1f;
  
  if (isRightAngle) {
    // Render Square Symbol
    m_arc.clear();
    m_arc.setPrimitiveType(sf::LineStrip);
    m_arc.resize(5); // V -> P1 -> P2 -> P3 -> V (Closed loop or just open?)
                 // Typically right angle symbol is just the two internal lines.
                 // Let's do: Start(on A side) -> Corner -> End(on B side)
                 // But typically it's a square.
                 // Points: P_A = V + dirA * R
                 //         P_Corner = V + dirA * R + dirB * R
                 //         P_B = V + dirB * R
                 // Path: P_A -> P_Corner -> P_B
    
    double dirAx = std::cos(m_startAngle);
    double dirAy = std::sin(m_startAngle);
    double dirBx = std::cos(m_startAngle + m_sweepAngle); // dirB
    double dirBy = std::sin(m_startAngle + m_sweepAngle);

    // If reflex, we might need to be careful, but 90 degrees usually isn't reflex in this context unless explicit.
    // The sweep handles direction naturally.

    double pxA = cx + dirAx * m_arcRadius;
    double pyA = cy + dirAy * m_arcRadius;

    double pxB = cx + dirBx * m_arcRadius;
    double pyB = cy + dirBy * m_arcRadius;

    double pxCorner = cx + (dirAx + dirBx) * m_arcRadius; 
    // Actually exact square corner is V + dA*R + dB*R
    // Check math: If A is X-axis, B is Y-axis. V=0.
    // dirA=(1,0), dirB=(0,1). P_A=(R,0), P_B=(0,R).
    // P_Corner = (R, R). Correct.
    
    double pyCorner = cy + (dirAy + dirBy) * m_arcRadius;

    // We can draw the full square or just the L-bracket. 
    // Standard is L-bracket from the sides.
    // Let's draw: P_A -> P_Corner -> P_B.
    
    m_arc.resize(3);
    m_arc[0].position = sf::Vector2f(static_cast<float>(pxA), static_cast<float>(pyA));
    m_arc[1].position = sf::Vector2f(static_cast<float>(pxCorner), static_cast<float>(pyCorner));
    m_arc[2].position = sf::Vector2f(static_cast<float>(pxB), static_cast<float>(pyB));

    for(int i=0; i<3; ++i) m_arc[i].color = m_outlineColor;

    // Fill for Right Angle (Square sector)
    if (hasFill) {
      m_fillFan.clear();
      m_fillFan.setPrimitiveType(sf::Quads); // Just a quad
      m_fillFan.resize(4);
      m_fillFan[0].position = sf::Vector2f(static_cast<float>(vx), static_cast<float>(vy)); // V
      m_fillFan[1].position = sf::Vector2f(static_cast<float>(pxA), static_cast<float>(pyA));
      m_fillFan[2].position = sf::Vector2f(static_cast<float>(pxCorner), static_cast<float>(pyCorner));
      m_fillFan[3].position = sf::Vector2f(static_cast<float>(pxB), static_cast<float>(pyB));
      
      sf::Color transparentFill = m_fillColor;
      transparentFill.a = 50;
      for(int i=0; i<4; ++i) m_fillFan[i].color = transparentFill;
    }

  } else {
      // --- STANDARD ARC VISUALIZATION ---
      int segments = std::max(12, static_cast<int>(sweepAbs / (kPi / 18.0)));
      
      // Update Outline (Arc)
      m_arc.clear();
      m_arc.setPrimitiveType(sf::LineStrip);
      m_arc.resize(static_cast<size_t>(segments + 1));

      // Update Fill (Sector)
      m_fillFan.clear();
      m_fillFan.setPrimitiveType(sf::TriangleFan);
      if (hasFill) {
          m_fillFan.resize(static_cast<size_t>(segments + 2)); // Center + arc points
          m_fillFan[0].position = sf::Vector2f(static_cast<float>(vx), static_cast<float>(vy));
         sf::Color transparentFill = m_fillColor;
          transparentFill.a = 50;
          m_fillFan[0].color = transparentFill;
      }

      for (int i = 0; i <= segments; ++i) {
        double t = (segments == 0) ? 0.0 : (static_cast<double>(i) / segments);
        double ang = m_startAngle + m_sweepAngle * t;
        float px = static_cast<float>(cx + m_arcRadius * std::cos(ang));
        float py = static_cast<float>(cy + m_arcRadius * std::sin(ang));
        
        // Outline point
        m_arc[static_cast<size_t>(i)].position = sf::Vector2f(px, py);
        m_arc[static_cast<size_t>(i)].color = m_outlineColor;

        // Fill point
        if (hasFill) {
            m_fillFan[static_cast<size_t>(i+1)].position = sf::Vector2f(px, py);
            sf::Color transparentFill = m_fillColor;
            transparentFill.a = 50;
            m_fillFan[static_cast<size_t>(i+1)].color = transparentFill;
        }
      }
  }

  float midAngle = static_cast<float>(m_startAngle + m_sweepAngle * 0.5);
  float textRadius = static_cast<float>(m_arcRadius + 12.0);
  sf::Vector2f textPos(static_cast<float>(cx) + textRadius * std::cos(midAngle),
                       static_cast<float>(cy) + textRadius * std::sin(midAngle));
  m_text.setString(std::to_string(static_cast<int>(std::round(m_currentDegrees))) + "\xC2\xB0");
  m_text.setPosition(textPos);

  // Update logic for interaction handles
  m_resizeHandle.setRadius(4.0f);
  m_resizeHandle.setOrigin(4.0f, 4.0f);
  float hx = static_cast<float>(cx + m_arcRadius * std::cos(midAngle));
  float hy = static_cast<float>(cy + m_arcRadius * std::sin(midAngle));
  m_resizeHandle.setPosition(hx, hy);
  m_resizeHandle.setFillColor(sf::Color(100, 100, 255, 200));
  m_resizeHandle.setOutlineColor(sf::Color::Black);
  m_resizeHandle.setOutlineThickness(1.0f);

  sf::FloatRect bounds;
  if (!m_arc.getVertexCount()) {
    bounds = sf::FloatRect();
  } else {
    float minX = m_arc[0].position.x;
    float minY = m_arc[0].position.y;
    float maxX = minX;
    float maxY = minY;
    // Include center for sector bounds
    if (hasFill) {
        minX = std::min(minX, (float)cx);
        minY = std::min(minY, (float)cy);
        maxX = std::max(maxX, (float)cx);
        maxY = std::max(maxY, (float)cy);
    }
    for (size_t i = 1; i < m_arc.getVertexCount(); ++i) {
      minX = std::min(minX, m_arc[i].position.x);
      minY = std::min(minY, m_arc[i].position.y);
      maxX = std::max(maxX, m_arc[i].position.x);
      maxY = std::max(maxY, m_arc[i].position.y);
    }
    bounds = sf::FloatRect(minX, minY, maxX - minX, maxY - minY);
  }
  m_bounds = bounds;
}

void Angle::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  if (!isVisible() && !forceVisible) return;
  if (!m_isValid) return;

  // GHOST MODE logic
  sf::Color overrideColor = m_color;
  bool isGhost = (!isVisible() && forceVisible);
  if (isGhost) {
      overrideColor.a = 50;
  }

    // Draw Fill
    if (m_fillFan.getVertexCount() > 0 && !isGhost) {
       bool emphasize = isSelected() || isHovered();
       if (emphasize) {
         sf::VertexArray highlightFan = m_fillFan;
         for (size_t i = 0; i < highlightFan.getVertexCount(); ++i) {
           auto c = highlightFan[i].color;
           c.a = std::max<sf::Uint8>(c.a, 100);
           highlightFan[i].color = c;
         }
         window.draw(highlightFan);
       } else {
         window.draw(m_fillFan);
       }
    }

  // Draw Outline (Arc)
  if (m_arc.getVertexCount() > 0) {
      if (isGhost) {
          sf::VertexArray ghostArc = m_arc;
           for(size_t i=0; i<ghostArc.getVertexCount(); ++i) {
               ghostArc[i].color = overrideColor;
           }
           window.draw(ghostArc);
      } else {
          window.draw(m_arc);
      }
  }

  // Draw Text (only if fully visible)
  if (s_fontLoaded && !isGhost) {
    window.draw(m_text);
  }

  // Selection/Hover highlight logic could be added here similar to other shapes
  if (isSelected() || isHovered()) {
      drawVertexHandles(window, scale);
  }
}

void Angle::drawVertexHandles(sf::RenderWindow &window, float scale) const {
    // scale handles
    sf::CircleShape handle = m_resizeHandle;
    float r = handle.getRadius() * scale;
    handle.setRadius(r);
    handle.setOrigin(r, r);
    // Position is already set in updateSFMLShape but we might need to verify if position scaling is needed?
    // Angles are world space, so position is correct.
    handle.setOutlineThickness(1.0f * scale);
    window.draw(handle);
}

void Angle::setRadius(double radius) {
    m_customRadius = radius;
    updateSFMLShape();
}

bool Angle::contains(const sf::Vector2f &worldPos, float tolerance) const {
  if (!isVisible()) return false;
  if (!m_arc.getVertexCount()) return false;

  double cx = CGAL::to_double(m_vertexPoint.x());
  double cy = CGAL::to_double(m_vertexPoint.y());
  double dx = worldPos.x - cx;
  double dy = worldPos.y - cy;
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist > m_arcRadius + tolerance) return false;

  double ang = std::atan2(dy, dx);
  double start = normalizePositive(m_startAngle);
  double sweep = m_sweepAngle;
  double end = normalizePositive(m_startAngle + sweep);
  double target = normalizePositive(ang);

  if (sweep >= 0.0) {
    if (start <= end) {
      return target >= start && target <= end;
    }
    return target >= start || target <= end;
  }

  if (start >= end) {
    return target <= start && target >= end;
  }
  return target <= start || target >= end;
}

bool Angle::isMouseOverArc(const sf::Vector2f &worldPos, float tolerance) const {
  if (!isVisible()) return false;
  if (!m_arc.getVertexCount()) return false;

  double cx = CGAL::to_double(m_vertexPoint.x());
  double cy = CGAL::to_double(m_vertexPoint.y());
  double dx = worldPos.x - cx;
  double dy = worldPos.y - cy;
  double dist = std::sqrt(dx * dx + dy * dy);
  
  // Specific check for arc radius alignment
  if (std::abs(dist - m_arcRadius) > tolerance) return false;

  double ang = std::atan2(dy, dx);
  double start = normalizePositive(m_startAngle);
  double sweep = m_sweepAngle;
  double end = normalizePositive(m_startAngle + sweep);
  double target = normalizePositive(ang);

  if (sweep >= 0.0) {
    if (start <= end) {
      return target >= start && target <= end;
    }
    return target >= start || target <= end;
  }

  if (start >= end) {
    return target <= start && target >= end;
  }
  return target <= start || target >= end;
}

void Angle::setColor(const sf::Color& c) {
    GeometricObject::setColor(c); // Update base m_color

    // Update visual components
    m_fillColor = c;
    m_fillColor.a = 50; // Force low alpha for the wedge/fill

    m_outlineColor = c;
    m_outlineColor.a = 255; // Full alpha for the arc line

    // Rebuild or update the VertexArray colors immediately
    updateSFMLShape();
}
