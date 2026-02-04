# Infinite Line Rendering: View-Space Re-centering

## The Problem
Rendering infinite lines using large fixed coordinates (e.g., `±1,000,000`) causes visual artifacts when zooming in.
1.  **Jitter:** Floating-point precision loss at high coordinates causes the line to shake.
2.  **Disappearance:** If the "anchor" point of the line is far off-screen, the graphics pipeline may clip the entire line, even if it passes through the current view.

## The Solution
We use **View-Space Re-centering**. Instead of drawing the line from its original mathematical definition, we calculate the segment of the line that passes through the **Current Viewport**.

### Algorithm
1.  **Calculate Center:** Find the center of the current camera view ($C$).
2.  **Project:** Find the point on the infinite line closest to $C$. Let's call this anchor $M$ (Midpoint).
3.  **Extend:** Draw the line from $M - E$ to $M + E$, where $E$ is a length large enough to cover the screen diagonal.

### Code Implementation (`Line::draw`)
```cpp
// 1. Get View Metrics
sf::Vector2f viewCenter = currentView.getCenter();
float viewDiagonal = length(currentView.getSize());
float extension = viewDiagonal * 2.0f;

// 2. Normalize Line Direction
sf::Vector2f dir = normalize(p2 - p1);

// 3. Find Anchor Point (M) closest to View Center
sf::Vector2f toCenter = viewCenter - p1;
float proj = dot(toCenter, dir);
sf::Vector2f mid = p1 + dir * proj;

// 4. Draw from M outward
drawStart = mid - dir * extension;
drawEnd   = mid + dir * extension;

==============================================

# Infinite Line Rendering: Dynamic View Clipping

## The Concept
Instead of drawing a "really long line," this method calculates the exact mathematical intersection between the infinite line and the rectangular camera view. It then draws a finite segment *exactly* matching the visible portion of the line.

## Algorithm (Liang-Barsky approach)
1.  **Define View Bounds:** Get the top-left $(x_{min}, y_{min})$ and bottom-right $(x_{max}, y_{max})$ coordinates of the current camera view.
2.  **Define Line:** Represent the line parametrically as $P(t) = P_0 + t \cdot \vec{D}$.
3.  **Calculate Intersections:**
    * For each of the 4 view edges (Left, Right, Top, Bottom), calculate the $t$ value where the line crosses that edge.
    * Determine the entry point ($t_{enter}$) and exit point ($t_{exit}$) into the visible rectangle.
4.  **Draw:**
    * If $t_{enter} \le t_{exit}$, the line is visible.
    * Draw segment from $P_0 + t_{enter} \cdot \vec{D}$ to $P_0 + t_{exit} \cdot \vec{D}$.

## Code Implementation
```cpp
// 1. Get View Bounds
sf::FloatRect viewBounds = ...; // Current view rectangle

// 2. Define Parametric Line (P + t*D)
sf::Vector2f p1 = ...;
sf::Vector2f dir = ...;

// 3. Clip against axes (Simplified Liang-Barsky)
float t_enter = -infinity;
float t_exit  = +infinity;

auto clip = [&](float p, float dir, float min, float max) {
    if (abs(dir) < epsilon) return (p >= min && p <= max); // Parallel line
    float t1 = (min - p) / dir;
    float t2 = (max - p) / dir;
    t_enter = std::max(t_enter, std::min(t1, t2));
    t_exit  = std::min(t_exit,  std::max(t1, t2));
    return true;
};

if (clip(p1.x, dir.x, viewBounds.left, viewBounds.right) &&
    clip(p1.y, dir.y, viewBounds.top,  viewBounds.bottom)) {
    
    // 4. Draw Valid Segment
    if (t_enter <= t_exit) {
        drawStart = p1 + dir * t_enter;
        drawEnd   = p1 + dir * t_exit;
    }
}

## Old function implementation works very well but we replaced with even better !
void Line::draw(sf::RenderWindow &window, float scale, bool forceVisible) const {
  try {
    if ((!m_visible && !forceVisible) || (!m_startPoint || !m_startPoint->isValid()) ||
        (m_endPoint && !m_endPoint->isValid())) {
      return;
    }

    // --- COLOR & STATE SETUP ---
    sf::Color drawColor = m_color;
    if (m_selected) {
      drawColor = Constants::SELECTION_COLOR;
    } else if (m_hovered) {
      drawColor = Constants::HOVER_COLOR;
    }
    // Ghost mode transparency
    if (!m_visible && forceVisible) {
      drawColor.a = 50;
    }

    float basePixelThickness = Constants::LINE_THICKNESS_DEFAULT;
    if (m_selected) basePixelThickness = 4.0f;
    else if (m_hovered) basePixelThickness = 3.0f;
    
    // Convert thickness to world units
    float worldThickness = basePixelThickness * scale;

    // --- COORDINATE CALCULATION ---
    const Point_2 startCgal = m_startPoint->getCGALPosition();
    const Point_2 endCgal = m_endPoint->getCGALPosition();
    sf::Vector2f p1(static_cast<float>(CGAL::to_double(startCgal.x())),
                    static_cast<float>(CGAL::to_double(startCgal.y())));
    sf::Vector2f p2(static_cast<float>(CGAL::to_double(endCgal.x())),
                    static_cast<float>(CGAL::to_double(endCgal.y())));

    sf::Vector2f dir = p2 - p1;
    float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (len < 1e-9f) return; // Degenerate
    sf::Vector2f unitDir = dir / len;

    // --- TYPE-SPECIFIC DRAWING LOGIC ---
    sf::Vector2f drawStart = p1;
    sf::Vector2f drawEnd = p2;

    ObjectType type = getType(); // Line, LineSegment, Ray, Vector

    if (m_lineType == LineType::Infinite || type == ObjectType::Line) {
        // Infinite Line: Extend both ways
        sf::View currentView = window.getView();
        sf::Vector2f viewSize = currentView.getSize();
        float viewDiagonal = std::sqrt(viewSize.x * viewSize.x + viewSize.y * viewSize.y);
        float extension = viewDiagonal * 2.0f;
        drawStart = p1 - unitDir * extension;
        drawEnd = p2 + unitDir * extension;
    } 
    else if (m_lineType == LineType::Ray || type == ObjectType::Ray) {
        // Ray: Extend only from Start towards End
        sf::View currentView = window.getView();
        sf::Vector2f viewSize = currentView.getSize();
        float viewDiagonal = std::sqrt(viewSize.x * viewSize.x + viewSize.y * viewSize.y);
        float extension = viewDiagonal * 2.0f;
        drawStart = p1; // Start is fixed
        drawEnd = p2 + unitDir * extension; // Extend past P2
    }
    // Else: Segment or Vector (draw directly from p1 to p2)

    // --- DRAW MAIN LINE BODY ---
    sf::Vector2f normal(-unitDir.y, unitDir.x);
    sf::Vector2f offset = normal * (worldThickness * 0.5f);

    sf::VertexArray quad(sf::Quads, 4);
    quad[0].position = drawStart + offset;
    quad[1].position = drawEnd + offset;
    quad[2].position = drawEnd - offset;
    quad[3].position = drawStart - offset;
    
    for(int i=0; i<4; ++i) quad[i].color = drawColor;
    window.draw(quad);

    // --- DRAW VECTOR ARROWHEAD ---
    if (type == ObjectType::Vector) {
        float arrowSize = 15.0f * scale; // Adjust size as needed
        sf::Vector2f arrowTip = p2;
        sf::Vector2f arrowBase = arrowTip - unitDir * arrowSize;
        sf::Vector2f arrowLeft = arrowBase + normal * (arrowSize * 0.35f);
        sf::Vector2f arrowRight = arrowBase - normal * (arrowSize * 0.35f);

        sf::VertexArray arrow(sf::Triangles, 3);
        arrow[0].position = arrowTip;
        arrow[1].position = arrowLeft;
        arrow[2].position = arrowRight;
        
        for(int i=0; i<3; ++i) arrow[i].color = drawColor;
        window.draw(arrow);
    }
  }