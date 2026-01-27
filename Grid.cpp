#include "Grid.h"
#include "Constants.h"
#include "GUI.h" // For Button::getFontLoaded and Button::getFont
#include <SFML/Graphics.hpp>
#include <cmath>   // For std::floor, std::ceil
#include <iomanip> // For std::fixed, std::setprecision
#include <sstream> // For std::ostringstream

Grid::Grid(float size, bool visible)
    : m_gridSize(size), 
    m_visible(visible),
    m_xAxisPseudoLine(Point_2(0, 0), Point_2(1, 0), false, Constants::GRID_AXIS_COLOR),
    m_yAxisPseudoLine(Point_2(0,0), Point_2(0,1), false, Constants::GRID_AXIS_COLOR),
    m_font(), // Default initialize m_font
    m_fontLoaded(false)  {
        if (Button::getFontLoaded()) {
            m_font = Button::getFont(); // Get the loaded font
            m_fontLoaded = true;
            m_axisText.setFont(m_font);
            m_axisLabelText.setFont(m_font);
        } else {
            std::cerr << "Grid: Font not loaded by Button class!" << std::endl;
            // Attempt to load a fallback font or handle error
            if (m_font.loadFromFile("arial.ttf")) { // Ensure arial.ttf is accessible
                m_fontLoaded = true;
                m_axisText.setFont(m_font);
                m_axisLabelText.setFont(m_font);
                std::cout << "Grid: Fallback font arial.ttf loaded." << std::endl;
            } else {
                std::cerr << "Grid: Failed to load fallback font arial.ttf." << std::endl;
            }
        }
        // Call setupGridLines with a default view to initialize pseudo lines properly
        sf::View defaultView;
        defaultView.setSize(static_cast<float>(Constants::WINDOW_WIDTH), static_cast<float>(Constants::WINDOW_HEIGHT));
        defaultView.setCenter(static_cast<float>(Constants::WINDOW_WIDTH) / 2.0f, static_cast<float>(Constants::WINDOW_HEIGHT) / 2.0f);
        setupGridLines(defaultView, sf::Vector2u(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT));
      }

void Grid::setVisible(bool visible) { m_visible = visible; }

bool Grid::isVisible() const { return m_visible; }

void Grid::toggleVisibility() { m_visible = !m_visible; }

void Grid::update(const sf::View &view, const sf::Vector2u &windowSize) {
  // Update the cached view state
  m_lastView = view;

  // 1. Calculate how many world units equal ~50 pixels (the desired gap)
  if (windowSize.x == 0 || view.getSize().x == 0) return;

  float pixelsPerUnit = static_cast<float>(windowSize.x) / view.getSize().x;
  float minScreenGap = 50.0f; // Minimum pixels between lines
  float minWorldGap = minScreenGap / pixelsPerUnit;

  // 2. Determine best step size (1, 2, 5, 10, 20, 50, etc.)
  // We use log10 to find the magnitude (power of 10)
  float magnitude = std::pow(10.0f, std::floor(std::log10(minWorldGap)));
  float residual = minWorldGap / magnitude;

  // Pick the cleanest multiplier
  if (residual > 5.0f)
    m_currentGridSpacing = 10.0f * magnitude;
  else if (residual > 2.0f)
    m_currentGridSpacing = 5.0f * magnitude;
  else if (residual > 1.0f)
    m_currentGridSpacing = 2.0f * magnitude;
  else
    m_currentGridSpacing = 1.0f * magnitude;

  // 3. Robust Loop Bounds (Infinite Scrolling)
  sf::Vector2f center = view.getCenter();
  sf::Vector2f size = view.getSize();

  // Safety padding to ensure lines don't pop out at edges.
  // We add 'step' to the half-sizes to ensure we cover the full view plus a bit more.
  float step = m_currentGridSpacing;
  float extraPad = step * 2.0f; // Generous padding

  float left = center.x - size.x / 2.0f - extraPad;
  float right = center.x + size.x / 2.0f + extraPad;
  float top = center.y - size.y / 2.0f - extraPad;
  float bottom = center.y + size.y / 2.0f + extraPad;

  float startX = std::floor(left / step) * step;
 
  // Handle inverted Y or negative sizing: simply find min/max for loops
  float minY = std::min(top, bottom);
  float maxY = std::max(top, bottom);
  float startY = std::floor(minY / step) * step;

  // Recreate grid lines
  m_gridLines.clear();

  // Vertical lines
  for (float x = startX; x <= right; x += step) {
    if (std::abs(x) < step * 0.001f) continue; // Skip near-zero (axis handled separately usually, or simpler to draw)
    
    sf::VertexArray line(sf::Lines, 2);
    line[0].position = sf::Vector2f(x, minY);
    line[1].position = sf::Vector2f(x, maxY);
    line[0].color = Constants::GRID_COLOR;
    line[1].color = Constants::GRID_COLOR;
    m_gridLines.push_back(line);
  }

  // Horizontal lines
  for (float y = startY; y <= maxY; y += step) {
     if (std::abs(y) < step * 0.001f) continue;

    sf::VertexArray line(sf::Lines, 2);
    line[0].position = sf::Vector2f(left, y);
    line[1].position = sf::Vector2f(right, y);
    line[0].color = Constants::GRID_COLOR;
    line[1].color = Constants::GRID_COLOR;
    m_gridLines.push_back(line);
  }
}

void Grid::setupGridLines(const sf::View &currentView, const sf::Vector2u &windowSize) {
  update(currentView, windowSize);
}
bool Grid::isAxisLine(const sf::Vector2f& worldPos_sfml, float tolerance, bool checkXAxis) const {
    if (checkXAxis) {
        return m_xAxisPseudoLine.isValid() && m_xAxisPseudoLine.contains(worldPos_sfml, tolerance);
    } else {
        return m_yAxisPseudoLine.isValid() && m_yAxisPseudoLine.contains(worldPos_sfml, tolerance);
    }
}

Line& Grid::getXAxisPseudoLine() {
    return m_xAxisPseudoLine;
}

Line& Grid::getYAxisPseudoLine() { return m_yAxisPseudoLine; }

void Grid::draw(sf::RenderWindow &window, const sf::View &drawingView,
                const sf::View &guiView) const {
  if (!m_visible)
    return;

  // Add conditional debug output
  if (Constants::DEBUG_GRID_DRAWING) {
    std::cout << "Grid::draw: Drawing grid with current zoom level: "
              << (drawingView.getSize().y /
                  static_cast<float>(Constants::WINDOW_HEIGHT))
              << std::endl;
  }

  // Check if view has changed since last update (e.g. panning)
  // We use a small epsilon or direct comparison. Center and Size check is usually enough.
  if (drawingView.getCenter() != m_lastView.getCenter() || 
      drawingView.getSize() != m_lastView.getSize()) {
      // Auto-update to handle panning/zooming not caught by explicit calls
      // Cast away constness to call update (which updates mutable members)
      const_cast<Grid*>(this)->update(drawingView, window.getSize());
  }

  // Draw grid lines using drawingView (pre-calculated in update)
  window.setView(drawingView);
  for (const auto &line : m_gridLines) {
    window.draw(line);
  }

  // Draw axis labels using guiView for sharpness
  if (m_fontLoaded) {
    sf::Vector2f viewCenter = drawingView.getCenter();
    sf::Vector2f viewSize = drawingView.getSize();
    
    // Bounds for labels (same logic as update to prevent popping)
    float step = m_currentGridSpacing;
    float extraPad = step * 2.0f;

    float left = viewCenter.x - viewSize.x / 2.0f - extraPad;
    float right = viewCenter.x + viewSize.x / 2.0f + extraPad;
    float top = viewCenter.y - viewSize.y / 2.0f - extraPad;
    float bottom = viewCenter.y + viewSize.y / 2.0f + extraPad;

    float dynamicGridSize = m_currentGridSpacing; // Use the pre-calculated spacing

    m_axisText.setCharacterSize(Constants::GRID_LABEL_FONT_SIZE);
    m_axisText.setFont(m_font);
    m_axisText.setFillColor(Constants::AXIS_LABEL_COLOR);

    // X-axis labels
    float startX = std::floor(left / dynamicGridSize) * dynamicGridSize;
    for (float x = startX; x <= right; x += dynamicGridSize) {
      if (std::abs(x) < dynamicGridSize * 0.01f)
        continue; // Skip origin label

      sf::Vector2f worldLabelPos(x, 0.f); // Position on X-axis
      sf::Vector2i pixelLabelPos =
          window.mapCoordsToPixel(worldLabelPos, drawingView);

      // Convert float to string with controlled precision
      std::ostringstream oss;
      if (std::abs(x) >= 1000000.0f) {
        oss << std::scientific << std::setprecision(3) << x;
      } else if (dynamicGridSize < 1.0f) {
        oss << std::fixed << std::setprecision(3) << x;
      } else {
        oss << std::fixed << std::setprecision(0) << x;
      }
      m_axisText.setString(oss.str());

      sf::FloatRect textBounds = m_axisText.getLocalBounds();
      m_axisText.setOrigin(textBounds.left + textBounds.width / 2.f,
                           textBounds.top +
                               textBounds.height / 2.f); // Center text
      m_axisText.setPosition(
          static_cast<float>(pixelLabelPos.x),
          static_cast<float>(pixelLabelPos.y + Constants::GRID_LABEL_OFFSET));

      window.setView(guiView); // Switch to GUI view for sharp text
      window.draw(m_axisText);
      window.setView(drawingView); // Switch back
    }

    // Y-axis labels
    // Handle inverted Y or negative sizing for labels too
    float minY = std::min(top, bottom);
    float maxY = std::max(top, bottom);
    float startY = std::floor(minY / dynamicGridSize) * dynamicGridSize;
    
    for (float y = startY; y <= maxY; y += dynamicGridSize) {
       if (std::abs(y) < dynamicGridSize * 0.01f)
        continue; // Skip origin label

      sf::Vector2f worldLabelPos(0.f, y); // Position on Y-axis
      sf::Vector2i pixelLabelPos =
          window.mapCoordsToPixel(worldLabelPos, drawingView);

      // Convert float to string with controlled precision
      std::ostringstream oss;
      if (std::abs(y) >= 1000000.0f) {
        oss << std::scientific << std::setprecision(3) << y;
      } else if (dynamicGridSize < 1.0f) {
        oss << std::fixed << std::setprecision(3) << y;
      } else {
        oss << std::fixed << std::setprecision(0) << y;
      }
      m_axisText.setString(oss.str());

      sf::FloatRect textBounds = m_axisText.getLocalBounds();
      m_axisText.setOrigin(textBounds.left + textBounds.width + 5.f,
                           textBounds.top +
                               textBounds.height / 2.f); // Right-align
      m_axisText.setPosition(
          static_cast<float>(pixelLabelPos.x - Constants::GRID_LABEL_OFFSET),
          static_cast<float>(pixelLabelPos.y));

      window.setView(guiView); // Switch to GUI view for sharp text
      window.draw(m_axisText);
      window.setView(drawingView); // Switch back
    }
  }
}