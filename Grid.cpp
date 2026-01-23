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

void Grid::update(const sf::View &view, const sf::Vector2u &/* windowSize */) {
  // Get the visible area in world coordinates
  sf::Vector2f viewSize = view.getSize();
  sf::Vector2f viewCenter = view.getCenter();
  float left = viewCenter.x - viewSize.x / 2.f;
  float top = viewCenter.y - viewSize.y / 2.f;
  float right = viewCenter.x + viewSize.x / 2.f;
  float bottom = viewCenter.y + viewSize.y / 2.f;

  // Get current zoom level
  float zoomLevel =
      view.getSize().y / static_cast<float>(Constants::WINDOW_HEIGHT);

  // Extra guard for extreme zoom levels
  // if (zoomLevel < 0.001f || zoomLevel > 1000.0f) {
  //   m_gridLines.clear(); // Clear grid to prevent artifacts
  //   return;
  // }

  // Calculate appropriate grid spacing based on zoom level
  // This creates a dynamic grid that adjusts with zoom
  float worldUnitsPerMajorGridLine =
      std::pow(10, std::floor(std::log10(viewSize.x / 5.0f)));

  // Ensure minimum grid spacing
  if (worldUnitsPerMajorGridLine < Constants::MIN_GRID_SPACING_WORLD) {
    worldUnitsPerMajorGridLine = Constants::MIN_GRID_SPACING_WORLD;
  }

  // Additional sanity check
  if (worldUnitsPerMajorGridLine <= 0.0f) {
    worldUnitsPerMajorGridLine = 1.0f; // Fallback to positive value
  }

  float gridSpacing = worldUnitsPerMajorGridLine;

  // Recreate grid lines for the visible area with some margin
  float margin = viewSize.x * 0.2f; // 20% margin on each side

  // Clear previous grid lines
  m_gridLines.clear();

  // Only create grid if not at extreme zoom levels
  //if (zoomLevel >= 0.01f && zoomLevel <= 100.0f)
    // Create vertical grid lines
    for (float x = std::floor((left - margin) / gridSpacing) * gridSpacing;
         x <= right + margin; x += gridSpacing) {
      sf::Vertex line[] = {
          sf::Vertex(sf::Vector2f(x, top - margin), Constants::GRID_COLOR),
          sf::Vertex(sf::Vector2f(x, bottom + margin), Constants::GRID_COLOR)};

      // Add thicker lines for axes
      if (std::abs(x) < 0.001f * gridSpacing) {
        line[0].color = Constants::GRID_AXIS_COLOR;
        line[1].color = Constants::GRID_AXIS_COLOR;
      }

      m_gridLines.push_back(sf::VertexArray(sf::Lines, 2));
      m_gridLines.back()[0] = line[0];
      m_gridLines.back()[1] = line[1];
    }

    // Create horizontal grid lines
    for (float y = std::floor((top - margin) / gridSpacing) * gridSpacing;
         y <= bottom + margin; y += gridSpacing) {
      sf::Vertex line[] = {
          sf::Vertex(sf::Vector2f(left - margin, y), Constants::GRID_COLOR),
          sf::Vertex(sf::Vector2f(right + margin, y), Constants::GRID_COLOR)};

      // Add thicker lines for axes
      if (std::abs(y) < 0.001f * gridSpacing) {
        line[0].color = Constants::GRID_AXIS_COLOR;
        line[1].color = Constants::GRID_AXIS_COLOR;
      }

      m_gridLines.push_back(sf::VertexArray(sf::Lines, 2));
      m_gridLines.back()[0] = line[0];
      m_gridLines.back()[1] = line[1];
    }
}

void Grid::setupGridLines(const sf::View &currentView, const sf::Vector2u &) {
  m_gridLines.clear();
  if (!m_visible || m_gridSize <= 0)
    return;

  sf::Vector2f viewCenter = currentView.getCenter();
  sf::Vector2f viewSize = currentView.getSize();
  float left = viewCenter.x - viewSize.x / 2.f;
  float right = viewCenter.x + viewSize.x / 2.f;
  float top = viewCenter.y - viewSize.y / 2.f;
  float bottom = viewCenter.y + viewSize.y / 2.f;

  // Determine a dynamic grid spacing based on zoom level to avoid clutter
  float worldUnitsPerMajorGridLine =
      std::pow(10, std::floor(std::log10(
                       viewSize.x / 5.0f))); // Aim for ~5 major lines on screen
  if (worldUnitsPerMajorGridLine < Constants::MIN_GRID_SPACING_WORLD) {
    worldUnitsPerMajorGridLine = Constants::MIN_GRID_SPACING_WORLD;
  }
  float dynamicGridSize = worldUnitsPerMajorGridLine;

  // Vertical lines
  float startX = std::floor(left / dynamicGridSize) * dynamicGridSize;
  for (float x = startX; x <= right; x += dynamicGridSize) {
    sf::VertexArray line(sf::Lines, 2);
    line[0].position = sf::Vector2f(x, top - viewSize.y); // Extend beyond view
    line[1].position = sf::Vector2f(x, bottom + viewSize.y);
    bool isAxis =
        (std::abs(x) < dynamicGridSize * 0.1f); // Check if it's the Y-axis
    line[0].color = isAxis ? Constants::GRID_AXIS_COLOR : Constants::GRID_COLOR;
    line[1].color = isAxis ? Constants::GRID_AXIS_COLOR : Constants::GRID_COLOR;
    m_gridLines.push_back(line);
  }

  // Horizontal lines
  float startY = std::floor(top / dynamicGridSize) * dynamicGridSize;
  for (float y = startY; y <= bottom; y += dynamicGridSize) {
    sf::VertexArray line(sf::Lines, 2);
    line[0].position = sf::Vector2f(left - viewSize.x, y); // Extend beyond view
    line[1].position = sf::Vector2f(right + viewSize.x, y);
    bool isAxis =
        (std::abs(y) < dynamicGridSize * 0.1f); // Check if it's the X-axis
    line[0].color = isAxis ? Constants::GRID_AXIS_COLOR : Constants::GRID_COLOR;
    line[1].color = isAxis ? Constants::GRID_AXIS_COLOR : Constants::GRID_COLOR;
    m_gridLines.push_back(line);
  }
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

  // Get current zoom level
  float zoomLevel =
      drawingView.getSize().y / static_cast<float>(Constants::WINDOW_HEIGHT);

  // Skip grid rendering at extreme zoom levels to prevent performance issues
  // or visual artifacts that might cause the red screen
  // const float MIN_GRID_RENDER_ZOOM = 0.01f;
  // const float MAX_GRID_RENDER_ZOOM = 50.0f;

  // if (zoomLevel < MIN_GRID_RENDER_ZOOM || zoomLevel > MAX_GRID_RENDER_ZOOM) {
  //   // At extreme zoom levels, just render a simple background
  //   // This prevents potential rendering artifacts
  //   return;
  // }

  // Draw grid lines using drawingView
  window.setView(drawingView);
  for (const auto &line : m_gridLines) {
    window.draw(line);
  }

  // Draw axis labels using guiView for sharpness
  if (m_fontLoaded) {
    sf::Vector2f viewCenter = drawingView.getCenter();
    sf::Vector2f viewSize = drawingView.getSize();
    float left = viewCenter.x - viewSize.x / 2.f;
    float right = viewCenter.x + viewSize.x / 2.f;
    float top = viewCenter.y - viewSize.y / 2.f;
    float bottom = viewCenter.y + viewSize.y / 2.f;

    // Determine a dynamic grid spacing for labels (consistent with
    // setupGridLines)
    float worldUnitsPerMajorGridLine =
        std::pow(10, std::floor(std::log10(viewSize.x / 5.0f)));
    if (worldUnitsPerMajorGridLine < Constants::MIN_GRID_SPACING_WORLD) {
      worldUnitsPerMajorGridLine = Constants::MIN_GRID_SPACING_WORLD;
    }
    float dynamicGridSize = worldUnitsPerMajorGridLine;

    m_axisText.setCharacterSize(
        Constants::GRID_LABEL_FONT_SIZE); // Fixed pixel size
    m_axisText.setFont(m_font);
    m_axisText.setFillColor(Constants::AXIS_LABEL_COLOR);

    // Add a check to prevent rendering too many labels at extreme zoom levels
    if (dynamicGridSize < 0.0001f) {
      // At extreme zoom levels, limit label rendering
      return;
    }

    // X-axis labels
    float startX = std::floor(left / dynamicGridSize) * dynamicGridSize;
    for (float x = startX; x <= right; x += dynamicGridSize) {
      if (std::abs(x) < dynamicGridSize * 0.01f &&
          std::abs(0.0f) < dynamicGridSize * 0.01f)
        continue; // Skip origin label if too close to Y-axis labels

      sf::Vector2f worldLabelPos(x, 0.f); // Position on X-axis
      sf::Vector2i pixelLabelPos =
          window.mapCoordsToPixel(worldLabelPos, drawingView);

      // Convert float to string with controlled precision
      std::ostringstream oss;
      // Use scientific notation for large numbers (above 999,999)
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

    // Y-axis labels (similar modification needed here)
    float startY = std::floor(top / dynamicGridSize) * dynamicGridSize;
    for (float y = startY; y <= bottom; y += dynamicGridSize) {
      if (std::abs(y) < dynamicGridSize * 0.01f &&
          std::abs(0.0f) < dynamicGridSize * 0.01f)
        continue; // Skip origin label if too close to X-axis labels

      sf::Vector2f worldLabelPos(0.f, y); // Position on Y-axis
      sf::Vector2i pixelLabelPos =
          window.mapCoordsToPixel(worldLabelPos, drawingView);

      // Convert float to string with controlled precision
      std::ostringstream oss;
      // Use scientific notation for large numbers (above 999,999)
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