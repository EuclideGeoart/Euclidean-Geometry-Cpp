#pragma once
#include "CharTraitsFix.h"
#include "Constants.h"
#include <SFML/Graphics.hpp>
#include <string> // Required by CharTraitsFix potentially, and good practice
#include "Line.h"

class Grid {
public:
  Grid(float initialGridSize = Constants::GRID_SIZE,
       bool initiallyVisible = true); // Constructor

  void update(const sf::View &view,
              const sf::Vector2u &/* windowSize */); // To recalculate based on view
  void draw(sf::RenderWindow &window, const sf::View &drawingView,
            const sf::View &guiView) const; // Modified draw signature

  void setGridSize(float size);
  float getGridSize() const;
  float getCurrentGridSpacing() const;
  void toggleVisibility();
  bool isVisible() const;
  void setVisible(bool visible);
  void setAdaptive(bool adaptive);
  bool isAdaptive() const;
  bool isAxisLine(const sf::Vector2f& worldPos, float tolerance, bool checkXAxis) const;
  Line& getXAxisPseudoLine(); // Consider how this pseudo line is managed
  Line& getYAxisPseudoLine(); // Consider how this pseudo line is managed
  
private:
  // Helper methods can be non-static or static if they don't use member data
  void drawGridLines(sf::RenderWindow &window, const sf::View &view,
                     float spacing) const;
  float calculateAdaptiveGridSpacing(
      const sf::View &view) const; // Renamed and adapted
  void drawAxisLabels(sf::RenderWindow &window, const sf::View &view,
                      float spacing) const;
  void drawAxes(sf::RenderWindow &window, const sf::View &view) const;
  void rebuildVertices(const sf::View &view);
  std::string
  formatNumber(float number) const; // Helper to format numbers for labels
  void setupGridLines(const sf::View &currentView,
                      const sf::Vector2u &windowSize);
  mutable std::vector<sf::VertexArray> m_gridLines;

  sf::VertexArray m_vertices;
  float m_gridSize;
  bool m_visible;
  bool m_adaptive;            // Whether the grid spacing adapts to zoom
  float m_baseGridSpacing;    // The initial grid spacing
  mutable float m_currentGridSpacing; // Current grid spacing, adapted for zoom
  Line m_xAxisPseudoLine;
  Line m_yAxisPseudoLine;
  sf::Font m_font;               // Font for labels
  bool m_fontLoaded;                // Flag to check if font was loaded
  mutable sf::Text m_axisLabelText; // Reusable sf::Text for labels
  mutable sf::Text m_axisText;      // Reusable sf::Text for drawing labels
  mutable sf::View m_lastView;      // Cache last view to detect changes

  // Constants for adaptive grid behavior (can be tuned)
  float m_minPixelSpacing; // Minimum desired pixel spacing for grid lines on
                           // screen
  float m_maxPixelSpacing; // Maximum desired pixel spacing

  // Offset for labels from the axis lines (in screen pixels, adjust as needed)
  sf::Vector2f labelOffsetScreen;
};
