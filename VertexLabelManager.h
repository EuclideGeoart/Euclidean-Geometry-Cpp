#pragma once

#include "CharTraitsFix.h"
#include <SFML/Graphics.hpp>
#include <string>
#include <map>
#include <vector>
#include "Constants.h"

/**
 * @brief Manages vertex labels across all shapes with toggle, rename, and auto-subscript features
 */
class VertexLabelManager {
public:
  static VertexLabelManager& instance() {
    static VertexLabelManager mgr;
    return mgr;
  }

  // Visibility toggle
  void setVisible(bool visible) { m_visible = visible; }
  bool isVisible() const { return m_visible; }
  void toggleVisible() { m_visible = !m_visible; }

  // Font size control
  void setFontSize(unsigned int size) { m_fontSize = size; }
  unsigned int getFontSize() const { return m_fontSize; }

  /**
   * @brief Get a unique label, adding subscript if duplicate
   * @param baseLabel The base label (e.g., "A", "P")
   * @return Unique label with subscript if needed (e.g., "A", "A₁", "A₂")
   */
  std::string registerLabel(const std::string& baseLabel);

  /**
   * @brief Release a label when object is deleted
   */
  void releaseLabel(const std::string& label);

  /**
   * @brief Rename a label
   * @param oldLabel Current label
   * @param newLabel New label (will be made unique if duplicate)
   * @return The actual new label (may have subscript)
   */
  std::string renameLabel(const std::string& oldLabel, const std::string& newLabel);

  /**
   * @brief Draw a label at specified position
   * @param fontSize Font size in pixels (default: 18)
   */
  void drawLabel(sf::RenderWindow& window, const sf::Vector2f& pos, const std::string& label, unsigned int fontSize = 18) const;

  /**
   * @brief Initialize font (call once at startup)
   */
  bool initFont();

private:
  VertexLabelManager() = default;
  VertexLabelManager(const VertexLabelManager&) = delete;
  VertexLabelManager& operator=(const VertexLabelManager&) = delete;

  bool m_visible = true;
  unsigned int m_fontSize = 18;  // Default font size
  std::map<std::string, int> m_labelCounts;  // Track usage counts for base labels
  sf::Font m_font;
  mutable bool m_fontLoaded = false;

  // Helper to create subscript string
  static std::string toSubscript(int number);
};
