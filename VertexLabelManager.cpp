#include "VertexLabelManager.h"
#include <iostream>
#include <sstream>

std::string VertexLabelManager::toSubscript(int number) {
  if (number == 0) return "";
  
  // Unicode subscript digits: ₀₁₂₃₄₅₆₇₈₉
  const std::string subscripts[] = {"₀", "₁", "₂", "₃", "₄", "₅", "₆", "₇", "₈", "₉"};
  
  std::string result;
  std::string numStr = std::to_string(number);
  for (char c : numStr) {
    if (c >= '0' && c <= '9') {
      result += subscripts[c - '0'];
    }
  }
  return result;
}

bool VertexLabelManager::initFont() {
  if (m_fontLoaded) return true;
  
  if (!m_font.loadFromFile(Constants::DEFAULT_FONT_PATH)) {
    std::cerr << "VertexLabelManager: Failed to load font: " << Constants::DEFAULT_FONT_PATH << std::endl;
    return false;
  }
  
  m_fontLoaded = true;
  return true;
}

std::string VertexLabelManager::registerLabel(const std::string& baseLabel) {
  if (baseLabel.empty()) return "?";
  
  int& count = m_labelCounts[baseLabel];
  std::string result;
  
  if (count == 0) {
    // First usage of this label, no subscript needed
    result = baseLabel;
  } else {
    // Need subscript
    result = baseLabel + toSubscript(count);
  }
  
  count++;
  return result;
}

void VertexLabelManager::releaseLabel(const std::string& label) {
  // Find base label (strip any subscripts)
  std::string baseLabel;
  for (char c : label) {
    // Stop at first subscript character
    if ((unsigned char)c > 127) break;  // Unicode subscripts are multi-byte
    baseLabel += c;
  }
  
  auto it = m_labelCounts.find(baseLabel);
  if (it != m_labelCounts.end() && it->second > 0) {
    it->second--;
  }
}

std::string VertexLabelManager::renameLabel(const std::string& oldLabel, const std::string& newLabel) {
  releaseLabel(oldLabel);
  return registerLabel(newLabel);
}

void VertexLabelManager::drawLabel(sf::RenderWindow& window, const sf::Vector2f& pos, const std::string& label, unsigned int fontSize) const {
  if (!m_visible || label.empty()) return;
  
  // Ensure font is loaded (const_cast for lazy init)
  if (!m_fontLoaded) {
    if (!const_cast<VertexLabelManager*>(this)->initFont()) {
      return;
    }
  }
  
  // Use provided fontSize, or fall back to instance fontSize if default (18)
  unsigned int actualSize = (fontSize == 18) ? m_fontSize : fontSize;
  
  sf::Text text;
  text.setFont(m_font);
  text.setString(label);
  text.setCharacterSize(actualSize);
  text.setFillColor(sf::Color::Black);
  
  // Position label with offset (slightly above and to the right of vertex)
  sf::FloatRect bounds = text.getLocalBounds();
  text.setOrigin(bounds.width / 2.f, bounds.height + 5.f);  // Center horizontally, above vertex
  text.setPosition(pos.x, pos.y - 8.f);  // 8 pixels above vertex
  
  // Draw background for readability
  sf::RectangleShape bg;
  bg.setSize(sf::Vector2f(bounds.width + 4.f, bounds.height + 2.f));
  bg.setFillColor(sf::Color(255, 255, 255, 200));
  bg.setOrigin(bounds.width / 2.f + 2.f, bounds.height + 6.f);
  bg.setPosition(pos.x, pos.y - 8.f);
  
  window.draw(bg);
  window.draw(text);
}
