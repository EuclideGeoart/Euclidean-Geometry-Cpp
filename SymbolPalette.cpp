#include "SymbolPalette.h"

#include <algorithm>

SymbolPalette::SymbolPalette() {
  m_entries = {
      {"\\angle", "\\angle", "Geometry"},
      {"\\triangle", "\\triangle", "Geometry"},
      {"\\parallel", "\\parallel", "Geometry"},
      {"\\perp", "\\perp", "Geometry"},
      {"\\cong", "\\cong", "Geometry"},
      {"\\sim", "\\sim", "Geometry"},
      {"\\approx", "\\approx", "Geometry"},
      {"\\neq", "\\neq", "Geometry"},
      {"\\degrees", "\\degrees", "Geometry"},

      {"\\int", "\\int", "Calculus"},
      {"\\sum", "\\sum", "Calculus"},
      {"\\lim", "\\lim", "Calculus"},
      {"\\infty", "\\infty", "Calculus"},
      {"\\partial", "\\partial", "Calculus"},
      {"\\sqrt{}", "\\sqrt{}", "Calculus"},
      {"\\frac{}{}", "\\frac{}{}", "Calculus"},

      {"\\implies", "\\implies", "Logic"},
      {"\\iff", "\\iff", "Logic"},
      {"\\therefore", "\\therefore", "Logic"},
      {"\\because", "\\because", "Logic"},

      {"\\alpha", "\\alpha", "Greek"},
      {"\\beta", "\\beta", "Greek"},
      {"\\theta", "\\theta", "Greek"},
      {"\\pi", "\\pi", "Greek"},
      {"\\Delta", "\\Delta", "Greek"}
  };
}

void SymbolPalette::setPosition(const sf::Vector2f& pos) {
  m_position = pos;
}

sf::FloatRect SymbolPalette::getBounds() const {
  int rows = static_cast<int>((m_entries.size() + m_columns - 1) / m_columns);
  float width = m_columns * m_cellSize + m_padding * 2.0f;
  float height = rows * m_cellSize + m_padding * 2.0f;
  return sf::FloatRect(m_position.x, m_position.y, width, height);
}

void SymbolPalette::draw(sf::RenderWindow& window, const sf::Font& font, bool visible) {
  if (!visible) return;

  sf::FloatRect bounds = getBounds();
  sf::RectangleShape bg(sf::Vector2f(bounds.width, bounds.height));
  bg.setPosition(bounds.left, bounds.top);
  bg.setFillColor(sf::Color(255, 255, 255, 240));
  bg.setOutlineColor(sf::Color(0, 120, 255, 200));
  bg.setOutlineThickness(1.0f);
  window.draw(bg);

  for (size_t i = 0; i < m_entries.size(); ++i) {
    int col = static_cast<int>(i % m_columns);
    int row = static_cast<int>(i / m_columns);
    float x = m_position.x + m_padding + col * m_cellSize;
    float y = m_position.y + m_padding + row * m_cellSize;

    sf::RectangleShape cell(sf::Vector2f(m_cellSize - 2.0f, m_cellSize - 2.0f));
    cell.setPosition(x + 1.0f, y + 1.0f);
    cell.setFillColor(sf::Color(245, 245, 245));
    cell.setOutlineThickness(1.0f);
    cell.setOutlineColor(sf::Color(210, 210, 210));
    window.draw(cell);

    sf::Text text;
    text.setFont(font);
    text.setString(m_entries[i].label);
    text.setCharacterSize(14);
    text.setFillColor(sf::Color::Black);
    sf::FloatRect tb = text.getLocalBounds();
    text.setPosition(x + (m_cellSize - tb.width) * 0.5f - tb.left,
                     y + (m_cellSize - tb.height) * 0.5f - tb.top);
    window.draw(text);
  }
}

bool SymbolPalette::handleClick(const sf::Vector2f& mousePos, std::string& outLatex) const {
  if (!getBounds().contains(mousePos)) return false;

  float localX = mousePos.x - m_position.x - m_padding;
  float localY = mousePos.y - m_position.y - m_padding;
  if (localX < 0 || localY < 0) return false;

  int col = static_cast<int>(localX / m_cellSize);
  int row = static_cast<int>(localY / m_cellSize);
  if (col < 0 || col >= m_columns) return false;

  size_t index = static_cast<size_t>(row * m_columns + col);
  if (index >= m_entries.size()) return false;

  outLatex = m_entries[index].latex;
  return true;
}
