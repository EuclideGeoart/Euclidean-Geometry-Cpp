#pragma once

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

class SymbolPalette {
 public:
  struct Entry {
    std::string label;
    std::string latex;
    std::string category;
  };

  SymbolPalette();

  void setPosition(const sf::Vector2f& pos);
  const sf::Vector2f& getPosition() const { return m_position; }

  void draw(sf::RenderWindow& window, const sf::Font& font, bool visible);

  // Returns true if a click hit a symbol; outLatex gets inserted.
  bool handleClick(const sf::Vector2f& mousePos, std::string& outLatex) const;

  const std::vector<Entry>& getEntries() const { return m_entries; }

  sf::FloatRect getBounds() const;

 private:
  std::vector<Entry> m_entries;
  sf::Vector2f m_position{0.f, 0.f};
  float m_cellSize = 28.0f;
  float m_padding = 6.0f;
  int m_columns = 6;
};
