#pragma once
#include <SFML/Graphics.hpp>
#include <memory>

class GeometryEditor;

class Benchmark {
 public:
  Benchmark();
  void spawnStressTest(GeometryEditor& editor);
  void renderHUD(sf::RenderWindow& window, const GeometryEditor& editor, sf::Time logicTime);
  void clearStressTest(GeometryEditor& editor);

 private:
  sf::Font m_font;
  sf::Text m_text;
};