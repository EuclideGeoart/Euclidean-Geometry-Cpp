#pragma once

#include <SFML/Graphics.hpp>
#include <memory>
#include <string>
#include <unordered_map>

// MicroTeX integration is optional; this bridge can be compiled even if MicroTeX
// is not yet available in the include path.
#if __has_include("microtex.h")
#include "microtex.h"
#define HAS_MICROTEX 1
#else
#define HAS_MICROTEX 0
#endif

class LatexRenderer {
 public:
  explicit LatexRenderer(sf::RenderTarget& target);

  void setTarget(sf::RenderTarget& target);
  void setColor(const sf::Color& color);
  void drawText(const std::string& text, float x, float y, float size);
  void drawLine(float x1, float y1, float x2, float y2, float thickness = 1.0f);
  void fillRect(float x, float y, float w, float h);
  void strokeRect(float x, float y, float w, float h, float thickness = 1.0f);

  static void setDefaultFont(const sf::Font& font);
  static const sf::Font* getFont();

  // Renders a LaTeX (or plain) string into a cached texture for fast drawing.
  static std::shared_ptr<sf::Texture> RenderLatex(const std::string& latex, float size, float maxWidth, sf::Color color);

  static sf::Vector2f MeasureText(const std::string& text, float size);

  // High-DPI Scaling Constants
  static constexpr float HD_FACTOR = 4.0f;
  static constexpr float VISUAL_SCALE = 2.5f;
  static constexpr float HD_INVERSE = 1.0f / (HD_FACTOR * VISUAL_SCALE);

 private:
  static std::unordered_map<std::string, std::shared_ptr<sf::Texture>> s_textureCache;
  sf::RenderTarget* m_target = nullptr;
  sf::Color m_color = sf::Color::Black;
  // Helper to generate unique keys for the cache
  static std::string generateCacheKey(const std::string& latex, float size, sf::Color color);
  static const sf::Font* s_font;
  static sf::Font s_fallbackFont;
  static bool s_fallbackLoaded;
};
