#ifndef MICROTEX_SFML_H
#define MICROTEX_SFML_H

#include <SFML/Graphics.hpp>
#include <memory>
#include "graphic/graphic.h"
#include "common.h"

namespace tex {

class Font_sfml : public Font {
 public:
  Font_sfml(const std::shared_ptr<sf::Font>& font, float size, int style)
      : m_font(font), m_size(size), m_style(style) {}

  float getSize() const override { return m_size; }

  sptr<Font> deriveFont(int style) const override {
    return sptrOf<Font_sfml>(m_font, m_size, style);
  }

  bool operator==(const Font& f) const override { return this == &f; }

  bool operator!=(const Font& f) const override { return this != &f; }

  const std::shared_ptr<sf::Font>& getSfFont() const { return m_font; }
  int getStyle() const { return m_style; }

 private:
  std::shared_ptr<sf::Font> m_font;
  float m_size = 12.0f;
  int m_style = 0; // PLAIN
};

} // namespace tex

#endif // MICROTEX_SFML_H
