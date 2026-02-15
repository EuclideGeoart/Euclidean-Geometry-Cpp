#include <SFML/Graphics.hpp>
#include <cmath>

#include "Constants.h"

#include "graphic/graphic.h"
#include "utils/utf.h"

#include "MicroTeXSFML.h"
#include "graphic/graphic.h"
#include "utils/utf.h"

namespace tex {

namespace {
/* Font_sfml moved to MicroTeXSFML.h */


std::shared_ptr<sf::Font> loadFontFromFile(const std::string& path) {
  auto font = std::make_shared<sf::Font>();
  if (!path.empty() && font->loadFromFile(path)) {
    return font;
  }
  if (font->loadFromFile(Constants::DEFAULT_FONT_PATH)) {
    return font;
  }
  return nullptr;
}

std::shared_ptr<sf::Font> loadDefaultFont() {
  return loadFontFromFile(Constants::DEFAULT_FONT_PATH);
}

}  // namespace

Font* Font::create(const std::string& file, float size) {
  auto font = loadFontFromFile(file);
  if (!font) return nullptr;
  return new Font_sfml(font, size, PLAIN);
}

sptr<Font> Font::_create(const std::string& name, int style, float size) {
  (void)name;
  auto font = loadDefaultFont();
  if (!font) return nullptr;
  return sptrOf<Font_sfml>(font, size, style);
}

class TextLayout_sfml : public TextLayout {
 public:
  TextLayout_sfml(const std::wstring& src, const sptr<Font_sfml>& font)
      : m_text(src), m_font(font) {}

  void getBounds(Rect& bounds) override {
    if (!m_font || !m_font->getSfFont()) {
      bounds = Rect(0, 0, 0, 0);
      return;
    }
    sf::Text text;
    text.setFont(*m_font->getSfFont());
    text.setString(sf::String(m_text));
    text.setCharacterSize(static_cast<unsigned int>(std::round(m_font->getSize())));
    sf::FloatRect b = text.getLocalBounds();
    bounds.x = b.left;
    bounds.y = b.top;
    bounds.w = b.width;
    bounds.h = b.height;
  }

  void draw(Graphics2D& g2, float x, float y) override {
    g2.drawText(m_text, x, y);
  }

 private:
  std::wstring m_text;
  sptr<Font_sfml> m_font;
};

sptr<TextLayout> TextLayout::create(const std::wstring& src, const sptr<Font>& font) {
  auto f = std::dynamic_pointer_cast<Font_sfml>(font);
  if (!f) {
    return sptr<TextLayout>(new TextLayout_sfml(src, sptrOf<Font_sfml>(loadDefaultFont(), 12.0f, PLAIN)));
  }
  return sptr<TextLayout>(new TextLayout_sfml(src, f));
}

}  // namespace tex
