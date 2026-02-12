#include "LatexRenderer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Constants.h"
#include "graphic/graphic.h"
#include "latex.h"
#include "render.h"
#include "utils/utf.h"
#include "core/formula.h"

namespace {

tex::color toTexColor(const sf::Color& color) {
  return tex::argb(static_cast<int>(color.a), static_cast<int>(color.r), static_cast<int>(color.g), static_cast<int>(color.b));
}

sf::Color toSfColor(tex::color c) {
  return sf::Color(static_cast<sf::Uint8>(tex::color_r(c)), static_cast<sf::Uint8>(tex::color_g(c)), static_cast<sf::Uint8>(tex::color_b(c)),
                   static_cast<sf::Uint8>(tex::color_a(c)));
}

std::string resolveMicroTeXResPath() {
  namespace fs = std::filesystem;
  const std::vector<fs::path> candidates = {
      fs::path("microtex-res"), 
      fs::path("res"), 
      fs::path("libs/MicroTeX/res"), 
      fs::path("../libs/MicroTeX/res"), 
      fs::path("../../libs/MicroTeX/res")
  };
  
  std::cout << "[LatexRenderer] Searching for resources..." << std::endl;
  for (const auto& candidate : candidates) {
    fs::path marker = candidate / ".clatexmath-res_root";
    if (fs::exists(marker)) {
      std::cout << "  FOUND at: " << fs::absolute(candidate) << std::endl;
      return candidate.string();
    }
  }
  std::cerr << "  [LatexRenderer] ERROR: Could not find resource folder!" << std::endl;
  return "";
}

class SfmlGraphics : public tex::Graphics2D {
 public:
  explicit SfmlGraphics(sf::RenderTarget& target) : m_target(&target) {}

  void setColor(tex::color c) override { m_color = c; }
  tex::color getColor() const override { return m_color; }
  void setStroke(const tex::Stroke& s) override { m_stroke = s; }
  const tex::Stroke& getStroke() const override { return m_stroke; }
  void setStrokeWidth(float w) override { m_stroke.lineWidth = w; }
  const tex::Font* getFont() const override { return m_font; }
  void setFont(const tex::Font* font) override { m_font = font; }
  void translate(float dx, float dy) override { m_transform.translate(dx, dy); }
  void scale(float sx, float sy) override { m_transform.scale(sx, sy); m_sx *= sx; m_sy *= sy; }
  void rotate(float angle) override { m_transform.rotate(angle * 180.0f / 3.14159f); }
  void rotate(float angle, float px, float py) override { m_transform.rotate(angle * 180.0f / 3.14159f, px, py); }
  void reset() override { m_transform = sf::Transform::Identity; m_sx = 1.0f; m_sy = 1.0f; }
  float sx() const override { return m_sx; }
  float sy() const override { return m_sy; }

  void drawChar(wchar_t c, float x, float y) override {
    std::wstring text(1, c);
    drawText(text, x, y);
  }

  void drawText(const std::wstring& c, float x, float y) override {
    if (!m_target) return;
    const sf::Font* font = LatexRenderer::getFont();
    if (!font) return;

    sf::Text text;
    text.setFont(*font);
    text.setString(sf::String(c));
    
    // 1. Setup Size
    unsigned int charSize = static_cast<unsigned int>(std::round(m_font ? m_font->getSize() : 20));
    text.setCharacterSize(charSize);
    text.setFillColor(toSfColor(m_color));

    // 2. CRITICAL FIX: Baseline Correction
    // MicroTeX sends 'y' as the Baseline. SFML draws from Top-Left.
    // We must shift the SFML text UP by the character ascent (approx. character size).
    // This aligns the SFML "bottom" roughly with the MicroTeX "baseline".
    
    // Note: getLocalBounds().top helps adjust for exact glyph bounding
    sf::FloatRect bounds = text.getLocalBounds();
    
    // Move 'top' to align baseline. 
    // A heuristic that works well for standard fonts is (y - charSize) + offset
    // But 'bounds.top' is usually negative (distance from baseline to top).
    // So 'y + bounds.top' aligns the visual top to 'y', which is wrong.
    // We want the visual bottom to be near 'y'.
    
    text.setOrigin(bounds.left, bounds.top); // Center origin on glyph visual
    text.setPosition(x, y - charSize);       // Shift up by full height to emulate baseline
    
    // Alternative simple fix if the above jitters:
    // text.setPosition(x, y - charSize * 0.8f); 

    sf::RenderStates states;
    states.transform = m_transform;
    m_target->draw(text, states);
  }

  void drawLine(float x1, float y1, float x2, float y2) override {
    if (!m_target) return;
    sf::Vertex line[] = {
        sf::Vertex(sf::Vector2f(x1, y1), toSfColor(m_color)),
        sf::Vertex(sf::Vector2f(x2, y2), toSfColor(m_color))
    };
    sf::RenderStates states;
    states.transform = m_transform;
    m_target->draw(line, 2, sf::Lines, states);
  }

  void drawRect(float x, float y, float w, float h) override {
    if (!m_target) return;
    sf::RectangleShape rect(sf::Vector2f(w, h));
    rect.setPosition(x, y);
    rect.setFillColor(sf::Color::Transparent);
    rect.setOutlineColor(toSfColor(m_color));
    rect.setOutlineThickness(1.0f);
    m_target->draw(rect, sf::RenderStates(m_transform));
  }

  void fillRect(float x, float y, float w, float h) override {
    if (!m_target) return;
    sf::RectangleShape rect(sf::Vector2f(w, h));
    rect.setPosition(x, y);
    rect.setFillColor(toSfColor(m_color));
    m_target->draw(rect, sf::RenderStates(m_transform));
  }
  
  void drawRoundRect(float x, float y, float w, float h, float rx, float ry) override { drawRect(x,y,w,h); }
  void fillRoundRect(float x, float y, float w, float h, float rx, float ry) override { fillRect(x,y,w,h); }

 private:
  sf::RenderTarget* m_target = nullptr;
  tex::color m_color = tex::black;
  const tex::Font* m_font = nullptr;
  tex::Stroke m_stroke;
  sf::Transform m_transform = sf::Transform::Identity;
  float m_sx = 1.0f;
  float m_sy = 1.0f;
};

static bool g_texInitialized = false;

bool ensureTexInit() {
  if (g_texInitialized) return true;
  std::string resPath = resolveMicroTeXResPath();
  if (resPath.empty()) return false;
  try {
    tex::LaTeX::init(resPath);
    g_texInitialized = true;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[LatexRenderer] Init Failed: " << e.what() << std::endl;
    return false;
  }
}

}  // namespace

// =========================================================
// GLOBAL IMPLEMENTATIONS
// =========================================================

std::unordered_map<std::string, std::shared_ptr<sf::Texture>> LatexRenderer::s_textureCache;

std::string LatexRenderer::generateCacheKey(const std::string& latex, float size, sf::Color color) {
  return latex + "|" + std::to_string(size) + "|" + std::to_string(color.r);
}

const sf::Font* LatexRenderer::s_font = nullptr;
sf::Font LatexRenderer::s_fallbackFont;
bool LatexRenderer::s_fallbackLoaded = false;

LatexRenderer::LatexRenderer(sf::RenderTarget& target) : m_target(&target) {}
void LatexRenderer::setTarget(sf::RenderTarget& target) { m_target = &target; }
void LatexRenderer::setColor(const sf::Color& color) { m_color = color; }

const sf::Font* LatexRenderer::getFont() {
  if (s_font) return s_font;
  if (!s_fallbackLoaded) {
    // Try standard Windows fonts
    if (s_fallbackFont.loadFromFile("C:\\Windows\\Fonts\\seguisym.ttf")) {
         std::cout << "[LatexRenderer] Loaded Segoe UI Symbol" << std::endl;
    } else if (s_fallbackFont.loadFromFile("C:\\Windows\\Fonts\\arial.ttf")) {
         std::cout << "[LatexRenderer] Loaded Arial" << std::endl;
    }
    s_fallbackLoaded = true;
  }
  return &s_fallbackFont;
}

void LatexRenderer::setDefaultFont(const sf::Font& font) { s_font = &font; }
void LatexRenderer::drawText(const std::string& text, float x, float y, float size) { /* Unused */ }
void LatexRenderer::drawLine(float x1, float y1, float x2, float y2, float thickness) { /* Unused */ }
void LatexRenderer::fillRect(float x, float y, float w, float h) { /* Unused */ }
void LatexRenderer::strokeRect(float x, float y, float w, float h, float thickness) { /* Unused */ }
sf::Vector2f LatexRenderer::MeasureText(const std::string& text, float size) { return {0,0}; }

std::shared_ptr<sf::Texture> LatexRenderer::RenderLatex(const std::string& latex, float size, float maxWidth, sf::Color color) {
    // Error Texture
    static sf::Texture errorTex;
    static bool errorInit = false;
    if (!errorInit) {
        sf::Image img; img.create(20, 20, sf::Color::Red);
        errorTex.loadFromImage(img); errorInit = true;
    }
    if (latex.empty()) return nullptr;

    // Cache Check
    std::string key = generateCacheKey(latex, size, color);
    if (s_textureCache.count(key)) return s_textureCache[key];

    if (!ensureTexInit()) return std::make_shared<sf::Texture>(errorTex);

    float finalSize = size * HD_FACTOR * LatexRenderer::VISUAL_SCALE;

    try {
        tex::Formula formula(tex::utf82wide(latex));
        tex::TeXRenderBuilder builder;
        builder.setStyle(tex::TexStyle::text);
        builder.setTextSize(finalSize);
        builder.setForeground(toTexColor(color));

        tex::TeXRender* render = builder.build(formula);
        
        int w = static_cast<int>(std::ceil(render->getWidth())) + 4;
        int h = static_cast<int>(std::ceil(render->getHeight())) + 10;

        // LOGGING SIZES (Check Console!)
        std::cout << "[Render] Latex: " << latex << " | Size: " << w << "x" << h << std::endl;

        sf::RenderTexture rt;
        if (!rt.create(w, h)) { delete render; return std::make_shared<sf::Texture>(errorTex); }
        rt.clear(sf::Color::Transparent);

        // DEBUG: Draw Green Box to verify sprite visibility
        sf::RectangleShape dbg(sf::Vector2f((float)w, (float)h));
        dbg.setFillColor(sf::Color(0, 255, 0, 100)); // Transparent Green
        rt.draw(dbg);

        SfmlGraphics graphics(rt); 
        
        // COORDINATE FIX: Draw slightly higher up?
        // render->getHeight() puts baseline at bottom.
        // Try shifting up by (finalSize * 0.2) to accommodate descent?
        // Or trust MicroTeX.
        render->draw(graphics, 2.0, static_cast<float>(render->getHeight()));
        
        rt.display();
        delete render; 

        auto texturePtr = std::make_shared<sf::Texture>(rt.getTexture());
        texturePtr->setSmooth(true);
        s_textureCache[key] = texturePtr;
        return texturePtr;

    } catch (const std::exception& e) {
        std::cerr << "[LatexRenderer] EXCEPTION: " << e.what() << std::endl;
        return std::make_shared<sf::Texture>(errorTex);
    }
}