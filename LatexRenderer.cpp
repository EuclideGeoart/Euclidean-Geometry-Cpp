#include "LatexRenderer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Constants.h"
#include "core/formula.h"
#include "graphic/graphic.h"
#include "latex.h"
#include "render.h"
#include "utils/utf.h"


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
  const std::vector<fs::path> candidates = {fs::path("libs/MicroTeX/res"), fs::path("../libs/MicroTeX/res"), fs::path("../../libs/MicroTeX/res"),
                                            fs::path("../../../libs/MicroTeX/res")};
  for (const auto& candidate : candidates) {
    fs::path check = candidate / ".clatexmath-res_root";
    if (fs::exists(check)) {
      return candidate.string();
    }
  }
  return "res";
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
  void scale(float sx, float sy) override {
    m_transform.scale(sx, sy);
    m_sx *= sx;
    m_sy *= sy;
  }
  void rotate(float angle) override { m_transform.rotate(angle * 180.0f / 3.14159265358979323846f); }
  void rotate(float angle, float px, float py) override { m_transform.rotate(angle * 180.0f / 3.14159265358979323846f, px, py); }
  void reset() override {
    m_transform = sf::Transform::Identity;
    m_sx = 1.0f;
    m_sy = 1.0f;
  }

  float sx() const override { return m_sx; }
  float sy() const override { return m_sy; }

  void drawChar(wchar_t c, float x, float y) override {
    std::wstring text(1, c);
    drawText(text, x, y);
  }

  void drawText(const std::wstring& c, float x, float y) override {
    if (!m_target) return;
    const sf::Font* font = LatexRenderer::getFont();
    unsigned int size = 12;
    if (m_font) {
      size = static_cast<unsigned int>(std::round(m_font->getSize()));
    }

    if (!font) return;
    sf::Text text;
    text.setFont(*font);
    text.setString(sf::String(c));
    text.setCharacterSize(size);
    text.setFillColor(toSfColor(m_color));
    text.setPosition(x, y);

    sf::RenderStates states;
    states.transform = m_transform;
    m_target->draw(text, states);
  }

  void drawLine(float x1, float y1, float x2, float y2) override {
    if (!m_target) return;
    sf::VertexArray line(sf::Lines, 2);
    line[0].position = sf::Vector2f(x1, y1);
    line[1].position = sf::Vector2f(x2, y2);
    sf::Color c = toSfColor(m_color);
    line[0].color = c;
    line[1].color = c;

    sf::RenderStates states;
    states.transform = m_transform;
    m_target->draw(line, states);
  }

  void drawRect(float x, float y, float w, float h) override {
    if (!m_target) return;
    sf::RectangleShape rect(sf::Vector2f(w, h));
    rect.setPosition(x, y);
    rect.setFillColor(sf::Color::Transparent);
    rect.setOutlineColor(toSfColor(m_color));
    rect.setOutlineThickness(std::max(1.0f, m_stroke.lineWidth));

    sf::RenderStates states;
    states.transform = m_transform;
    m_target->draw(rect, states);
  }

  void fillRect(float x, float y, float w, float h) override {
    if (!m_target) return;
    sf::RectangleShape rect(sf::Vector2f(w, h));
    rect.setPosition(x, y);
    rect.setFillColor(toSfColor(m_color));
    rect.setOutlineThickness(0.f);

    sf::RenderStates states;
    states.transform = m_transform;
    m_target->draw(rect, states);
  }

  void drawRoundRect(float x, float y, float w, float h, float rx, float ry) override {
    (void)rx;
    (void)ry;
    drawRect(x, y, w, h);
  }

  void fillRoundRect(float x, float y, float w, float h, float rx, float ry) override {
    (void)rx;
    (void)ry;
    fillRect(x, y, w, h);
  }

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

void ensureTexInit() {
  if (g_texInitialized) return;

  // Check for math font in assets directory (for future font loading)
  namespace fs = std::filesystem;
  std::string fontPath;
  const std::vector<fs::path> fontCandidates = {fs::path("assets/latinmodern-math.otf"), fs::path("../assets/latinmodern-math.otf"),
                                                fs::path("libs/MicroTeX/res/fonts/latinmodern-math.otf")};

  bool fontFound = false;
  for (const auto& candidate : fontCandidates) {
    if (fs::exists(candidate)) {
      fontPath = candidate.string();
      fontFound = true;
      break;
    }
  }

  // Initialize MicroTeX with resource path
  std::string resPath = resolveMicroTeXResPath();
  std::cout << "[LatexRenderer] Initializing MicroTeX with resource path: " << resPath << std::endl;

  if (fontFound) {
    std::cout << "[LatexRenderer] Found math font at: " << fontPath << std::endl;
  } else {
    std::cout << "[LatexRenderer] WARNING: Math font not found in any candidate location" << std::endl;
  }

  tex::LaTeX::init(resPath);

  if (fontFound) {
    // Font found - future: can use for custom font loading
    // Currently MicroTeX loads its own fonts from res directory
  }

  g_texInitialized = true;
}

}  // namespace

const sf::Font* LatexRenderer::s_font = nullptr;
sf::Font LatexRenderer::s_fallbackFont;
bool LatexRenderer::s_fallbackLoaded = false;

LatexRenderer::LatexRenderer(sf::RenderTarget& target) : m_target(&target) {}

void LatexRenderer::setTarget(sf::RenderTarget& target) { m_target = &target; }

void LatexRenderer::setColor(const sf::Color& color) { m_color = color; }

const sf::Font* LatexRenderer::getFont() {
  if (s_font) return s_font;
  if (!s_fallbackLoaded) {
    // Try Segoe UI Symbol first (best for Math/Greek in plain text)
    if (!s_fallbackFont.loadFromFile("C:\\Windows\\Fonts\\seguisym.ttf")) {
       if (!s_fallbackFont.loadFromFile("C:\\Windows\\Fonts\\segoeui.ttf")) {
           s_fallbackFont.loadFromFile(Constants::DEFAULT_FONT_PATH); // Fallback to arial
       }
    }
    s_fallbackLoaded = true;
  }
  return &s_fallbackFont;
}

void LatexRenderer::setDefaultFont(const sf::Font& font) { s_font = &font; }

void LatexRenderer::drawText(const std::string& text, float x, float y, float size) {
  if (!m_target) return;
  const sf::Font* font = getFont();
  if (!font) return;

  sf::Text drawable;
  drawable.setFont(*font);
  drawable.setString(text);
  drawable.setCharacterSize(static_cast<unsigned int>(std::round(size)));
  drawable.setFillColor(m_color);
  drawable.setPosition(x, y);
  m_target->draw(drawable);
}

void LatexRenderer::drawLine(float x1, float y1, float x2, float y2, float thickness) {
  if (!m_target) return;
  sf::Vector2f p1(x1, y1);
  sf::Vector2f p2(x2, y2);
  sf::Vector2f delta = p2 - p1;
  float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
  if (length <= 0.0f) return;

  sf::RectangleShape line(sf::Vector2f(length, std::max(1.0f, thickness)));
  line.setFillColor(m_color);
  line.setOrigin(0.f, line.getSize().y * 0.5f);
  line.setPosition(p1);
  float angle = std::atan2(delta.y, delta.x) * 180.0f / 3.14159265358979323846f;
  line.setRotation(angle);
  m_target->draw(line);
}

void LatexRenderer::fillRect(float x, float y, float w, float h) {
  if (!m_target) return;
  sf::RectangleShape rect(sf::Vector2f(w, h));
  rect.setPosition(x, y);
  rect.setFillColor(m_color);
  rect.setOutlineThickness(0.f);
  m_target->draw(rect);
}

void LatexRenderer::strokeRect(float x, float y, float w, float h, float thickness) {
  if (!m_target) return;
  sf::RectangleShape rect(sf::Vector2f(w, h));
  rect.setPosition(x, y);
  rect.setFillColor(sf::Color::Transparent);
  rect.setOutlineColor(m_color);
  rect.setOutlineThickness(std::max(1.0f, thickness));
  m_target->draw(rect);
}

sf::Vector2f LatexRenderer::MeasureText(const std::string& text, float size) {
  const sf::Font* font = getFont();
  if (!font) return sf::Vector2f(0.f, 0.f);

  sf::Text drawable;
  drawable.setFont(*font);
  drawable.setString(text.empty() ? " " : text);
  drawable.setCharacterSize(static_cast<unsigned int>(std::round(size)));
  sf::FloatRect bounds = drawable.getLocalBounds();
  float w = std::max(1.0f, bounds.width + 6.0f);
  float h = std::max(1.0f, bounds.height + 6.0f);
  return sf::Vector2f(w, h);
}

sf::Texture* LatexRenderer::RenderLatex(const std::string& latex, float size, float maxWidth, sf::Color color) {
  static bool init = false;
  static sf::Texture errorTex;

  if (!init) {
    // --- MICROTEX INITIALIZATION ---
    // MicroTeX expects a RESOURCE DIRECTORY containing:
    //   - .clatexmath-res_root (marker file)
    //   - fonts/ subdirectory with font files
    //   - symbol definitions, etc.
    // It does NOT take a font file path directly.
    
    std::cout << "[LatexRenderer] Initializing MicroTeX..." << std::endl;
    std::cout << "[DEBUG] Current Working Directory: " << std::filesystem::current_path() << std::endl;

    // Check for the critical 'res' folder with marker file
    bool resExists = std::filesystem::exists("res/.clatexmath-res_root");
    std::cout << "[DEBUG] Check ./res/.clatexmath-res_root: " << (resExists ? "FOUND" : "MISSING") << std::endl;

    if (resExists) {
      // Pass the resource directory, not a font file
      tex::LaTeX::init("res");
      std::cout << "[LatexRenderer] Initialized with res/ directory" << std::endl;
    } else {
      // Try parent directory (if running from different location)
      bool parentResExists = std::filesystem::exists("../res/.clatexmath-res_root");
      if (parentResExists) {
        tex::LaTeX::init("../res");
        std::cout << "[LatexRenderer] Initialized with ../res/ directory" << std::endl;
      } else {
        std::cerr << "[CRITICAL] MicroTeX resource folder 'res/' not found!" << std::endl;
        std::cerr << "           Expected to find 'res/.clatexmath-res_root'" << std::endl;
        errorTex.create(64, 32);
        sf::Uint8 pixels[64 * 32 * 4];
        for (int i = 0; i < 64 * 32 * 4; i += 4) {
          pixels[i] = 255;     // R
          pixels[i + 1] = 0;   // G
          pixels[i + 2] = 255; // B (Magenta = error)
          pixels[i + 3] = 255; // A
        }
        errorTex.update(pixels);
        return &errorTex;
      }
    }
    init = true;
  }

  if (latex.empty()) return nullptr;

  // ===== HIGH-DPI RENDERING FIX =====
  float finalSize = size * HD_FACTOR * VISUAL_SCALE;

  try {
    std::wstring wtext = tex::utf82wide(latex);
    tex::Formula formula;
    formula.setLaTeX(wtext);

    tex::TeXRenderBuilder builder;
    
    // Construct ARGB color manually to avoid tex::argb namespace issues
    unsigned int argb = (255u << 24) | ((unsigned int)color.r << 16) | ((unsigned int)color.g << 8) | (unsigned int)color.b;
    
    builder.setStyle(tex::TexStyle::display).setTextSize(finalSize);
    builder.setForeground(argb);

    // Pass maxWidth for text wrapping (scaled to HD space)
    if (maxWidth > 0) {
      builder.setWidth(tex::UnitType::pixel, maxWidth * HD_FACTOR * VISUAL_SCALE, tex::Alignment::left)
             .setIsMaxWidth(true);
    }

    std::unique_ptr<tex::TeXRender> render(builder.build(formula));
    if (!render) {
      std::cerr << "[LatexRenderer] Failed to build formula: " << latex << std::endl;
      return new sf::Texture(errorTex);
    }

    // Get exact dimensions (width + height + descender depth)
    int w = std::max(1, static_cast<int>(std::ceil(render->getWidth())));
    int h = std::max(1, static_cast<int>(std::ceil(render->getHeight() + render->getDepth())));
    
    // DEBUG: Log render dimensions (ENABLED)
    static int logLimit = 0;
    if (logLimit++ < 50) std::cout << "[LatexRenderer] Formula '" << latex << "' -> Size: " << w << "x" << h << std::endl;

    // Create render texture sized exactly to content
    sf::RenderTexture rt;
    if (!rt.create(w, h)) {
      std::cerr << "[LatexRenderer] Failed to create RenderTexture " << w << "x" << h << std::endl;
      return new sf::Texture(errorTex);
    }
    rt.clear(sf::Color::Transparent);

    // DEBUG: Draw a semi-transparent GREEN box to prove texture exists
    sf::RectangleShape debugBg(sf::Vector2f((float)w, (float)h));
    debugBg.setFillColor(sf::Color(0, 255, 0, 128)); // Green, semi-transparent
    rt.draw(debugBg);

    if (w < 1 || h < 1) return nullptr;
    // Draw formula (MicroTeX uses baseline-origin, so offset by height)
    SfmlGraphics graphics(rt);
    render->draw(graphics, 0, static_cast<float>(render->getHeight()));
    rt.display();

    // Create final texture with exact size
    sf::Texture* result = new sf::Texture(rt.getTexture());
    result->setSmooth(true);
    return result;
    
  } catch (const std::exception& e) {
    std::cerr << "[LatexRenderer] Exception: " << e.what() << std::endl;
    return new sf::Texture(errorTex);
  }
}
