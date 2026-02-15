#include "LatexRenderer.h"
#include <filesystem>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

// --- PORTABLE PATH FIX START ---
#ifdef _WIN32
  #include <windows.h>
#else
  #include <unistd.h>
  #include <limits.h>
  #include <libgen.h>
#endif

#include "Constants.h"
#include "graphic/graphic.h"
#include "latex.h"
#include "render.h"
#include "utils/utf.h"
#include "core/formula.h"
#include "MicroTeXSFML.h" // Include the shared header

namespace {

tex::color toTexColor(const sf::Color& color) {
  return tex::argb(static_cast<int>(color.a), static_cast<int>(color.r), static_cast<int>(color.g), static_cast<int>(color.b));
}

sf::Color toSfColor(tex::color c) {
  return sf::Color(static_cast<sf::Uint8>(tex::color_r(c)), static_cast<sf::Uint8>(tex::color_g(c)), static_cast<sf::Uint8>(tex::color_b(c)),
                   static_cast<sf::Uint8>(tex::color_a(c)));
}

namespace fs = std::filesystem;

// Helper: Get the directory where the actual binary (FluxGeo) lives
static fs::path getExecutableDir() {
#ifdef _WIN32
  char buf[MAX_PATH] = {};
  DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
  if (n > 0) return fs::path(buf).parent_path();
#else
  char buf[PATH_MAX] = {};
  // /proc/self/exe is a symlink to the executable on Linux
  ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n > 0) { 
      buf[n] = '\0'; 
      return fs::path(buf).parent_path(); 
  }
#endif
  return fs::current_path(); // Fallback
}

// Helper: Hunt for the "res" folder in common locations
static std::string resolveMicroTeXResPath() {
  const fs::path exeDir = getExecutableDir();
  const fs::path cwd = fs::current_path();

  const std::vector<fs::path> candidates = {
    exeDir / "res",
    exeDir / "microtex-res",
    cwd / "res",
    cwd / "microtex-res",
    cwd / "libs" / "MicroTeX" / "res",
    exeDir.parent_path() / "libs" / "MicroTeX" / "res" // In case exe is in build/
  };

  // std::cout << "[LatexRenderer] Debugging Paths:\n";
  // std::cout << "  Executable Dir: " << exeDir << "\n";
  // std::cout << "  Working Dir:    " << cwd << "\n";

  for (const auto& c : candidates) {
    std::error_code ec;
    if (fs::exists(c, ec) && fs::is_directory(c, ec)) {
         // std::cout << "[LatexRenderer] FOUND resources at: " << fs::absolute(c) << "\n";
         return fs::absolute(c).string();
    }
  }

  std::cerr << "  [LatexRenderer] ERROR: Could not find resource folder!" << std::endl;
  return "";
}

// --- MATH FONT SEPARATION ---
static sf::Font s_mathFont;
static bool s_mathFontLoaded = false;

static const sf::Font* getMathFont() {
  if (s_mathFontLoaded) return &s_mathFont;
  // Fallback loading if MicroTeX font manager fails
  if (s_mathFont.loadFromFile("latinmodern-math.otf")) {
       std::cout << "[LatexRenderer] Loaded Fallback Math Font\n";
  }
  s_mathFontLoaded = true;
  return &s_mathFont;
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
  
  void setFont(const tex::Font* font) override { 
      m_font = font;
      m_sfFont = nullptr;
      if (m_font) {
          // Cast to Font_sfml to get the underlying SFML font
          // We use static_cast because we know MicroTeXSFML creates Font_sfml
          const tex::Font_sfml* f = static_cast<const tex::Font_sfml*>(m_font);
          if (f) {
              m_sfFont = f->getSfFont().get();
          }
      }
  }

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
    
    // Use the Font provided by MicroTeX (via setFont)
    const sf::Font* font = m_sfFont;
    
    // Fallback if something went wrong
    if (!font) font = getMathFont();
    
    if (!font) {
        std::cerr << "[SfmlGraphics] ERROR: No font available!\n";
        return;
    }

    sf::Text text;
    text.setFont(*font);
    text.setString(sf::String(c));
    
    // CORRECT SCALING STRATEGY:
    constexpr float HIGH_RES_SIZE = 100.0f;
    unsigned int textureSize = static_cast<unsigned int>(HIGH_RES_SIZE);
    text.setCharacterSize(textureSize);
    
    float intendedSize = m_font ? m_font->getSize() : 1.0f;
    float scaleFactor = intendedSize / HIGH_RES_SIZE;
    text.setScale(scaleFactor, scaleFactor);
    
    sf::Color sCol = toSfColor(m_color);
    text.setFillColor(sCol);

    // Using simple offset implementation matching MicroTeX baseline
    text.setPosition(x, y - intendedSize); 
    
    // DEBUG LOGS CLEARED
    // if (!c.empty()) { ... }

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
  const sf::Font* m_sfFont = nullptr; // Mapped SFML font
  tex::Stroke m_stroke;
  sf::Transform m_transform = sf::Transform::Identity;
  float m_sx = 1.0f;
  float m_sy = 1.0f;
};

static bool g_texInitialized = false;

bool ensureTexInit() {
  if (g_texInitialized) return true;
  std::string bestPath = resolveMicroTeXResPath();
  try {
     if (!bestPath.empty()) {
         tex::LaTeX::init(bestPath);
     } else {
         tex::LaTeX::init("res"); // Last ditch effort
     }
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

// ADAPTIVE DPI IMPLEMENTATION
float LatexRenderer::VISUAL_SCALE = 2.5f; // Default fallback
// Initial inverse calculation based on default
float LatexRenderer::HD_INVERSE = 1.0f / (LatexRenderer::HD_FACTOR * 2.5f);

static bool s_dpiInitialized = false;

void LatexRenderer::initDPI() {
    if (s_dpiInitialized) return;
    
    auto mode = sf::VideoMode::getDesktopMode();
    float screenHeight = static_cast<float>(mode.height);
    
    // Baseline: 1080p -> 2.5f scale
    // Formula:  2.5 * (ScreenHeight / 1080)
    float calculatedScale = 2.5f * (screenHeight / 1080.0f);
    
    // Clamp to minimum 2.5f to ensure readability on standard/low-res screens
    if (calculatedScale < 2.5f) calculatedScale = 2.5f;
    
    // Clamp to maximum 10.0f just in case of weird reporting
    if (calculatedScale > 10.0f) calculatedScale = 10.0f;
    
    VISUAL_SCALE = calculatedScale;
    
    // FIX: To make labels adaptive (larger on high-res screens), we must NOT
    // completely invert the visual scale. We should invert based on the BASE scale (2.5).
    // This allows the extra resolution from VISUAL_SCALE to translate into physical size.
    // effective_scale = VISUAL_SCALE * HD_INVERSE
    //               = (2.5 * ratio) * (1.0 / (4.0 * 2.5))
    //               = ratio / 4.0 <-- Wait, TextLabel applies scale * HD_INVERSE.
    // Actually, TextLabel applies `scale * HD_INVERSE`.
    // We want `HD_INVERSE` to be such that at 1080p (scale 2.5), result is 1.0.
    // At 5K (scale 6.6), result is 2.66.
    
    // Constant inverse based on BASE_SCALE (2.5f)
    HD_INVERSE = 1.0f / (HD_FACTOR * 2.5f); 
    
    std::cout << "[LatexRenderer] Adaptive DPI Scanned: Height=" << screenHeight 
              << "px -> Scale=" << VISUAL_SCALE 
              << " -> inv=" << HD_INVERSE << "\n";
              
    s_dpiInitialized = true;
}

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
    // STANDARD UI FONTS ONLY
    if (s_fallbackFont.loadFromFile("C:\\Windows\\Fonts\\seguisym.ttf")) {
         std::cout << "[LatexRenderer] Loaded Segoe UI Symbol (default)" << std::endl;
    } else if (s_fallbackFont.loadFromFile("C:\\Windows\\Fonts\\arial.ttf")) {
         std::cout << "[LatexRenderer] Loaded Arial (default)" << std::endl;
    } else {
         std::cerr << "[LatexRenderer] CRITICAL: No system fonts found for UI!" << std::endl;
    }
    s_fallbackLoaded = true;
  }
  return &s_fallbackFont;
}

void LatexRenderer::setDefaultFont(const sf::Font& font) { s_font = &font; }
void LatexRenderer::drawText(const std::string& text, float x, float y, float size) { /* Unused */ }
void LatexRenderer::drawLine(float x1, float y1, float x2, float y2, float thickness) { /* Unused */ }
void LatexRenderer::fillRect(float x, float y, float w, float h) { (void)x; (void)y; (void)w; (void)h; /* Unused */ }
void LatexRenderer::strokeRect(float x, float y, float w, float h, float thickness) { (void)x; (void)y; (void)w; (void)h; (void)thickness; /* Unused */ }
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

    // Initializate DPI settings if not done
    initDPI();

    float finalSize = size * HD_FACTOR * LatexRenderer::VISUAL_SCALE;

    try {
        // AUTOMATIC SPACING FIX:
        // Users expect "a b" to render with a space, but LaTeX math mode ignores spaces.
        // We replace " " with "~" (non-breaking space) to fulfill this expectation.
        std::string processedLatex = latex;
        // Simple replace all spaces. 
        // Note: This might break some complex LaTeX that relies on spaces (like macro arguments), 
        // but for a geometry tool's label input, it's the desired behavior.
        // We avoid replacing if it looks like it's inside a command? No, simple replacement is safer for casual users.
        // Actually, let's just replace ' ' with '~'
        size_t pos = 0;
        while ((pos = processedLatex.find(' ', pos)) != std::string::npos) {
            processedLatex.replace(pos, 1, "~");
            pos += 1; // Move past the replacement
        }

        tex::Formula formula(tex::utf82wide(processedLatex));
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
        // sf::RectangleShape dbg(sf::Vector2f((float)w, (float)h)); 
        (void)w; (void)h; // Suppress unused warning since debug code is commented out
        // dbg.setFillColor(sf::Color(0, 255, 0, 100)); // Transparent Green
        // rt.draw(dbg);

        SfmlGraphics graphics(rt); 
        
        // COORDINATE FIX: Draw at top-left (y=0) to avoid double-offsetting
        // render->getHeight() puts baseline at bottom, but our matrix might already handle this or it pushes it off screen.
        render->draw(graphics, 2.0f, 0.0f);
        
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