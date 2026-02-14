#include "TextLabel.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "GeometryEditor.h"

TextLabel::TextLabel(const Point_2& pos,
                     const std::string& content,
                     bool richText,
                     float fontSize,
                     const sf::Color& color,
                     unsigned int id)
    : GeometricObject(ObjectType::TextLabel, color, pos, id),
      m_rawContent(content),
      m_isRichText(richText),
      m_fontSize(fontSize),
      m_position(pos) {
  m_dirty = true;
}

void TextLabel::setRawContent(const std::string& content, bool richText) {
  m_rawContent = content;
  m_isRichText = richText;
  m_dirty = true;
}

const std::string& TextLabel::getRawContent() const {
  return m_rawContent;
}

bool TextLabel::isRichText() const {
  return m_isRichText;
}

void TextLabel::setRichText(bool richText) {
  if (m_isRichText == richText) return;
  m_isRichText = richText;
  m_dirty = true;
}

void TextLabel::setFontSize(float size) {
  m_fontSize = std::max(6.0f, size);
  m_dirty = true;
}

float TextLabel::getFontSize() const {
  return m_fontSize;
}

void TextLabel::setBoxWidthWorld(float widthWorld) {
  m_boxWidthWorld = std::max(0.0f, widthWorld);
  m_dirty = true;
}

float TextLabel::getBoxWidthWorld() const {
  return m_boxWidthWorld;
}

void TextLabel::setColor(const sf::Color& color) {
  GeometricObject::setColor(color);
  m_dirty = true;
}

Point_2 TextLabel::getCGALPosition() const {
  return m_position;
}

void TextLabel::setCGALPosition(const Point_2& newPos) {
  m_position = newPos;
}

void TextLabel::setPosition(const sf::Vector2f& newSfmlPos) {
  m_position = Point_2(newSfmlPos.x, newSfmlPos.y);
}

void TextLabel::translate(const Vector_2& offset) {
  if (isDependent() || isLocked()) return;
  m_position = m_position + offset;
}

void TextLabel::refreshCache(float scale) {
  float safeScale = (scale > 0.0f) ? scale : 1.0f;
  float boxWidthPixels = (m_boxWidthWorld > 0.0f) ? (m_boxWidthWorld / safeScale) : 0.0f;

  bool useLatex = m_isRichText;
  if (!useLatex && !m_rawContent.empty()) {
    if (m_rawContent.rfind("$", 0) == 0 || m_rawContent.rfind("\\(", 0) == 0 || m_rawContent.rfind("\\[", 0) == 0) {
      useLatex = true;
    }
  }

  if (useLatex) {
    m_cachedTexture = std::shared_ptr<sf::Texture>(LatexRenderer::RenderLatex(m_rawContent, m_fontSize, boxWidthPixels, m_color));
  } else {
    const sf::Font* font = LatexRenderer::getFont();
    if (font) {
      std::string text = m_rawContent;
      
      // Calculate high-resolution font size to match HD scaling
      float hdFontSize = m_fontSize * LatexRenderer::HD_FACTOR * LatexRenderer::VISUAL_SCALE;
      unsigned int charSize = static_cast<unsigned int>(std::round(hdFontSize));
      float hdBoxWidth = (boxWidthPixels > 0.0f) ? (boxWidthPixels * LatexRenderer::HD_FACTOR * LatexRenderer::VISUAL_SCALE) : 0.0f;

      if (boxWidthPixels > 0.0f) {
        std::istringstream iss(m_rawContent);
        std::string word;
        std::string line;
        std::string wrapped;
        sf::Text measure;
        measure.setFont(*font);
        measure.setCharacterSize(charSize);

        while (iss >> word) {
          std::string candidate = line.empty() ? word : (line + " " + word);
          measure.setString(candidate);
          // Check against high-res width
          if (measure.getLocalBounds().width > hdBoxWidth && !line.empty()) {
            if (!wrapped.empty()) wrapped += "\n";
            wrapped += line;
            line = word;
          } else {
            line = candidate;
          }
        }
        if (!line.empty()) {
          if (!wrapped.empty()) wrapped += "\n";
          wrapped += line;
        }
        text = wrapped.empty() ? m_rawContent : wrapped;
      }

      sf::Text drawable;
      drawable.setFont(*font);
      drawable.setString(text.empty() ? " " : text);
      drawable.setCharacterSize(charSize);
      drawable.setFillColor(m_color);

      // Add padding for high-res texture
      sf::FloatRect bounds = drawable.getLocalBounds();
      unsigned int width = static_cast<unsigned int>(std::ceil(bounds.width + 20.0f));
      unsigned int height = static_cast<unsigned int>(std::ceil(bounds.height + 20.0f));
      width = std::max(1u, width);
      height = std::max(1u, height);

      sf::RenderTexture renderTexture;
      if (renderTexture.create(width, height)) {
        renderTexture.clear(sf::Color::Transparent);
        drawable.setPosition(-bounds.left + 5.0f, -bounds.top + 5.0f);
        renderTexture.draw(drawable);
        renderTexture.display();
        m_cachedTexture = std::make_shared<sf::Texture>(renderTexture.getTexture());
      }
    }
  }
  if (m_cachedTexture) {
    auto size = m_cachedTexture->getSize();
    m_width = static_cast<float>(size.x);
    m_height = static_cast<float>(size.y);
    m_cachedRender.setTexture(*m_cachedTexture, true);
    
    // Calculate local bounds for hit-testing
    // HD_FACTOR=4.0, VISUAL_SCALE=2.5 -> HD_INVERSE = 0.1
    const float HD_INVERSE = 1.0f / (4.0f * 2.5f);
    m_localBounds = sf::FloatRect(0.0f, 0.0f, 
                                  m_width * HD_INVERSE, 
                                  m_height * HD_INVERSE);
  } else {
    m_width = 0.0f;
    m_height = 0.0f;
    m_localBounds = sf::FloatRect(0.0f, 0.0f, 0.0f, 0.0f);
  }
  m_dirty = false;
}

void TextLabel::update() {
  if (m_dirty || !m_cachedTexture) {
    refreshCache(m_lastScale > 0.0f ? m_lastScale : 1.0f);
  }
}

void TextLabel::draw(sf::RenderWindow& window, float scale, bool forceVisible) const {
  if (!isVisible() && !forceVisible) return;

  if (m_dirty || !m_cachedTexture || std::abs(scale - m_lastScale) > 1e-6f) {
    const_cast<TextLabel*>(this)->refreshCache(scale);
  }

  if (!m_cachedTexture) return;

  // ===== HIGH-DPI INVERSE SCALING =====
  // HD_FACTOR=4.0, VISUAL_SCALE=2.5 -> Render at 10x, display at 1x
  // Inverse = 1.0 / (HD_FACTOR * VISUAL_SCALE) = 0.1
  const float HD_INVERSE = 1.0f / (4.0f * 2.5f); // = 0.1
  
  // Calculate the scaled dimensions for hit-testing
  float displayW = m_width * scale * HD_INVERSE;
  float displayH = m_height * scale * HD_INVERSE;
  
  // Get anchor position
  float anchorX = static_cast<float>(CGAL::to_double(m_position.x()));
  float anchorY = static_cast<float>(CGAL::to_double(m_position.y()));

  // Create sprite with texture
  sf::Sprite sprite(*m_cachedTexture);
  sprite.setColor(m_color);
  
  // The Y-flip draws the sprite upwards from the anchor point.
  // So the visual box spans [anchorX, anchorX + displayW] x [anchorY - displayH, anchorY]
  sprite.setScale(scale * HD_INVERSE, -scale * HD_INVERSE);
  sprite.setPosition(anchorX, anchorY);

  window.draw(sprite);
  
  // ===== SELECTION OUTLINE (only when selected) =====
  if (isSelected()) {
    // The sprite is drawn upward from anchor due to Y-flip
    // So bounds.top = anchorY - displayH, bounds.bottom = anchorY
    sf::RectangleShape outline(sf::Vector2f(displayW, displayH));
    outline.setPosition(anchorX, anchorY - displayH);
    outline.setFillColor(sf::Color::Transparent);
    outline.setOutlineColor(sf::Color(100, 150, 255, 200)); // Blue selection
    outline.setOutlineThickness(2.0f * scale);
    window.draw(outline);
  }
  
  m_lastScale = scale;
}

sf::FloatRect TextLabel::getGlobalBounds() const {
  float scale = (m_lastScale > 0.0f) ? m_lastScale : 1.0f;
  
  // HD_FACTOR=4.0, VISUAL_SCALE=2.5 -> HD_INVERSE = 0.1
  const float HD_INVERSE = 1.0f / (4.0f * 2.5f);
  float displayW = m_width * scale * HD_INVERSE;
  float displayH = m_height * scale * HD_INVERSE;
  
  float anchorX = static_cast<float>(CGAL::to_double(m_position.x()));
  float anchorY = static_cast<float>(CGAL::to_double(m_position.y()));
  
  // The sprite draws UPWARD from anchor due to Y-flip
  // So bounds are: left=anchorX, top=anchorY-displayH, width=displayW, height=displayH
  sf::FloatRect bounds;
  bounds.left = anchorX;
  bounds.top = anchorY - displayH;
  bounds.width = displayW;
  bounds.height = displayH;
  return bounds;
}

bool TextLabel::contains(const sf::Vector2f& worldPos, float tolerance) const {
  if (m_localBounds.width <= 0.0f || m_localBounds.height <= 0.0f) {
    return false;
  }
  
  float scale = (m_lastScale > 0.0f) ? m_lastScale : 1.0f;
  
  // HD_FACTOR=4.0, VISUAL_SCALE=2.5 -> HD_INVERSE = 0.1  
  const float HD_INVERSE = 1.0f / (4.0f * 2.5f);
  float displayW = m_width * scale * HD_INVERSE;
  float displayH = m_height * scale * HD_INVERSE;
  
  float anchorX = static_cast<float>(CGAL::to_double(m_position.x()));
  float anchorY = static_cast<float>(CGAL::to_double(m_position.y()));
  
  // The sprite draws UPWARD from anchor due to Y-flip
  // So hit-box is: [anchorX, anchorX + displayW] x [anchorY - displayH, anchorY]
  sf::FloatRect worldBounds;
  worldBounds.left = anchorX - tolerance;
  worldBounds.top = anchorY - displayH - tolerance;
  worldBounds.width = displayW + tolerance * 2.0f;
  worldBounds.height = displayH + tolerance * 2.0f;
  
  return worldBounds.contains(worldPos);
}

void TextLabel::edit(GeometryEditor& editor) {
  editor.isTextEditing = true;
  editor.textEditingLabel = std::dynamic_pointer_cast<TextLabel>(editor.findSharedPtr(this));
  editor.textEditBuffer = m_rawContent;
  editor.textEditIsRich = m_isRichText;
  editor.textEditFontSize = m_fontSize;
}
