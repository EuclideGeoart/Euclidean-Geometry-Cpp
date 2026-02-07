#pragma once

#include <SFML/Graphics.hpp>
#include <memory>
#include <string>

#include "GeometricObject.h"
#include "LatexRenderer.h"

class GeometryEditor;

class TextLabel : public GeometricObject {
 public:
  TextLabel(const Point_2& pos,
            const std::string& content,
            bool richText,
            float fontSize,
            const sf::Color& color,
            unsigned int id);

  void draw(sf::RenderWindow& window, float scale, bool forceVisible = false) const override;
  bool contains(const sf::Vector2f& worldPos, float tolerance) const override;
  sf::FloatRect getGlobalBounds() const override;

  Point_2 getCGALPosition() const override;
  void setCGALPosition(const Point_2& newPos) override;
  void setPosition(const sf::Vector2f& newSfmlPos) override;
  void translate(const Vector_2& offset) override;
  void update() override;

  void setRawContent(const std::string& content, bool richText);
  const std::string& getRawContent() const;
  bool isRichText() const;
  void setRichText(bool richText);

  void setFontSize(float size);
  float getFontSize() const;

  void setBoxWidthWorld(float widthWorld);
  float getBoxWidthWorld() const;

  void edit(GeometryEditor& editor);

  void setColor(const sf::Color& color) override;

 private:
  void refreshCache(float scale);

  std::string m_rawContent;
  bool m_isRichText = false;
  float m_width = 0.0f;
  float m_height = 0.0f;
  float m_fontSize = 18.0f;
  mutable float m_lastScale = 1.0f;
  float m_boxWidthWorld = 0.0f;

  Point_2 m_position;
  std::shared_ptr<sf::Texture> m_cachedTexture;
  mutable sf::Sprite m_cachedRender;
  bool m_dirty = true;
  
  // Hit-testing bounds
  sf::FloatRect m_localBounds;
};
