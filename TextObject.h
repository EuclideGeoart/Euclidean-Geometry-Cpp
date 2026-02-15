#pragma once

#include <SFML/Graphics.hpp>
#include <string>
#include "GeometricObject.h"
#include "Types.h"

class TextObject : public GeometricObject {
public:
    TextObject(const Point_2& pos, const std::string& content, bool isLatex = false, float fontSize = 18.0f);

    void draw(sf::RenderWindow& window, float scale, bool forceVisible = false) const override;
    bool contains(const sf::Vector2f& worldPos, float tolerance) const override;
    sf::FloatRect getGlobalBounds() const override;

    Point_2 getCGALPosition() const override { return m_position; }
    void setCGALPosition(const Point_2& newPos) override { m_position = newPos; m_dirty = true; }
    void setPosition(const sf::Vector2f& newSfmlPos) override { m_position = Point_2(newSfmlPos.x, newSfmlPos.y); m_dirty = true; }
    
    void setRawContent(const std::string& content, bool isLatex) { m_content = content; m_isLatex = isLatex; m_dirty = true; }
    const std::string& getRawContent() const { return m_content; }
    bool isRichText() const { return m_isLatex; }

    void setFontSize(float size) { m_fontSize = size; m_dirty = true; }
    float getFontSize() const { return m_fontSize; }

    ObjectType getType() const override { return ObjectType::TextObject; }
    void update() override;

private:
    void refreshCache(float scale) const;

    Point_2 m_position;
    std::string m_content;
    bool m_isLatex;
    float m_fontSize;

    mutable sf::Sprite m_cachedRender;
    mutable std::shared_ptr<sf::Texture> m_cachedTexture;
    mutable bool m_dirty = true;
    mutable float m_lastScale = 1.0f;
    mutable sf::FloatRect m_localBounds;
};
