#include "TextObject.h"
#include "LatexRenderer.h"
#include <cmath>

TextObject::TextObject(const Point_2& pos, const std::string& content, bool isLatex, float fontSize)
    : GeometricObject(ObjectType::TextObject, sf::Color::Black, pos, 0),
      m_position(pos), m_content(content), m_isLatex(isLatex), m_fontSize(fontSize) {
        m_type = ObjectType::TextObject;
    m_dirty = true;
}

void TextObject::draw(sf::RenderWindow& window, float scale, bool forceVisible) const {
    if (!m_visible && !forceVisible) return;

    if (m_dirty || std::abs(scale - m_lastScale) > 1e-6f) {
        refreshCache(scale);
    }

    if (m_cachedTexture) {
        sf::Sprite sprite(*m_cachedTexture);
        
        const float HD_INVERSE = LatexRenderer::HD_INVERSE;
        sprite.setScale(HD_INVERSE * scale, HD_INVERSE * scale);
        
        sf::Vector2f sfmlPos(static_cast<float>(CGAL::to_double(m_position.x())), 
                             static_cast<float>(CGAL::to_double(m_position.y())));
        sprite.setPosition(sfmlPos);
        
        sf::FloatRect bounds = sprite.getLocalBounds();
        sprite.setOrigin(bounds.width / 2.0f, bounds.height / 2.0f);
        
        window.draw(sprite);
    }
}

void TextObject::refreshCache(float scale) const {
    float safeScale = (scale > 0.0f) ? scale : 1.0f;
    
    // For now, always use LatexRenderer as it handles both plain text and LaTeX in this project
    m_cachedTexture = std::shared_ptr<sf::Texture>(LatexRenderer::RenderLatex(m_content, m_fontSize, 0.0f, m_color));
    
    if (m_cachedTexture) {
        m_cachedTexture->setSmooth(true);
        const float HD_INVERSE = LatexRenderer::HD_INVERSE;
        m_localBounds = sf::FloatRect(0.0f, 0.0f, 
                                      m_cachedTexture->getSize().x * HD_INVERSE, 
                                      m_cachedTexture->getSize().y * HD_INVERSE);
    } else {
        m_localBounds = sf::FloatRect(0.0f, 0.0f, 0.0f, 0.0f);
    }

    m_dirty = false;
    m_lastScale = scale;
}

void TextObject::update() {
    m_dirty = true;
}

bool TextObject::contains(const sf::Vector2f& worldPos, float tolerance) const {
    if (!m_visible) return false;
    
    sf::FloatRect bounds = getGlobalBounds();
    bounds.left -= tolerance;
    bounds.top -= tolerance;
    bounds.width += 2 * tolerance;
    bounds.height += 2 * tolerance;
    
    return bounds.contains(worldPos);
}

sf::FloatRect TextObject::getGlobalBounds() const {
    // Current draw logic centers the sprite at m_position
    sf::Vector2f sfmlPos(static_cast<float>(CGAL::to_double(m_position.x())), 
                         static_cast<float>(CGAL::to_double(m_position.y())));
    
    float w = m_localBounds.width * m_lastScale;
    float h = m_localBounds.height * m_lastScale;
    
    return sf::FloatRect(sfmlPos.x - w/2.0f, sfmlPos.y - h/2.0f, w, h);
}
