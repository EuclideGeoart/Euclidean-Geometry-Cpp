#include "ContextMenu.h"
#include "Constants.h"
#include "GeometryEditor.h" // Needed for incomplete type in callbacks if used here, but mainly for consistent types

ContextMenu::ContextMenu() {
    if (!m_font.loadFromFile(Constants::DEFAULT_FONT_PATH)) {
        std::cerr << "ContextMenu failed to load font: " << Constants::DEFAULT_FONT_PATH << std::endl;
        // Fallback or error handling
    }
}

void ContextMenu::addItem(const std::string& label, std::function<void(GeometryEditor&)> action) {
    MenuItem item;
    item.label = label;
    item.action = action;
    
    // Setup visuals
    item.shape.setSize(sf::Vector2f(m_width, m_itemHeight));
    item.shape.setFillColor(item.defaultColor);
    item.shape.setOutlineThickness(1.0f);
    item.shape.setOutlineColor(sf::Color(100, 100, 100));
    
    item.text.setFont(m_font);
    item.text.setString(label);
    item.text.setCharacterSize(14);
    item.text.setFillColor(sf::Color::Black);
    
    m_items.push_back(item);
}

void ContextMenu::open(const sf::Vector2f& position, GeometryEditor& editor) {
    m_position = position;
    m_isOpen = true;
    updateLayout();
}

void ContextMenu::close() {
    m_isOpen = false;
}

void ContextMenu::updateLayout() {
    for (size_t i = 0; i < m_items.size(); ++i) {
        float yPos = m_position.y + i * m_itemHeight;
        m_items[i].shape.setPosition(m_position.x, yPos);
        
        // Center text vertically
        sf::FloatRect textBounds = m_items[i].text.getLocalBounds();
        float textX = m_position.x + 10.0f; // Padding
        float textY = yPos + (m_itemHeight - textBounds.height) / 2.0f - textBounds.top;
        m_items[i].text.setPosition(textX, textY);
    }
}

bool ContextMenu::handleEvent(const sf::Event& event, const sf::Vector2f& mousePos, GeometryEditor& editor) {
    if (!m_isOpen) return false;

    if (event.type == sf::Event::MouseMoved) {
        for (auto& item : m_items) {
            if (item.shape.getGlobalBounds().contains(mousePos)) {
                item.isHovered = true;
                item.shape.setFillColor(item.hoverColor);
            } else {
                item.isHovered = false;
                item.shape.setFillColor(item.defaultColor);
            }
        }
        return true; // We swallow moves if over menu? Or maybe just let it pass.
                     // Better to swallow if over menu to avoid highlighting objects behind it.
        // Check if mouse is over ANY item
        sf::FloatRect menuBounds(m_position.x, m_position.y, m_width, m_items.size() * m_itemHeight);
        if (menuBounds.contains(mousePos)) return true;
    }

    if (event.type == sf::Event::MouseButtonPressed) {
        if (event.mouseButton.button == sf::Mouse::Left) {
            for (auto& item : m_items) {
                if (item.shape.getGlobalBounds().contains(mousePos)) {
                    if (item.action) {
                        item.action(editor);
                    }
                    close(); // Close after action
                    return true;
                }
            }
            // If clicked outside, close
            close();
            return true; // Swallowed the close click?
                         // If we click empty space, we just close. 
                         // But we might want to let that click select something else?
                         // Usually Context Menu consumes the "dismiss" click.
        }
    }
    
    return false;
}

void ContextMenu::draw(sf::RenderWindow& window) {
    if (!m_isOpen) return;
    
    for (const auto& item : m_items) {
        window.draw(item.shape);
        window.draw(item.text);
    }
}
