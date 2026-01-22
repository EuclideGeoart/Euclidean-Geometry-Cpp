#ifndef CONTEXT_MENU_H
#define CONTEXT_MENU_H

#include "CharTraitsFix.h"  // Must be before SFML includes
#include <SFML/Graphics.hpp>
#include <vector>
#include <functional>
#include <string>
#include <iostream>

class GeometryEditor; // Forward decl

struct MenuItem {
    std::string label;
    std::function<void(GeometryEditor&)> action;
    sf::RectangleShape shape;
    sf::Text text;
    sf::Color hoverColor = sf::Color(200, 200, 200);
    sf::Color defaultColor = sf::Color(240, 240, 240);
    bool isHovered = false;
};

class ContextMenu {
public:
    ContextMenu();
    
    void addItem(const std::string& label, std::function<void(GeometryEditor&)> action);
    void open(const sf::Vector2f& position, GeometryEditor& editor);
    void close();
    bool isOpen() const { return m_isOpen; }
    
    bool handleEvent(const sf::Event& event, const sf::Vector2f& mousePos, GeometryEditor& editor);
    void draw(sf::RenderWindow& window);

private:
    std::vector<MenuItem> m_items;
    sf::Vector2f m_position;
    bool m_isOpen = false;
    sf::Font m_font;
    float m_width = 150.0f;
    float m_itemHeight = 30.0f;
    
    void updateLayout();
};

#endif // CONTEXT_MENU_H
