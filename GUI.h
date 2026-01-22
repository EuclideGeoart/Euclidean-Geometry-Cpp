#ifndef GUI_H
#define GUI_H
#include "CharTraitsFix.h"  // Ensure this is very early
#include <SFML/Graphics.hpp>
#include <algorithm>  // For std::clamp
#include <cmath>      // For std::sqrt, std::pow in distance
#include <iostream>   // For debugging
#include <stdexcept>  // For std::runtime_error
#include <string>     // Ensure standard string is included very early


#include "Constants.h"  // For button colors, sizes, font path. Constants.h now handles string/SFML order.
#include "ContextMenu.h"

class Circle;          // Forward declaration for previewCircle
class GeometryEditor;  // Forward declaration for handleEvent

class ColorPicker {
 private:
  sf::RectangleShape m_colorPreview;
  sf::RectangleShape m_colorPalette[16];  // Pre-defined colors
  sf::Color m_currentColor;
    float m_alpha = 0.0f;
  bool m_isOpen = false;
  bool m_isInApplicationMode = false;  // True when picker is open and waiting for object clicks
  sf::Vector2f m_position;
    sf::RectangleShape m_alphaTrack;
    sf::RectangleShape m_alphaKnob;
    bool m_draggingAlpha = false;

 public:
  ColorPicker(sf::Vector2f position) : m_currentColor(sf::Color::Blue), m_position(position) {
    setupPalette();
  }

  void setupPalette() {
    // Pre-defined colors
    sf::Color colors[] = {
        sf::Color::Red,           sf::Color::Green,   sf::Color::Blue,
        sf::Color::Yellow,        sf::Color::Magenta, sf::Color::Cyan,
        sf::Color::Black,         sf::Color::White,   sf::Color(255, 165, 0),  // Orange
        sf::Color(128, 0, 128),                                                // Purple
        sf::Color(165, 42, 42),                                                // Brown
        sf::Color(255, 192, 203),                                              // Pink
        sf::Color(128, 128, 128),                                              // Gray
        sf::Color(0, 128, 0),                                                  // Dark Green
        sf::Color(0, 0, 139),                                                  // Dark Blue
        sf::Color(139, 0, 0)                                                   // Dark Red
    };

    for (int i = 0; i < 16; i++) {
      m_colorPalette[i].setSize(sf::Vector2f(30, 30));
      m_colorPalette[i].setFillColor(colors[i]);
      m_colorPalette[i].setPosition(m_position.x + (i % 4) * 35, m_position.y + (i / 4) * 35);
      m_colorPalette[i].setOutlineThickness(2);
      m_colorPalette[i].setOutlineColor(sf::Color::Black);
    }

    m_colorPreview.setSize(sf::Vector2f(50, 50));
    m_colorPreview.setPosition(m_position.x + 150, m_position.y);
    m_colorPreview.setFillColor(m_currentColor);
    m_colorPreview.setOutlineThickness(2);
    m_colorPreview.setOutlineColor(sf::Color::Black);

    // Alpha slider visuals
    m_alphaTrack.setSize(sf::Vector2f(140.f, 8.f));
    m_alphaTrack.setPosition(m_position.x, m_position.y + 160.f);
    m_alphaTrack.setFillColor(sf::Color(150, 150, 150));
    m_alphaTrack.setOutlineThickness(1);
    m_alphaTrack.setOutlineColor(sf::Color::Black);

    m_alphaKnob.setSize(sf::Vector2f(12.f, 12.f));
    m_alphaKnob.setOrigin(6.f, 6.f);
    m_alphaKnob.setFillColor(sf::Color::White);
    m_alphaKnob.setOutlineThickness(1);
    m_alphaKnob.setOutlineColor(sf::Color::Black);
    setAlpha(m_alpha);
  }

  bool handleEvent(const sf::Event &event, const sf::Vector2f &mousePos);
  bool handleMouseMove(const sf::Event &event, const sf::Vector2f &mousePos);

  void draw(sf::RenderWindow &window) {
    if (m_isOpen) {
      for (int i = 0; i < 16; i++) {
        window.draw(m_colorPalette[i]);
      }
      window.draw(m_colorPreview);
      window.draw(m_alphaTrack);
      window.draw(m_alphaKnob);
    }
  }

  sf::Color getCurrentColor() const { return m_currentColor; }
  sf::Color getCurrentColorWithAlpha() const {
    auto c = m_currentColor;
    c.a = static_cast<sf::Uint8>(std::clamp(m_alpha, 0.0f, 255.0f));
    return c;
  }
  void setOpen(bool open) { 
    m_isOpen = open;
    if (!open) {
      m_isInApplicationMode = false;  // Exit application mode when picker closes
      m_draggingAlpha = false;
    }
  }
  bool isOpen() const { return m_isOpen; }
  bool isInApplicationMode() const { return m_isInApplicationMode; }
  
  // Get color palette for checking bounds in application mode
  const sf::RectangleShape* getColorPalette() const { return m_colorPalette; }
  void setAlpha(float alpha);
  float getAlpha() const { return m_alpha; }
  void setCurrentColor(sf::Color color) {
    m_currentColor = color;
    m_colorPreview.setFillColor(getCurrentColorWithAlpha());
  }
};

class Button {
 public:
  Button(sf::Vector2f position, sf::Vector2f size, std::string label, sf::Color defaultColor,
         sf::Color activeColor, sf::Color hoverColor);

  static void loadFont() {
    if (!fontLoaded) {
      std::cout << "Button::loadFont() attempting to load font from: "
                << Constants::DEFAULT_FONT_PATH << std::endl;
      if (!font.loadFromFile(Constants::DEFAULT_FONT_PATH)) {
        std::cerr << "Button::loadFont() FAILED to load font from: " << Constants::DEFAULT_FONT_PATH
                  << std::endl;
        fontLoaded = false;
      } else {
        std::cout << "Button::loadFont() SUCCESSFULLY loaded font from: "
                  << Constants::DEFAULT_FONT_PATH << std::endl;
        fontLoaded = true;
      }
    } else {
      std::cout << "Button::loadFont() - Font already loaded." << std::endl;
    }
  }
  static const sf::Font &getFont() { return font; }   // Static getter for the font
  static bool getFontLoaded() { return fontLoaded; }  // Static getter for font loaded status

  void draw(sf::RenderWindow &window) const;
  bool isMouseOver(const sf::RenderWindow &window,
                   const sf::View &view) const;  // Uses provided view

  void setLabel(const std::string &label);
  std::string getLabel() const;
  void updateVisualState();  // Updates color based on active/hover
  bool isActive() const;
  void setActive(bool active);
  sf::FloatRect getGlobalBounds() const {
    return shape.getGlobalBounds();
  }  // Might need adjustment if view changes
  bool isHovered() const;
  void setHovered(bool hover);  // Added missing declaration

 private:
  void centerText();
  static sf::Font font;
  static bool fontLoaded;

  sf::RectangleShape shape;
  sf::Text text;
  std::string m_label;
  bool m_active;
  bool m_hovered;  // To manage hover state internally

  sf::Color m_defaultColor;
  sf::Color m_activeColor;
  sf::Color m_hoverColor;
};

class GUI {
 public:
  GUI();
  // Draw GUI elements. drawingView is needed if GUI draws preview objects in
  // world space.
  void draw(sf::RenderWindow &window, const sf::View &drawingView, GeometryEditor &editor) const;
  void toggleButton(const std::string &buttonName, bool state);
  bool isButtonActive(const std::string &label) const;
  // handleEvent now takes GeometryEditor to interact with the main application
  // logic
  bool handleEvent(sf::RenderWindow &window, const sf::Event &event, GeometryEditor &editor);
  void updateView(const sf::View &newGuiView);            // Updates the GUI's own view
  void setView(const sf::View &view) { guiView = view; }  // Sets the GUI's view
  void update(sf::Time deltaTime);                        // Add declaration for update method

  bool isGridActive() const;
  bool isPointActive() const;
  bool isObjPointActive() const;
  bool isLineActive() const;
  bool isLineSegmentActive() const;
  bool isMoveActive() const { return isButtonActive("Move"); }
  bool isParallelLineActive() const;
  bool isPerpendicularLineActive() const;
  bool isAnyToolActive() const;
  bool isIntersectionActive() const;
  bool isCircleActive() const;
  bool isInitialized() const;
  void displayMessage(const std::string &message);
  void clearMessage();
  void deactivateAllTools();
  void setMessage(const std::string &message);
  // COLOR METHODS:
  sf::Color getCurrentColor() const { return m_currentColor; }
  void setCurrentColor(sf::Color color) { m_currentColor = color; }
  void toggleColorPicker();
  std::unique_ptr<ColorPicker> &getColorPicker() { return m_colorPicker; }
  ContextMenu& getContextMenu() { return m_contextMenu; }
  
 private:
  std::vector<Button> buttons;
  sf::View guiView;  // View for positioning and drawing GUI elements like buttons

  mutable sf::Text guiMessage;  // Make mutable to allow setPosition in const draw
  sf::Font messageFont;         // Font for messages
  sf::Clock messageTimer;
  bool messageActive = false;
  bool m_isInitialized;  // Flag to check if GUI is initialized
  bool m_fontLoaded;     // Flag to check if font is loaded
  std::unique_ptr<ColorPicker> m_colorPicker;
  ContextMenu m_contextMenu;
  sf::Color m_currentColor = sf::Color::Blue;

  // Utility function
  float distance(const sf::Vector2f &a, const sf::Vector2f &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  }
};
#endif  // GUI_H