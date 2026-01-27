#include "CharTraitsFix.h"  // Include very early
#include "GUI.h"
#include "FileDialogs.h"

#include <iostream>  // For std::cerr
#include <string>    // Include very early for any string operations
#include <algorithm>  // For std::clamp

#include "Constants.h"       // For Constants::BUTTON_TEXT_SIZE, etc.
#include "GeometryEditor.h"  // Include the full definition of GeometryEditor
#include "LineToolMode.h"    // Include this to access LineToolMode enum and globals
#include "ObjectType.h"      // Include the definition for ObjectType enum
#include "Angle.h"

bool ColorPicker::handleEvent(const sf::Event &event, const sf::Vector2f &mousePos) {
  if (!m_isOpen) return false;

  if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
    for (int i = 0; i < 16; i++) {
      auto bounds = m_colorPalette[i].getGlobalBounds();
      float tolerance = 2.0f;
      if (mousePos.x >= bounds.left - tolerance && mousePos.x <= bounds.left + bounds.width + tolerance &&
          mousePos.y >= bounds.top - tolerance && mousePos.y <= bounds.top + bounds.height + tolerance) {
        m_currentColor = m_colorPalette[i].getFillColor();
        m_colorPreview.setFillColor(getCurrentColorWithAlpha());
        m_isInApplicationMode = true;
        return true;
      }
    }

    auto track = m_alphaTrack.getGlobalBounds();
    if (track.contains(mousePos)) {
      float ratio = (mousePos.x - track.left) / track.width;
      setAlpha(std::clamp(ratio * 255.0f, 0.0f, 255.0f));
      m_draggingAlpha = true;
      return true;
    }
  }

  if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
    m_draggingAlpha = false;
  }

  return false;
}

bool ColorPicker::handleMouseMove(const sf::Event &event, const sf::Vector2f &mousePos) {
  (void)event;
  if (!m_isOpen || !m_draggingAlpha) return false;
  auto track = m_alphaTrack.getGlobalBounds();
  float clampedX = std::clamp(mousePos.x, track.left, track.left + track.width);
  float ratio = (clampedX - track.left) / track.width;
  setAlpha(std::clamp(ratio * 255.0f, 0.0f, 255.0f));
  return true;
}

bool ColorPicker::isMouseOver(const sf::Vector2f &mousePos) const {
  if (!m_isOpen) return false;
  for (int i = 0; i < 16; i++) {
    if (m_colorPalette[i].getGlobalBounds().contains(mousePos)) return true;
  }
  if (m_colorPreview.getGlobalBounds().contains(mousePos)) return true;
  if (m_alphaTrack.getGlobalBounds().contains(mousePos)) return true;
  if (m_alphaKnob.getGlobalBounds().contains(mousePos)) return true;
  return false;
}

void ColorPicker::setAlpha(float alpha) {
  m_alpha = std::clamp(alpha, 0.0f, 255.0f);
  auto track = m_alphaTrack.getGlobalBounds();
  float ratio = m_alpha / 255.0f;
  m_alphaKnob.setPosition(track.left + ratio * track.width, track.top + track.height / 2.0f);
  m_colorPreview.setFillColor(getCurrentColorWithAlpha());
}

// Static member initialization
sf::Font Button::font;
bool Button::fontLoaded = false;

// Button Constructor
Button::Button(sf::Vector2f position, sf::Vector2f size, std::string label, sf::Color defaultColor,
               sf::Color activeColor, sf::Color hoverColor)
    : m_label(label),
      m_active(false),
      m_hovered(false),
      m_defaultColor(defaultColor),
      m_activeColor(activeColor),
      m_hoverColor(hoverColor) {
  shape.setPosition(position);
  shape.setSize(size);

  // Ensure font is loaded (typically called once at GUI initialization or first
  // button creation)
  if (!Button::getFontLoaded()) {
    Button::loadFont();  // Static method to load the font
  }

  // It's safer to check fontLoaded before every use of Button::getFont()
  // or ensure loadFont() is absolutely called first.
  if (Button::getFontLoaded()) {
    text.setFont(Button::getFont());
  } else {
    // Fallback: Don't set font, or use a system default if possible, or log
    // error. This state might lead to issues if text is drawn.
    std::cerr << "Button '" << m_label << "': Font not available for text." << std::endl;
  }

  text.setString(m_label);
  // CRUCIAL: Set character size to a fixed pixel height.
  // Do NOT use setScale for text size.
  text.setCharacterSize(Constants::BUTTON_TEXT_SIZE);  // e.g., 12, 14, or whatever pixel size
                                                       // you want
  text.setFillColor(Constants::BUTTON_TEXT_COLOR);
  centerText();
  updateVisualState();  // Set initial color
}

void Button::centerText() {
  if (!Button::getFontLoaded()) return;  // Don't try to center if font isn't there

  sf::FloatRect textRect = text.getLocalBounds();
  text.setOrigin(std::round(textRect.left + textRect.width / 2.0f), 
                 std::round(textRect.top + textRect.height / 2.0f));
  text.setPosition(std::round(shape.getPosition().x + shape.getSize().x / 2.0f),
                   std::round(shape.getPosition().y + shape.getSize().y / 2.0f));
}

void Button::setLabel(const std::string &label) {
  m_label = label;
  text.setString(m_label);
  centerText();
}

// Add missing Button method definitions here:
std::string Button::getLabel() const { return m_label; }

bool Button::isActive() const { return m_active; }

void Button::setActive(bool active) {
  if (m_active != active) {
    m_active = active;
    updateVisualState();
  }
}

bool Button::isHovered() const { return m_hovered; }

void Button::setHovered(bool hover) {
  if (m_hovered != hover) {
    m_hovered = hover;
    updateVisualState();
  }
}

void Button::updateVisualState() {
  if (m_active) {
    shape.setFillColor(m_activeColor);
  } else if (m_hovered) {
    shape.setFillColor(m_hoverColor);
  } else {
    shape.setFillColor(m_defaultColor);
  }
}

void Button::draw(sf::RenderWindow &window) const {
  window.draw(shape);
  if (Button::getFontLoaded() && !text.getString().isEmpty()) {
    window.draw(text);
  }
}

bool Button::isMouseOver(const sf::RenderWindow &window, const sf::View &view) const {
  sf::Vector2i mousePixelPos = sf::Mouse::getPosition(window);
  sf::Vector2f mouseViewPos = window.mapPixelToCoords(mousePixelPos, view);
  return shape.getGlobalBounds().contains(mouseViewPos);
}

void Button::setPosition(sf::Vector2f pos) {
  shape.setPosition(pos);
  centerText();
}

void Button::setSize(sf::Vector2f size) {
  shape.setSize(size);
  centerText();
}
void Button::updateFontSize() {
  text.setCharacterSize(Constants::BUTTON_TEXT_SIZE);
  centerText();
}

// --- GUI Implementation ---
GUI::GUI() : messageActive(false), m_isInitialized(false), m_fontLoaded(false) {
  // Load font first to ensure it's available for text
  Button::loadFont();

  // Initialize message font for GUI messages
  if (!messageFont.loadFromFile(Constants::DEFAULT_FONT_PATH)) {
    std::cerr << "Error loading GUI message font." << std::endl;
  } else {
    m_fontLoaded = true;
  }

  guiMessage.setFont(messageFont);
  guiMessage.setCharacterSize(Constants::GUI_MESSAGE_FONT_SIZE);
  guiMessage.setFillColor(Constants::GUI_MESSAGE_COLOR);

  // Fix: Use sf::Vector2f instead of just a float for position
  guiMessage.setPosition(
      sf::Vector2f(Constants::GUI_MESSAGE_POSITION, Constants::GUI_MESSAGE_POSITION));

  // Initialize GUI view
  guiView.setCenter(Constants::WINDOW_WIDTH / 2.0f, Constants::WINDOW_HEIGHT / 2.0f);
  guiView.setSize(static_cast<float>(Constants::WINDOW_WIDTH),
                  static_cast<float>(Constants::WINDOW_HEIGHT));

  // Define spacing and position variables once
  float spacing = 5.0f;
  float buttonWidth = Constants::BUTTON_SIZE.x;
  float currentX = 0.0f;
  float currentY = 0.0f;

  // Create all tool buttons in a single sequence with proper spacing

  // Move tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Move",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Point tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Point",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Line tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Line",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Segment tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Segment",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Circle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Circle",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Parallel line tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Parallel",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Perpendicular line tool - use "Perp" for the label to match
  // isPerpendicularLineActive()
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Perp",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Perpendicular Bisector tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "PerpBis",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Angle Bisector tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "AngBis",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Tangent tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Tangent",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // ObjPoint tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "ObjPoint",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Rectangle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Rect",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Rotatable Rectangle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "RotRect",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Polygon tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Polygon",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Regular Polygon tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "RegPoly",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Triangle tool (3-sided regular polygon)
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Triangle",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Intersection tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Intersect",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Angle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Angle",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Hide tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Hide",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Detach tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Detach",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Grid toggle
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Grid",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  buttons.back().setActive(true);  // Grid on by default
  currentX += buttonWidth + spacing;
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Color",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;
  
  // Save project button
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Save",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;
  
  // Load project button
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Load",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;
  
  // Export SVG button
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "SVG",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;
  
  m_colorPicker = std::make_unique<ColorPicker>(sf::Vector2f(10, 200));
  
  // Initialize Context Menu
  m_contextMenu.addItem("Change Color", [](GeometryEditor& ed) {
      // Toggle color picker to allow choosing new color
       ed.getGUI().toggleColorPicker();
  });
  
  m_contextMenu.addItem("Delete Object", [](GeometryEditor& ed) {
      ed.deleteSelected();
  });
  
  // Future transformation placeholders
  m_contextMenu.addItem("Rotate Object", [](GeometryEditor&) {
     std::cout << "Rotation tool coming soon..." << std::endl;
  });
  m_contextMenu.addItem("Scale Object", [](GeometryEditor&) {
     std::cout << "Scaling tool coming soon..." << std::endl;
  });

  // Angle inspector controls
  m_angleReflexBox.setSize(sf::Vector2f(14.f, 14.f));
  m_angleReflexBox.setOutlineThickness(1.f);
  m_angleReflexBox.setOutlineColor(sf::Color::Black);
  m_angleReflexBox.setFillColor(sf::Color::Transparent);

  if (m_fontLoaded) {
    m_angleReflexLabel.setFont(messageFont);
  } else if (Button::getFontLoaded()) {
    m_angleReflexLabel.setFont(Button::getFont());
  }
  m_angleReflexLabel.setCharacterSize(Constants::BUTTON_TEXT_SIZE);
  m_angleReflexLabel.setFillColor(sf::Color::White);
  m_angleReflexLabel.setString("Show Reflex Angle (0-360\xC2\xB0)");
  
  // Mark GUI as fully initialized
  m_isInitialized = true;
}
bool GUI::isInitialized() const { return m_isInitialized; }

void GUI::setMessage(const std::string &message) {
  if (m_fontLoaded) {
    guiMessage.setString(message);
    messageActive = true;
    messageTimer.restart();
    std::cout << "GUI::setMessage: " << message << std::endl;
  } else {
    std::cerr << "GUI::setMessage: Font not loaded, cannot set message." << std::endl;
  }
}
void GUI::draw(sf::RenderWindow &window, const sf::View &drawingView,
               GeometryEditor &editor) const {
  (void)drawingView;          // Mark as unused
  window.setView(window.getDefaultView());  // Use Default View (Screen Space)

  // Dynamic left sidebar layout: scale button dimensions with font size
  const float scaleBase = static_cast<float>(Constants::BUTTON_TEXT_SIZE);
  const float buttonHeight = std::round(scaleBase * 1.8f);
  const float buttonWidth = std::round(std::max(80.0f, scaleBase * 2.5f));
  const float padding = std::round(std::max(4.0f, scaleBase * 0.4f));
  const float sidebarWidth = std::round(buttonWidth + padding * 2.0f);
  float currentY = padding;
  for (auto &button : const_cast<std::vector<Button>&>(buttons)) {
    // Snap button dimensions/position to integers
    button.setSize(sf::Vector2f(std::round(sidebarWidth - 2 * padding), std::round(buttonHeight)));
    button.setPosition(sf::Vector2f(std::round(padding), std::round(currentY)));
    currentY += buttonHeight + padding;
  }
  // After laying out buttons, determine where to place the color picker
  float colorPickerX = padding;
  float colorPickerY = currentY + padding * 2.0f; // Add extra space below last button
  if (m_colorPicker) {
    m_colorPicker->setPosition(sf::Vector2f(colorPickerX, colorPickerY));
  }

  for (const auto &button : buttons) {
    button.draw(window);
    if (button.getLabel() == "Color") {
      const float iconSize = std::round(std::max(20.0f, scaleBase * 1.2f));
      sf::RectangleShape colorPreview;
      colorPreview.setSize(sf::Vector2f(iconSize, iconSize));
      float iconX = button.getGlobalBounds().left + std::round(padding * 0.5f);
      float iconY = button.getGlobalBounds().top + std::round((buttonHeight - iconSize) * 0.5f);
      colorPreview.setPosition(iconX, iconY);
      colorPreview.setFillColor(m_currentColor);
      colorPreview.setOutlineThickness(1);
      colorPreview.setOutlineColor(sf::Color::Black);
      window.draw(colorPreview);
    }
  }
  if (m_colorPicker && m_colorPicker->isOpen()) {
    m_colorPicker->draw(window, Button::getFont());

    if (m_fontLoaded) {
      sf::Text paletteLabel;
      paletteLabel.setFont(messageFont);
      paletteLabel.setCharacterSize(Constants::BUTTON_TEXT_SIZE);
      paletteLabel.setFillColor(sf::Color::White);
      paletteLabel.setString("Selected Object Color");
      // Place label just above the color picker
      paletteLabel.setPosition(colorPickerX, colorPickerY - Constants::BUTTON_TEXT_SIZE - 4.0f);
      window.draw(paletteLabel);
    }
  }

  if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Angle) {
    auto *angle = dynamic_cast<Angle *>(editor.selectedObject);
    if (angle) {
      sf::Vector2f basePos(10.f, 60.f);
      sf::RectangleShape box = m_angleReflexBox;
      sf::Text label = m_angleReflexLabel;

      box.setPosition(basePos);
      box.setFillColor(angle->isReflex() ? sf::Color(100, 200, 100) : sf::Color::Transparent);
      label.setPosition(basePos.x + 20.f, basePos.y - 2.f);

      window.draw(box);
      window.draw(label);
    }
  }
  // ContextMenu::draw is non-const; cast away const to call it from this const method
  const_cast<ContextMenu&>(m_contextMenu).draw(window);
  if (messageActive) {
    // Position message (e.g., bottom center of window)
    sf::FloatRect messageBounds = guiMessage.getLocalBounds();
    sf::Vector2u winSize = window.getSize();
    guiMessage.setPosition(std::round(winSize.x / 2.f - messageBounds.width / 2.f),
                           std::round(winSize.y - messageBounds.height - 20.f));
    window.draw(guiMessage);
  }

  // Render Tool Hint
  if (m_fontLoaded) {
      sf::Text hintText;
      hintText.setFont(messageFont);
      hintText.setCharacterSize(Constants::GUI_MESSAGE_TEXT_SIZE);
      hintText.setFillColor(sf::Color(200, 200, 200)); // Light grey
      hintText.setString(editor.getToolHint());
      
      // Position at bottom center, above message if active
      sf::FloatRect bounds = hintText.getLocalBounds();
      sf::Vector2u winSize = window.getSize();
      float yPos = winSize.y - bounds.height - 10.f;
      if (messageActive) yPos -= 30.f; // Move up if message is significant
      
      hintText.setPosition(std::round(winSize.x / 2.f - bounds.width / 2.f), std::round(yPos));
      window.draw(hintText);
  }
  //   // let editor manage views
  // }
}

void GUI::updateFontSizes() {
  for (auto &button : buttons) {
    button.updateFontSize();
  }
  guiMessage.setCharacterSize(Constants::GUI_MESSAGE_TEXT_SIZE);
  m_angleReflexLabel.setCharacterSize(Constants::BUTTON_TEXT_SIZE);
}

void GUI::toggleColorPicker() {
  if (m_colorPicker) {
    m_colorPicker->setOpen(!m_colorPicker->isOpen());
  }
}

void GUI::toggleButton(const std::string &buttonName, bool state) {
  for (auto &button : buttons) {
    if (button.getLabel() == buttonName) {
      button.setActive(state);
      return;
    }
  }
}

bool GUI::isButtonActive(const std::string &label) const {
  for (const auto &button : buttons) {
    if (button.getLabel() == label) {
      return button.isActive();
    }
  }
  return false;
}

bool GUI::isMouseOverPalette(const sf::Vector2f &guiPos) const {
  if (!m_colorPicker || !m_colorPicker->isOpen()) return false;
  return m_colorPicker->isMouseOver(guiPos);
}

bool GUI::handleEvent(sf::RenderWindow &window, const sf::Event &event, GeometryEditor &editor) {
  // Use DefaultView for UI Hit Testing to match Screen Space rendering
  sf::Vector2f currentMousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window), window.getDefaultView());
  if (m_contextMenu.handleEvent(event, currentMousePos, editor)) return true;

  // Ensure sidebar layout matches draw() before hit-tests
  {
    const float scaleBase = static_cast<float>(Constants::BUTTON_TEXT_SIZE);
    const float buttonHeight = std::round(scaleBase * 1.8f);
    const float buttonWidth = std::round(std::max(80.0f, scaleBase * 2.5f));
    const float padding = std::round(std::max(4.0f, scaleBase * 0.4f));
    const float sidebarWidth = std::round(buttonWidth + padding * 2.0f);
    float currentY = padding;
    for (auto &button : buttons) {
      button.setSize(sf::Vector2f(std::round(sidebarWidth - 2 * padding), std::round(buttonHeight)));
      button.setPosition(sf::Vector2f(std::round(padding), std::round(currentY)));
      currentY += buttonHeight + padding;
    }
    if (m_colorPicker) {
      float colorPickerX = padding;
      float colorPickerY = currentY + padding * 2.0f;
      m_colorPicker->setPosition(sf::Vector2f(colorPickerX, colorPickerY));
    }
  }

  auto applyColorToSelection = [&](const sf::Color &color) -> bool {
    bool applied = false;
    if (!editor.selectedObjects.empty()) {
      for (auto *obj : editor.selectedObjects) {
        if (obj) {
          obj->setColor(color);
          applied = true;
        }
      }
    } else if (editor.selectedObject) {
      editor.selectedObject->setColor(color);
      applied = true;
    }
    return applied;
  };

  if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
    if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Angle) {
      auto *angle = dynamic_cast<Angle *>(editor.selectedObject);
      if (angle) {
        sf::Vector2f basePos(10.f, 60.f);
        m_angleReflexBox.setPosition(basePos);
        if (m_angleReflexBox.getGlobalBounds().contains(currentMousePos)) {
          angle->setReflex(!angle->isReflex());
          return true;
        }
      }
    }
  }

  if (event.type == sf::Event::MouseMoved) {
    // Convert mouse position to GUI view (Screen Space) coordinates once
    sf::Vector2f mousePosView =
        window.mapPixelToCoords(sf::Vector2i(event.mouseMove.x, event.mouseMove.y), window.getDefaultView());

    if (m_colorPicker && m_colorPicker->isOpen()) {
      if (m_colorPicker->handleMouseMove(event, mousePosView)) {
        m_currentColor = m_colorPicker->getCurrentColorWithAlpha();
        editor.setCurrentColor(m_currentColor);
        if (editor.fillTarget) {
          editor.fillTarget->setColor(m_currentColor);
        }
        applyColorToSelection(m_currentColor);
        return true;
      }
    }

    for (auto &button : buttons) {
      // Assuming Button::isMouseOver uses the window and the GUI view,
      // or takes the mousePosView directly.
      // If Button::isMouseOver is simple (e.g. getGlobalBounds().contains()),
      // it might need adjustment if the button's coordinate system isn't
      // directly guiView. For this example, we'll assume Button::isMouseOver
      // correctly uses guiView.
      if (button.getGlobalBounds().contains(mousePosView)) {  // More direct check
        button.setHovered(true);
      } else {
        button.setHovered(false);
      }
    }
    // Mouse move events over GUI buttons usually don't "consume" the event
    // fully, unless dragging a GUI element. So, no 'return true' here
    // typically.
  }
  
  // Handle color picker - but allow clicks outside the picker in application mode
  if (m_colorPicker && m_colorPicker->isOpen()) {
    sf::Vector2f mousePosView =
      window.mapPixelToCoords(sf::Mouse::getPosition(window), window.getDefaultView());

    // If in application mode, check if click is actually on the color picker
    if (m_colorPicker->isInApplicationMode()) {
      // Check if this click is on a color square
      bool clickedColorSquare = false;
      auto palette = m_colorPicker->getColorPalette();
      for (int i = 0; i < 16; i++) {
        auto bounds = palette[i].getGlobalBounds();
        float tolerance = 2.0f;
        if (mousePosView.x >= bounds.left - tolerance && 
            mousePosView.x <= bounds.left + bounds.width + tolerance &&
            mousePosView.y >= bounds.top - tolerance && 
            mousePosView.y <= bounds.top + bounds.height + tolerance) {
          clickedColorSquare = true;
          break;
        }
      }
      
      // If clicked on a color square, handle it
      if (clickedColorSquare && m_colorPicker->handleEvent(event, mousePosView)) {
        m_currentColor = m_colorPicker->getCurrentColorWithAlpha();
        editor.setCurrentColor(m_currentColor);
        if (editor.fillTarget) {
          editor.fillTarget->setColor(m_currentColor);
        }
        applyColorToSelection(m_currentColor);
        return true;
      }
      
      // Otherwise, click is on an object - apply the color
      if (event.type == sf::Event::MouseButtonPressed || event.type == sf::Event::MouseButtonReleased) {
        sf::Vector2i pixelPos(event.mouseButton.x, event.mouseButton.y);
        sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos, editor.drawingView);
        
        GeometricObject *obj = editor.lookForObjectAt(worldPos, 5.0f);
        if (obj) {
          // Cancel any active dragging to prevent interference
          editor.dragMode = DragMode::None;
          editor.selectedObject = obj;
          editor.selectedObjects.clear();
          editor.selectedObjects.push_back(obj);
          applyColorToSelection(m_currentColor);
          return true;
        }
      }
    } else {
      // Not in application mode, just handle normal color picker selection
      if (m_colorPicker->handleEvent(event, mousePosView)) {
        // Enforce selection before applying
        if (!editor.selectedObjects.empty() || editor.selectedObject) {
             m_currentColor = m_colorPicker->getCurrentColorWithAlpha();
             editor.setCurrentColor(m_currentColor);
             if (applyColorToSelection(m_currentColor)) {
               editor.setGUIMessage("Updated Color");
             }
        } else {
             // Revert picker visual maybe? Or just warn.
             editor.setGUIMessage("Select an object first to change color");
             // m_colorPicker->setCurrentColor(editor.getCurrentColor()); // Revert visual? Optional.
        }
        
        if (editor.fillTarget) {
          editor.fillTarget->setColor(m_currentColor);
        }
        
        return true;
      }
    }
  }

  // Inside the button click handler, when buttons are clicked:
  if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
    for (auto &button : buttons) {
      if (button.isMouseOver(window, window.getDefaultView())) {
        std::cout << "Button clicked: " << button.getLabel() << std::endl;

        if (button.getLabel() == "Parallel") {
          editor.setCurrentTool(ObjectType::ParallelLine);
          editor.setToolHint("Click a reference line, then place the parallel line.");

          // Reset the parallel line creation state
          resetParallelLineState();  // Reset state to ensure clean start
          std::cout << "Parallel line tool activated. Click on a reference "
                       "line first."
                    << std::endl;
          return true;
        } else if (button.getLabel() == "Perp") {
          editor.setCurrentTool(ObjectType::PerpendicularLine);
          editor.setToolHint("Click a reference line, then place the perpendicular line.");

          // Reset the perpendicular line creation state
          resetPerpLineState();  // Reset state to ensure clean start
          std::cout << "Perpendicular line tool activated. Click on a "
                       "reference line first."
                    << std::endl;
          return true;
        } else if (button.getLabel() == "PerpBis") {
          editor.setCurrentTool(ObjectType::PerpendicularBisector);
          editor.setToolHint("Pick two points or one segment to build its perpendicular bisector.");
          editor.isCreatingPerpendicularBisector = false;
          editor.perpBisectorP1 = nullptr;
          editor.perpBisectorP2 = nullptr;
          editor.perpBisectorLineRef = nullptr;
          return true;
        } else if (button.getLabel() == "AngBis") {
          editor.setCurrentTool(ObjectType::AngleBisector);
          editor.setToolHint("Select A, vertex B, C or two lines to bisect the angle.");
          editor.isCreatingAngleBisector = false;
          editor.angleBisectorPoints.clear();
          editor.angleBisectorLine1 = nullptr;
          editor.angleBisectorLine2 = nullptr;
          return true;
        } else if (button.getLabel() == "Tangent") {
          editor.setCurrentTool(ObjectType::TangentLine);
          editor.setToolHint("Select a point then a circle to draw tangents.");
          editor.isCreatingTangent = false;
          editor.tangentAnchorPoint = nullptr;
          editor.tangentCircle = nullptr;
          return true;
        } else if (button.getLabel() == "Point") {
          editor.setCurrentTool(ObjectType::Point);
          editor.setToolHint("Click to create a point.");
          return true;
        } else if (button.getLabel() == "Line") {
          editor.setCurrentTool(ObjectType::Line);
          editor.setToolHint("Click start point, then click end point (Ctrl to snap).");
          return true;
        } else if (button.getLabel() == "Segment") {
          editor.setCurrentTool(ObjectType::LineSegment);
          editor.setToolHint("Click start point, then click end point (Ctrl to snap).");
          return true;
        } else if (button.getLabel() == "ObjPoint") {
          editor.setCurrentTool(ObjectType::ObjectPoint);
          editor.setToolHint("Click on a line or circle to attach a point.");
          return true;
        } else if (button.getLabel() == "Circle") {
          editor.setCurrentTool(ObjectType::Circle);
          editor.setToolHint("Click center, then drag radius.");
          return true;
        } else if (button.getLabel() == "Rect") {
          editor.setCurrentTool(ObjectType::Rectangle);
          editor.setToolHint("Click corner, then drag to opposite corner.");
          return true;
        } else if (button.getLabel() == "RotRect") {
          editor.setCurrentTool(ObjectType::RectangleRotatable);
          editor.setToolHint("Click first corner, drag side, then drag width.");
          return true;
        } else if (button.getLabel() == "Polygon") {
          editor.setCurrentTool(ObjectType::Polygon);
          editor.setToolHint("Click vertices. Press Enter to finish.");
          return true;
        } else if (button.getLabel() == "RegPoly") {
          editor.setCurrentTool(ObjectType::RegularPolygon);
          editor.setToolHint("Click center, then drag for size/rotation.");
          return true;
        } else if (button.getLabel() == "Triangle") {
          editor.setCurrentTool(ObjectType::Triangle);
          editor.setToolHint("Click 3 points to create a triangle.");
          return true;
        } else if (button.getLabel() == "Move") {
          editor.setCurrentTool(ObjectType::None);
          editor.setToolHint("Click to select. Drag to move. Ctrl+Drag to snap.");
          return true;
        } else if (button.getLabel() == "Grid") {
          editor.toggleGrid();
          button.setActive(!button.isActive());
          return true;
        } else if (button.getLabel() == "Intersect") {
          // Switch to intersection mode to allow targeted intersection creation
          editor.setCurrentTool(ObjectType::Intersection);
          editor.setToolHint("Click two objects to find their intersection.");
          return true;
        } else if (button.getLabel() == "Angle") {
          editor.setCurrentTool(ObjectType::Angle);
          editor.setToolHint("Click Point A, Vertex, then Point B.");
          return true;
        } else if (button.getLabel() == "Hide") {
          editor.setCurrentTool(ObjectType::Hide);
          editor.setToolHint("Click objects to hide them. Click ghosts (force visible) to unhide.");
          return true;
        } else if (button.getLabel() == "Detach") {
          editor.setCurrentTool(ObjectType::Detach);
          editor.setToolHint("Click a shared line endpoint to detach it.");
          return true;
        } else if (button.getLabel() == "Color") {
          std::cout << "Color button clicked!" << std::endl;
          if (m_colorPicker) {
            m_colorPicker->setOpen(!m_colorPicker->isOpen());
            if (m_colorPicker->isOpen()) {
              sf::Color baseColor = editor.getCurrentColor();
              if (editor.selectedObject) {
                baseColor = editor.selectedObject->getColor();
              }
              m_colorPicker->setCurrentColor(baseColor);
              m_currentColor = m_colorPicker->getCurrentColorWithAlpha();
              editor.setCurrentColor(m_currentColor);
            }
            std::cout << "Color picker " << (m_colorPicker->isOpen() ? "opened" : "closed")
                      << std::endl;
          }
          return true;
        } else if (button.getLabel() == "Save") {
          std::cout << "Save button clicked!" << std::endl;
          std::string path = FileDialogs::SaveFile("JSON Project (*.json)\0*.json\0", "json");
          if (!path.empty()) {
            editor.saveProject(path);
            editor.setGUIMessage("Project Saved");
          }
          return true;
        } else if (button.getLabel() == "Load") {
          std::cout << "Load button clicked!" << std::endl;
          std::string path = FileDialogs::OpenFile("JSON Project (*.json)\0*.json\0");
          if (!path.empty()) {
            editor.loadProject(path);
            editor.setGUIMessage("Project Loaded");
          }
          return true;
        } else if (button.getLabel() == "SVG") {
          std::cout << "SVG Export button clicked!" << std::endl;
          std::string path =
              FileDialogs::SaveFile("Scalable Vector Graphics (*.svg)\0*.svg\0", "svg");
          if (!path.empty()) {
            editor.exportSVG(path);
            editor.setGUIMessage("Exported to SVG");
          }
          return true;
        }
        // ...existing code for other buttons...
      }
    }
  }

  // Remove or comment out the old logic block below, as tool activation
  // is now handled by editor.setCurrentTool when buttons are clicked.
  /*
  // Circle Creation Logic (using world coordinates via editor)
  if (isButtonActive("Circle")) {
    // ...
  } else if (isButtonActive("dragCircle")) {
    // ...
  } else if (isButtonActive("Point")) {
    // ...
  }
  */

  return false;  // Event not handled by GUI buttons (e.g., click on canvas)
}

void GUI::update(sf::Time deltaTime) {
  // Example update logic for GUI, like clearing timed messages
  (void)deltaTime;  // Mark as unused if deltaTime is not used yet
  clearMessage();
  // You could add other time-dependent GUI updates here,
  // like animations for buttons or fading messages.
}

void GUI::updateView(const sf::View &newGuiView) {
  guiView = newGuiView;
  // If button positions are relative to view center/size, update them here.
  // For now, assuming absolute positions within the guiView's coordinate
  // system. Example: Reposition message if view size changes
  if (messageActive) {
    sf::FloatRect messageBounds = guiMessage.getLocalBounds();
    guiMessage.setPosition(10.f, guiView.getSize().y - messageBounds.height - 10.f);
  }
}

bool GUI::isGridActive() const { return isButtonActive("Grid"); }
bool GUI::isPointActive() const { return isButtonActive("Point"); }
bool GUI::isObjPointActive() const { return isButtonActive("ObjPoint"); }
bool GUI::isLineActive() const { return isButtonActive("Line"); }
bool GUI::isLineSegmentActive() const { return isButtonActive("Segment"); }
bool GUI::isIntersectionActive() const { return isButtonActive("Intersect"); }
bool GUI::isCircleActive() const { return isButtonActive("Circle"); }
bool GUI::isParallelLineActive() const { return isButtonActive("Parallel"); }
bool GUI::isPerpendicularLineActive() const { return isButtonActive("Perp"); }

bool GUI::isAnyToolActive() const {
  return isPointActive() || isLineActive() || isLineSegmentActive() || isCircleActive() ||
         isIntersectionActive() || isObjPointActive() || isMoveActive();
}

void GUI::displayMessage(const std::string &message) {
  guiMessage.setString(message);
  // Position the message (example: bottom center of guiView)
  sf::FloatRect textRect = guiMessage.getLocalBounds();
  guiMessage.setOrigin(0, 0); // Reset origin
  guiMessage.setPosition(
      10.f,
      guiView.getSize().y - textRect.height - 10.f);

  messageActive = true;
  messageTimer.restart();
}

void GUI::clearMessage() {
  if (messageActive && messageTimer.getElapsedTime().asSeconds() > 2.0f) {  // Display for 2 seconds
    messageActive = false;
    guiMessage.setString("");
  }
}

void GUI::deactivateAllTools() {
  for (auto &button : buttons) {
    // Deactivate all tool buttons, except persistent ones like "Grid" or "Move"
    // if desired
    const std::string &label = button.getLabel();
    if (label != "Grid" /* && label != "Move" */) {  // Keep Grid active state
      button.setActive(false);
    }
  }
  // Resetting editor's creation state should be handled by the editor itself
  // when a tool is deactivated or Escape is pressed.
  // if (isCreatingCircle) { // REMOVE
  //   isCreatingCircle = false; // REMOVE
  //   previewCircle.reset(); // REMOVE
  // }
  // Reset other creation-specific states here if any
  std::cout << "GUI: All tools deactivated (except persistent)." << std::endl;
}



void GUI::updateLayout(float windowWidth) {
  if (buttons.empty()) return;
  
  float margin = 5.0f;
  float buttonWidth = Constants::BUTTON_SIZE.x;
  float buttonHeight = Constants::BUTTON_SIZE.y;

  float startX = margin;
  float currentX = startX;
  float currentY = margin;

  for (auto &button : buttons) {
    if (currentX + buttonWidth > windowWidth) {
      currentX = startX;
      currentY += buttonHeight + margin;
    }
    button.setSize(sf::Vector2f(buttonWidth, buttonHeight));
    button.setPosition(sf::Vector2f(currentX, currentY));
    currentX += buttonWidth + margin;
  }

  m_toolbarHeight = currentY + buttonHeight + margin;
}