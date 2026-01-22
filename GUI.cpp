#include "CharTraitsFix.h"  // Include very early
#include "GUI.h"

#include <iostream>  // For std::cerr
#include <string>    // Include very early for any string operations
#include <algorithm>  // For std::clamp

#include "Constants.h"       // For Constants::BUTTON_TEXT_SIZE, etc.
#include "GeometryEditor.h"  // Include the full definition of GeometryEditor
#include "LineToolMode.h"    // Include this to access LineToolMode enum and globals
#include "ObjectType.h"      // Include the definition for ObjectType enum

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
  text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
  text.setPosition(shape.getPosition().x + shape.getSize().x / 2.0f,
                   shape.getPosition().y + shape.getSize().y / 2.0f);
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

// --- GUI Implementation ---
GUI::GUI() : messageActive(false), m_isInitialized(false), m_fontLoaded(false) {
  // Load font first to ensure it's available for text
  Button::loadFont();

  // Initialize message font for GUI messages
  if (!messageFont.loadFromFile(Constants::DEFAULT_FONT_PATH)) {
    std::cerr << "Error loading GUI message font." << std::endl;
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

  // Grid toggle
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Grid",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
  buttons.back().setActive(true);  // Grid on by default
  currentX += buttonWidth + spacing;
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Color",
                       Constants::BUTTON_DEFAULT_COLOR, Constants::BUTTON_ACTIVE_COLOR,
                       Constants::BUTTON_HOVER_COLOR);
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
}
bool GUI::isInitialized() const { return m_isInitialized; }

void GUI::setMessage(const std::string &message) {
  if (m_fontLoaded) {
    guiMessage.setString(message);
  } else {
    std::cerr << "GUI::setMessage: Font not loaded, cannot set message." << std::endl;
  }
}
void GUI::draw(sf::RenderWindow &window, const sf::View &drawingView,
               GeometryEditor &editor) const {
  (void)drawingView;        // Mark as unused if no longer needed for preview circle
  window.setView(guiView);  // Set view for drawing GUI elements

  for (const auto &button : buttons) {
    button.draw(window);
    if (button.getLabel() == "Color") {
      sf::RectangleShape colorPreview;
      colorPreview.setSize(sf::Vector2f(20, 20));
      colorPreview.setPosition(button.getGlobalBounds().left + 5, button.getGlobalBounds().top + 5);
      colorPreview.setFillColor(m_currentColor);
      colorPreview.setOutlineThickness(1);
      colorPreview.setOutlineColor(sf::Color::Black);
      window.draw(colorPreview);
    }
  }
  if (m_colorPicker && m_colorPicker->isOpen()) {
    m_colorPicker->draw(window);
  }
  // ContextMenu::draw is non-const; cast away const to call it from this const method
  const_cast<ContextMenu&>(m_contextMenu).draw(window);
  if (messageActive) {
    // Position message (e.g., bottom center of guiView)
    sf::FloatRect messageBounds = guiMessage.getLocalBounds();
    guiMessage.setPosition(guiView.getSize().x / 2.f - messageBounds.width / 2.f,
                           guiView.getSize().y - messageBounds.height - 20.f);
    window.draw(guiMessage);
  }

  // Draw preview circle in the drawingView (world space)
  // This is now handled by GeometryEditor::render()
  // if (isCreatingCircle && previewCircle) { // REMOVE THIS BLOCK
  //   window.setView(drawingView); // Switch to world view for preview
  //   previewCircle->draw(window);
  //   // window.setView(guiView); // Switch back if more GUI elements followed,
  //   or
  //   // let editor manage views
  // }
  //   // let editor manage views
  // }
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

bool GUI::handleEvent(sf::RenderWindow &window, const sf::Event &event, GeometryEditor &editor) {
  sf::Vector2f currentMousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window), guiView);
  if (m_contextMenu.handleEvent(event, currentMousePos, editor)) return true;

  if (event.type == sf::Event::MouseMoved) {
    // Convert mouse position to GUI view coordinates once
    sf::Vector2f mousePosView =
        window.mapPixelToCoords(sf::Vector2i(event.mouseMove.x, event.mouseMove.y), guiView);

    if (m_colorPicker && m_colorPicker->isOpen()) {
      if (m_colorPicker->handleMouseMove(event, mousePosView)) {
        m_currentColor = m_colorPicker->getCurrentColorWithAlpha();
        editor.setCurrentColor(m_currentColor);
        if (editor.fillTarget) {
          editor.fillTarget->setColor(m_currentColor);
        }
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
      window.mapPixelToCoords(sf::Mouse::getPosition(window), guiView);

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
          editor.changeSelectedObjectColor(m_currentColor);
          return true;
        }
      }
    } else {
      // Not in application mode, just handle normal color picker selection
      if (m_colorPicker->handleEvent(event, mousePosView)) {
        m_currentColor = m_colorPicker->getCurrentColorWithAlpha();
        editor.setCurrentColor(m_currentColor);
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
      if (button.isMouseOver(window, guiView)) {
        std::cout << "Button clicked: " << button.getLabel() << std::endl;

        if (button.getLabel() == "Parallel") {
          editor.setCurrentTool(ObjectType::ParallelLine);

          // Reset the parallel line creation state
          resetParallelLineState();  // Reset state to ensure clean start
          std::cout << "Parallel line tool activated. Click on a reference "
                       "line first."
                    << std::endl;
          return true;
        } else if (button.getLabel() == "Perp") {
          editor.setCurrentTool(ObjectType::PerpendicularLine);

          // Reset the perpendicular line creation state
          resetPerpLineState();  // Reset state to ensure clean start
          std::cout << "Perpendicular line tool activated. Click on a "
                       "reference line first."
                    << std::endl;
          return true;
        } else if (button.getLabel() == "Point") {
          editor.setCurrentTool(ObjectType::Point);
          return true;
        } else if (button.getLabel() == "Line") {
          editor.setCurrentTool(ObjectType::Line);
          return true;
        } else if (button.getLabel() == "Segment") {
          editor.setCurrentTool(ObjectType::LineSegment);
          return true;
        } else if (button.getLabel() == "ObjPoint") {
          editor.setCurrentTool(ObjectType::ObjectPoint);
          return true;
        } else if (button.getLabel() == "Circle") {
          editor.setCurrentTool(ObjectType::Circle);
          return true;
        } else if (button.getLabel() == "Rect") {
          editor.setCurrentTool(ObjectType::Rectangle);
          return true;
        } else if (button.getLabel() == "RotRect") {
          editor.setCurrentTool(ObjectType::RectangleRotatable);
          return true;
        } else if (button.getLabel() == "Polygon") {
          editor.setCurrentTool(ObjectType::Polygon);
          return true;
        } else if (button.getLabel() == "RegPoly") {
          editor.setCurrentTool(ObjectType::RegularPolygon);
          return true;
        } else if (button.getLabel() == "Triangle") {
          editor.setCurrentTool(ObjectType::Triangle);
          return true;
        } else if (button.getLabel() == "Move") {
          editor.setCurrentTool(ObjectType::None);
          return true;
        } else if (button.getLabel() == "Grid") {
          editor.toggleGrid();
          button.setActive(!button.isActive());
          return true;
        } else if (button.getLabel() == "Intersect") {
          // Switch to intersection mode to allow targeted intersection creation
          editor.setCurrentTool(ObjectType::Intersection);
          return true;
        } else if (button.getLabel() == "Color") {
          std::cout << "Color button clicked!" << std::endl;
          if (m_colorPicker) {
            m_colorPicker->setOpen(!m_colorPicker->isOpen());
            std::cout << "Color picker " << (m_colorPicker->isOpen() ? "opened" : "closed")
                      << std::endl;
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
    guiMessage.setPosition(guiView.getSize().x / 2.f - messageBounds.width / 2.f,
                           guiView.getSize().y - messageBounds.height - 20.f);
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
  guiMessage.setOrigin(textRect.left + textRect.width / 2.0f,
                       textRect.top + textRect.height / 2.0f);
  guiMessage.setPosition(
      guiView.getSize().x / 2.f,
      guiView.getSize().y - (Constants::BUTTON_SIZE.y / 2.f) - 5.f);  // Adjust y-offset as needed

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