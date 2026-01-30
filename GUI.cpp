#include "GUI.h"

#include <SFML/Graphics/Color.hpp>
#include <algorithm>  // For std::clamp
#include <iostream>   // For std::cerr
#include <string>     // Include very early for any string operations

#include "Angle.h"
#include "CharTraitsFix.h"  // Include very early
#include "Constants.h"      // For Constants::BUTTON_TEXT_SIZE, etc.
#include "FileDialogs.h"
#include "GeometryEditor.h"  // Include the full definition of GeometryEditor
#include "LineToolMode.h"    // Include this to access LineToolMode enum and globals
#include "ObjectType.h"      // Include the definition for ObjectType enum
#include <imgui.h>
#include <imgui-SFML.h>

float g_transformRotationDegrees = 45.0f;
float g_transformDilationFactor = 2.0f;
static bool s_showTransformTools = false;

static bool isTransformButtonLabel(const std::string& label) {
  return label == "RflLine" || label == "RflPt" || label == "Invert" ||
         label == "Rotate" || label == "Translate" || label == "Dilate" ||
         label == "Rot+" || label == "Rot-" || label == "Dil+" || label == "Dil-";
}

static ImVec4 toImVec4(const sf::Color& color) {
  return ImVec4(color.r / 255.0f, color.g / 255.0f, color.b / 255.0f, color.a / 255.0f);
}

static void drawOverlays(GeometryEditor& editor, const sf::Text& guiMessage, bool messageActive) {
  ImGuiIO& io = ImGui::GetIO();
  float padding = 15.0f;

  // Top-right: static keybinds (use semi-opaque grey background so white text is visible)
  ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - padding, padding + 20.0f), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.18f, 0.18f, 0.18f, 0.85f));
  ImGuiWindowFlags keyFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                              ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                              ImGuiWindowFlags_NoInputs;
  if (ImGui::Begin("KeysOverlay", nullptr, keyFlags)) {
    ImGui::TextColored(ImVec4(1,1,1,1), "Alt: Snap to Point");
    ImGui::TextColored(ImVec4(1,1,1,1), "Shift: Snap to Grid");
    ImGui::TextColored(ImVec4(1,1,1,1), "Ctrl: Multi-Select");
    ImGui::TextColored(ImVec4(1,1,1,1), "F+/-: Scale UI");
    ImGui::TextColored(ImVec4(1,1,1,1), "Del: Delete Object");
    ImGui::TextColored(ImVec4(1,1,1,1), "H: Toggle Labels");
    ImGui::TextColored(ImVec4(1,1,1,1), "Select point and click R to change its label");
  }
  ImGui::End();
  ImGui::PopStyleColor();

  // Top-middle: dynamic tool hint
  std::string msg;
  if (messageActive) {
    msg = guiMessage.getString().toAnsiString();
  }
  if (msg.empty()) {
    msg = editor.getToolHint();
  }
  if (!msg.empty()) {
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, padding + 20.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.5f, 0.5f, 0.5f, 0.9f));
    ImGuiWindowFlags hintFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                 ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                                 ImGuiWindowFlags_NoInputs;
    if (ImGui::Begin("HintOverlay", nullptr, hintFlags)) {
      ImGui::TextColored(toImVec4(Constants::TOOL_HINT_COLOR), "%s", msg.c_str());
    }
    ImGui::End();
    ImGui::PopStyleColor();
  }
}


bool ColorPicker::handleEvent(const sf::Event& event, const sf::Vector2f& mousePos) {
  if (!m_isOpen) return false;

  if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
    for (int i = 0; i < 16; i++) {
      auto bounds = m_colorPalette[i].getGlobalBounds();
      float tolerance = 2.0f;
      if (mousePos.x >= bounds.left - tolerance && mousePos.x <= bounds.left + bounds.width + tolerance && mousePos.y >= bounds.top - tolerance &&
          mousePos.y <= bounds.top + bounds.height + tolerance) {
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

bool ColorPicker::handleMouseMove(const sf::Event& event, const sf::Vector2f& mousePos) {
  (void)event;
  if (!m_isOpen || !m_draggingAlpha) return false;
  auto track = m_alphaTrack.getGlobalBounds();
  float clampedX = std::clamp(mousePos.x, track.left, track.left + track.width);
  float ratio = (clampedX - track.left) / track.width;
  setAlpha(std::clamp(ratio * 255.0f, 0.0f, 255.0f));
  return true;
}

bool ColorPicker::isMouseOver(const sf::Vector2f& mousePos) const {
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
Button::Button(sf::Vector2f position, sf::Vector2f size, std::string label, sf::Color defaultColor, sf::Color activeColor, sf::Color hoverColor)
    : m_label(label), m_active(false), m_hovered(false), m_defaultColor(defaultColor), m_activeColor(activeColor), m_hoverColor(hoverColor) {
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
  text.setOrigin(std::round(textRect.left + textRect.width / 2.0f), std::round(textRect.top + textRect.height / 2.0f));
  text.setPosition(std::round(shape.getPosition().x + shape.getSize().x / 2.0f), std::round(shape.getPosition().y + shape.getSize().y / 2.0f));
}

void Button::setLabel(const std::string& label) {
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

void Button::draw(sf::RenderWindow& window) const {
  window.draw(shape);
  if (Button::getFontLoaded() && !text.getString().isEmpty()) {
    window.draw(text);
  }
}

bool Button::isMouseOver(const sf::RenderWindow& window, const sf::View& view) const {
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
  guiMessage.setPosition(sf::Vector2f(Constants::GUI_MESSAGE_POSITION, Constants::GUI_MESSAGE_POSITION));

  // Initialize GUI view
  guiView.setCenter(Constants::WINDOW_WIDTH / 2.0f, Constants::WINDOW_HEIGHT / 2.0f);
  guiView.setSize(static_cast<float>(Constants::WINDOW_WIDTH), static_cast<float>(Constants::WINDOW_HEIGHT));

  // Define spacing and position variables once
  float spacing = 5.0f;
  float buttonWidth = Constants::BUTTON_SIZE.x;
  float currentX = 0.0f;
  float currentY = 0.0f;

  // Create all tool buttons in a single sequence with proper spacing

  // Move tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Move", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Point tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Point", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Line tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Line", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Segment tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Segment", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Circle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Circle", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Parallel line tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Parallel", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Perpendicular line tool - use "Perp" for the label to match
  // isPerpendicularLineActive()
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Perp", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Perpendicular Bisector tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "PerpBis", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Angle Bisector tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "AngBis", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Tangent tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Tangent", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // ObjPoint tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "ObjPoint", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Rectangle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Rect", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Rotatable Rectangle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "RotRect", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Polygon tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Polygon", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Regular Polygon tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "RegPoly", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Triangle tool (3-sided regular polygon)
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Triangle", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Intersection tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Intersect", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Midpoint tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Midpoint", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Compass tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Compass", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Angle tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Angle", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Transformations header (toggle)
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Transform", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Transformations sub-tools
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "RflLine", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "RflPt", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Invert", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Rotate", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Translate", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Dilate", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Rotation/Dilation inputs
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Rot+", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Rot-", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Dil+", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Dil-", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Hide tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Hide", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Detach tool
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Detach", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Grid toggle
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Grid", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  buttons.back().setActive(true);  // Grid on by default
  currentX += buttonWidth + spacing;

  // Axes toggle
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Axes", Constants::BUTTON_INACTIVE_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  buttons.back().setActive(true); // Axes visible by default
  currentX += buttonWidth + spacing;

  // Reset view button
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Reset", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Color", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Save project button
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Save", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Load project button
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "Load", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  // Export SVG button
  buttons.emplace_back(sf::Vector2f(currentX, currentY), Constants::BUTTON_SIZE, "SVG", Constants::BUTTON_DEFAULT_COLOR,
                       Constants::BUTTON_ACTIVE_COLOR, Constants::BUTTON_HOVER_COLOR);
  currentX += buttonWidth + spacing;

  m_colorPicker = std::make_unique<ColorPicker>(sf::Vector2f(10, 200));

  // Initialize Context Menu
  m_contextMenu.addItem("Change Color", [](GeometryEditor& ed) {
    // Toggle color picker to allow choosing new color
    ed.getGUI().toggleColorPicker();
  });

  m_contextMenu.addItem("Delete Object", [](GeometryEditor& ed) { ed.deleteSelected(); });

  // Future transformation placeholders
  m_contextMenu.addItem("Rotate Object", [](GeometryEditor&) { std::cout << "Rotation tool coming soon..." << std::endl; });
  m_contextMenu.addItem("Scale Object", [](GeometryEditor&) { std::cout << "Scaling tool coming soon..." << std::endl; });

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

void GUI::setMessage(const std::string& message) {
  if (m_fontLoaded) {
    guiMessage.setString(message);
    messageActive = true;
    messageTimer.restart();
    std::cout << "GUI::setMessage: " << message << std::endl;
  } else {
    std::cerr << "GUI::setMessage: Font not loaded, cannot set message." << std::endl;
  }
}

void GUI::updateThicknessSliderLayout(const sf::Vector2u& windowSize, float currentValue) const {
  const float minValue = 1.0f;
  const float maxValue = 10.0f;
  const float trackWidth = 220.0f;
  const float trackHeight = 6.0f;
  const float topPadding = 8.0f;

  // Snap to integer steps
  int snappedValue = static_cast<int>(std::round(std::clamp(currentValue, minValue, maxValue)));
  float clampedValue = static_cast<float>(snappedValue);
  float t = (clampedValue - minValue) / (maxValue - minValue);

  float centerX = static_cast<float>(windowSize.x) * 0.5f;
  float trackX = std::round(centerX - trackWidth * 0.5f);
  float trackY = std::round(topPadding);

  m_thicknessTrack.setSize(sf::Vector2f(trackWidth, trackHeight));
  m_thicknessTrack.setPosition(trackX, trackY);
  m_thicknessTrack.setFillColor(sf::Color(120, 120, 120, 180));
  m_thicknessTrack.setOutlineThickness(1.0f);
  m_thicknessTrack.setOutlineColor(sf::Color(0, 0, 0, 180));

  const float knobRadius = 7.0f;
  m_thicknessKnob.setRadius(knobRadius);
  m_thicknessKnob.setOrigin(knobRadius, knobRadius);
  float knobX = trackX + t * trackWidth;
  float knobY = trackY + trackHeight * 0.5f;
  m_thicknessKnob.setPosition(std::round(knobX), std::round(knobY));
  m_thicknessKnob.setFillColor(sf::Color(240, 240, 240));
  m_thicknessKnob.setOutlineThickness(1.0f);
  m_thicknessKnob.setOutlineColor(sf::Color::Black);

  // Draw label to the left of the slider, vertically centered
  if (m_fontLoaded) {
    m_thicknessLabel.setFont(messageFont);
  } else if (Button::getFontLoaded()) {
    m_thicknessLabel.setFont(Button::getFont());
  }
  m_thicknessLabel.setCharacterSize(std::max(12u, Constants::BUTTON_TEXT_SIZE - 6));
  m_thicknessLabel.setFillColor(sf::Color::White);
  m_thicknessLabel.setString("Line Thickness: " + std::to_string(snappedValue));
  // Calculate label size and position
  sf::FloatRect labelBounds = m_thicknessLabel.getLocalBounds();
  float labelX = trackX - labelBounds.width - 18.0f;  // 18px padding from slider
  float labelY = trackY + (trackHeight * 0.5f) - (labelBounds.height * 0.5f) - labelBounds.top;
  m_thicknessLabel.setPosition(labelX, labelY);
}

bool GUI::handleSliderInteraction(const sf::Vector2i& mousePos, GeometryEditor& editor) {
  updateThicknessSliderLayout(editor.window.getSize(), editor.currentThickness);

  sf::Vector2f mouse(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
  sf::FloatRect trackBounds = m_thicknessTrack.getGlobalBounds();
  sf::FloatRect knobBounds = m_thicknessKnob.getGlobalBounds();

  bool leftDown = sf::Mouse::isButtonPressed(sf::Mouse::Left);
  if (leftDown && (trackBounds.contains(mouse) || knobBounds.contains(mouse) || m_draggingThickness)) {
    m_draggingThickness = true;
    float t = (mouse.x - trackBounds.left) / trackBounds.width;
    t = std::clamp(t, 0.0f, 1.0f);
    // Snap to integer steps
    int snappedValue = static_cast<int>(std::round(1.0f + t * 9.0f));
    snappedValue = std::clamp(snappedValue, 1, 10);
    editor.currentThickness = static_cast<float>(snappedValue);
    if (editor.selectedObject &&
        (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
      editor.selectedObject->setThickness(editor.currentThickness);
    }
    return true;
  }

  if (!leftDown && m_draggingThickness) {
    m_draggingThickness = false;
    return true;
  }

  return false;
}
void GUI::draw(sf::RenderWindow& window, const sf::View& drawingView, GeometryEditor& editor) const {
  (void)drawingView;

  // Main Menu Bar
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Save As...")) {
        std::string path = FileDialogs::SaveFile("JSON Project (*.json)\0*.json\0", "json");
        if (!path.empty()) {
          editor.saveProject(path);
          editor.setGUIMessage("Project Saved");
        }
      }
      if (ImGui::MenuItem("Load...")) {
        std::string path = FileDialogs::OpenFile("JSON Project (*.json)\0*.json\0");
        if (!path.empty()) {
          editor.loadProject(path);
          editor.setGUIMessage("Project Loaded");
        }
      }
      if (ImGui::MenuItem("Export SVG")) {
        std::string path = FileDialogs::SaveFile("Scalable Vector Graphics (*.svg)\0*.svg\0", "svg");
        if (!path.empty()) {
          editor.exportSVG(path);
          editor.setGUIMessage("Exported to SVG");
        }
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Reset View")) {
        editor.resetView();
      }
      bool gridVisible = editor.isGridVisible();
      if (ImGui::MenuItem("Toggle Grid", nullptr, &gridVisible)) {
        editor.toggleGrid();
      }
      bool axesVisible = editor.areAxesVisible();
      if (ImGui::MenuItem("Toggle Axes", nullptr, &axesVisible)) {
        editor.toggleAxes();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  // Sidebar Tool Panel
  const float windowHeight = static_cast<float>(editor.window.getSize().y);
  
  // Set position always, but allow size to be changed by user
  ImGui::SetNextWindowPos(ImVec2(0, 18), ImGuiCond_Always); 
  ImGui::SetNextWindowSizeConstraints(ImVec2(150, windowHeight - 18), ImVec2(800, windowHeight - 18));
  ImGui::SetNextWindowSize(ImVec2(m_sidebarWidth, windowHeight - 18), ImGuiCond_FirstUseEver);

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;

  if (ImGui::Begin("Tools", nullptr, flags)) {
    m_sidebarWidth = ImGui::GetWindowWidth(); // Update tracked width
    
    ImGui::Text("Mode: %s", editor.getCurrentToolName().c_str());
    ImGui::Separator();

  // Tool Button Helper
  auto DrawToolButton = [&](const char* label, ObjectType toolType, const char* hint, std::function<void()> onActivate = nullptr) {
    bool isActive = (editor.getCurrentTool() == toolType);
    if (isActive) {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.2f, 0.8f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.3f, 0.9f, 1.0f));
    }
    if (ImGui::Button(label, ImVec2(-1, 0))) {
      editor.setCurrentTool(toolType);
      editor.clearSelection();
      if (hint && *hint) editor.setToolHint(hint);
      if (onActivate) onActivate();
    }
    if (isActive) ImGui::PopStyleColor(2);
  };

  // --- RESTORED VIEW & PROJECT SECTION ---
  if (ImGui::CollapsingHeader("View & Project", ImGuiTreeNodeFlags_DefaultOpen)) {
    bool grid = editor.isGridVisible();
    if (ImGui::Checkbox("Show Grid", &grid)) editor.toggleGrid();
    ImGui::SameLine();
    bool axes = editor.areAxesVisible();
    if (ImGui::Checkbox("Show Axes", &axes)) editor.toggleAxes();

    if (ImGui::Button("Reset View", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f, 0))) {
        editor.resetView();
    }
    ImGui::SameLine();
    if (ImGui::Button("Export SVG", ImVec2(-1, 0))) {
        std::string path = FileDialogs::SaveFile("Scalable Vector Graphics (*.svg)\0*.svg\0", "svg");
        if (!path.empty()) {
            editor.exportSVG(path);
            editor.setGUIMessage("Exported to SVG");
        }
    }
    
    if (ImGui::Button("Save As...", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f, 0))) {
        std::string path = FileDialogs::SaveFile("JSON Project (*.json)\0*.json\0", "json");
        if (!path.empty()) editor.saveProject(path);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load Project", ImVec2(-1, 0))) {
        std::string path = FileDialogs::OpenFile("JSON Project (*.json)\0*.json\0");
        if (!path.empty()) editor.loadProject(path);
    }
  }

  // --- ANGLE PROPERTIES (ONLY IF ANGLE SELECTED) ---
  if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Angle) {
    if (ImGui::CollapsingHeader("Angle Properties", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto angle = std::dynamic_pointer_cast<Angle>(editor.findSharedPtr(editor.selectedObject));
        if (angle) {
            bool reflex = angle->isReflex();
            if (ImGui::Checkbox("Show Reflex Angle (0-360\xC2\xB0)", &reflex)) {
                angle->setReflex(reflex);
            }
        }
    }
  }

  if (ImGui::CollapsingHeader("Construction", ImGuiTreeNodeFlags_DefaultOpen)) {
    DrawToolButton("Move/Select", ObjectType::None, "Click to select. Drag to move. Ctrl+Drag to snap.");
    DrawToolButton("Point", ObjectType::Point, "Click to create a point.");
    DrawToolButton("Line", ObjectType::Line, "Click start point, then click end point (Ctrl to snap).");
    DrawToolButton("Segment", ObjectType::LineSegment, "Click start point, then click end point (Ctrl to snap).");
    DrawToolButton("Circle", ObjectType::Circle, "Click center, then drag radius.");
    DrawToolButton("ObjPoint", ObjectType::ObjectPoint, "Click on a line or circle to attach a point.");
    DrawToolButton("Intersection", ObjectType::Intersection, "Click two objects to find their intersection.");
    DrawToolButton("Midpoint", ObjectType::Midpoint, "Select two points (or a line) to find the middle.");
    DrawToolButton("Compass", ObjectType::Compass, "Select segment/2 points for radius, then a center point.");
    DrawToolButton("Parallel", ObjectType::ParallelLine, "Click a reference line, then place the parallel line.", [](){ resetParallelLineState(); });
    DrawToolButton("Perpendicular", ObjectType::PerpendicularLine, "Click a reference line, then place the perpendicular line.", [](){ resetPerpLineState(); });
    DrawToolButton("Perp Bisector", ObjectType::PerpendicularBisector, "Pick two points or one segment.", [&](){
        editor.isCreatingPerpendicularBisector = false;
        editor.perpBisectorP1 = nullptr;
        editor.perpBisectorP2 = nullptr;
        editor.perpBisectorLineRef = nullptr;
    });
    DrawToolButton("Angle Bisector", ObjectType::AngleBisector, "Select A, vertex B, C or two lines.", [&](){
        editor.isCreatingAngleBisector = false;
        editor.angleBisectorPoints.clear();
        editor.angleBisectorLine1 = nullptr;
        editor.angleBisectorLine2 = nullptr;
    });
    DrawToolButton("Tangent", ObjectType::TangentLine, "Select a point then a circle.", [&](){
        editor.isCreatingTangent = false;
        editor.tangentAnchorPoint = nullptr;
        editor.tangentCircle = nullptr;
    });
    DrawToolButton("Detach", ObjectType::Detach, "Click a shared line endpoint to detach it.");
    DrawToolButton("Hide/Show", ObjectType::Hide, "Click objects to hide them. Click ghosts (force visible) to unhide.");
  }


  if (ImGui::CollapsingHeader("Shapes", ImGuiTreeNodeFlags_DefaultOpen)) {
    DrawToolButton("Rectangle", ObjectType::Rectangle, "Click corner, then drag to opposite corner.");
    DrawToolButton("Rotated Rect", ObjectType::RectangleRotatable, "Click first corner, drag side, then drag width.");
    DrawToolButton("Triangle", ObjectType::Triangle, "Click 3 points to create a triangle.");
    DrawToolButton("Polygon", ObjectType::Polygon, "Click vertices. Press Enter to finish.");
    DrawToolButton("Regular Polygon", ObjectType::RegularPolygon, "Click center, then drag for size/rotation.");
  }

  if (ImGui::CollapsingHeader("Measure", ImGuiTreeNodeFlags_DefaultOpen)) {
    DrawToolButton("Angle", ObjectType::Angle, "Click Point A, Vertex, then Point B.");
  }

  if (ImGui::CollapsingHeader("Transformations", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InputFloat("Rotation (deg)", &g_transformRotationDegrees, 1.0f, 5.0f, "%.1f");
    ImGui::InputFloat("Dilation factor", &g_transformDilationFactor, 0.1f, 0.5f, "%.2f");
    
    DrawToolButton("Reflect about Line", ObjectType::ReflectAboutLine, "Select source point, then a line.");
    DrawToolButton("Reflect about Point", ObjectType::ReflectAboutPoint, "Select source point, then a center point.");
    DrawToolButton("Invert (Circle)", ObjectType::ReflectAboutCircle, "Select source point, then a circle.");
    DrawToolButton("Rotate around Point", ObjectType::RotateAroundPoint, "Select source point, then pivot point.");
    DrawToolButton("Translate by Vector", ObjectType::TranslateByVector, "Select source point, then vector start and end points.");
    DrawToolButton("Dilate from Point", ObjectType::DilateFromPoint, "Select source point, then dilation center.");
  }

  if (ImGui::CollapsingHeader("Properties", ImGuiTreeNodeFlags_DefaultOpen)) {
    // Colors
    ImGui::Text("Color");
    ImVec4 currentColor = toImVec4(editor.getCurrentColor());
    if (ImGui::ColorButton("##current_color", currentColor, ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop, ImVec2(ImGui::GetContentRegionAvail().x, 25))) {
        ImGui::OpenPopup("ColorPickerPopup");
    }
    
    // Quick Palette
    const ImVec4 palette[] = {
        ImVec4(1, 1, 1, 1),       ImVec4(1, 0, 0, 1),       ImVec4(0, 1, 0, 1),       ImVec4(0, 0, 1, 1),
        ImVec4(1, 1, 0, 1),       ImVec4(0, 1, 1, 1),       ImVec4(1, 0, 1, 1),       ImVec4(0.5f, 0.5f, 0.5f, 1),
        ImVec4(1.0f, 0.65f, 0.0f, 1), ImVec4(0.0f, 0.5f, 0.5f, 1), ImVec4(0.5f, 0.0f, 0.5f, 1), ImVec4(1.0f, 0.75f, 0.8f, 1), ImVec4(0.2f, 0.2f, 0.2f, 1)
    };
    for (int i = 0; i < 13; ++i) {
      if (i > 0 && i % 7 != 0) ImGui::SameLine();
      ImGui::PushID(i);
      if (ImGui::ColorButton("##pal", palette[i])) {
         sf::Color c(static_cast<sf::Uint8>(palette[i].x * 255), static_cast<sf::Uint8>(palette[i].y * 255), static_cast<sf::Uint8>(palette[i].z * 255), 255);
         editor.setCurrentColor(c);
         if (editor.selectedObject) editor.selectedObject->setColor(c);
         for(auto* obj : editor.selectedObjects) if(obj) obj->setColor(c);
      }
      ImGui::PopID();
    }
    
    // Embedded Color Picker
    static float colorBuf[4] = {0,0,0,1};
    ImGui::ColorEdit4("Custom", colorBuf, ImGuiColorEditFlags_AlphaBar | ImGuiColorEditFlags_DisplayRGB); 
    if (ImGui::IsItemEdited()) {
         sf::Color c(static_cast<sf::Uint8>(colorBuf[0] * 255), static_cast<sf::Uint8>(colorBuf[1] * 255), static_cast<sf::Uint8>(colorBuf[2] * 255), static_cast<sf::Uint8>(colorBuf[3] * 255));
         editor.setCurrentColor(c);
         if (editor.selectedObject) editor.selectedObject->setColor(c);
         for(auto* obj : editor.selectedObjects) if(obj) obj->setColor(c);
    }

    ImGui::Separator();
    
    // Thickness
    int thickness = static_cast<int>(editor.currentThickness);
    if (ImGui::SliderInt("Thickness", &thickness, 1, 10)) {
        editor.currentThickness = static_cast<float>(thickness);
        if (editor.selectedObject && (editor.selectedObject->getType() == ObjectType::Line || editor.selectedObject->getType() == ObjectType::LineSegment)) {
            editor.selectedObject->setThickness(editor.currentThickness);
        }
    }
    
    // Point Size
    if (ImGui::SliderFloat("Point Size", &editor.currentPointSize, 2.0f, 20.0f, "%.1f")) {
        if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Point) {
            auto pt = std::dynamic_pointer_cast<Point>(editor.findSharedPtr(editor.selectedObject));
            if(pt) pt->setRadius(editor.currentPointSize);
        }
    }
    }
  }   

  ImGui::End();

  // Draw Overlays
  drawOverlays(editor, guiMessage, messageActive);

  // Context Menu
  const_cast<ContextMenu&>(m_contextMenu).draw(window);

  ImGui::SFML::Render(window);
}

void GUI::updateFontSizes() {
  for (auto& button : buttons) {
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

void GUI::toggleButton(const std::string& buttonName, bool state) {
  for (auto& button : buttons) {
    if (button.getLabel() == buttonName) {
      button.setActive(state);
      return;
    }
  }
}

bool GUI::isButtonActive(const std::string& label) const {
  for (const auto& button : buttons) {
    if (button.getLabel() == label) {
      return button.isActive();
    }
  }
  return false;
}

bool GUI::isMouseOverPalette(const sf::Vector2f& guiPos) const {
  if (!m_colorPicker || !m_colorPicker->isOpen()) return false;
  return m_colorPicker->isMouseOver(guiPos);
}

bool GUI::handleEvent(sf::RenderWindow& window, const sf::Event& event, GeometryEditor& editor) {
  // Use DefaultView for UI Hit Testing to match Screen Space rendering
  sf::Vector2f currentMousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window), window.getDefaultView());
  if (m_contextMenu.handleEvent(event, currentMousePos, editor)) return true;

  // Handle Angle Reflex Toggle (Overlay)
  if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
    if (editor.selectedObject && editor.selectedObject->getType() == ObjectType::Angle) {
      auto* angle = dynamic_cast<Angle*>(editor.selectedObject);
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

  return false;
}

void GUI::update(sf::Time deltaTime) {
  // Example update logic for GUI, like clearing timed messages
  (void)deltaTime;  // Mark as unused if deltaTime is not used yet
  clearMessage();
  // You could add other time-dependent GUI updates here,
  // like animations for buttons or fading messages.
}

void GUI::updateView(const sf::View& newGuiView) {
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
  return isPointActive() || isLineActive() || isLineSegmentActive() || isCircleActive() || isIntersectionActive() || isObjPointActive() ||
         isMoveActive();
}

void GUI::displayMessage(const std::string& message) {
  guiMessage.setString(message);
  // Position the message (example: bottom center of guiView)
  sf::FloatRect textRect = guiMessage.getLocalBounds();
  guiMessage.setOrigin(0, 0);  // Reset origin
  guiMessage.setPosition(10.f, guiView.getSize().y - textRect.height - 10.f);

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
  for (auto& button : buttons) {
    // Deactivate all tool buttons, except persistent ones like "Grid" or "Move"
    // if desired
    const std::string& label = button.getLabel();
    if (label != "Grid" && label != "Axes" /* && label != "Move" */) {  // Keep Grid/Axes active state
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

  for (auto& button : buttons) {
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
