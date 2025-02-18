#include "GUI.h"
#include <iostream>
#include "Constants.h"
#include <SFML/Graphics.hpp>

sf::Font Button::font;
bool Button::fontLoaded = false;
float xOffset = 10.0f;
const float yPos = 10.0f;
const float buttonSpacing = 10.0f;

Button::Button(sf::Vector2f position, sf::Vector2f size, std::string label, sf::Color color)
    : shape(size), active(false) {

    loadFont();  // This will only load the font once

    // Set up the shape
    shape.setPosition(position);
    shape.setFillColor(color);
    shape.setOutlineThickness(2.0f);
    shape.setOutlineColor(sf::Color::Black);

    // Set up the text with the static font
    text.setFont(font);
    text.setString(label);
    text.setCharacterSize(14);
    text.setFillColor(sf::Color::White);

    centerText();
}


void Button::centerText() {
    // Get the global bounds of the text (includes any potential spacing/padding)
    sf::FloatRect textBounds = text.getGlobalBounds();
    sf::Vector2f buttonPos = shape.getPosition();
    sf::Vector2f buttonSize = shape.getSize();

    // Calculate centered position
    float xPos = buttonPos.x + (buttonSize.x - textBounds.width) / 2.0f;
    // Adjust vertical position to account for text baseline
    float yPos = buttonPos.y + (buttonSize.y - textBounds.height) / 2.0f - textBounds.top / 2.0f;

    text.setPosition(xPos, yPos);
}

void Button::draw(sf::RenderWindow& window) const {
    if (!window.isOpen()) {
        return;
    }

    // Draw the button shape first
    window.draw(shape);

     //Draw the text if font is loaded
    if (text.getFont() != nullptr) {
        window.draw(text);  // Re-enabled text drawing
    }
}

// Rest of the implementation stays the same...
void Button::setLabel(const std::string& label) {
    text.setString(label);
    centerText();
}

bool Button::isMouseOver(const sf::RenderWindow& window) const {
    sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
    sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos);
    return shape.getGlobalBounds().contains(worldPos);
}

std::string Button::getLabel() const {
    return text.getString().toAnsiString();
}

void Button::setColor(sf::Color color) {
    shape.setFillColor(color);
}

sf::Color Button::getColor() const {
    return shape.getFillColor();
}

bool Button::isActive() const {
    return active;
}

void Button::setActive(bool active) {
    this->active = active;
    shape.setFillColor(active ? Constants::BUTTON_ACTIVE_COLOR : Constants::BUTTON_INACTIVE_COLOR);
}

// GUI class implementation
GUI::GUI() {
    float xOffset = 10.0f;
    const float yPos = 10.0f;
    const float buttonSpacing = 10.0f;

    try {
        // Create buttons with proper spacing
        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "Grid", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;

        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "Point", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;
        // --- ADDED: "Point on Object" button ---
        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "ObjPoint", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;
        // --- ADDED: "Line" button ---
        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "Line", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;

        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "Line Segment", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;
        // --- ADDED: "Circle" button ---
        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "Circle", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;

        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "Intersect", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;
        // --- ADDED: "Move" button ---
        buttons.emplace_back(sf::Vector2f(xOffset, yPos), Constants::BUTTON_SIZE, "Move", Constants::BUTTON_INACTIVE_COLOR);
        xOffset += Constants::BUTTON_SIZE.x + buttonSpacing;
    }
    catch (const std::runtime_error& e) {
        std::cerr << "Error creating GUI: " << e.what() << std::endl;
    }
}

void GUI::draw(sf::RenderWindow& window) const {
    for (const auto& button : buttons) {
        button.draw(window);
    }
    if (messageActive) {
        window.draw(guiMessage);
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

void GUI::handleEvent(const sf::RenderWindow& window,const sf::Event& event ) {
    if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
        for (auto& button : buttons) {
            if (button.isMouseOver(window)) {
                if (button.getLabel() == "Grid") {
                    button.setActive(!button.isActive());
                }
                else if (button.getLabel() == "Move") {
                    button.setActive(!button.isActive()); // Toggle Move button
                    // Deactivate other buttons (except Grid)
                    for (auto& otherButton : buttons) {
                        if (otherButton.getLabel() != "Grid" && otherButton.getLabel() != "Move") {
                            otherButton.setActive(false);
                        }
                    }
                }
                else {
                    // If clicking an already active button, deactivate it
                    if (button.isActive()) {
                        button.setActive(false);
                    }
                    else {
                        // Deactivate other non-grid buttons
                        for (auto& otherButton : buttons) {
                            if (otherButton.getLabel() != "Grid") {
                                otherButton.setActive(false);
                            }
                        }
                        button.setActive(true);
                    }
                }
                break; // Important: Break after handling a button
            }
        }
    }
}

void GUI::displayMessage(const std::string& message) {  
   // Assuming there's a member sf::Text guiMessage;  
   guiMessage.setString(message);  
   guiMessage.setPosition(10.0f, 10.0f);  // Set to a fixed position or desired coordinates  
   guiMessage.setFillColor(sf::Color::Black);  
   // Possibly set a timer to auto-clear after a few seconds  
   messageTimer.restart();  
   messageActive = true;  
}

void GUI::clearMessage() {
    if (messageActive && messageTimer.getElapsedTime().asSeconds() > 2.0f) { // Display for 2 seconds
        guiMessage.setString("");
        messageActive = false;
    }
}

bool GUI::isGridActive() const {
    return isButtonActive("Grid");
}

bool GUI::isPointActive() const {
    return isButtonActive("Point");
}
bool GUI::isObjPointActive() const {
	return isButtonActive("ObjPoint");
}
bool GUI::isLineActive() const {
    return isButtonActive("Line");
}

bool GUI::isLineSegmentActive() const {
    return isButtonActive("Line Segment");
}

bool GUI::isIntersectionActive() const {
    return isButtonActive("Intersect");
}
bool GUI::isCircleActive() const {
    return isButtonActive("Circle");
}
