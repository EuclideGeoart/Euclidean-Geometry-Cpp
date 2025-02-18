#ifndef GUI_H
#define GUI_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>

class Button {
public:
	Button(sf::Vector2f position, sf::Vector2f size, std::string label, sf::Color color);
	static void loadFont() {
		if (!fontLoaded) {
			if (!font.loadFromFile("arial.ttf")) {
				throw std::runtime_error("Font loading failed");
			}
			fontLoaded = true;
		}
	}
	void draw(sf::RenderWindow& window) const;
	bool isMouseOver(const sf::RenderWindow& window) const;
	void setLabel(const std::string& label);
	std::string getLabel() const;
	void setColor(sf::Color color);
	sf::Color getColor() const;
	bool isActive() const;
	void setActive(bool active);

private:
	void centerText();  // New helper function
	static sf::Font font;
	static bool fontLoaded;
	sf::RectangleShape shape;
	sf::Text text;
	bool active;
};

class GUI {
public:
	GUI();
	void draw(sf::RenderWindow& window) const;
	bool isButtonActive(const std::string& label) const;
	void handleEvent(const sf::RenderWindow& window, const sf::Event& event);
	void setView(const sf::View& view) { guiView = view; }
	bool isGridActive() const;
	bool isPointActive() const;
	bool isObjPointActive() const;
	bool isLineActive() const;
	bool isLineSegmentActive() const;
	bool isMoveActive() const {
		return isButtonActive("Move");
	}
	bool isAnyToolActive() const {
		return isLineActive() || isLineSegmentActive() || isPointActive() || isIntersectionActive() || isCircleActive();
	}
	bool isIntersectionActive() const;
	//void resetCreationMode() {
	//	// Deactivate all creation-related buttons
	//	for (auto& button : buttons) {
	//		if (button.getLabel() != "Grid" && button.getLabel() != "Move") {
	//			button.setActive(false);
	//		}
	//	}
	//}
	bool isCircleActive() const;

    // Displays a transient message on the GUI
    void displayMessage(const std::string& message);

    // Clears any displayed message
    void clearMessage();

private:
	std::vector<Button> buttons;
	sf::View guiView;
	enum class ButtonType { Point, Line, LineSegment, Intersection, Grid };
	ButtonType activeButton;

    sf::Text guiMessage;
    sf::Clock messageTimer;
    bool messageActive = false;
};

#endif // GUI_H