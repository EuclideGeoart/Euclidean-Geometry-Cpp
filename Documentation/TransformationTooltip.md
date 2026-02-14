# Transformation Hover Tooltip

## Overview
The Transformation Hover Tooltip provides real-time visual feedback about an object's transformation history when the user hovers over it with the mouse.

## Implementation Details

### 1. State Definition
A new member `currentTooltip` was added to the `GeometryEditor` class to store the tooltip text for the current frame.

**File:** `GeometryEditor.h`
```cpp
class GeometryEditor {
public:
    // ...
    std::string currentTooltip; // Stores the formatted message for the current frame
    // ...
};
```

### 2. Logic (Detection & Formatting)
The logic for detecting hovered objects and generating the tooltip message is implemented in `handleMouseMove`. It performs hit-testing against points and shapes, prioritizing points. If a transformed (dependent) object is found, it extracts the transformation type and the source object's ID.

**File:** `HandleEvents.cpp`
```cpp
void handleMouseMove(GeometryEditor& editor, const sf::Event::MouseMoveEvent& moveEvent) {
    // ...
    editor.currentTooltip = ""; // Reset every frame

    if (hoveredObj && hoveredObj->isDependent()) {
        TransformationType type = hoveredObj->getTransformType();
        unsigned int sourceID = hoveredObj->getParentSourceID();
        
        if (type != TransformationType::None) {
            std::string typeStr = getTransformationName(type);
            editor.currentTooltip = typeStr + "\nSource: " + getObjectTypeName(hoveredObj->getType()) + " #" + std::to_string(sourceID);
        }
    }
}
```

### 3. Rendering
The tooltip is rendered in the `render` function. It draws a semi-transparent black background box and white text near the mouse cursor position, mapped to the GUI view space.

**File:** `GeometryEditor.cpp`
```cpp
void GeometryEditor::render() {
    // ... after drawing all objects ...
    if (!currentTooltip.empty()) {
        // Map world mouse pos to GUI space
        sf::Vector2f uiPos = window.mapPixelToCoords(window.mapCoordsToPixel(lastMousePos_sfml, drawingView), guiView);

        sf::Text tooltipText;
        tooltipText.setFont(Button::getFont());
        tooltipText.setString(currentTooltip);
        tooltipText.setCharacterSize(14); 
        tooltipText.setFillColor(sf::Color::White); 
        tooltipText.setPosition(uiPos + sf::Vector2f(15, 15));

        sf::FloatRect bounds = tooltipText.getGlobalBounds();
        sf::RectangleShape bg(sf::Vector2f(bounds.width + 10.f, bounds.height + 6.f));
        bg.setPosition(bounds.left - 5.f, bounds.top - 3.f);
        bg.setFillColor(sf::Color(0, 0, 0, 200)); 
        bg.setOutlineColor(sf::Color(200, 200, 200));
        bg.setOutlineThickness(1.0f);

        window.draw(bg);
        window.draw(tooltipText);
    }
    // ...
}
```

## Summary of Code Order
1.  **Definitions**: The field `currentTooltip` must exist in `GeometryEditor.h`.
2.  **Detection**: `HandleEvents.cpp` computes the string during mouse movement.
3.  **Drawing**: `GeometryEditor.cpp` renders the final UI overlay.
