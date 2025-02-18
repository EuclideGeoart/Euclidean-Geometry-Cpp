#ifndef HANDLE_EVENTS_H
#define HANDLE_EVENTS_H

#include <SFML/Graphics.hpp>
#include "GeometryEditor.h"

// Forward declare the GeometryEditor class
class GeometryEditor;

void handleMousePress(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent);
void handleMouseMove(GeometryEditor& editor, const sf::Event::MouseMoveEvent& moveEvent);
void handleMouseRelease(GeometryEditor& editor, const sf::Event::MouseButtonEvent& mouseEvent);
void handlePanning(GeometryEditor& editor, const sf::Event::MouseMoveEvent& mouseEvent);
void handleZoom(GeometryEditor& editor, float delta, const sf::Vector2i& mousePos);
void handleResize(GeometryEditor& editor, unsigned int width, unsigned int height);
void handleEvents(GeometryEditor& editor);

#endif // HANDLE_EVENTS_H	