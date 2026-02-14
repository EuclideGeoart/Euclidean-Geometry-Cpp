#ifndef HANDLE_EVENTS_H
#define HANDLE_EVENTS_H

#include "GeometryEditor.h"
#include <SFML/Graphics.hpp>

// Forward declarations for event handler functions
void handleMouseMove(GeometryEditor &editor,
                     const sf::Event::MouseMoveEvent &mouseMoveEvent);
void handleMouseRelease(GeometryEditor &editor,
                        const sf::Event::MouseButtonEvent &mouseEvent);
void handleZoom(GeometryEditor &editor, float delta,
                const sf::Vector2i &mousePos);
void handleResize(GeometryEditor &editor, unsigned int width,
                  unsigned int height);
void handleKeyPress(GeometryEditor &editor,
                    const sf::Event::KeyEvent &keyEvent);
void handleEvents(GeometryEditor &editor);
void deleteObject(GeometryEditor &editor, GeometricObject *objToDelete);

// Helper function for deselecting objects and clearing interaction state
void deselectAllAndClearInteractionState(GeometryEditor &editor, bool preserveSelection = false);

void markObjectForDeletion(GeometricObject* obj);
void unmarkObjectForDeletion(GeometricObject* obj);
bool isObjectBeingDeleted(GeometricObject *obj);
void clearAllDeletionTracking();

// Clears transient selection used by construction/transformation tools
void clearTempSelectedObjects();
#endif // HANDLE_EVENTS_H