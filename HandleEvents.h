#ifndef HANDLE_EVENTS_H
#define HANDLE_EVENTS_H

#include "GeometryEditor.h"
#include <SFML/Graphics.hpp>

// Forward declarations for event handler functions
void handleMousePress(GeometryEditor &editor,
                      const sf::Event::MouseButtonEvent &mouseEvent);
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

// Add forward declarations for the creation handler functions
void handlePointCreation(GeometryEditor &editor,
                         const sf::Event::MouseButtonEvent &mouseEvent);
void handleLineCreation(GeometryEditor &editor,
                        const sf::Event::MouseButtonEvent &mouseEvent);
void handleParallelLineCreation(GeometryEditor &editor,
                                const sf::Event::MouseButtonEvent &mouseEvent);
void handlePerpendicularLineCreation(
    GeometryEditor &editor, const sf::Event::MouseButtonEvent &mouseEvent);
void handleObjectPointCreation(GeometryEditor &editor,
                               const sf::Event::MouseButtonEvent &mouseEvent);
void handleRectangleCreation(GeometryEditor &editor,
                             const sf::Event::MouseButtonEvent &mouseEvent);
void handleRotatableRectangleCreation(GeometryEditor &editor,
                                      const sf::Event::MouseButtonEvent &mouseEvent);
void handlePolygonCreation(GeometryEditor &editor,
                           const sf::Event::MouseButtonEvent &mouseEvent);
void handleRegularPolygonCreation(GeometryEditor &editor,
                                  const sf::Event::MouseButtonEvent &mouseEvent);
void handleTriangleCreation(GeometryEditor &editor,
                            const sf::Event::MouseButtonEvent &mouseEvent);

// Helper function for deselecting objects and clearing interaction state
void deselectAllAndClearInteractionState(GeometryEditor &editor);
void createObjectPointOnLine(GeometryEditor &editor, Line *lineHost,
                             const Point_2 &cgalWorldPos);
void createObjectPointOnCircle(GeometryEditor &editor, Circle *circleHost,
                               const Point_2 &cgalWorldPos);

// Helper function to snap to nearby objects (lines, circles, etc)
// Returns ObjectPoint if snapped, nullptr otherwise
std::shared_ptr<Point> trySnapToNearbyObject(GeometryEditor &editor,
                                            const sf::Vector2f &worldPos_sfml,
                                            const Point_2 &cgalWorldPos,
                                            float snapTolerance);

void markObjectForDeletion(GeometricObject* obj);
void unmarkObjectForDeletion(GeometricObject* obj);
bool isObjectBeingDeleted(GeometricObject *obj);
void clearAllDeletionTracking();
#endif // HANDLE_EVENTS_H