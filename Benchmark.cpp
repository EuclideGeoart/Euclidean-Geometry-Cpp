#include "Benchmark.h"
#include "GeometryEditor.h"
#include "Line.h"
#include <sstream>

Benchmark::Benchmark() {
    // Try loading Arial from Windows fonts, fallback to default if not found
    if (!m_font.loadFromFile("C:/Windows/Fonts/arial.ttf")) {
        m_font.loadFromFile("consola.ttf"); // Fallback, or add your own asset path
    }
    m_text.setFont(m_font);
    m_text.setCharacterSize(18);
    m_text.setFillColor(sf::Color::White);
    m_text.setOutlineColor(sf::Color::Black);
    m_text.setOutlineThickness(2.f);
}

void Benchmark::spawnStressTest(GeometryEditor& editor) {
    std::cout << "[Stress Test] Starting... Press ESC to Abort." << std::endl;
    int count = 0;
    for (int x = 0; x < 50; ++x) {
        // Emergency brake: ESC abort
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) {
            std::cout << "[Stress Test] ABORTED by user." << std::endl;
            return;
        }
        for (int y = 0; y < 50; ++y) {
            auto p1 = std::make_shared<Point>(sf::Vector2f(x * 10.f, y * 10.f), 1.0f);
            auto p2 = std::make_shared<Point>(sf::Vector2f((x + 1) * 10.f, y * 10.f), 1.0f);
            editor.addObject(p1);
            editor.addObject(p2);
            auto line = std::make_shared<Line>(p1, p2, true);
            editor.addObject(line);
            ++count;
            if (count % 250 == 0) {
                std::cout << "[Stress Test] Progress: " << count << " objects..." << std::endl;
            }
        }
    }
    std::cout << "[Stress Test] Complete." << std::endl;
}

void Benchmark::renderHUD(sf::RenderWindow& window, const GeometryEditor& editor, sf::Time logicTime) {
    float fps = 0.f;
    if (logicTime.asSeconds() > 0.f)
        fps = 1.f / logicTime.asSeconds();

    std::ostringstream oss;
    oss << "FPS: " << static_cast<int>(fps)
        << " | Objects: " << editor.getAllObjects().size()
        << " | Logic: " << logicTime.asMicroseconds() << " µs";

    m_text.setString(oss.str());
    m_text.setPosition(10.f, 10.f);

    // Draw HUD in screen space
    auto oldView = window.getView();
    window.setView(window.getDefaultView());
    window.draw(m_text);
    window.setView(oldView);
}

void Benchmark::clearStressTest(GeometryEditor& editor) {
    editor.clearScene();
}