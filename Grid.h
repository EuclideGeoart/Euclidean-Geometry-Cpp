#pragma once
#include <SFML/Graphics.hpp>
#include "Constants.h"
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel_1;
typedef Kernel_1::Point_2 Point_2;
typedef Kernel_1::Line_2 Line_2;

class Grid {
public:
    static void draw(sf::RenderWindow& window, const sf::View& view);

private:
    static void drawGridLines(sf::RenderWindow& window, const sf::View& view, float spacing);
    static void calculateGridSpacing(const sf::View& view, float& spacing, const sf::RenderWindow& window);
    static void drawAxisLabels(sf::RenderWindow& window, const sf::View& view, float spacing);
    static void drawAxes(sf::RenderWindow& window, const sf::View& view);
    Point_2 xAxisStart;
    Point_2 xAxisEnd;
    Point_2 yAxisStart;
    Point_2 yAxisEnd;
    Line_2 xAxis;
    Line_2 yAxis;
};
