#include <CGAL/number_utils.h>
#include <gtest/gtest.h>
#include "Types.h"
#include "Point.h"
#include "Line.h"
#include <SFML/Graphics.hpp>
#include <iostream>
unsigned int id_counter = 0;
// Test to investigate Line constructor and internal state
TEST(LineDebug, ConstructorInspection) {
    
    // Create points with very different coordinates for clarity
    Point_2 p1(0, 0);
    Point_2 p2(100, 100);
    float default_zoom = 1.0f;
    
    // Create Point objects
    auto point1 = std::make_shared<Point>(p1, default_zoom, sf::Color::Red);
    auto point2 = std::make_shared<Point>(p2, default_zoom, sf::Color::Blue);
    
    // Print addresses to check they're different
    std::cout << "Point 1 address: " << point1.get() << std::endl;
    std::cout << "Point 2 address: " << point2.get() << std::endl;
    
    // Verify Point values before line creation
    std::cout << "Point 1 CGAL: (" 
    << CGAL::to_double(point1->getCGALPosition().x()) << "," 
    << CGAL::to_double(point1->getCGALPosition().y()) << ")" << std::endl;
    std::cout << "Point 2 CGAL: (" 
    << CGAL::to_double(point2->getCGALPosition().x()) << "," 
    << CGAL::to_double(point2->getCGALPosition().y()) << ")" << std::endl;
    
    // Create Line and check the stored points
    std::cout << "\nCreating line with these points..." << std::endl;
    Line line(point1, point2, true, sf::Color::Green);
    
    // Check what the line stored
    auto startPt = line.getStartPoint();
    auto endPt = line.getEndPoint();
    
    std::cout << "Line start point: (" 
              << CGAL::to_double(startPt.x()) << "," 
              << CGAL::to_double(startPt.y()) << ")" << std::endl;
    std::cout << "Line end point: (" 
              << CGAL::to_double(endPt.x()) << "," 
              << CGAL::to_double(endPt.y()) << ")" << std::endl;
    
    // Print the raw memory addresses stored in the Line
    if (line.getStartPointPtr()) {
        std::cout << "Line's stored start point address: " << line.getStartPointPtr() << std::endl;
    } else {
        std::cout << "Line's stored start point is NULL" << std::endl;
    }
    
    if (line.getEndPointPtr()) {
        std::cout << "Line's stored end point address: " << line.getEndPointPtr() << std::endl;
    } else {
        std::cout << "Line's stored end point is NULL" << std::endl;
    }
    
    // Check if original points were modified
    std::cout << "\nChecking if original points were modified:" << std::endl;
    std::cout << "Point 1 CGAL after: (" 
              << CGAL::to_double(point1->getCGALPosition().x()) << "," 
              << CGAL::to_double(point1->getCGALPosition().y()) << ")" << std::endl;
    std::cout << "Point 2 CGAL after: (" 
              << CGAL::to_double(point2->getCGALPosition().x()) << "," 
              << CGAL::to_double(point2->getCGALPosition().y()) << ")" << std::endl;
    
    // Add basic assertion that should pass if points are different
    ASSERT_NE(point1, point2) << "Points have the same memory address!";
    
    // These assertions will fail based on your current bug
    EXPECT_NE(CGAL::to_double(startPt.x()), CGAL::to_double(endPt.x())) 
        << "Line start and end X coordinates are the same!";
    EXPECT_NE(CGAL::to_double(startPt.y()), CGAL::to_double(endPt.y()))
        << "Line start and end Y coordinates are the same!";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}