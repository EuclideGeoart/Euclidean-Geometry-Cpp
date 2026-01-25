#include <CGAL/number_utils.h>
#include <gtest/gtest.h>
// Use relative paths to include project files
#include "Types.h"
#include "Point.h"
#include "Line.h"
#include "Circle.h"
#include <SFML/Graphics.hpp>
#include <iostream>

unsigned int id_counter = 0;
// Test to investigate Line constructor and internal state
TEST(LineDebug, ConstructorInspection) {
    float default_zoom = 1.0f;
    // Create points with very different coordinates for clarity
    Point_2 p1(0, 0);
    Point_2 p2(100, 100);
    
    // Create Point objects
    auto point1 = std::make_shared<Point>(p1,default_zoom, sf::Color::Red);
    auto point2 = std::make_shared<Point>(p2,default_zoom, sf::Color::Blue);
    
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
    Line line(point1, point2, true, sf::Color::Green, ++id_counter);
    
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
    std::cout << "Line's stored start point address: " << line.getStartPointPtr() << std::endl;
    std::cout << "Line's stored end point address: " << line.getEndPointPtr() << std::endl;
    
    // Check if original points were modified
    std::cout << "\nChecking if original points were modified:" << std::endl;
    std::cout << "Point 1 CGAL after: (" 
              << CGAL::to_double(point1->getCGALPosition().x()) << "," 
              << CGAL::to_double(point1->getCGALPosition().y()) << ")" << std::endl;
    std::cout << "Point 2 CGAL after: (" 
              << CGAL::to_double(point2->getCGALPosition().x()) << "," 
              << CGAL::to_double(point2->getCGALPosition().y()) << ")" << std::endl;
    
    // Add basic assertion that should pass if points are different
    ASSERT_NE(point1.get(), point2.get()) << "Points have the same memory address!";
    
    // These assertions will fail based on your current bug
    EXPECT_NE(CGAL::to_double(startPt.x()), CGAL::to_double(endPt.x())) 
        << "Line start and end X coordinates are the same!";
    EXPECT_NE(CGAL::to_double(startPt.y()), CGAL::to_double(endPt.y()))
        << "Line start and end Y coordinates are the same!";
}

// Simple test to verify testing works
TEST(BasicTest, AssertionWorks) {
    EXPECT_TRUE(true);
}

// Basic test for CGAL Point_2
TEST(CGALTest, Point2Works) {
    Point_2 p(3.0, 4.0);
    EXPECT_DOUBLE_EQ(3.0, CGAL::to_double(p.x()));
    EXPECT_DOUBLE_EQ(4.0, CGAL::to_double(p.y()));
}

TEST(CGALTest, LineTest) {
    Point_2 p1(0, 0);
    Point_2 p2(3, 4);
    Line_2 line(p1, p2);
    
    // Test basic line properties
    EXPECT_TRUE(line.has_on(p1));
    EXPECT_TRUE(line.has_on(p2));
}

// Check the Point class
TEST(PointTest, PointCreation) {
  Point_2 cgalPoint(5, 10);
  float default_zoom = 1.0f;
    
    // Create a unique_ptr to prevent memory leaks
    auto pointPtr = std::make_shared<Point>(cgalPoint, default_zoom, sf::Color::Red);
    
    // Test the CGAL point properties directly
    EXPECT_DOUBLE_EQ(5.0, CGAL::to_double(cgalPoint.x()));
    EXPECT_DOUBLE_EQ(10.0, CGAL::to_double(cgalPoint.y()));
}

// Test Line class with correct constructor and method calls
TEST(LineTest, LineProperties) {
    // Create points with clearly different coordinates
    Point_2 p1(0, 0);
    Point_2 p2(10, 10);  // Use more distinct values
    
    // Create Points with unique_ptr to prevent memory leaks
    auto point1Ptr = std::make_shared<Point>(p1, sf::Color::Red);
    auto point2Ptr = std::make_shared<Point>(p2, sf::Color::Red);
    
    // Debug print to see what's happening
    std::cout << "Creating line with points: (" 
              << CGAL::to_double(p1.x()) << "," << CGAL::to_double(p1.y()) << ") and ("
              << CGAL::to_double(p2.x()) << "," << CGAL::to_double(p2.y()) << ")" << std::endl;
    
    // Create a Line with raw pointers from the unique_ptrs
    Line line(point1Ptr, point2Ptr, true, sf::Color::Blue, id_counter++);
    
    // Test line endpoints
    auto startPoint = line.getStartPoint();
    auto endPoint = line.getEndPoint();
    
    // Debug print the returned endpoints
    std::cout << "Start point from line: (" 
              << CGAL::to_double(startPoint.x()) << "," << CGAL::to_double(startPoint.y()) << ")" << std::endl;
    std::cout << "End point from line: (" 
              << CGAL::to_double(endPoint.x()) << "," << CGAL::to_double(endPoint.y()) << ")" << std::endl;
              
    EXPECT_DOUBLE_EQ(0.0, CGAL::to_double(startPoint.x()));
    EXPECT_DOUBLE_EQ(0.0, CGAL::to_double(startPoint.y()));
    EXPECT_DOUBLE_EQ(10.0, CGAL::to_double(endPoint.x()));
    EXPECT_DOUBLE_EQ(10.0, CGAL::to_double(endPoint.y()));
}

// Test circle with correct constructor
TEST(CircleTest, CircleProperties) {
    Point_2 center(0, 0);
    
    // Create a centerPoint with unique_ptr to prevent memory leaks
    auto centerPointPtr = std::make_shared<Point>(center, sf::Color::Red);
    
    // Use the correct constructor signature - adding outline and fill colors
    Circle circle(centerPointPtr.get(), nullptr, 5.0, sf::Color::Green);
    
    // Adjust methods to match what Circle class actually has
    auto centerPt = circle.getCenterPoint();
    EXPECT_DOUBLE_EQ(0.0, CGAL::to_double(centerPt.x()));
    EXPECT_DOUBLE_EQ(0.0, CGAL::to_double(centerPt.y()));
    EXPECT_DOUBLE_EQ(5.0, circle.getRadius());
}

// Run the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
