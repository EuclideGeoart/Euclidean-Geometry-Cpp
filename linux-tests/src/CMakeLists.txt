# Add test executable
add_executable(PointTest PointTest.cpp)

# Explicitly link all SFML components needed
target_link_libraries(PointTest 
    GeometryToolLib 
    ${GTEST_LIBRARIES} 
    sfml-graphics
    sfml-window
    sfml-system
    pthread)

add_test(NAME PointTest COMMAND PointTest)

# Add debugging test for Line class
add_executable(LineDebugTest LineDebuggingTest.cpp)
target_link_libraries(LineDebugTest 
    GeometryToolLib 
    ${GTEST_LIBRARIES} 
    sfml-graphics
    sfml-window
    sfml-system
    pthread)

add_test(NAME LineDebugTest COMMAND LineDebugTest)

