cmake_minimum_required(VERSION 3.22)
project(GeometryTool2)

# Find packages with an explicit version constraint for SFML
find_package(CGAL REQUIRED)
find_package(SFML COMPONENTS graphics window system REQUIRED)

# Add the executable
add_executable(GeometryTool2
    Circle.cpp Circle.h Command.h CommandManager.h Config.cpp Config.h Constants.h GenericDeleteCommand.h 
    GeometricObject.cpp GeometricObject.h GeometryEditor.cpp GeometryEditor.h 
    GeometryFactory.h Grid.cpp Grid.h GUI.cpp GUI.h HandleEvents.cpp 
    HandleEvents.h Header.h Intersection.cpp Intersection.h Line.cpp 
    Line.h main.cpp ObjectPoint.cpp ObjectPoint.h ObjectType.h 
    Point.cpp Point.h ProjectionUtils.h Transforms.cpp Transforms.h Types.h
)

# Include directories
target_include_directories(GeometryTool2 PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}
    ${CGAL_INCLUDE_DIRS}
    ${SFML_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(GeometryTool2 PRIVATE
    CGAL::CGAL
    sfml-graphics
    sfml-window
    sfml-system
)