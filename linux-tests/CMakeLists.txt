cmake_minimum_required(VERSION 3.10)
project(GeometryTool2Tests)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

# Define consistent threading model for CGAL
add_definitions(-DCGAL_HAS_THREADS)

# Find required packages
find_package(SFML 2.6 COMPONENTS graphics window system REQUIRED)
find_package(CGAL REQUIRED)
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# The absolute path to your main project directory
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Include directories - be very explicit about paths
include_directories(
    ${CGAL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${PROJECT_ROOT_DIR}  # Main project directory
)

message(STATUS "Include directories:")
message(STATUS "  CGAL: ${CGAL_INCLUDE_DIRS}")
message(STATUS "  GTest: ${GTEST_INCLUDE_DIRS}")
message(STATUS "  Project root: ${PROJECT_ROOT_DIR}")

# Define the source files needed for testing
set(LIB_SOURCES
    ${PROJECT_ROOT_DIR}/Point.cpp
    ${PROJECT_ROOT_DIR}/GeometricObject.cpp
    ${PROJECT_ROOT_DIR}/Line.cpp
    ${PROJECT_ROOT_DIR}/Circle.cpp
    ${PROJECT_ROOT_DIR}/Transforms.cpp
    ${PROJECT_ROOT_DIR}/ObjectPoint.cpp  # Add this line
)

# Create a library from the main project sources
add_library(GeometryToolLib STATIC ${LIB_SOURCES})

# Set consistent threading options for CGAL
target_compile_definitions(GeometryToolLib PRIVATE 
    CGAL_HAS_THREADS)

target_link_libraries(GeometryToolLib ${SFML_LIBRARIES} ${CGAL_LIBRARIES})

# Add test subdirectory
add_subdirectory(src)

# Enable testing
enable_testing()
