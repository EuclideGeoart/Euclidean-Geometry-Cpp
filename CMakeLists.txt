cmake_minimum_required(VERSION 3.15)
project(GeometryTool2)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific compiler flags
if(WIN32)
    # Use lld only with Clang; GCC may not have lld available in MSYS2 by default
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=lld")
    endif()
    # MSYS2 math.h macro workaround - suppress problematic warnings
    add_compile_options(-Wno-expansion-to-defined)
    add_compile_options(-Wno-error)
    add_compile_options(-Wno-ignored-attributes)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=ld -U CGAL_USE_SSE2")
endif()

# Add compiler flags for warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Platform-specific SFML configuration
if(WIN32)
    # Prefer UCRT64 SFML (has 2.6); allow override via SFML_ROOT if set
    if (NOT DEFINED SFML_ROOT)
        if (EXISTS "C:/msys64/ucrt64")
            set(SFML_ROOT "C:/msys64/ucrt64")
        elseif (EXISTS "C:/msys64/clang64")
            set(SFML_ROOT "C:/msys64/clang64")
        endif()
    endif()
    find_package(SFML COMPONENTS graphics window system REQUIRED PATHS ${SFML_ROOT} NO_DEFAULT_PATH)
    message(STATUS "Windows: Found SFML at ${SFML_ROOT}")
else()
    find_package(SFML 2.6 COMPONENTS graphics window system audio network REQUIRED)
    if(SFML_FOUND)
        message(STATUS "Linux: Found system SFML version ${SFML_VERSION}")
    else()
        message(FATAL_ERROR "Linux: SFML 2.6 not found. Install with: sudo apt install libsfml-dev")
    endif()
endif()

# Platform-specific CGAL configuration
if(WIN32)
    # Prefer UCRT64 CGAL when using UCRT toolchain; fallback to clang64 if needed
    if (EXISTS "C:/msys64/ucrt64")
        set(CGAL_ROOT_HINT "C:/msys64/ucrt64")
    elseif (EXISTS "C:/msys64/clang64")
        set(CGAL_ROOT_HINT "C:/msys64/clang64")
    endif()
    find_package(CGAL REQUIRED PATHS ${CGAL_ROOT_HINT} NO_DEFAULT_PATH)
    message(STATUS "Windows: Found CGAL at ${CGAL_ROOT_HINT}")
else()
    find_package(CGAL REQUIRED)
    if(CGAL_FOUND)
        message(STATUS "Linux: Found system CGAL version ${CGAL_VERSION}")
    else()
        message(FATAL_ERROR "Linux: CGAL not found. Install with: sudo apt install libcgal-dev")
    endif()
endif()

# Platform-specific GMP/MPFR (needed for CGAL)
if(WIN32)
    message(STATUS "Windows: Using MSYS2 GMP/MPFR")
else()
    find_package(GMP REQUIRED)
    find_package(MPFR REQUIRED)
    if(GMP_FOUND AND MPFR_FOUND)
        message(STATUS "Linux: Found system GMP and MPFR")
    else()
        message(FATAL_ERROR "Linux: GMP/MPFR not found. Install with: sudo apt install libgmp-dev libmpfr-dev")
    endif()
endif()

# Platform-specific compile definitions
set(BASE_COMPILE_DEFINITIONS CGAL_USE_GMPXX=1 SFML_USE_CHAR32_STRING CGAL_HAS_THREADS)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include)

# MAIN EXECUTABLE - Only main program files, NO test files
add_executable(GeometryTool2
    main.cpp
    GeometryEditor.cpp
    GeometricObject.cpp
    Point.cpp
    Line.cpp
    Circle.cpp
    Rectangle.cpp
    Polygon.cpp
    RegularPolygon.cpp
    RegularPolygon.cpp
    ObjectPoint.cpp
    Triangle.cpp
    Angle.cpp
    GUI.cpp
    Grid.cpp
    HandleEvents.cpp
    Transforms.cpp
    Intersection.cpp
    IntersectionSystem.cpp
    GMP_exception_handler.cpp
    LineToolMode.cpp
    QuickProfiler.cpp
    Triangle.cpp
    ContextMenu.cpp
    VertexLabelManager.cpp
    PointUtils.cpp
    ProjectSerializer.cpp
)

# Platform-specific linking for main executable
if(WIN32)
    target_link_libraries(GeometryTool2 PRIVATE 
        sfml-graphics sfml-window sfml-system CGAL::CGAL
    )
else()
    target_link_libraries(GeometryTool2 PRIVATE 
        sfml-graphics sfml-window sfml-system CGAL::CGAL 
        ${GMP_LIBRARIES} ${MPFR_LIBRARIES}
    )
endif()

target_compile_definitions(GeometryTool2 PRIVATE ${BASE_COMPILE_DEFINITIONS})

# Platform-specific post-build steps
if(WIN32)
    add_custom_command(TARGET GeometryTool2 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/arial.ttf
            $<TARGET_FILE_DIR:GeometryTool2>/arial.ttf
    )
else()
    add_custom_command(TARGET GeometryTool2 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/fonts
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            /usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf
            ${CMAKE_BINARY_DIR}/arial.ttf
    )
endif()

# TESTS (Optional - only built when requested)
option(BUILD_TESTS "Build test executables" OFF)

if(BUILD_TESTS)
    # Find GTest only if building tests
    if(WIN32)
        find_package(PkgConfig QUIET)
        if(PKG_CONFIG_FOUND)
            pkg_check_modules(GTEST gtest)
            pkg_check_modules(GTEST_MAIN gtest_main)
        endif()
        if(NOT GTEST_FOUND)
            find_package(GTest PATHS "C:/msys64/clang64" NO_DEFAULT_PATH)
        endif()
    else()
        find_package(GTest REQUIRED)
    endif()

    if(GTEST_FOUND OR GTest_FOUND)
        enable_testing()
        
        add_executable(LineDebuggingTest EXCLUDE_FROM_ALL
            linux-tests/src/LineDebuggingTest.cpp
            Point.cpp Line.cpp GeometricObject.cpp ObjectPoint.cpp Circle.cpp
        )
        target_compile_definitions(LineDebuggingTest PRIVATE ${BASE_COMPILE_DEFINITIONS})
        
        if(WIN32)
            target_link_libraries(LineDebuggingTest PRIVATE 
                gtest gtest_main sfml-graphics CGAL::CGAL
            )
        else()
            target_link_libraries(LineDebuggingTest PRIVATE 
                GTest::GTest GTest::Main sfml-graphics CGAL::CGAL 
                ${GMP_LIBRARIES} ${MPFR_LIBRARIES}
            )
        endif()
        
        add_test(NAME LineDebuggingTest_CTest COMMAND LineDebuggingTest)
    endif()
endif()

# Exception test (separate from GTest)
add_executable(TestExceptions EXCLUDE_FROM_ALL test_exceptions.cpp)
target_compile_definitions(TestExceptions PRIVATE TEST_EXCEPTIONS_MAIN ${BASE_COMPILE_DEFINITIONS})

if(WIN32)
    target_link_libraries(TestExceptions PRIVATE CGAL::CGAL)
    target_link_options(TestExceptions PRIVATE "-Wl,--subsystem,console")
else()
    target_link_libraries(TestExceptions PRIVATE CGAL::CGAL ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
endif()

message(STATUS "=== Configuration Summary ===")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Build Tests: ${BUILD_TESTS}")
message(STATUS "==============================")