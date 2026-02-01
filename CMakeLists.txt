cmake_minimum_required(VERSION 3.15)
project(FluxGeo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific compiler flags
if(WIN32)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=lld")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=lld")
    endif()
    add_compile_options(-Wno-expansion-to-defined)
    add_compile_options(-Wno-error)
    add_compile_options(-Wno-ignored-attributes)
else()
    # Linux/Unix Flags
    # Removed -fuse-ld=ld (let cmake decide default)
    # Added -static-libgcc -static-libstdc++ to bake runtimes into the binary
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -U CGAL_USE_SSE2 -static-libgcc -static-libstdc++") 
endif()

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    # Optimizations for Release
    add_compile_options($<$<CONFIG:Release>:-O3>)
endif()

# --- Dependencies ---

# SFML
if(WIN32)
    if (NOT DEFINED SFML_ROOT)
        if (EXISTS "C:/msys64/ucrt64")
            set(SFML_ROOT "C:/msys64/ucrt64")
        # elseif (EXISTS "C:/msys64/clang64")
        #     set(SFML_ROOT "C:/msys64/clang64")
        endif()
    endif()
    find_package(SFML COMPONENTS graphics window system REQUIRED PATHS ${SFML_ROOT} NO_DEFAULT_PATH)
else()
    # Lowered requirement to 2.5 for better compatibility with WSL/Ubuntu Repos
    find_package(SFML 2.5 COMPONENTS graphics window system audio network REQUIRED)
endif()

# CGAL
if(WIN32)
    if (EXISTS "C:/msys64/ucrt64")
        set(CGAL_ROOT_HINT "C:/msys64/ucrt64")
    # elseif (EXISTS "C:/msys64/clang64")
    #     set(CGAL_ROOT_HINT "C:/msys64/clang64")
    endif()
    find_package(CGAL REQUIRED PATHS ${CGAL_ROOT_HINT} NO_DEFAULT_PATH)
else()
    find_package(CGAL REQUIRED)
endif()

# GMP/MPFR
if(NOT WIN32)
    find_package(GMP REQUIRED)
    find_package(MPFR REQUIRED)
endif()

# Compile Definitions
set(BASE_COMPILE_DEFINITIONS CGAL_USE_GMPXX=1 SFML_USE_CHAR32_STRING CGAL_HAS_THREADS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-sfml)

# Define Sources list dynamically to handle platform differences
set(SOURCES
    main.cpp
    GeometryEditor.cpp
    GeometricObject.cpp
    Point.cpp
    Line.cpp
    Circle.cpp
    Rectangle.cpp
    Polygon.cpp
    RegularPolygon.cpp  # Duplicate removed
    ObjectPoint.cpp
    Triangle.cpp
    Angle.cpp
    GUI.cpp
    Grid.cpp
    HandleEvents.cpp
    HandleMousePress.cpp
    Transforms.cpp
    Intersection.cpp
    IntersectionSystem.cpp
    GMP_exception_handler.cpp
    LineToolMode.cpp
    QuickProfiler.cpp
    ContextMenu.cpp
    VertexLabelManager.cpp
    PointUtils.cpp
    ProjectSerializer.cpp
    RayObject.cpp
    VectorObject.cpp
    libs/imgui/imgui.cpp
    libs/imgui/imgui_draw.cpp
    libs/imgui/imgui_tables.cpp
    libs/imgui/imgui_widgets.cpp
    libs/imgui/imgui_demo.cpp
    libs/imgui-sfml/imgui-SFML.cpp
)

# ImGui-SFML compatibility shim for ImGui 1.92 API changes
set(IMGUI_SFML_COMPAT_HEADER "${CMAKE_CURRENT_BINARY_DIR}/imgui_sfml_compat.h")
file(WRITE "${IMGUI_SFML_COMPAT_HEADER}" "#pragma once\n\n#include <imgui.h>\n\n#ifndef ImGuiKey_ModCtrl\n#define ImGuiKey_ModCtrl ImGuiMod_Ctrl\n#endif\n#ifndef ImGuiKey_ModShift\n#define ImGuiKey_ModShift ImGuiMod_Shift\n#endif\n#ifndef ImGuiKey_ModAlt\n#define ImGuiKey_ModAlt ImGuiMod_Alt\n#endif\n#ifndef ImGuiKey_ModSuper\n#define ImGuiKey_ModSuper ImGuiMod_Super\n#endif\n\n#ifndef ImGuiNavInput_Activate\nenum ImGuiNavInput_ {\n    ImGuiNavInput_Activate = 0,\n    ImGuiNavInput_Cancel = 1,\n    ImGuiNavInput_Input = 2,\n    ImGuiNavInput_Menu = 3,\n    ImGuiNavInput_FocusPrev = 4,\n    ImGuiNavInput_FocusNext = 5,\n    ImGuiNavInput_TweakSlow = 6,\n    ImGuiNavInput_TweakFast = 7\n};\n#endif\n\n#ifndef TextureId\n#define TextureId GetTexID()\n#endif\n\n#ifndef TexID\n#define TexID TexID.GetTexID()\n#endif\n\nnamespace ImGui {\ninline void Image(ImTextureID tex_id, const ImVec2& image_size, const ImVec2& uv0, const ImVec2& uv1, const ImVec4& tint_col, const ImVec4& border_col) {\n    ImGui::ImageWithBg(ImTextureRef(tex_id), image_size, uv0, uv1, border_col, tint_col);\n}\n\ninline bool ImageButton(ImTextureID tex_id, const ImVec2& image_size, const ImVec2& uv0, const ImVec2& uv1, int frame_padding, const ImVec4& bg_col, const ImVec4& tint_col) {\n    return ImGui::ImageButton(\"##imgui-sfml\", ImTextureRef(tex_id), image_size, uv0, uv1, bg_col, tint_col);\n}\n} // namespace ImGui\n")

set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-sfml/imgui-SFML.cpp
    PROPERTIES COMPILE_OPTIONS "-include;${IMGUI_SFML_COMPAT_HEADER}"
)

# Only add FileDialogs if on Windows, or if you have implemented Linux support in it
if(WIN32)
    list(APPEND SOURCES FileDialogs.cpp)
else()
    # On Linux, if you don't have a file dialog impl, we might need a stub
    # For now, let's include it, but if it fails, you'll need to wrap Windows code in #ifdef _WIN32
    list(APPEND SOURCES FileDialogs.cpp) 
endif()

# Main Executable
add_executable(FluxGeo ${SOURCES})

# Linking
if(WIN32)
#
    target_link_libraries(FluxGeo PRIVATE 
        sfml-graphics sfml-window sfml-system CGAL::CGAL comdlg32 opengl32 
    )
    target_link_options(FluxGeo PRIVATE "") #-mwindows
else()
    target_link_libraries(FluxGeo PRIVATE 
        sfml-graphics sfml-window sfml-system CGAL::CGAL 
        ${GMP_LIBRARIES} ${MPFR_LIBRARIES} GL
    )
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

target_compile_definitions(FluxGeo PRIVATE ${BASE_COMPILE_DEFINITIONS})

# Post-Build: Fonts
if(WIN32)
    add_custom_command(TARGET FluxGeo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/arial.ttf
            $<TARGET_FILE_DIR:FluxGeo>/arial.ttf
    )
else()
    # Fallback if system font doesn't exist
    add_custom_command(TARGET FluxGeo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/fonts
        # Try to copy local arial.ttf first if it exists, otherwise try system
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/arial.ttf
            ${CMAKE_BINARY_DIR}/arial.ttf
    )
endif()

# --- Tests (Optional) ---
option(BUILD_TESTS "Build test executables" OFF)

if(BUILD_TESTS)
    if(WIN32)
        find_package(GTest PATHS "C:/msys64/clang64" NO_DEFAULT_PATH)
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
            target_link_libraries(LineDebuggingTest PRIVATE gtest gtest_main sfml-graphics CGAL::CGAL)
        else()
            target_link_libraries(LineDebuggingTest PRIVATE GTest::GTest GTest::Main sfml-graphics CGAL::CGAL ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
        endif()
        add_test(NAME LineDebuggingTest_CTest COMMAND LineDebuggingTest)
    endif()
endif()

if(NOT WIN32)
    # Ensure the linker can resolve transitive shared library dependencies
    target_link_options(FluxGeo PRIVATE "-Wl,-rpath-link,/lib/x86_64-linux-gnu" "-Wl,-rpath-link,/usr/lib/x86_64-linux-gnu")
endif()