#pragma message("--- In main.cpp: Checking CGAL preprocessor flags ---")

#ifdef CGAL_HAS_THREADS
#pragma message("main.cpp: CGAL_HAS_THREADS is DEFINED before any action.")
#else
#pragma message("main.cpp: CGAL_HAS_THREADS is NOT DEFINED before any action.")
#endif

#ifdef CGAL_USE_SSE2
#pragma message("main.cpp: CGAL_USE_SSE2 is DEFINED before any action.")
#else
#pragma message("main.cpp: CGAL_USE_SSE2 is NOT DEFINED before any action.")
#endif

// Attempt to set the desired state
#define CGAL_HAS_THREADS 1
#undef CGAL_USE_SSE2
#pragma message(                                                               \
    "main.cpp: Action: Defined CGAL_HAS_THREADS, Undefined CGAL_USE_SSE2.")

// Verify after action
#ifdef CGAL_HAS_THREADS
#pragma message("main.cpp: CGAL_HAS_THREADS is DEFINED after action.")
#else
#pragma message(                                                               \
    "main.cpp: CGAL_HAS_THREADS is NOT DEFINED after action. (Problem!)")
#endif

#ifdef CGAL_USE_SSE2
#pragma message(                                                               \
    "main.cpp: CGAL_USE_SSE2 is STILL DEFINED after action. (Problem!)")
#else
#pragma message("main.cpp: CGAL_USE_SSE2 is UNDEFINED after action. (Good)")
#endif

#pragma message("--- End of preprocessor checks in main.cpp ---")

#include "CGALSafeUtils.h"
#include "CharTraitsFix.h"
#include "GMP_exception_handler.h"
#include "GeometryEditor.h"
#include "HandleEvents.h"
#include <imgui.h>
#include <imgui-SFML.h>
#include "cgal_error_handler.h"
#include <exception>
#include <iostream>
#include <memory>
#include <vector>
#include <filesystem>

void run_exception_tests(); // Declaration for the function in
                            // test_exceptions.cpp

static void setupImGuiStyle() {
  ImGuiStyle &style = ImGui::GetStyle();
  ImGui::StyleColorsDark();
  style.WindowRounding = 0.0f;
  style.FrameRounding = 0.0f;
  style.GrabRounding = 0.0f;
  style.PopupRounding = 0.0f;
  style.ScrollbarRounding = 0.0f;
}

int main() {
  // Install our custom CGAL error handlers that throw exceptions instead of
  // aborting the program
  install_custom_cgal_error_handlers();

#ifdef RUN_EXCEPTION_TESTS
  // Run the exception tests if the flag is defined
  run_exception_tests();
#endif
  try {
    std::cout << "Initializing CGAL..." << std::endl;

    // Test basic CGAL functionality
    try {
      Point_2 testPoint(0, 0);
      Vector_2 testVector(1, 0);
      std::cout << "CGAL initialization successful" << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "CGAL initialization failed: " << e.what() << std::endl;
      return 1;
    }

    // Create the editor instance here
    std::cout << "Creating GeometryEditor..." << std::endl;
    GeometryEditor editor;

    std::cout << "Starting application..." << std::endl;

    ImGui::SFML::Init(editor.window);
    setupImGuiStyle();

     // --- FONT LOADING (ROBUST WINDOWS FIX) ---
  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->Clear();

  // 1. Base Font: Segoe UI (Standard Windows UI Font)
  // We use this for readable English text.
  ImFontConfig baseConfig;
  ImFont* baseFont = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 20.0f, &baseConfig);
  if (!baseFont) {
      baseFont = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf", 18.0f);
  }

  // 2. Symbol Font: Segoe UI Symbol (Contains Math, Greek, Geometric shapes)
  // We MERGE this into the base font so they can be used together.
  ImFontConfig symbolConfig;
  symbolConfig.MergeMode = true; 
  symbolConfig.PixelSnapH = true;
  
  // Define extensive ranges for Math, Greek, and Arrows
  static const ImWchar icon_ranges[] = {
      0x0020, 0x00FF, // Basic Latin + Latin Supplement
      0x0370, 0x03FF, // Greek
      0x2000, 0x206F, // General Punctuation
      0x2070, 0x209F, // Superscripts and Subscripts
      0x20A0, 0x20CF, // Currency Symbols
      0x2100, 0x214F, // Letterlike Symbols
      0x2150, 0x218F, // Number Forms
      0x2190, 0x21FF, // Arrows
      0x2200, 0x22FF, // Mathematical Operators (KEY FOR GEOMETRY)
      0x2300, 0x23FF, // Miscellaneous Technical
      0x2500, 0x257F, // Box Drawing
      0x25A0, 0x25FF, // Geometric Shapes
      0,
  };

  // Try loading Segoe UI Symbol, fallback to Arial if missing
  if (std::filesystem::exists("C:\\Windows\\Fonts\\seguisym.ttf")) {
      io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\seguisym.ttf", 20.0f, &symbolConfig, icon_ranges);
      std::cout << "Loaded Segoe UI Symbol for robust character support." << std::endl;
  } else {
      std::cout << "Segoe UI Symbol not found, falling back to Arial for symbols." << std::endl;
      io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf", 18.0f, &symbolConfig, icon_ranges);
  }

  // 3. Build Texture
  ImGui::SFML::UpdateFontTexture(); 
  
  // Set default font for Geometry Tool buttons too
  Button::loadFont();
    io.FontGlobalScale = 1.0f;

    sf::Clock deltaClock;
    while (editor.window.isOpen()) {
      float dt = deltaClock.restart().asSeconds();

      // Process events
      try {
        handleEvents(editor);
      } catch (const CGAL::Uncertain_conversion_exception &e) {
        std::cerr << "main: Caught Uncertain_conversion_exception during event handling: "
                  << e.what() << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "main: Caught exception during event handling: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "main: Caught unknown exception during event handling" << std::endl;
      }

      ImGui::SFML::Update(editor.window, sf::seconds(dt));

      // Update
      try {
        editor.update(sf::seconds(dt));
      } catch (const CGAL::Uncertain_conversion_exception &e) {
        std::cerr << "main: Caught Uncertain_conversion_exception during update: "
                  << e.what() << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "main: Caught exception during update: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "main: Caught unknown exception during update" << std::endl;
      }

      // Render
      try {
        editor.render();
      } catch (const CGAL::Uncertain_conversion_exception &e) {
        std::cerr << "main: Caught Uncertain_conversion_exception during rendering: "
                  << e.what() << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "main: Caught exception during rendering: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "main: Caught unknown exception during rendering" << std::endl;
      }
    }

    ImGui::SFML::Shutdown();

    // Explicit cleanup before exit
    std::cout << "Cleaning up CGAL resources..." << std::endl;

    // GeometryEditor destructor will handle cleanup safely

    std::cout << "Application exiting cleanly." << std::endl;
    return 0;

  } catch (const std::exception &e) {
    std::cerr << "Exception in main: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Unknown fatal error occurred" << std::endl;
    return 2;
  }
}
