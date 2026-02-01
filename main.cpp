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
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    ImFont* font = nullptr;

#if defined(_WIN32) || defined(_WIN64)
    font = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 24.0f);
    if (!font) {
      font = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf", 24.0f);
    }
#elif defined(__linux__)
    // Common Linux font locations and fallbacks
    font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24.0f);
    if (!font) {
      font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/msttcorefonts/Arial.ttf", 24.0f);
    }
    if (!font) {
      font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", 24.0f);
    }
#else
    // Unknown platform: try common Windows paths as a last resort
    font = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 24.0f);
    if (!font) {
      font = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf", 24.0f);
    }
#endif

    // Ensure we always have at least a default font to avoid an empty UI
    if (!font) {
      font = io.Fonts->AddFontDefault();
    }

    ImGui::SFML::UpdateFontTexture();
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
