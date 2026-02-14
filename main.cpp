/**
 * ============================================================================
 * FluxGeo Geometry Engine
 * ============================================================================
 *
 * Created by: Mario Balit
 * Assisted by: AI Coding Tools
 * Year: 2026
 *
 * Description: An advanced, GeoGebra-style geometric construction 
 * and transformation editor built with C++, SFML, and ImGui.
 * ============================================================================
 */

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

#include "Benchmark.h"
#include "CGALSafeUtils.h"
#include "CharTraitsFix.h"
#include "GMP_exception_handler.h"
#include "GeometryEditor.h"
#include "HandleEvents.h" 
#include "cgal_error_handler.h"
#include <exception>
#include <filesystem>
#include <imgui-SFML.h>
#include <imgui.h>
#include <iostream>
#include <memory>
#include <vector>

void run_exception_tests(); 

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
  // Install our custom CGAL error handlers
  install_custom_cgal_error_handlers();

  Benchmark benchmark;

#ifdef RUN_EXCEPTION_TESTS
  run_exception_tests();
#endif
  try {
    std::cout << "Initializing CGAL..." << std::endl;

    try {
      Point_2 testPoint(0, 0);
      Vector_2 testVector(1, 0);
      std::cout << "CGAL initialization successful" << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "CGAL initialization failed: " << e.what() << std::endl;
      return 1;
    }

    std::cout << "Creating GeometryEditor..." << std::endl;
    GeometryEditor editor;

    std::cout << "Starting application..." << std::endl;

    ImGui::SFML::Init(editor.window);
    setupImGuiStyle();

    // --- FONT LOADING ---
    ImGuiIO &io = ImGui::GetIO();
    io.Fonts->Clear();

    // 1. Base Font
    ImFontConfig baseConfig;
    ImFont *baseFont = nullptr;
#if defined(_WIN32) || defined(_WIN64)
    try {
      if (std::filesystem::exists("C:/Windows/Fonts/segoeui.ttf")) {
        baseFont = io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 20.0f, &baseConfig);
      }
    } catch (...) {
      baseFont = nullptr;
    }
    if (!baseFont) {
      try {
        if (std::filesystem::exists("C:/Windows/Fonts/arial.ttf")) {
          baseFont = io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/arial.ttf", 18.0f);
        }
      } catch (...) {
        baseFont = nullptr;
      }
    }
#else
    // Try common Linux font paths
    if (std::filesystem::exists("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
      baseFont = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18.0f, &baseConfig);
    }
    if (!baseFont && std::filesystem::exists("/usr/share/fonts/truetype/freefont/FreeSans.ttf")) {
      baseFont = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 18.0f, &baseConfig);
    }
    if (!baseFont) {
      std::cerr << "Warning: Could not load a system font. Using ImGui default font." << std::endl;
      baseFont = io.Fonts->AddFontDefault();
    }
#endif

    // 2. Symbol Font
    ImFontConfig symbolConfig;
    symbolConfig.MergeMode = true;
    symbolConfig.PixelSnapH = true;

    static const ImWchar icon_ranges[] = {
        0x0020, 0x00FF, // Basic Latin + Latin Supplement
        0x0370, 0x03FF, // Greek
        0x2000, 0x206F, // General Punctuation
        0x2070, 0x209F, // Superscripts and Subscripts
        0x20A0, 0x20CF, // Currency Symbols
        0x2100, 0x214F, // Letterlike Symbols
        0x2150, 0x218F, // Number Forms
        0x2190, 0x21FF, // Arrows
        0x2200, 0x22FF, // Mathematical Operators
        0x2300, 0x23FF, // Miscellaneous Technical
        0x2500, 0x257F, // Box Drawing
        0x25A0, 0x25FF, // Geometric Shapes
        0,
    };

    // Try a list of candidate symbol fonts depending on platform. Only call
    // AddFontFromFileTTF for paths that actually exist to avoid ImGui asserts
    // when the file cannot be opened.
    std::vector<std::string> symbolCandidates;
#if defined(_WIN32) || defined(_WIN64)
    symbolCandidates = {"C:\\Windows\\Fonts\\seguisym.ttf", "C:\\Windows\\Fonts\\arial.ttf"};
#else
    symbolCandidates = {
        "/usr/share/fonts/truetype/seguisym/seguisym.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
    };
#endif

    bool loadedSymbol = false;
    for (const auto &path : symbolCandidates) {
      try {
        if (!std::filesystem::exists(path)) continue;
      } catch (...) {
        continue;
      }
      ImFont *sym = io.Fonts->AddFontFromFileTTF(path.c_str(), 18.0f, &symbolConfig, icon_ranges);
      if (sym) {
        std::cout << "Loaded symbol font for robust character support: " << path << std::endl;
        loadedSymbol = true;
        break;
      }
    }
    if (!loadedSymbol) {
      std::cout << "No symbol font found; skipping symbol merge." << std::endl;
    }

    ImGui::SFML::UpdateFontTexture();
    Button::loadFont();
    io.FontGlobalScale = 1.0f;

    sf::Clock deltaClock;
    sf::Clock logicTimer;
    
    // --- INPUT DEBOUNCING FLAGS ---
    // These prevent the stress test from running 60 times a second
    bool f5PressedLastFrame = false;
    bool f6PressedLastFrame = false;

    while (editor.window.isOpen()) {
      float dt = deltaClock.restart().asSeconds();

      // 1. SAFE BENCHMARK TRIGGER (F5)
      // We check the key state manually here, but enforce "Single Shot" logic
      bool f5Pressed = sf::Keyboard::isKeyPressed(sf::Keyboard::F5);
      if (f5Pressed && !f5PressedLastFrame) {
          // Key just went down - trigger ONCE
          benchmark.spawnStressTest(editor);
      }
      f5PressedLastFrame = f5Pressed; // Remember state for next frame

      // 2. SAFE CLEAR TRIGGER (F6)
      bool f6Pressed = sf::Keyboard::isKeyPressed(sf::Keyboard::F6);
      if (f6Pressed && !f6PressedLastFrame) {
          benchmark.clearStressTest(editor);
      }
      f6PressedLastFrame = f6Pressed;

      // 3. RESTORED EVENT HANDLING (Linker Fix)
      // This function handles the poll loop internally, so drawing works again.
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
      sf::Time logicTime;
      try {
        logicTimer.restart();
        editor.update(sf::seconds(dt));
        logicTime = logicTimer.getElapsedTime();
      } catch (const CGAL::Uncertain_conversion_exception &e) {
        std::cerr << "main: Caught Uncertain_conversion_exception during update: "
                  << e.what() << std::endl;
        logicTime = sf::Time::Zero;
      } catch (const std::exception &e) {
        std::cerr << "main: Caught exception during update: " << e.what()
                  << std::endl;
        logicTime = sf::Time::Zero;
      } catch (...) {
        std::cerr << "main: Caught unknown exception during update" << std::endl;
        logicTime = sf::Time::Zero;
      }

      // Render
      try {
        editor.render();
        // Step 4: Draw performance HUD after rendering
        benchmark.renderHUD(editor.window, editor, logicTime);
      } catch (const CGAL::Uncertain_conversion_exception &e) {
        std::cerr
            << "main: Caught Uncertain_conversion_exception during rendering: "
            << e.what() << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "main: Caught exception during rendering: " << e.what()
                  << std::endl;
      } catch (...) {
        std::cerr << "main: Caught unknown exception during rendering"
                  << std::endl;
      }
    }

    ImGui::SFML::Shutdown();
    std::cout << "Cleaning up CGAL resources..." << std::endl;
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