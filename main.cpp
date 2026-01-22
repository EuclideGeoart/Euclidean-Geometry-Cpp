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
#include "cgal_error_handler.h"
#include <exception>
#include <iostream>
#include <memory>
#include <vector>

void run_exception_tests(); // Declaration for the function in
                            // test_exceptions.cpp

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

    // Run the main application
    std::cout << "Starting application..." << std::endl;
    editor.run();

    // Explicit cleanup before exit
    std::cout << "Cleaning up CGAL resources..." << std::endl;

    // Clear all geometry containers
    editor.points.clear();
    editor.lines.clear();
    editor.circles.clear();
    editor.ObjectPoints.clear();

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
