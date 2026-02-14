#include "CGALExceptions.h"
#include <iostream>

// Keep implementation as a separate function for reuse
int test_exceptions_main() {
  try {
    // Test that GeometricInconsistencyException is properly defined
    throw GeometricInconsistencyException("Test exception");
  } catch (const GeometricInconsistencyException &e) {
    std::cout << "Caught expected exception: " << e.what() << std::endl;
    return 0; // Success
  } catch (const std::exception &e) {
    std::cout << "Caught unexpected exception: " << e.what() << std::endl;
    return 1; // Error - wrong exception type
  } catch (...) {
    std::cout << "Caught unknown exception type" << std::endl;
    return 2; // Error - unknown exception
  }

  // If no exception was caught (shouldn't happen)
  std::cout << "Error: No exception was thrown" << std::endl;
  return 3;
}

// Add a function that can be called from the real main if needed
void run_exception_tests() {
  std::cout << "Running exception tests..." << std::endl;
  int result = test_exceptions_main();
  std::cout << "Exception tests completed with result: " << result << std::endl;
}

// Add back a proper main function for the standalone test executable
#ifdef TEST_EXCEPTIONS_MAIN
int main(int argc, char *argv[]) {
  std::cout << "TestExceptions standalone executable running..." << std::endl;

  int result = test_exceptions_main();

  std::cout << "TestExceptions completed with result code: " << result
            << std::endl;
  return result;
}
#endif
