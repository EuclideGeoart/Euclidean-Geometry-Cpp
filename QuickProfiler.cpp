#include "QuickProfiler.h"

// Static member definitions (only once in .cpp file)
std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point>
    QuickProfiler::timers;
std::unordered_map<std::string, double> QuickProfiler::totals;
std::unordered_map<std::string, int> QuickProfiler::counts;

void QuickProfiler::start(const std::string &name) {
  timers[name] = std::chrono::high_resolution_clock::now();
}

void QuickProfiler::end(const std::string &name) {
  auto it = timers.find(name);
  if (it != timers.end()) {
    auto duration = std::chrono::high_resolution_clock::now() - it->second;
    double ms = std::chrono::duration<double, std::milli>(duration).count();
    totals[name] += ms;
    counts[name]++;

  }
}

void QuickProfiler::report() {
  std::cout << "\n=== PERFORMANCE SUMMARY ===" << std::endl;
  for (const auto &pair : totals) {
    std::cout << pair.first << ": " << pair.second << "ms total, "
              << (pair.second / counts[pair.first]) << "ms avg, "
              << counts[pair.first] << " calls" << std::endl;
  }
}