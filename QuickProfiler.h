#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <unordered_map>


class QuickProfiler {
private:
  static std::unordered_map<std::string,
                            std::chrono::high_resolution_clock::time_point>
      timers;
  static std::unordered_map<std::string, double> totals;
  static std::unordered_map<std::string, int> counts;

public:
  static void start(const std::string &name);
  static void end(const std::string &name);
  static void report();
};

#define QUICK_PROFILE(name)                                                    \
  QuickProfiler::start(name);                                                  \
  auto _qp_##__LINE__ = [=]() { QuickProfiler::end(name); };                   \
  std::shared_ptr<void> _qp_guard_##__LINE__(                                  \
      nullptr, [=](void *) { QuickProfiler::end(name); });
