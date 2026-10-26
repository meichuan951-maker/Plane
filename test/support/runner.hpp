#pragma once

#include <exception>
#include <iostream>
#include <utility>

namespace test_support {

// Small shared runner for suites with named cases; no external test framework.
class Runner {
public:
  template <typename Function>
  void operator()(const char *name, Function &&function) {
    try {
      std::forward<Function>(function)();
      ++passed_;
      std::cout << "PASS " << name << '\n';
    } catch (const std::exception &error) {
      ++failed_;
      std::cerr << "FAIL " << name << ": " << error.what() << '\n';
    }
  }

  int finish() const {
    std::cout << passed_ << " passed, " << failed_ << " failed\n";
    return failed_ ? 1 : 0;
  }

private:
  int passed_ = 0;
  int failed_ = 0;
};

} // namespace test_support
