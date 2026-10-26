#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace test_support {

// Tests must still check their expectations in Release builds (NDEBUG).
inline void check(bool condition, const std::string &message = "check failed") {
  if (!condition)
    throw std::runtime_error(message);
}

// Keep exception type requirements explicit at each call site.
template <typename Exception = std::exception, typename Function>
void expectThrows(Function &&function,
                  const std::string &message = "expected exception") {
  bool thrown = false;
  try {
    std::forward<Function>(function)();
  } catch (const Exception &) {
    thrown = true;
  }
  check(thrown, message);
}

} // namespace test_support
