#pragma once

#include <stdexcept>
#include <string>

namespace aim::app {

// std::stoi accepts trailing characters; CLI arguments must consume the value.
inline int parseInteger(const std::string &value,
                        const std::string &error_prefix = "Invalid integer: ") {
  std::size_t consumed = 0;
  const int result = std::stoi(value, &consumed);
  if (consumed != value.size())
    throw std::invalid_argument(error_prefix + value);
  return result;
}

} // namespace aim::app
