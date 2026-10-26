#include "demo_aim/detail/app/common/parse.hpp"
#include <stdexcept>

namespace aim::app {

int parseInteger(const std::string &value, const std::string &error_prefix) {
  std::size_t consumed = 0;
  const int result = std::stoi(value, &consumed);
  if (consumed != value.size())
    throw std::invalid_argument(error_prefix + value);
  return result;
}

} // namespace aim::app
