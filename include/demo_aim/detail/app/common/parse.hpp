#pragma once

#include <string>

namespace aim::app {

// std::stoi accepts trailing characters; CLI arguments must consume the value.
int parseInteger(const std::string &value,
                 const std::string &error_prefix = "Invalid integer: ");

} // namespace aim::app
