#pragma once
#include "demo_aim/armor_auto.hpp"
namespace aim {
int calibrateArmorCamera(const ArmorAutoConfig &config, int columns, int rows,
                         double square_mm, const std::string &output);
}
