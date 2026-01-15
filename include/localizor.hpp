#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config.hpp"

Result localize2(std::pair<std::vector<float>, std::vector<float>> data, float heading);

extern bool confident;
extern std::string wallsString;
extern std::vector<int> xHist;
extern std::vector<int> yHist;
extern float binWidth;

