#pragma once
#include "Config.h"
#include <vector>
#include <random>

class LidarSimulator {
public:
    LidarSimulator();
    std::pair<std::vector<float>, std::vector<float>> getScan(float tx, float ty, float heading, const std::vector<Obstacle>& obstacles);

private:
    std::mt19937 gen;
};