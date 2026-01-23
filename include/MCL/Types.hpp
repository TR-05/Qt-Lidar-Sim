#pragma once
#include <vector>

struct Pose {
    double x, y, theta; // inches, radians
};

struct Particle {
    Pose pose;
    double weight;
};

struct LidarData {
    double length; // distance
    double angle;  // relative to robot 0
};