#include <qmath.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "MCL/Types.hpp"
#include "config.hpp"

Pose true_pose;

std::mt19937 gen(std::random_device{}());

std::vector<LidarData> lidar_get_scan() {
    std::vector<LidarData> data(Config::ScanPoints);

    std::normal_distribution<double> noise(0.0, 0.4);

    //for (int i = 0; i < Config::ScanPoints; ++i) {
    for (int i = 0; i < Config::ScanPoints; i++) {

        float ang = (- i * Config::AngularResolution) * Config::DegToRad;
        ang += Config::DegToRad * ( - true_pose.theta - Config::LidarAngleOffset - Config::ScanStartAngle);
        
        double dx = std::cos(ang);
        double dy = std::sin(ang);
        if (std::abs(dx) < 1e-6) dx = 1e-6;
        if (std::abs(dy) < 1e-6) dy = 1e-6;

        // --- Wall Intersection ---
        double tMin = 1e6;
        if (dx > 0)
            tMin = std::min(tMin, (Config::RoomSize - true_pose.x) / dx);
        else if (dx < 0)
            tMin = std::min(tMin, -true_pose.x / dx);
        if (dy > 0)
            tMin = std::min(tMin, (Config::RoomSize - true_pose.y) / dy);
        else if (dy < 0)
            tMin = std::min(tMin, -true_pose.y / dy);

        // --- Circle Obstacle Intersection ---
        for (const auto& obs : WorldObstacles) {
            float ocX = true_pose.x - obs.x;
            float ocY = true_pose.y - obs.y;
            float b = ocX * dx + ocY * dy;
            float c = (ocX * ocX + ocY * ocY) - (obs.radius * obs.radius);
            float discriminant = b * b - c;

            if (discriminant > 0) {
                float t = -b - std::sqrt(discriminant);
                if (t > 0 && t < tMin) tMin = t;
            }
        }
        data[i] = {tMin + noise(gen)*1.2, ang};
    }
    return data;
}