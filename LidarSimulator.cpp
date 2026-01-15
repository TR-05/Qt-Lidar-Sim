#include "LidarSimulator.h"
#include "Config.h"
#include "config.h"
#include <algorithm>
#include <cmath>

LidarSimulator::LidarSimulator() : gen(std::random_device{}()) {}

std::pair<std::vector<float>, std::vector<float>>
LidarSimulator::getScan(float tx, float ty, float heading,
                        const std::vector<Obstacle> &obstacles) {
  std::vector<float> distances(Config::ScanPoints);
  std::vector<float> headings(Config::ScanPoints);

  std::normal_distribution<float> noise(0.0, 0.4);

  for (int i = 0; i < Config::ScanPoints; ++i) {
    float angleRad =
        (Config::ScanStartAngle + i * Config::AngularResolution + heading + Config::LidarAngleOffset) *
        Config::DegToRad;
        angleRad *= -1; // convert to CW rotation
    float dx = std::cos(angleRad);
    float dy = std::sin(angleRad);

    // --- Wall Intersection ---
    float tMin = 1e6;
    if (dx > 0)
      tMin = std::min(tMin, (Config::RoomSize - tx) / dx);
    else if (dx < 0)
      tMin = std::min(tMin, -tx / dx);
    if (dy > 0)
      tMin = std::min(tMin, (Config::RoomSize - ty) / dy);
    else if (dy < 0)
      tMin = std::min(tMin, -ty / dy);

    // --- Circle Obstacle Intersection ---
    for (const auto &obs : obstacles) {
      float ocX = tx - obs.x;
      float ocY = ty - obs.y;
      float b = ocX * dx + ocY * dy;
      float c = (ocX * ocX + ocY * ocY) - (obs.radius * obs.radius);
      float discriminant = b * b - c;

      if (discriminant > 0) {
        float t = -b - std::sqrt(discriminant);
        if (t > 0 && t < tMin)
          tMin = t;
      }
    }

    distances[i] = tMin + noise(gen) * 1;
    headings[i] = angleRad * Config::RadToDeg - heading;
  }
  return {distances, headings};
}