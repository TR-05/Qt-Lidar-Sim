#pragma once
#include <numbers>
#include <vector>

namespace Config {
constexpr float RoomSize = 144.0f;
constexpr int LidarAngleOffset = 180;
// start and end angles from lidars perspective
constexpr int ScanStartAngle = -160;
constexpr int ScanEndAngle = 50;
//constexpr int ScanStartAngle = 0;
//constexpr int ScanEndAngle = 180;
constexpr float AngularResolution = 0.72;
constexpr int ScanPoints = (ScanEndAngle - ScanStartAngle) / AngularResolution;
constexpr float PixelsPerInch = 30.0f;
constexpr float DegToRad = std::numbers::pi / 180.0;
constexpr float RadToDeg = 180.0 / std::numbers::pi;

} // namespace Config

struct Point {
  float x, y;
};
struct Obstacle {
  float x, y, radius;
  bool moving;
};
const std::vector<Obstacle> WorldObstacles = {{Config::RoomSize / 2, Config::RoomSize / 2, 3, false},
                                              {24, 48, 1, false},
                                              {24, Config::RoomSize - 48, 1, false},
                                              {Config::RoomSize - 24, 48, 1, false},
                                              {Config::RoomSize - 24, Config::RoomSize - 48, 1, false}};

struct Result {
  float x, y, heading, confidence;
};