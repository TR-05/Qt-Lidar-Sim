#pragma once
#include <numbers>

namespace Config {
constexpr float RoomSize = 144.0f;
constexpr int LidarAngleOffset = 180;
constexpr int ScanStartAngle = -120;
constexpr int ScanEndAngle = 85;
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
struct Result {
  float x, y, heading, confidence;
};