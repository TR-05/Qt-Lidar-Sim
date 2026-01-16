#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config.hpp"

Result localize2(std::pair<std::vector<float>, std::vector<float>> data, float real_x, float real_y, float heading);


extern bool confident;
extern std::string wallsString;
extern std::vector<int> xHist;
extern std::vector<int> yHist;
extern float binWidth;


namespace LidarProcessor {
    struct Point2d {
    float x;
    float y;
};
struct Line {
    Point2d p1, p2;
};
void segment(const std::pair<std::vector<float>, std::vector<float>>& rawData, Point2d pose, float h);
void localizeFromLines(Point2d& pose);

extern std::vector<Line> finalLines;
}  // namespace LidarProcessor
