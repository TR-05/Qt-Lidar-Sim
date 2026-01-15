#include <math.h>

#include <algorithm>
#include <array>
#include <vector>

#include "Config.hpp"
#include "localizor.hpp"

bool confident;
std::string wallsString;
std::vector<int> xHist;
std::vector<int> yHist;
float binWidth;

// --- Helper Structs ---
struct Point2d {
    float x;
    float y;
};

struct WallResult {
    float position;
    bool found;
};

// --- Constants ---
const float BIN_SIZE = 0.5f;
const float INV_BIN_SIZE = 2.0f;
const int HIST_SIZE = 2 * Config::RoomSize;
const int OFFSET = HIST_SIZE / 2;

// --- Internal Functions ---

float calculate_score_fast(float c, float s, const std::array<Point2d, Config::ScanPoints>& points, int step) {
    static float hist_x[HIST_SIZE];
    static float hist_y[HIST_SIZE];
    std::fill(std::begin(hist_x), std::end(hist_x), 0.0f);
    std::fill(std::begin(hist_y), std::end(hist_y), 0.0f);

    for (size_t i = 0; i < points.size(); i += step) {
        const auto& p = points[i];
        float distSq = p.x * p.x + p.y * p.y;

        // Dynamic Mask: Ignore near-field noise and robot body
        if (distSq < 324.0f) continue;

        float rx_raw = (p.x * c - p.y * s) * INV_BIN_SIZE + OFFSET;
        float ry_raw = (p.x * s + p.y * c) * INV_BIN_SIZE + OFFSET;

        // Stability Weight: Distant points define lines better
        float weight = 1.0f + (distSq * 0.0005f);

        int x0 = static_cast<int>(rx_raw);
        if (x0 > 0 && x0 < HIST_SIZE - 1) {
            float x_frac = rx_raw - (float)x0;
            hist_x[x0] += (1.0f - x_frac) * weight;
            hist_x[x0 + 1] += x_frac * weight;
        }

        int y0 = static_cast<int>(ry_raw);
        if (y0 > 0 && y0 < HIST_SIZE - 1) {
            float y_frac = ry_raw - (float)y0;
            hist_y[y0] += (1.0f - y_frac) * weight;
            hist_y[y0 + 1] += y_frac * weight;
        }
    }

    float max_x = 0, max_y = 0;
    for (int i = 0; i < HIST_SIZE; ++i) {
        if (hist_x[i] > max_x) max_x = hist_x[i];
        if (hist_y[i] > max_y) max_y = hist_y[i];
    }

    // Geometry check: penalize if we only see one axis (likely an obstacle)
    float ratio = (max_x > max_y) ? (max_y / max_x) : (max_x / max_y);
    if (ratio < 0.15f) return (max_x + max_y) * 0.5f;

    return max_x + max_y;
}

WallResult validateSingleWall(const std::vector<Point2d>& points, bool isXAxis) {
    if (points.size() < 12) return {0, false};

    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    float minCross = 1e9, maxCross = -1e9;

    for (const auto& p : points) {
        float cross = (isXAxis ? p.y : p.x);
        float depth = (isXAxis ? p.x : p.y);
        sumX += cross;
        sumY += depth;
        sumXY += (double)cross * depth;
        sumX2 += (double)cross * cross;
        if (cross < minCross) minCross = cross;
        if (cross > maxCross) maxCross = cross;
    }

    // Ensure the wall has sufficient physical span
    if (maxCross - minCross < 18.0f) return {0, false};

    double denom = (points.size() * sumX2 - sumX * sumX);
    if (fabs(denom) < 1e-6) return {0, false};

    float slope = (points.size() * sumXY - sumX * sumY) / denom;
    float intercept = (sumY - slope * sumX) / points.size();

    // Line Quality Checks
    if (fabs(slope) > 0.25f) return {0, false};

    return {intercept, true};
}

// --- Main Localization ---

Result localize2(std::pair<std::vector<float>, std::vector<float>> data, float heading) {
    static float lastX = 72.0f, lastY = 72.0f;
    static bool initialized = false;

    // 1. Initial Translation to Cartesian using Fuzzy Heading (+20 bias)
    std::array<Point2d, Config::ScanPoints> points;
    float fuzzyHeading = heading + 20.0f;
    for (size_t i = 0; i < data.first.size(); i++) {
        float a = (data.second[i] + fuzzyHeading) * Config::DegToRad;
        points[i] = {data.first[i] * cosf(a), data.first[i] * sinf(a)};
    }

    // 2. Multi-Pass Heading Search
    float best_offset = 0, max_score = -1.0f;

    // Coarse Search (Low Res / Fast)
    for (float a = -45.0f; a <= 45.0f; a += 1.5f) {
        float score = calculate_score_fast(cosf(a * Config::DegToRad), sinf(a * Config::DegToRad), points, 4);
        if (score > max_score) {
            max_score = score;
            best_offset = a;
        }
    }

    // Fine Search (High Res / Full Points)
    for (float a = best_offset - 2.0f; a <= best_offset + 2.0f; a += 0.05f) {
        float score = calculate_score_fast(cosf(a * Config::DegToRad), sinf(a * Config::DegToRad), points, 1);
        if (score > max_score) {
            max_score = score;
            best_offset = a;
        }
    }

    float correctedHeading = fuzzyHeading + best_offset;

    // 3. Final Rotation for Coordinate Extraction
    for (size_t i = 0; i < data.first.size(); i++) {
        float a = (data.second[i] + correctedHeading) * Config::DegToRad;
        points[i] = {data.first[i] * cosf(a), data.first[i] * sinf(a)};
    }

    // 4. Wall Isolation
    std::vector<Point2d> L, R, B, T;
    float minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (auto& p : points) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }

    for (auto& p : points) {
        if (p.x < minX + 4.0f) L.push_back(p);
        if (p.x > maxX - 4.0f) R.push_back(p);
        if (p.y < minY + 4.0f) B.push_back(p);
        if (p.y > maxY - 4.0f) T.push_back(p);
    }

    WallResult resL = validateSingleWall(L, true), resR = validateSingleWall(R, true);
    WallResult resB = validateSingleWall(B, false), resT = validateSingleWall(T, false);

    float curX = lastX, curY = lastY;

    // Determine X relative to Global Frame
    if (resL.found)
        curX = fabs(resL.position);
    else if (resR.found)
        curX = Config::RoomSize - resR.position;

    // Determine Y relative to Global Frame
    if (resB.found)
        curY = fabs(resB.position);
    else if (resT.found)
        curY = Config::RoomSize - resT.position;

    // 5. Jump Guard: Detect and discard "Mirroring" or "Teleporting"
    if (initialized) {
        float deltaX = fabs(curX - lastX);
        float deltaY = fabs(curY - lastY);
        // If we move > 10 inches in 16ms, it's a sensor flip/bad heading match
        if (deltaX > 10.0f || deltaY > 10.0f) {
            curX = lastX;
            curY = lastY;
        }
    }

    lastX = curX;
    lastY = curY;
    initialized = true;

    // Output formatted for the log monitoring
    printf("X: %6.2f, Y: %6.2f, H: %7.2f\n", curX, curY, heading - correctedHeading);

    return {curX, curY, correctedHeading, max_score};
}