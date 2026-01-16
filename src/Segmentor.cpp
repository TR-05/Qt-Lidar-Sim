#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <span>
#include <vector>

#include "config.hpp"

// Use a namespace to prevent global scope pollution
namespace LidarProcessor {

struct Point2d {
    float x;
    float y;
};

struct Line {
    Point2d p1, p2;
    [[nodiscard]] Point2d direction() const noexcept { return {p2.x - p1.x, p2.y - p1.y}; }
};

// --- Geometry Helpers ---

[[nodiscard]] float getLineAngle(const Line& l) noexcept { return std::atan2(l.p2.y - l.p1.y, l.p2.x - l.p1.x) * (180.0f / std::numbers::pi_v<float>); }

[[nodiscard]] float getLineLength(const Line& l) noexcept { return std::hypot(l.p2.x - l.p1.x, l.p2.y - l.p1.y); }

void setLineAngle(Line& l, float targetAngleDeg) noexcept {
    const float length = getLineLength(l);
    const float rad = targetAngleDeg * (std::numbers::pi_v<float> / 180.0f);
    l.p2 = {l.p1.x + length * std::cos(rad), l.p1.y + length * std::sin(rad)};
}

[[nodiscard]] float distToLine(Point2d p, Point2d a, Point2d b) noexcept {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float magSq = dx * dx + dy * dy;
    if (magSq < 1e-6f) return std::hypot(p.x - a.x, p.y - a.y);
    return std::abs(dy * p.x - dx * p.y + b.x * a.y - b.y * a.x) / std::sqrt(magSq);
}

bool getIntersection(const Line& l1, const Line& l2, Point2d& outIntersect) noexcept {
    const auto [x1, y1] = l1.p1;
    const auto [x2, y2] = l1.p2;
    const auto [x3, y3] = l2.p1;
    const auto [x4, y4] = l2.p2;

    const float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(denom) < 1e-5f) return false;

    outIntersect.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
    outIntersect.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
    return true;
}

// --- Core Algorithms ---

Line fitBestLine(std::span<const Point2d> points) {
    if (points.empty()) return {{0, 0}, {0, 0}};
    if (points.size() < 3) return {points.front(), points.back()};

    float sumX = 0, sumY = 0;
    for (const auto& p : points) {
        sumX += p.x;
        sumY += p.y;
    }
    const float centerX = sumX / points.size();
    const float centerY = sumY / points.size();

    float sXX = 0, sYY = 0, sXY = 0;
    for (const auto& p : points) {
        const float dx = p.x - centerX;
        const float dy = p.y - centerY;
        sXX += dx * dx;
        sYY += dy * dy;
        sXY += dx * dy;
    }

    const float angle = 0.5f * std::atan2(2.0f * sXY, sXX - sYY);
    const float dirX = std::cos(angle), dirY = std::sin(angle);

    auto project = [&](Point2d p) -> Point2d {
        const float t = (p.x - centerX) * dirX + (p.y - centerY) * dirY;
        return {centerX + t * dirX, centerY + t * dirY};
    };

    return {project(points.front()), project(points.back())};
}

void splitAndMerge(std::span<const Point2d> points, float threshold, std::vector<Line>& outLines) {
    if (points.size() < 2) return;

    float maxDist = 0;
    size_t index = 0;
    for (size_t i = 1; i < points.size() - 1; ++i) {
        float d = distToLine(points[i], points.front(), points.back());
        if (d > maxDist) {
            maxDist = d;
            index = i;
        }
    }

    if (maxDist > threshold) {
        splitAndMerge(points.subspan(0, index + 1), threshold, outLines);
        splitAndMerge(points.subspan(index), threshold, outLines);
    } else {
        outLines.push_back(fitBestLine(points));
    }
}

// --- Main Segmentation Function ---

std::vector<Line> finalLines;
std::vector<Line> correctedFinalLines;
const float maxJumpSq = std::pow(4.0f, 2);

void segment(const std::pair<std::vector<float>, std::vector<float>>& rawData, Point2d pose, float h) {
    if (rawData.first.empty()) return;

    // 1. Clean & Normalize
    struct RawPoint {
        float dist, angle;
    };
    std::vector<RawPoint> cleanData;
    cleanData.reserve(rawData.first.size());
    for (size_t i = 0; i < rawData.first.size(); ++i) {
        float a = std::fmod(rawData.second[i], 360.0f);
        if (a < 0) a += 360.0f;
        cleanData.push_back({rawData.first[i], a});
    }

    // 2. Sort by angle
    std::ranges::sort(cleanData, {}, &RawPoint::angle);

    // 3. Euclidean Clustering
    std::vector<std::vector<Point2d>> clusters(1);

    for (size_t i = 0; i < cleanData.size(); ++i) {
        const float rad = (cleanData[i].angle + h) * Config::DegToRad;
        const Point2d currentPt{cleanData[i].dist * std::cos(rad), cleanData[i].dist * std::sin(rad)};

        if (!clusters.back().empty()) {
            const auto& prevPt = clusters.back().back();
            const float actualDistSq = std::pow(currentPt.x - prevPt.x, 2) + std::pow(currentPt.y - prevPt.y, 2);
            const float radialDiff = std::abs(cleanData[i].dist - cleanData[i - 1].dist);

            if (actualDistSq > maxJumpSq || radialDiff > 10.0f) clusters.emplace_back();
        }
        clusters.back().push_back(currentPt);
    }

    // 4. Wrap-around Merge
    if (clusters.size() > 1) {
        const float wrapDistSq = std::pow(clusters.front().front().x - clusters.back().back().x, 2) + std::pow(clusters.front().front().y - clusters.back().back().y, 2);
        if (wrapDistSq < maxJumpSq) {
            clusters.front().insert(clusters.front().begin(), clusters.back().begin(), clusters.back().end());
            clusters.pop_back();
        }
    }

    // 5. Split & Merge
    finalLines.clear();
    for (const auto& cluster : clusters) {
        if (cluster.size() >= 2) splitAndMerge(cluster, 1.5f, finalLines);
    }

    // 6. Collinear Pass 1 (Rough)
    auto mergePass = [](std::vector<Line>& lines, float angleThresh, float latThresh, float gapThreshSq) {
        for (size_t i = 0; i + 1 < lines.size();) {
            float diff = std::abs(getLineAngle(lines[i]) - getLineAngle(lines[i + 1]));
            while (diff > 180) diff = std::abs(diff - 180);

            if (diff < angleThresh && distToLine(lines[i + 1].p1, lines[i].p1, lines[i].p2) < latThresh &&
                (std::pow(lines[i + 1].p1.x - lines[i].p2.x, 2) + std::pow(lines[i + 1].p1.y - lines[i].p2.y, 2)) < gapThreshSq) {
                lines[i].p2 = lines[i + 1].p2;
                lines.erase(lines.begin() + i + 1);
            } else
                i++;
        }
    };
    mergePass(finalLines, 5.0f, 3.0f, 100.0f);

    // 7. Robust Orthogonal Snapping
    for (size_t i = 0; i + 1 < finalLines.size(); ++i) {
        Line &lineA = finalLines[i], &lineB = finalLines[i + 1];
        float diff = std::abs(getLineAngle(lineA) - getLineAngle(lineB));
        while (diff > 180) diff = std::abs(diff - 180);

        if (std::abs(diff - 90.0f) < 10.0f) {
            Point2d origP1 = lineA.p1;
            if (getLineLength(lineA) >= getLineLength(lineB))
                setLineAngle(lineB, getLineAngle(lineA) + 90.0f);
            else {
                setLineAngle(lineA, getLineAngle(lineB) - 90.0f);  // Simplification of the P1 logic
            }
            if (std::hypot(lineA.p1.x - origP1.x, lineA.p1.y - origP1.y) > 5.0f) lineA.p1 = origP1;
        }

        Point2d intersect;
        if (getIntersection(lineA, lineB, intersect)) {
            if (std::hypot(lineA.p2.x - lineB.p1.x, lineA.p2.y - lineB.p1.y) < 3.0f && std::hypot(lineA.p2.x - intersect.x, lineA.p2.y - intersect.y) < 5.0f) {
                lineA.p2 = lineB.p1 = intersect;
            }
        }
    }

    // 8. Clamp & Filter Noise
    finalLines.erase(std::ranges::remove_if(finalLines, [](const Line& l) { return getLineLength(l) < 10.0f; }).begin(), finalLines.end());

    // 9. Final Weighted Fusion
    mergePass(finalLines, 5.0f, 4.0f, 500.0f);

    // Output results
    // std::cout << "--- Frame Segments: " << finalLines.size() << " ---\n";
}

float normalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}
void localizeFromLines(Point2d& pose, float imuHeading) {
    if (LidarProcessor::finalLines.empty()) return;

    // --- 1. Calculate Rotation Offset ---
    float totalErrorWeighted = 0;
    float totalLength = 0;

    for (const auto& l : LidarProcessor::finalLines) {
        float dx = l.p2.x - l.p1.x;
        float dy = l.p2.y - l.p1.y;
        float len = std::hypot(dx, dy);
        if (len < 8.0f) continue;

        float ang = LidarProcessor::getLineAngle(l);
        float absA = std::abs(ang);
        float error = 0;
        bool isWall = false;

        if (absA < 20.0f || absA > 160.0f) {  // Horizontal
            float target = (absA < 45.0f) ? 0.0f : (ang > 0 ? 180.0f : -180.0f);
            error = normalizeAngle(ang - target);
            isWall = true;
        } else if (absA > 70.0f && absA < 110.0f) {  // Vertical
            float target = (ang > 0) ? 90.0f : -90.0f;
            error = normalizeAngle(ang - target);
            isWall = true;
        }

        if (isWall) {
            totalErrorWeighted += error * len;
            totalLength += len;
        }
    }

    float measuredRoomAngle = (totalLength > 0) ? (totalErrorWeighted / totalLength) : 0;

    // --- 2. Rotate finalLines to align with Global Grid ---
    // We rotate by NEGATIVE measuredRoomAngle to "un-tilt" the room
    float rad = -measuredRoomAngle * (M_PI / 180.0f);
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);

    correctedFinalLines = finalLines;
    for (auto& l : LidarProcessor::correctedFinalLines) {
        // Rotate P1
        float x1 = l.p1.x;
        float y1 = l.p1.y;
        l.p1.x = x1 * cosA - y1 * sinA;
        l.p1.y = x1 * sinA + y1 * cosA;

        // Rotate P2
        float x2 = l.p2.x;
        float y2 = l.p2.y;
        l.p2.x = x2 * cosA - y2 * sinA;
        l.p2.y = x2 * sinA + y2 * cosA;
    }

    // --- 3. Bucket the NOW-ROTATED lines ---
    struct WallData {
        float dist;
        float length;
    };
    std::vector<WallData> top, bottom, left, right;
    for (const auto& l : LidarProcessor::correctedFinalLines) {
        float dx = l.p2.x - l.p1.x;
        float dy = l.p2.y - l.p1.y;
        float len = std::hypot(dx, dy);
        if (len < 8.0f) continue;

        float midX = (l.p1.x + l.p2.x) / 2.0f;
        float midY = (l.p1.y + l.p2.y) / 2.0f;

        // After rotation, we check orientation purely by the new delta values
        if (std::abs(dy) < std::abs(dx)) {  // Horizontal-ish
            if (midY > 0)
                top.push_back({midY, len});
            else
                bottom.push_back({midY, len});
        } else {  // Vertical-ish
            if (midX > 0)
                right.push_back({midX, len});
            else
                left.push_back({midX, len});
        }
    }

    // --- 4. Solve Axis (same logic as before) ---
    auto solveAxis = [&](std::vector<WallData>& lowWalls, std::vector<WallData>& highWalls, float& currentCoord, float maxRoom) {
        float weightedLow = 0, weightL = 0;
        for (auto& w : lowWalls) {
            weightedLow += w.dist * w.length;
            weightL += w.length;
        }
        if (weightL > 0) weightedLow /= weightL;

        float weightedHigh = 0, weightH = 0;
        for (auto& w : highWalls) {
            weightedHigh += w.dist * w.length;
            weightH += w.length;
        }
        if (weightH > 0) weightedHigh /= weightH;

        float targetCoord = currentCoord;
        if (weightL > 0 && weightH > 0) {
            if (std::abs(std::abs(weightedHigh - weightedLow) - maxRoom) < 15.0f) targetCoord = std::abs(weightedLow);
        } else if (weightH > 0)
            targetCoord = maxRoom - weightedHigh;
        else if (weightL > 0)
            targetCoord = 0.0f - weightedLow;

        if (weightL > 0 || weightH > 0) currentCoord = (currentCoord * 0.3) + (targetCoord * 0.7);
    };

    solveAxis(bottom, top, pose.y, 144.0f);
    solveAxis(left, right, pose.x, 144.0f);

    pose.x = std::clamp(pose.x, 0.0f, 144.0f);
    pose.y = std::clamp(pose.y, 0.0f, 144.0f);
}
}  // namespace LidarProcessor