#include <QApplication>
#include <QHBoxLayout>
#include <QTimer>
#include <QWidget>
#include <random>
#include <vector>

// Project Headers
#include "LidarSimulator.hpp"
#include "LidarVisualizer.hpp"
#include "config.hpp"
#include "localizor.hpp"


// --- Simulation Constants & Initial State ---
namespace Simulation {
constexpr float LoopIntervalMs = 16.0f;

const std::vector<Obstacle> WorldObstacles = {{Config::RoomSize / 2, Config::RoomSize / 2, 3, false},
                                              {24, 48, 1, false},
                                              {24, Config::RoomSize - 48, 1, false},
                                              {Config::RoomSize - 24, 48, 1, false},
                                              {Config::RoomSize - 24, Config::RoomSize - 48, 1, false}};
}  // namespace Simulation

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setStyle("Fusion");

    // --- 1. UI Setup ---
    QWidget mainWin;
    mainWin.setWindowTitle("LiDAR Simulator & Localizer");
    mainWin.setStyleSheet("background-color: #0d1117; color: white;");
    mainWin.resize(1000, 800);

    QHBoxLayout* layout = new QHBoxLayout(&mainWin);
    layout->setContentsMargins(80, 20, 20, 20);
    auto* viz = new LidarVisualizer();
    viz->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(viz);

    // --- 2. Simulation State ---
    LidarSimulator sim;
    float currentTime = 0.0f;

    // Random engine (static so it persists between timer ticks)
    static std::mt19937 gen(std::random_device{}());
    std::normal_distribution<float> noiseDist(0.0, 0.4);

    // --- 3. Main Loop ---
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        currentTime += Simulation::LoopIntervalMs / 1000.0f;

        // Target Pose (Ground Truth)
        float rx = viz->poseX;
        float ry = viz->poseY;
        float heading = currentTime * 180;  // 90 deg/s
        // heading = 90;
        //  A. Simulate Lidar Scan
        auto scanData = sim.getScan(rx, ry, heading, Simulation::WorldObstacles);

        // B. Run Localization
        // Note: noise calculation 'n' is here if you want to add it to your localization input
        float n = noiseDist(gen) * 5 + 12;

        LidarProcessor::segment(scanData, {rx, ry}, heading + n * 1);
        static LidarProcessor::Point2d estPose = {40, 72};
        float estH = heading;
        LidarProcessor::localizeFromLines(estPose, heading);

        // C. Process Data for Visualization
        std::vector<Point> scanPoints;
        scanPoints.reserve(Config::ScanPoints);

        for (int i = 0; i < Config::ScanPoints; ++i) {
            float range = scanData.first.at(i);
            float angle = scanData.second.at(i);

            if (range > 0) {
                float rad = (angle + heading) * Config::DegToRad;
                scanPoints.push_back({range * std::cos(rad) + rx, range * std::sin(rad) + ry});
            }
        }

        // D. Update Display
        viz->updateFrame(scanPoints, scanPoints, Simulation::WorldObstacles, {estPose.x, estPose.y, estH}, {rx, ry}, heading);
    });

    // --- 4. Start ---
    timer.start(Simulation::LoopIntervalMs);
    mainWin.show();

    return app.exec();
}