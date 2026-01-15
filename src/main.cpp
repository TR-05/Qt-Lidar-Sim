#include <QApplication>
#include <QHBoxLayout>
#include <QTimer>
#include <QWidget>
#include <random>
#include <vector>

// Project Headers
#include "config.hpp"
#include "LidarSimulator.hpp"
#include "LidarVisualizer.hpp"
#include "localizor.hpp"

// --- Simulation Constants & Initial State ---
namespace Simulation {
    constexpr float LoopIntervalMs = 200.0f;
    
    const std::vector<Obstacle> WorldObstacles = {
        {Config::RoomSize / 2, Config::RoomSize / 2, 3, false},
        {24, 48, 1, false},
        {24, Config::RoomSize - 48, 1, false},
        {Config::RoomSize - 24, 48, 1, false},
        {Config::RoomSize - 24, Config::RoomSize - 48, 1, false}
    };
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setStyle("Fusion");

    // --- 1. UI Setup ---
    QWidget mainWin;
    mainWin.setWindowTitle("LiDAR Simulator & Localizer");
    mainWin.setStyleSheet("background-color: #0d1117; color: white;");
    mainWin.resize(1000, 800);

    QHBoxLayout* layout = new QHBoxLayout(&mainWin);
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
        float heading = currentTime * 90; // 90 deg/s
        heading = 90;
        // A. Simulate Lidar Scan
        auto scanData = sim.getScan(rx, ry, heading, Simulation::WorldObstacles);

        // B. Run Localization
        // Note: noise calculation 'n' is here if you want to add it to your localization input
        float n = noiseDist(gen) * 30.0f; 
        Result localizationResult = localize2(scanData, rx, ry, heading);

        // C. Process Data for Visualization
        std::vector<Point> scanPoints;
        scanPoints.reserve(Config::ScanPoints);

        for (int i = 0; i < Config::ScanPoints; ++i) {
            float range = scanData.first.at(i);
            float angle = scanData.second.at(i);
            
            if (range > 0) {
                float rad = (angle + heading) * Config::DegToRad;
                scanPoints.push_back({
                    range * std::cos(rad) + rx,
                    range * std::sin(rad) + ry
                });
            }
        }

        std::vector<Point> scanPointsRotated;
        scanPointsRotated.reserve(Config::ScanPoints);

        for (int i = 0; i < Config::ScanPoints; ++i) {
            float range = scanData.first.at(i);
            float angle = scanData.second.at(i);
            
            if (range > 5) {
                float rad = (angle + localizationResult.heading) * Config::DegToRad;
                scanPointsRotated.push_back({
                    range * std::cos(rad) + localizationResult.x,
                    range * std::sin(rad) + localizationResult.y
                });
            }
        }

        // D. Update Display
        viz->updateFrame(scanPoints, scanPointsRotated, Simulation::WorldObstacles, localizationResult, {rx, ry}, heading);
    });

    // --- 4. Start ---
    timer.start(Simulation::LoopIntervalMs);
    mainWin.show();
    
    return app.exec();
}