#include <QApplication>
#include <QHBoxLayout>
#include <QTimer>
#include <QWidget>
#include <random>
#include <vector>

// Project Headers
#include "LidarVisualizer.hpp"
#include "MCL/Lidar.hpp"
#include "MCL/MCL.hpp"
#include "config.hpp"
#include "localizor.hpp"
std::mt19937 gen2(std::random_device{}());

// --- Simulation Constants & Initial State ---
namespace Simulation {
constexpr float LoopIntervalMs = 16.0f;
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
    float currentTime = 0.0f;
    QTimer timer;
    MCL mcl_filter(25, 0.05);
    Pose lastPose;
    viz->poseX = 72;
    viz->poseY = 72;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        currentTime += Simulation::LoopIntervalMs / 1000.0f;

        // Target Pose (Ground Truth)
        true_pose = {viz->poseX, viz->poseY, currentTime * 0};
        double cur_theta = (90.0 - true_pose.theta) * Config::DegToRad;
        double last_theta = (90.0 - lastPose.theta) * Config::DegToRad;

        // 2. Calculate Delta in pure Cartesian
            std::normal_distribution<double> noise(0.0, 0.4);

        Pose delta = {true_pose.x - lastPose.x + noise(gen2)*.15, true_pose.y-lastPose.y + noise(gen2)*.15, cur_theta - last_theta + noise(gen2)*.15};

        // Wrap angle
        if (!std::isnan(delta.theta)) {
            delta.theta = fmod(delta.theta + M_PI, 2 * M_PI);
            if (delta.theta < 0) delta.theta += 2 * M_PI;
            delta.theta -= M_PI;
        }

        // 3. Run Step
        auto data = lidar_get_scan();
        delta.theta *= (M_PI / 180.0);  // Convert deg to rad
        while (delta.theta > M_PI) delta.theta -= 2 * M_PI;
        while (delta.theta < -M_PI) delta.theta += 2 * M_PI;

        //printf("DEBUG DELTA: x: %.4f, y: %.4f, theta: %.4f\n", delta.x, delta.y, delta.theta);
        mcl_filter.step(delta, data);

        Pose est = mcl_filter.getEstimatedPose();

        // Cartesian CCW to Compass CW: compass = 90 - cartesian
        Pose viz_est = {est.x, est.y, true_pose.theta};

        std::vector<Point> rayCastedScan;
        for (auto& d : data) {
            float l = d.length;
            float a = d.angle;
            float x = l * cosf(a) + true_pose.x;
            float y = l * sinf(a) + true_pose.y;
            rayCastedScan.push_back({x, y});
        }

        lastPose = true_pose;
        viz->updateFrame(rayCastedScan, mcl_filter.getParticles(), WorldObstacles, viz_est, true_pose);
    });

    // --- 4. Start ---
    timer.start(Simulation::LoopIntervalMs);
    mainWin.show();

    return app.exec();
}