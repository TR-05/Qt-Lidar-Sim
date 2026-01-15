#include "LidarSimulator.hpp"
#include "LidarVisualizer.hpp"
#include "LiveErrorPlot.hpp"
#include "config.hpp"
#include "localizor.hpp"
#include <QApplication>
#include <QHBoxLayout>
#include <QTimer>
#include <qnamespace.h>
#include <qsizepolicy.h>

// basically the lidar part works. localization part ehh no
 bool confident;
 std::string wallsString;
 std::vector<int> xHist;
 std::vector<int> yHist;
 float binWidth;

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  a.setStyle("Fusion");

  QWidget mainWin;
  QHBoxLayout *layout = new QHBoxLayout(&mainWin);
  mainWin.setStyleSheet("background-color: #0d1117; color: white;");

  auto *viz = new LidarVisualizer();
  viz->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  layout->addWidget(viz);
  layout->setAlignment(Qt::AlignLeft | Qt::AlignTop);

  LidarSimulator sim;
  std::vector<Obstacle> obstacles = {{Config::RoomSize/2, Config::RoomSize/2, 4, false},   {24, 48, 2, false},
                                     {24, Config::RoomSize - 48, 2, false},  {Config::RoomSize-24, 48, 2, false},
                                     {Config::RoomSize-24, Config::RoomSize - 48, 2, false}};

  float loopMs = 16;
  float time = 0.0f;
  QTimer timer;

  QObject::connect(&timer, &QTimer::timeout, [&]() {
    time += loopMs/1000.0;
    float rx = viz->poseX;
    float ry = viz->poseY;
    float heading = time * 180; //(60 deg/s);
    //heading = 0;
    // simulate lidar data
    auto data = sim.getScan(rx, ry, heading, obstacles);

    std::mt19937 gen;
    std::normal_distribution<float> noise(0.0, 0.4);
    float n = noise(gen) * 30;

    auto p = localize2(data, heading);
    //printf("ex: %6.2f, ey: %6.2f, eh: %7.2f, rx: %6.2f, ry: %6.2f, rh: %7.2f\n", p.x,p.y,p.heading, rx, ry, heading);
    // localization here:
    Result res;//localize2(data, heading);
    res.x = p.x;
    res.y = p.y;
    res.heading = p.heading;
    // Convert raw lidar data to cartesian for plotting
    std::vector<Point> points;
    for (int i = 0; i < Config::ScanPoints; ++i) {
      if (data.first.at(i) > 0) {
        float rad = (data.second.at(i) + heading) * Config::DegToRad;
        points.push_back({data.first.at(i) * std::cos(rad) + rx,
                          data.first.at(i) * std::sin(rad)+ ry} );
      }
    }
    
    viz->updateFrame(points, obstacles, res, {rx, ry}, heading);
  });

  timer.start(loopMs);
  mainWin.show();
  return a.exec();
}