#pragma once
#include <QWidget>
#include "config.hpp"
#include "MCL/Types.hpp"
#include <vector>

class LidarVisualizer : public QWidget {
    Q_OBJECT
public:
    explicit LidarVisualizer(QWidget* parent = nullptr);
    void updateFrame(const std::vector<Point>& scan, std::vector<Particle> particles, const std::vector<Obstacle>& obs, Pose est, Pose truth);
    float poseX;
    float poseY;

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
private:
    void drawRobot(class QPainter& p, QPointF pos, float heading, bool isGhost, int size);
    std::vector<Point> scanPoints;
    std::vector<Obstacle> obstacles;
    Pose estPose;
    bool m_isDragging = false;
    QPointF m_offset;      // Distance between mouse click and object center
    QPointF m_objectPos;   // The current (x, y) of your object/map
};