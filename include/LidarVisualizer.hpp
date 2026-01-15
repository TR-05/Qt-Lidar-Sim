#pragma once
#include <QWidget>
#include "Config.hpp"
#include <vector>

class LidarVisualizer : public QWidget {
    Q_OBJECT
public:
    explicit LidarVisualizer(QWidget* parent = nullptr);
    void updateFrame(const std::vector<Point>& scan, const std::vector<Obstacle>& obs, Result est, Point truth, float true_heading);
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
    Result estPose;
    Point truePose;
    float true_heading;
    bool m_isDragging = false;
    QPointF m_offset;      // Distance between mouse click and object center
    QPointF m_objectPos;   // The current (x, y) of your object/map
};