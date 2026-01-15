#include "LidarVisualizer.hpp"
#include "Config.hpp"
#include "localizor.hpp"
#include <QMouseEvent>
#include <QPainter>
#include <qnamespace.h>

LidarVisualizer::LidarVisualizer(QWidget *parent) : QWidget(parent) {
  // Increased width to 1350 to fit both histograms side-by-side
  // Increased height to 850 to give the X-axis labels room
  setFixedSize(1350, 850);
  this->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  m_objectPos = QPointF(200, 200);
  setMouseTracking(true);
}

void LidarVisualizer::updateFrame(const std::vector<Point>& scan, const std::vector<Point>& rotScan, const std::vector<Obstacle>& obs, Result est, Point truth, float true_heading) {
  scanPoints = scan;
  rotatedScan = rotScan;
  obstacles = obs;
  estPose = est;
  truePose = truth;
  this->true_heading = true_heading;
  update();
}

void LidarVisualizer::paintEvent(QPaintEvent *event) {
  QPainter p(this);

  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor(13, 17, 23));

  // Map area scale
  const float scale = (0.75 * this->height()) / Config::RoomSize;

  auto toScreen = [&](float x, float y) {
    return QPointF(x * scale, (Config::RoomSize - y) * scale);
  };

  auto fromScreen = [&](QPointF screenPos) {
    float x = screenPos.x() / scale;
    float y = Config::RoomSize - (screenPos.y() / scale);
    return QPointF(x, y);
  };

  // 1. Draw Room (Fixed at top-left)
  p.setPen(QPen(QColor(60, 70, 90), 2));
  p.drawRect(0, 0, Config::RoomSize * scale, Config::RoomSize * scale);
  // --- 2. Draw Grid (24x24 cells) ---
  p.setPen(
      QPen(QColor(60, 70, 90, 100), 1)); // Lower alpha (100) for a subtle look

  int tiles = 6;
  float cellSize = (float)Config::RoomSize / tiles; // This will be 6.0

  for (int i = 0; i <= tiles; ++i) {
    float pos = i * cellSize * scale;

    // Draw Vertical Lines
    p.drawLine(pos, 0, pos, Config::RoomSize * scale);

    // Draw Horizontal Lines
    p.drawLine(0, pos, Config::RoomSize * scale, pos);
  }

  // 2. Draw Obstacles
  p.setPen(Qt::NoPen);
  for (const auto &obs : obstacles) {
    p.setBrush(obs.moving ? QColor(200, 50, 50) : QColor(60, 75, 90));
    p.drawEllipse(toScreen(obs.x, obs.y), obs.radius * scale,
                  obs.radius * scale);
  }

  // 3. Draw Scan Points
  p.setBrush(QColor(0, 255, 220, 150));
  for (const auto &pt : scanPoints) {
    p.drawEllipse(toScreen(pt.x, pt.y), 1.5, 1.5);
  }

    p.setBrush(QColor(255, 128, 128, 150));
  for (const auto &pt : rotatedScan) {
    p.drawEllipse(toScreen(pt.x, pt.y), 1.5, 1.5);
  }


  // 4. Histogram Logic
  const int chartHeight = 200;
  const int chartBottom = this->height() - 18;
  const int barWidth = 2;
  const int spacing = 50;
  const int yScale = 4;
  const int xRange = 140;

  auto drawHistogram = [&](QPainter &painter, const std::vector<int> &hist,
                           int xOffset, QString title) {
    if (hist.empty())
      return;
    int chartWidth = hist.size() * barWidth;

    painter.setPen(QPen(QColor(60, 70, 90), 1));
    painter.setBrush(QColor(20, 25, 35));
    painter.drawRect(xOffset, chartBottom - chartHeight, chartWidth,
                     chartHeight);

    // Y-Axis Labels
    painter.setFont(QFont("Arial", 7));
    for (int val = 0; val <= (chartHeight / yScale); val += 10) {
      int yPos = chartBottom - (val * yScale);
      painter.setPen(QColor(40, 45, 55));
      painter.drawLine(xOffset, yPos, xOffset + chartWidth, yPos);
      painter.setPen(QColor(180, 180, 180));
      painter.drawText(xOffset - 25, yPos + 4, QString::number(val));
    }

    // X-Axis Scale
    for (int val = -xRange; val <= xRange; val += 20) {
      float t = (val + xRange) / (float)(2 * xRange);
      int xPos = xOffset + (t * chartWidth);
      painter.setPen(QColor(60, 70, 90, 150));
      painter.drawLine(xPos, chartBottom, xPos, chartBottom + 5);
      painter.setPen(QColor(150, 150, 150));
      painter.drawText(xPos - 12, chartBottom + 18, QString::number(val));
      if (val == 0) {
        painter.setPen(QPen(QColor(255, 255, 255, 50), 1, Qt::DashLine));
        painter.drawLine(xPos, chartBottom, xPos, chartBottom - chartHeight);
      }
    }

    // Bars
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 255, 150, 200));
    for (size_t i = 0; i < hist.size(); ++i) {
      int h = std::min(chartHeight, hist[i] * yScale);
      painter.drawRect(xOffset + (i * barWidth), chartBottom, barWidth, -h);
    }

    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 10, QFont::Bold));
    painter.drawText(xOffset, chartBottom - chartHeight - 15, title);
  };

  drawHistogram(p, xHist, 30, "X-Axis Point Density");
  drawHistogram(p, yHist, 30 + (280 * barWidth) + spacing,
                "Y-Axis Point Density");

  // 5. Draw Robots
  auto pose = fromScreen(m_objectPos);
  poseX = pose.x();
  poseY = pose.y();
  drawRobot(p, {m_objectPos.x(), m_objectPos.y()}, true_heading, false, 20);
  drawRobot(p, toScreen(estPose.x, estPose.y), estPose.heading, true, 30);

  // ---------------------------------------------------------
  // 6. TOP RIGHT STATUS BOX (Coordinates)
  // ---------------------------------------------------------
  int boxW = 220;
  int boxH = 200;
  int margin = 20;
  QRect statusRect(this->width() - boxW - margin, margin, boxW, boxH);

  // Draw Box Background
  p.setPen(QPen(QColor(60, 70, 90), 2));
  p.setBrush(QColor(20, 25, 35, 200)); // Slightly transparent
  p.drawRoundedRect(statusRect, 8, 8);

  // Draw Text
  p.setPen(Qt::white);
  p.setFont(QFont("Consolas", 10)); // Monospace for better alignment
  int textMargin = 15;
  int lineSpacing = 20;

  // Title
  p.setFont(QFont("Arial", 10, QFont::Bold));
  p.drawText(statusRect.adjusted(textMargin, 10, 0, 0), "Pose Comparison");

  p.setFont(QFont("Consolas", 9));
  // True Pose (Truth)
  p.setPen(QColor(150, 150, 150));
  p.drawText(statusRect.adjusted(textMargin, 35, 0, 0),
             QString("Truth: X:%1 Y:%2")
                 .arg(truePose.x, 0, 'f', 2)
                 .arg(truePose.y, 0, 'f', 2));

  // Localizer Pose (Estimated)
  p.setPen(QColor(0, 255, 220));
  p.drawText(statusRect.adjusted(textMargin, 35 + lineSpacing, 0, 0),
             QString("Localized: X:%1 Y:%2")
                 .arg(estPose.x, 0, 'f', 2)
                 .arg(estPose.y, 0, 'f', 2));

  // Pose Error
  p.setPen(Qt::white);
  float dx = truePose.x - estPose.x;
  float dy = truePose.y - estPose.y;
  float error = sqrt(dx*dx+dy*dy);
  p.drawText(statusRect.adjusted(textMargin, 35 + (lineSpacing * 2), 0, 0),
             QString("Pose Error: %1")
                 .arg((float)error, 0, 'f', 2));

  // Heading Comparison
  p.setPen(Qt::white);
  p.drawText(statusRect.adjusted(textMargin, 35 + (lineSpacing * 3), 0, 0),
             QString("Angle: Truth %1 | Est %2 | Diff %3")
                 .arg((int)true_heading)
                 .arg((int)estPose.heading)
                 .arg((float)(true_heading - estPose.heading), 0, 'f', 2));

  // 6. STATUS INDICATOR
  QString statusText = wallsString.c_str();
    QColor statusColor = Qt::red;

  if (confident) {
    statusColor = Qt::green;
  }
  p.setPen(statusColor);
  p.drawText(statusRect.adjusted(textMargin, 60 + (lineSpacing * 4), 0, 0),
             statusText);

  // Draw Origin Helper (0,0 marker)
  p.setPen(QPen(Qt::magenta, 2));
  QPointF origin = toScreen(0, 0);
  p.drawLine(origin.x(), origin.y() - 10, origin.x(), origin.y() + 10);
  p.drawLine(origin.x() - 10, origin.y(), origin.x() + 10, origin.y());
  p.drawText(origin + QPointF(5, -5), "Origin (0,0)");
}

void LidarVisualizer::drawRobot(QPainter &p, QPointF pos, float heading,
                                bool isGhost, int size) {
  p.save();
  p.translate(pos);
  p.rotate(heading);
  float s = size;
  if (isGhost) {
    p.setBrush(QColor(255, 255, 255, 40));
    p.setPen(QPen(QColor(255, 255, 255, 80), 1, Qt::DashLine));
  } else {
    p.setBrush(QColor(200, 200, 200));
    p.setPen(QPen(QColor(50, 50, 50), 2));
  }
  p.drawRect(-s / 2, -s / 2, s, s);
  p.setPen(Qt::black);
  p.drawLine(0, 0, s / 2, 0);
  p.restore();
}

void LidarVisualizer::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    printf("Pressed\n");
    printf("m_pos: %.2f,%.2f\n", m_objectPos.x(), m_objectPos.y());

    // Simple hit test: check if click is near m_objectPos
    // (You may need to scale m_objectPos to screen coordinates first)
    float distance = QLineF(event->position(), m_objectPos).length();
    if (distance < 50) { // If within 50 pixels
      m_isDragging = true;
      m_offset = event->position() - m_objectPos;
      setCursor(Qt::ClosedHandCursor); // Change cursor look
    }
  }
}

void LidarVisualizer::mouseMoveEvent(QMouseEvent *event) {
  QPointF currentPos = event->position();

  if (m_isDragging) {
    // 1. Calculate the raw new position
    QPointF newPos = currentPos - m_offset;

    // 2. Define your bounding box (e.g., the size of your widget)
    // Or hardcode it: QRectF boundary(0, 0, 800, 600);
    const float scale = (0.75 * this->height()) / Config::RoomSize;

    int padding = 7;
    QRectF boundary(padding, padding, scale * Config::RoomSize - 2 * padding,
                    scale * Config::RoomSize - 2 * padding);
    // 3. Clamp the X and Y coordinates
    float clampedX = std::clamp((float)newPos.x(), (float)boundary.left(),
                                (float)boundary.right());
    float clampedY = std::clamp((float)newPos.y(), (float)boundary.top(),
                                (float)boundary.bottom());

    m_objectPos = QPointF(clampedX, clampedY);
    update();
  } else {
    // Create a QRectF representing the room's area on screen
    float boxWidth = 30;
    QRectF roomRect(m_objectPos.x() - boxWidth / 2,
                    m_objectPos.y() - boxWidth / 2, boxWidth, boxWidth);

    if (roomRect.contains(currentPos)) {
      setCursor(Qt::OpenHandCursor); // Show "grab" hand
    } else {
      unsetCursor(); // Back to standard arrow
    }
  }
}

void LidarVisualizer::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    m_isDragging = false;
    unsetCursor();
  }
}
