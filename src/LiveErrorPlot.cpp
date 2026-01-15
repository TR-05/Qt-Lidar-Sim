#include "LiveErrorPlot.hpp"
#include "Config.hpp"
#include <QPainter>

LiveErrorPlot::LiveErrorPlot(QWidget* parent) : QWidget(parent) {
    setFixedSize(300, 250);
}

void LiveErrorPlot::addError(float err) {
    errorHistory.push_back(err);
    if (errorHistory.size() > 100) errorHistory.pop_front();
    update();
}

void LiveErrorPlot::paintEvent(QPaintEvent* event) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(25, 30, 40));

    const int baselineY = height() - 40;
    const int graphLeft = 40;

    // Draw Grid
    p.setPen(QColor(60, 70, 80));
    for (int i = 0; i <= 5; ++i) {
        int y = baselineY - (i * Config::PixelsPerInch);
        p.drawLine(graphLeft, y, width(), y);
        p.setPen(Qt::gray);
        p.drawText(5, y + 5, QString("%1\"").arg(i));
        p.setPen(QColor(60, 70, 80));
    }

    p.setPen(Qt::white);
    p.drawText(10, 20, "Positional Error (inches)");

    if (errorHistory.size() < 2) return;

    // Draw Error Curve
    p.setPen(QPen(QColor(0, 255, 200), 2));
    float step = static_cast<float>(width() - graphLeft) / 100.0f;
    
    for (size_t i = 1; i < errorHistory.size(); ++i) {
        float x1 = graphLeft + (i - 1) * step;
        float y1 = baselineY - (errorHistory[i - 1] * Config::PixelsPerInch);
        float x2 = graphLeft + i * step;
        float y2 = baselineY - (errorHistory[i] * Config::PixelsPerInch);
        p.drawLine(QPointF(x1, y1), QPointF(x2, y2));
    }
}