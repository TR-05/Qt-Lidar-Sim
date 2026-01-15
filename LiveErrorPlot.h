#pragma once
#include <QWidget>
#include <deque>

class LiveErrorPlot : public QWidget {
    Q_OBJECT
public:
    explicit LiveErrorPlot(QWidget* parent = nullptr);
    void addError(float err);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    std::deque<float> errorHistory;
};