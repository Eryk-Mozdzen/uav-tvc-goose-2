#pragma once

#include <QMainWindow>
#include <QChart>
#include <QLineSeries>
#include <QValueAxis>
#include <drake/systems/framework/leaf_system.h>

class Chart : public QMainWindow, public drake::systems::LeafSystem<double> {
    static constexpr double horizion = 10;
    static constexpr double frequency = 20;
    static constexpr double period = 1/frequency;
    static constexpr double samples = horizion*frequency;

    struct Series {
        QtCharts::QLineSeries *series;
        Eigen::VectorX<double> selector;
        drake::systems::InputPortIndex port;
    };

    QtCharts::QChart *chart;
    QtCharts::QValueAxis *axisX;
    QtCharts::QValueAxis *axisY;
    std::vector<Series> series;

    drake::systems::EventStatus update(const drake::systems::Context<double> &context, drake::systems::State<double> *state) const;

public:
    Chart(const QString title, const QString yLabel, const QString yFormat, const float yMin, const float yMax);

    void AddSeries(const QString name, const Eigen::VectorX<double> selector, const QColor color, const Qt::PenStyle style, const int width);
    void AddSeries(const QString name, const Eigen::MatrixX<double> selector);
};
