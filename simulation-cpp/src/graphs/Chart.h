#pragma once

#include <QMainWindow>
#include <QChart>
#include <QLineSeries>
#include <QValueAxis>

class Chart : public QMainWindow {
    static const qint64 start;

    QtCharts::QChart *chart;
    QtCharts::QValueAxis *axisX;
    QtCharts::QValueAxis *axisY;

    static float getTime();

public:
    class Series {
        QtCharts::QLineSeries s;

    public:
        Series(Chart &chart, const QColor color=Qt::black, const Qt::PenStyle style=Qt::SolidLine, const int width=2);

        void append(const float value);
    };

    Chart(const QString title, const QString yLabel, const QString yFormat, const float yMin, const float yMax);
};
