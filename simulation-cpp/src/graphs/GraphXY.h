#pragma once

#include <QMainWindow>
#include <QChart>
#include <QLineSeries>
#include <QValueAxis>

class GraphXY : public QMainWindow {
    static const qint64 start;

    QtCharts::QChart *chart;
    QtCharts::QValueAxis *axisX;
    QtCharts::QValueAxis *axisY;

    static float getTime();

public:
    class Series {
        QtCharts::QLineSeries s;

    public:
        Series(GraphXY &chart, const QColor color=Qt::black, const Qt::PenStyle style=Qt::SolidLine, const int width=2);

        void append(const float x, const float y);
    };

    GraphXY(const QString title, const QString format, const float range);
};
