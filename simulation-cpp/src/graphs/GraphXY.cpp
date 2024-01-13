#include <QDateTime>
#include <QDebug>
#include <QChartView>
#include <QLegend>
#include <QTimer>

#include "GraphXY.h"

const qint64 GraphXY::start = QDateTime::currentMSecsSinceEpoch();

GraphXY::Series::Series(GraphXY &chart, const QColor color, const Qt::PenStyle style, const int width) {
    chart.chart->addSeries(&s);

    s.setPen(QPen(color, width, style));
    s.attachAxis(chart.axisX);
    s.attachAxis(chart.axisY);

    QTimer *timer = new QTimer();

    connect(timer, &QTimer::timeout, [&]() {
        while(s.count()>250) {
            s.remove(0);
        }
    });

    timer->start(100);
}

void GraphXY::Series::append(const float x, const float y) {
    s.append(x, y);
}

GraphXY::GraphXY(const QString title, const QString format, const float range) : QMainWindow{nullptr} {
    resize(600, 600);
    setWindowTitle(title);

    chart = new QtCharts::QChart();
    chart->legend()->hide();

    axisX = new QtCharts::QValueAxis(this);
    axisX->setTitleText("X [m]");
    axisX->setLabelFormat(format);
    axisX->setRange(-range, range);
    chart->addAxis(axisX, Qt::AlignBottom);

    axisY = new QtCharts::QValueAxis(this);
    axisY->setTitleText("Y [m]");
    axisY->setLabelFormat(format);
    axisY->setRange(-range, range);
    chart->addAxis(axisY, Qt::AlignLeft);

    QtCharts::QChartView *view = new QtCharts::QChartView(chart);
    view->setRenderHint(QPainter::Antialiasing);
    setCentralWidget(view);

    show();
}

float GraphXY::getTime() {
    return static_cast<float>(QDateTime::currentMSecsSinceEpoch() - start)/1000.f;
}
