#include <QDateTime>
#include <QDebug>
#include <QChartView>
#include <QLegend>
#include <QTimer>

#include "Chart.h"

const qint64 Chart::start = QDateTime::currentMSecsSinceEpoch();

Chart::Series::Series(Chart &chart, const QColor color, const Qt::PenStyle style, const int width) {
    chart.chart->addSeries(&s);

    s.setPen(QPen(color, width, style));
    s.attachAxis(chart.axisX);
    s.attachAxis(chart.axisY);

    QTimer *timer = new QTimer();

    connect(timer, &QTimer::timeout, [&]() {
        while(s.points().first().x()<chart.axisX->min()) {
            s.remove(0);
        }
    });

    timer->start(100);
}

void Chart::Series::append(const float value) {
    s.append(Chart::getTime(), value);
}

Chart::Chart(const QString title, const QString yLabel, const QString yFormat, const float yMin, const float yMax) : QMainWindow{nullptr} {
    resize(600, 400);
    setWindowTitle(title);

    chart = new QtCharts::QChart();
    chart->legend()->hide();

    axisX = new QtCharts::QValueAxis(this);
    axisX->setTitleText("time [s]");
    axisX->setLabelFormat("%5.1f");
    chart->addAxis(axisX, Qt::AlignBottom);

    axisY = new QtCharts::QValueAxis(this);
    axisY->setTitleText(yLabel);
    axisY->setLabelFormat(yFormat);
    axisY->setRange(yMin, yMax);
    chart->addAxis(axisY, Qt::AlignLeft);

    QTimer *timer = new QTimer();
    timer->start(10);
    connect(timer, &QTimer::timeout, [&]() {
        const float t = Chart::getTime();

        axisX->setRange(t - 10, t);
    });

    QtCharts::QChartView *view = new QtCharts::QChartView(chart);
    view->setRenderHint(QPainter::Antialiasing);
    setCentralWidget(view);

    show();
}

float Chart::getTime() {
    return static_cast<float>(QDateTime::currentMSecsSinceEpoch() - start)/1000.f;
}
