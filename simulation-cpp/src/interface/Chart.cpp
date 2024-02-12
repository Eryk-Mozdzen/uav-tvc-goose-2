#include <QChartView>
#include <QLegend>

#include "Chart.h"

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

    QtCharts::QChartView *view = new QtCharts::QChartView(chart);
    view->setRenderHint(QPainter::Antialiasing);
    setCentralWidget(view);

    show();

    DeclarePeriodicUnrestrictedUpdateEvent(period, 0, &Chart::update);
}

void Chart::AddSeries(const QString name, const Eigen::VectorX<double> selector, const QColor color, const Qt::PenStyle style, const int width) {
    Series s;
    s.series = new QtCharts::QLineSeries();

    chart->addSeries(s.series);

    s.series->setPen(QPen(color, width, style));
    s.series->attachAxis(axisX);
    s.series->attachAxis(axisY);

    s.selector = selector;
    s.port = this->DeclareVectorInputPort(name.toStdString(), selector.size()).get_index();

    series.push_back(s);
}

drake::systems::EventStatus Chart::update(const drake::systems::Context<double> &context, drake::systems::State<double> *state) const {
    (void)state;

    const double t = context.get_time();

    axisX->setRange(t - horizion, t);

    for(const Series &s : series) {
        const Eigen::VectorX<double> &input = EvalVectorInput(context, s.port)->CopyToVector();
        const double value = s.selector.dot(input);

        while(s.series->count()>samples) {
            s.series->remove(0);
        }

        s.series->append(t, value);
    }

    return drake::systems::EventStatus::Succeeded();
}
