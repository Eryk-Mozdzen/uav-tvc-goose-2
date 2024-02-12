#include <QChartView>
#include <QLegend>

#include "GraphXY.h"

GraphXY::GraphXY(const QString title, const QString format, const float range) : QMainWindow{nullptr} {
    resize(600, 600);
    setWindowTitle(title);

    chart = new QtCharts::QChart();
    chart->legend()->hide();

    axisX = new QtCharts::QValueAxis(this);
    axisX->setTitleText("X");
    axisX->setLabelFormat(format);
    axisX->setRange(-range, range);
    chart->addAxis(axisX, Qt::AlignBottom);

    axisY = new QtCharts::QValueAxis(this);
    axisY->setTitleText("Y");
    axisY->setLabelFormat(format);
    axisY->setRange(-range, range);
    chart->addAxis(axisY, Qt::AlignLeft);

    QtCharts::QChartView *view = new QtCharts::QChartView(chart);
    view->setRenderHint(QPainter::Antialiasing);
    setCentralWidget(view);

    show();

    DeclarePeriodicUnrestrictedUpdateEvent(period, 0, &GraphXY::update);
}

void GraphXY::AddSeries(const QString name, const Eigen::MatrixX<double> selector, const QColor color, const Qt::PenStyle style, const int width) {
    Series s;
    s.series = new QtCharts::QLineSeries();
    chart->addSeries(s.series);
    s.series->setPen(QPen(color, width, style));
    s.series->attachAxis(axisX);
    s.series->attachAxis(axisY);

    s.selector = selector;
    s.port = DeclareVectorInputPort(name.toStdString(), selector.cols()).get_index();

    series.push_back(s);
}

drake::systems::EventStatus GraphXY::update(const drake::systems::Context<double> &context, drake::systems::State<double> *state) const {
    (void)state;

    for(const Series &s : series) {
        const Eigen::VectorX<double> &input = EvalVectorInput(context, s.port)->CopyToVector();
        const Eigen::VectorX<double> &value = s.selector*input;

        const double x = value(0);
        const double y = value(1);

        while(s.series->count()>samples) {
            s.series->remove(0);
        }

        s.series->append(x, y);
    }

    return drake::systems::EventStatus::Succeeded();
}
