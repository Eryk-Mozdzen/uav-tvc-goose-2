#pragma once

#include <QTcpSocket>

#include "VisualizationSink.h"

class VisualClient : public VisualizationSink<VisualClient> {
    static constexpr double rad2deg = 57.29578;
    static constexpr int port = 8080;

    QTcpSocket socket;

    void write(const QString &message);

public:
    VisualClient();

    void callback(const Eigen::VectorX<double> &value);
};
