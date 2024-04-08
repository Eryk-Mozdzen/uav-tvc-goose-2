#include "Map.h"

int Map::path_id_counter = 0;

Map::Map(QWidget *parent) : QWebEngineView{parent} {
    timer.setInterval(20);

    connect(&timer, &QTimer::timeout, [&]() {
        while(queue.size()>0) {
            page()->runJavaScript(queue.front());
            queue.dequeue();
        }
    });

    connect(this, &Map::loadFinished, [&]() {
        timer.start();
    });

    load(QUrl(QStringLiteral("qrc:///res/index.html")));
}

void Map::setView(double latitude, double longitude, int zoom) {
    const QString lat = QString::number(latitude, 'f', 20);
    const QString lng = QString::number(longitude, 'f', 20);
    const QString script = QString("setView(%1, %2, %3);").arg(lat).arg(lng).arg(zoom);

    queue.enqueue(script);
}

int Map::createPath(QString color) {
    const QString script = QString("createPolyLine('%1')").arg(color);

    queue.enqueue(script);

    return path_id_counter++;
}

void Map::append(double latitude, double longitude, int path_id) {
    const QString lat = QString::number(latitude, 'f', 20);
    const QString lng = QString::number(longitude, 'f', 20);
    const QString script = QString("appendPolyLine(%1, %2, %3)").arg(lat).arg(lng).arg(path_id);

    queue.enqueue(script);
}

void Map::append(double latitude, double longitude, QString color) {
    const QString lat = QString::number(latitude, 'f', 20);
    const QString lng = QString::number(longitude, 'f', 20);
    const QString script = QString("addPoint(%1, %2, '%3')").arg(lat).arg(lng).arg(color);

    queue.enqueue(script);
}
