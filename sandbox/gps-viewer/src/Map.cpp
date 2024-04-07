#include <QHBoxLayout>

#include "Map.h"

Map::Map(double latitude, double longitude, int zoom, QWidget *parent) :
        QWidget{parent},
        init_latitude{latitude},
        init_longitude{longitude},
        init_zoom{zoom},
        loaded{false} {

    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(&view);
    layout->setContentsMargins(0, 0, 0, 0);

    connect(view.page(), &QWebEnginePage::loadFinished, this, [&]() {
        loaded = true;
        set(init_latitude, init_longitude, init_zoom);
    });

    view.page()->load(QUrl(QStringLiteral("qrc:///res/index.html")));
}

void Map::mark(double latitude, double longitude) {
    if(!loaded) {
        return;
    }

    const QString lat = QString::number(latitude, 'f', 20);
    const QString lng = QString::number(longitude, 'f', 20);
    const QString script = QString("addMarker(%1, %2);").arg(lat).arg(lng);

    view.page()->runJavaScript(script);
}

void Map::set(double latitude, double longitude, int zoom) {
    if(!loaded) {
        return;
    }

    const QString lat = QString::number(latitude, 'f', 20);
    const QString lng = QString::number(longitude, 'f', 20);
    const QString script = QString("setView(%1, %2, %3);").arg(lat).arg(lng).arg(zoom);

    view.page()->runJavaScript(script);
}