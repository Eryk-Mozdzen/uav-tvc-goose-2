#pragma once

#include <QWidget>
#include <QWebEngineView>

class Map : public QWidget {
    const double init_latitude;
    const double init_longitude;
    const int init_zoom;

    QWebEngineView view;
    bool loaded;

public:
    explicit Map(double latitude, double longitude, int zoom = 19, QWidget *parent = nullptr);

    void mark(double latitude, double longitude);
    void set(double latitude, double longitude, int zoom);
};
