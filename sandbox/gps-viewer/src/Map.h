#pragma once

#include <QWebEngineView>
#include <QQueue>
#include <QTimer>

class Map : public QWebEngineView {
    static int path_id_counter;
    QQueue<QString> queue;
    QTimer timer;

public:
    Map(QWidget *parent = nullptr);

    void setView(double latitude, double longitude, int zoom = 19);
    int createPath(QString color);
    void append(double latitude, double longitude, QString color);
    void append(double latitude, double longitude, int path_id);
};
