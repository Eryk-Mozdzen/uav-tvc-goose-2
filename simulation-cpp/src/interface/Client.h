#pragma once

#include <QTcpSocket>

#include "Object.h"

class Client : public QObject {
    Q_OBJECT

    static constexpr double rad2deg = 57.29578;

    QTcpSocket socket;

    void write(const QString &message);

public:
    Client();

    void draw(const Object::State &state);

signals:
    void disconnect();
};
