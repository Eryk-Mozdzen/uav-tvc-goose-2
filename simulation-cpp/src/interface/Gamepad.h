#pragma once

#include <QObject>
#include <QMutex>

class Gamepad : public QObject {
    Q_OBJECT

public:
    enum Analog {
        LX,
        LY,
        RX,
        RY,
        L2,
        R2
    };

    enum Digital {
        CROSS_LEFT,
        CROSS_RIGHT,
        CROSS_UP,
        CROSS_DOWN,
        CIRCLE_A,
        CIRCLE_B,
        CIRCLE_X,
        CIRCLE_Y,
        L1,
        R1,
        L3,
        R3,
        SELECT,
        START,
        HOME
    };

    Gamepad(QObject *parent=nullptr);

    float get(Analog input);
    bool get(Digital input);

private slots:
    void updateA(const Analog input, const float value);
    void updateD(const Digital input, const bool value);

private:
    QVector<float> analog;
    QVector<bool> digital;
    QMutex mutex;
};
