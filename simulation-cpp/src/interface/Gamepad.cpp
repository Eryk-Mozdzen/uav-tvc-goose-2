#include <QGamepad>
#include <QDebug>
#include <QTimer>

#include "Gamepad.h"

Gamepad::Gamepad(QObject *parent) : QObject{parent}, analog{QVector<float>(6)}, digital{QVector<bool>(15)} {

    QTimer *timer = new QTimer(this);
    timer->start(100);

    connect(timer, &QTimer::timeout, [timer, this]() {
        const QList<int> list = QGamepadManager::instance()->connectedGamepads();

        if(list.isEmpty()) {
            return;
        }

        timer->stop();

        QGamepad *gamepad = new QGamepad(*list.begin(), this);

		qDebug() << "gamepad connected";

        connect(gamepad, &QGamepad::axisLeftXChanged,    this, std::bind(&Gamepad::updateA, this, Analog::LX,           std::placeholders::_1));
        connect(gamepad, &QGamepad::axisLeftYChanged,    this, std::bind(&Gamepad::updateA, this, Analog::LY,           std::placeholders::_1));
        connect(gamepad, &QGamepad::axisRightXChanged,   this, std::bind(&Gamepad::updateA, this, Analog::RX,           std::placeholders::_1));
        connect(gamepad, &QGamepad::axisRightYChanged,   this, std::bind(&Gamepad::updateA, this, Analog::RY,           std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonL2Changed,     this, std::bind(&Gamepad::updateA, this, Analog::L2,           std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonR2Changed,     this, std::bind(&Gamepad::updateA, this, Analog::R2,           std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonAChanged,      this, std::bind(&Gamepad::updateD, this, Digital::CIRCLE_A,    std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonBChanged,      this, std::bind(&Gamepad::updateD, this, Digital::CIRCLE_B,    std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonXChanged,      this, std::bind(&Gamepad::updateD, this, Digital::CIRCLE_X,    std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonYChanged,      this, std::bind(&Gamepad::updateD, this, Digital::CIRCLE_Y,    std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonDownChanged,   this, std::bind(&Gamepad::updateD, this, Digital::CROSS_DOWN,  std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonUpChanged,     this, std::bind(&Gamepad::updateD, this, Digital::CROSS_UP,    std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonLeftChanged,   this, std::bind(&Gamepad::updateD, this, Digital::CROSS_LEFT,  std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonRightChanged,  this, std::bind(&Gamepad::updateD, this, Digital::CROSS_RIGHT, std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonL1Changed,     this, std::bind(&Gamepad::updateD, this, Digital::L1,          std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonR1Changed,     this, std::bind(&Gamepad::updateD, this, Digital::R1,          std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonL3Changed,     this, std::bind(&Gamepad::updateD, this, Digital::L3,          std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonR3Changed,     this, std::bind(&Gamepad::updateD, this, Digital::R3,          std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonGuideChanged,	 this, std::bind(&Gamepad::updateD, this, Digital::HOME,        std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonSelectChanged, this, std::bind(&Gamepad::updateD, this, Digital::SELECT,      std::placeholders::_1));
        connect(gamepad, &QGamepad::buttonStartChanged,	 this, std::bind(&Gamepad::updateD, this, Digital::START,       std::placeholders::_1));

        connect(gamepad, &QGamepad::connectedChanged, [this, timer, gamepad](bool value) {
            if(value) {
                return;
            }

            QMutexLocker locker(&mutex);

			for(float &value : analog) {
				value = 0;
			}

			for(bool &value : digital) {
				value = false;
			}

            delete gamepad;

			qDebug() << "gamepad disconnected";

            timer->start();
        });
    });
}

float Gamepad::get(Analog input) {
    QMutexLocker locker(&mutex);
    return analog[input];
}

bool Gamepad::get(Digital input) {
    QMutexLocker locker(&mutex);
    return digital[input];
}

void Gamepad::updateA(const Analog input, const float value) {
    QMutexLocker locker(&mutex);
    analog[input] = value;
}

void Gamepad::updateD(const Digital input, const bool value) {
    QMutexLocker locker(&mutex);
    digital[input] = value;
}
