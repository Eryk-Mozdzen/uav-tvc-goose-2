#pragma once

#include <QWidget>

#include "protocol/protocol_data.h"

class Interface : public QWidget {
    QString name;

public:
    Interface(QString name, QWidget *parent=nullptr) : QWidget{parent}, name{name} {

    }

    QString getName() const {
        return name;
    }

    virtual void receive(const protocol_readings_t &readings) {
        (void)readings;
    };

    virtual void update(protocol_calibration_t &calibration) const = 0;
};
