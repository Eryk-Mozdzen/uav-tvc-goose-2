#pragma once

#include "Interface.h"
#include "Axis.h"

class Accelerometer : public Interface {
    Axis axisX;
    Axis axisY;
    Axis axisZ;

public:
    Accelerometer(QWidget *parent = nullptr);

    void receive(const protocol_readings_t &readings);
    void update(protocol_calibration_t &calibration) const;
};
