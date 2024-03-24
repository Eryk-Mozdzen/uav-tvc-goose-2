#include <QHBoxLayout>

#include "Accelerometer.h"

Accelerometer::Accelerometer(QWidget *parent) : Interface{"accelerometer", parent}, axisX{'X'}, axisY{'Y'}, axisZ{'Z'} {
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(&axisX);
    layout->addWidget(&axisY);
    layout->addWidget(&axisZ);
}

Interface * Accelerometer::create() const {
    return new Accelerometer();
}

void Accelerometer::receive(const protocol_readings_t &readings) {
    if(readings.valid.accelerometer) {
        axisX.set(readings.accelerometer[0]);
        axisY.set(readings.accelerometer[1]);
        axisZ.set(readings.accelerometer[2]);
    }
}

void Accelerometer::update(protocol_calibration_t &calibration) const {
    calibration.accelerometer[0] = axisX.getScale();
    calibration.accelerometer[1] = 0;
    calibration.accelerometer[2] = 0;
    calibration.accelerometer[3] = 0;
    calibration.accelerometer[4] = axisY.getScale();
    calibration.accelerometer[5] = 0;
    calibration.accelerometer[6] = 0;
    calibration.accelerometer[7] = 0;
    calibration.accelerometer[8] = axisZ.getScale();

    calibration.accelerometer[9] = axisX.getOffset();
    calibration.accelerometer[10] = axisY.getOffset();
    calibration.accelerometer[11] = axisZ.getOffset();
}
