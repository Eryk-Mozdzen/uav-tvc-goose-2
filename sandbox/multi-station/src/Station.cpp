#include "Station.h"

Station::Station(const float x, const float y) : MeasurementModel{1}, x{x}, y{y} {

}

float Station::getRSSI(const float x, const float y) const {
    const float dx = x - this->x;
    const float dy = y - this->y;

    return K1 + K2*std::log10(std::sqrt(dx*dx + dy*dy));
}

Eigen::Vector<float, 1> Station::h(const Eigen::Vector<float, 2> &x) const {
    const float dx = x(0) - this->x;
    const float dy = x(1) - this->y;

    return Eigen::Vector<float, 1>(
        K1 + K2*std::log10(std::sqrt(dx*dx + dy*dy))
    );
}

Eigen::Matrix<float, 1, 2> Station::dh(const Eigen::Vector<float, 2> &x) const {
    const float dx = x(0) - this->x;
    const float dy = x(1) - this->y;

    return Eigen::Matrix<float, 1, 2>(
        K2*dx/((dx*dx + dy*dy)*std::log(10)),
        K2*dy/((dx*dx + dy*dy)*std::log(10))
    );
}
