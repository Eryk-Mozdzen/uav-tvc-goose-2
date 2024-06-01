#pragma once

#include "EKF.h"

class Station : public EKF<float, 2>::MeasurementModel<1> {
    const float x;
    const float y;

    static constexpr float K1 = -2.031862;
    static constexpr float K2 = -3.9955;

public:
    Station(const float x, const float y);

    float getRSSI(const float x, const float y) const;

    Eigen::Vector<float, 1> h(const Eigen::Vector<float, 2> &x) const;
	Eigen::Matrix<float, 1, 2> dh(const Eigen::Vector<float, 2> &x) const;
};
