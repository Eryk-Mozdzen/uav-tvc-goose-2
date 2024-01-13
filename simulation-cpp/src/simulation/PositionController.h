#pragma once

#include <Eigen/Dense>

#include "Object.h"

class PositionController {
    static constexpr double dt = 0.01;

    const Eigen::Matrix<double, 2, 2> Kp = 3*Eigen::Matrix<double, 2, 2>::Identity();
    const Eigen::Matrix<double, 2, 2> Kd = 3*Eigen::Matrix<double, 2, 2>::Identity();

public:
    struct Desired {
        Eigen::Vector<double, 2> y;
        Eigen::Vector<double, 2> dy;
        Eigen::Vector<double, 2> ddy;

        Desired();
    };

    struct Output {
        Eigen::Vector<double, 2> y;
        Eigen::Vector<double, 2> dy;
        Eigen::Vector<double, 2> ddy;

        Output();
    };

    PositionController();

    Output operator()(const Object::State &state, const Desired &desired);

private:
    Output last;
};
