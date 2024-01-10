#pragma once

#include <Eigen/Dense>

#include "Object.h"

class HoverController {
    const Eigen::Matrix<double, 4, 4> Kp = 5*Eigen::Matrix<double, 4, 4>::Identity();
    const Eigen::Matrix<double, 4, 4> Kd = 10*Eigen::Matrix<double, 4, 4>::Identity();

public:
    struct Desired {
        Eigen::Vector<double, 4> y;
        Eigen::Vector<double, 4> dy;
        Eigen::Vector<double, 4> ddy;

        Desired();
    };

    HoverController();

    Eigen::Vector<double, 4> operator()(const Object::State &state, const Desired &desired) const;
};
