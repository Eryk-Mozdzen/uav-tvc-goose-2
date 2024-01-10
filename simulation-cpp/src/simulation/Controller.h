#pragma once

#include <Eigen/Dense>

#include "Object.h"
#include "HoverController.h"
#include "PositionController.h"

class Controller {
    HoverController hoverController;
    PositionController positionController;

public:
    struct Desired {
        Eigen::Vector<double, 4> y;
        Eigen::Vector<double, 4> dy;
        Eigen::Vector<double, 4> ddy;

        Desired();
    };

    Controller();

    Eigen::Vector<double, 4> operator()(const Object::State &state, const Desired &desired);
};
