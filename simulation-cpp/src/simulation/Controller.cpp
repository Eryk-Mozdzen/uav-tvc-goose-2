#include <cassert>
#include <iostream>

#include "Controller.h"

Controller::Desired::Desired() {
    y.setZero();
    dy.setZero();
    ddy.setZero();
}

Controller::Controller() {

}

Eigen::Vector<double, 4> Controller::operator()(const Object::State &state, const Controller::Desired &desired) {

    PositionController::Desired positionReference;
    positionReference.y = desired.y.block(0, 0, 2, 1);
    positionReference.dy = desired.dy.block(0, 0, 2, 1);
    positionReference.ddy = desired.ddy.block(0, 0, 2, 1);

    const PositionController::Output output = positionController(state, positionReference);

    HoverController::Desired hoverReference;
    hoverReference.y.block(0, 0, 2, 1) = output.y;
    hoverReference.dy.block(0, 0, 2, 1) = output.dy;
    hoverReference.ddy.block(0, 0, 2, 1) = output.ddy;
    hoverReference.y.block(2, 0, 2, 1) = desired.y.block(2, 0, 2, 1);
    hoverReference.dy.block(2, 0, 2, 1) = desired.dy.block(2, 0, 2, 1);
    hoverReference.ddy.block(2, 0, 2, 1) = desired.ddy.block(2, 0, 2, 1);

    return hoverController(state, hoverReference);
}
