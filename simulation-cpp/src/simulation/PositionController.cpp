#include <cassert>
#include <iostream>

#include "Params.h"
#include "Controller.h"

PositionController::Desired::Desired() {
    y.setZero();
    dy.setZero();
    ddy.setZero();
}

PositionController::Output::Output() {
    y.setZero();
    dy.setZero();
    ddy.setZero();
}

PositionController::PositionController() {

}

PositionController::Output PositionController::operator()(const Object::State &state, const PositionController::Desired &desired) {
    const double psi = state.q(5);

    const Eigen::Matrix2d Rz {
        { std::cos(psi), std::sin(psi)},
        {-std::sin(psi), std::cos(psi)}
    };

    const Eigen::Vector<double, 2> e = Rz*(desired.y - state.q.block(0, 0, 2, 1));
    const Eigen::Vector<double, 2> de = Rz*(desired.dy - state.dq.block(0, 0, 2, 1));
    const Eigen::Vector<double, 2> v = desired.ddy + Kd*de + Kp*e;

    const double ddx = v(0);
    const double ddy = v(1);

    const double phi_ref = std::atan(-ddy/Params::g);
    const double theta_ref = std::atan(ddx*std::cos(phi_ref)/Params::g);

    Output output;
    output.y = Eigen::Vector<double, 2>(phi_ref, theta_ref);
    output.dy = (output.y  - last.y)/dt;
    //output.ddy = (output.dy  - last.dy)/dt;

    last = output;

    return output;
}
