#include "PFL.h"
#include "Params.h"

PFL::HoverController::HoverController() {

}

PFL::HoverController::U PFL::HoverController::operator()(const Object::State &state, const Y &y, const Y &dy, const Y &ddy) const {
    assert(!state.q.hasNaN());
    assert(!state.dq.hasNaN());
    assert(!y.hasNaN());
    assert(!dy.hasNaN());
    assert(!ddy.hasNaN());

    const Eigen::Matrix<double, 4, 6> H {
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 1}
    };

    const Eigen::Vector<double, 4> e = y - H*state.q;
    const Eigen::Vector<double, 4> de = dy - H*state.dq;
    const Eigen::Vector<double, 4> v = ddy + Kd*de + Kp*e;

    const Eigen::Matrix<double, 6, 6> M = Object::M(state.q);
    const Eigen::Vector<double, 6> T = -Object::D(state.q) - Object::C(state.q, state.dq)*state.dq;

    const Eigen::Matrix<double, 2, 2> M11 = M.block(0, 0, 2, 2);
    const Eigen::Matrix<double, 2, 4> M12 = M.block(0, 2, 2, 4);
    const Eigen::Matrix<double, 4, 2> M21 = M.block(2, 0, 4, 2);
    const Eigen::Matrix<double, 4, 4> M22 = M.block(2, 2, 4, 4);
    const Eigen::Vector<double, 2> T1 = T.block(0, 0, 2, 1);
    const Eigen::Vector<double, 4> T2 = T.block(2, 0, 4, 1);

    const Eigen::Matrix<double, 4, 2> H1 = H.block(0, 0, 4, 2);
    const Eigen::Matrix<double, 4, 4> H2 = H.block(0, 2, 4, 4);
    const Eigen::Matrix<double, 4, 6> dH = Eigen::Matrix<double, 4, 6>::Zero();
    const Eigen::Matrix<double, 4, 4> Hd = H2 - H1*M11.inverse()*M12;
    const Eigen::Matrix<double, 4, 4> Hdp = Hd.transpose()*((Hd*Hd.transpose()).inverse());

    const Eigen::Vector<double, 4> ddq2 = Hdp*(v - dH*state.dq - H1*M11.inverse()*T1);
    const Eigen::Vector<double, 2> ddq1 = M11.inverse()*(T1 - M12*ddq2);
    const Eigen::Vector<double, 4> u = M21*ddq1 + M22*ddq2 - T2;

    assert(!e.hasNaN());
    assert(!de.hasNaN());
    assert(!v.hasNaN());
    assert(!T.hasNaN());
    assert(!H.hasNaN());
    assert(!dH.hasNaN());
    assert(!Hd.hasNaN());
    assert(!Hdp.hasNaN());
    assert(!ddq2.hasNaN());
    assert(!ddq1.hasNaN());
    assert(!u.hasNaN());

    return u;
}

PFL::PositionController::PositionController() {
    last_u.setZero();
}

PFL::PositionController::Output PFL::PositionController::operator()(const Object::State &state, const Y &y, const Y &dy, const Y &ddy) {
    const double psi = state.q(5);

    const Eigen::Matrix2d Rz {
        { std::cos(psi), std::sin(psi)},
        {-std::sin(psi), std::cos(psi)}
    };

    const Eigen::Vector<double, 2> e = y - state.q.block(0, 0, 2, 1);
    const Eigen::Vector<double, 2> de = dy - state.dq.block(0, 0, 2, 1);
    const Eigen::Vector<double, 2> v = Rz*(ddy + Kd*de + Kp*e);

    const double vddx = v(0);
    const double vddy = v(1);

    const double ref_phi = std::atan(-vddy/Params::g);
    const double ref_theta = std::atan(vddx*std::cos(ref_phi)/Params::g);

    Output output;
    output.u = Eigen::Vector<double, 2>(ref_phi, ref_theta);
    output.du = (output.u  - last_u)/dt;
    //output.ddu = (output.du  - last_du)/dt;

    last_u = output.u;

    return output;
}

PFL::PFL() {

}

Object::U PFL::operator()(const Object::State &state, const TrajectoryGenerator<4>::Trajectory &desired) {

    const PositionController::Output output = positionController(state,
        desired.y.block(0, 0, 2, 1),
        desired.dy.block(0, 0, 2, 1),
        desired.ddy.block(0, 0, 2, 1)
    );

    HoverController::Y y;
    HoverController::Y dy;
    HoverController::Y ddy;

    y.setZero();
    dy.setZero();
    ddy.setZero();

    y.block(0, 0, 2, 1) = output.u;
    dy.block(0, 0, 2, 1) = output.du;
    //ddy.block(0, 0, 2, 1) = output.ddu;
    y.block(2, 0, 2, 1) = desired.y.block(2, 0, 2, 1);
    dy.block(2, 0, 2, 1) = desired.dy.block(2, 0, 2, 1);
    ddy.block(2, 0, 2, 1) = desired.ddy.block(2, 0, 2, 1);

    return hoverController(state, y, dy, ddy);
}
