#include <cassert>

#include "Controller.h"

HoverController::Desired::Desired() {
    y.setZero();
    dy.setZero();
    ddy.setZero();
}

HoverController::HoverController() {
    assert(!Kp.hasNaN());
    assert(!Kd.hasNaN());
}

Eigen::Vector<double, 4> HoverController::operator()(const Object::State &state, const HoverController::Desired &desired) const {
    assert(!state.q.hasNaN());
    assert(!state.dq.hasNaN());
    assert(!desired.y.hasNaN());
    assert(!desired.dy.hasNaN());
    assert(!desired.ddy.hasNaN());

    const Eigen::Matrix<double, 4, 6> H {
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 1}
    };

    const Eigen::Vector<double, 4> e = desired.y - H*state.q;
    const Eigen::Vector<double, 4> de = desired.dy - H*state.dq;
    const Eigen::Vector<double, 4> v = desired.ddy + Kd*de + Kp*e;

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
