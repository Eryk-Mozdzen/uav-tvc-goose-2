#include "PFL.h"
#include "Params.h"
#include "Plant.h"

Eigen::Vector<double, 2> PFL::PositionController::last_u;
Eigen::Vector<double, 2> PFL::PositionController::last_du;

PFL::HoverController::HoverController() {

}

PFL::HoverController::U PFL::HoverController::operator()(const Q &q, const Q &dq, const Y &y, const Y &dy, const Y &ddy) const {
    const Eigen::Matrix<double, 4, 6> H {
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 1}
    };

    const Eigen::Vector<double, 4> e = y - H*q;
    const Eigen::Vector<double, 4> de = dy - H*dq;
    const Eigen::Vector<double, 4> v = ddy + Kd*de + Kp*e;

    const Eigen::Matrix<double, 6, 6> M = Plant::M(q);
    const Eigen::Vector<double, 6> T = -Plant::D(q) - Plant::C(q, dq)*dq;

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

    const Eigen::Vector<double, 4> ddq2 = Hdp*(v - dH*dq - H1*M11.inverse()*T1);
    const Eigen::Vector<double, 2> ddq1 = M11.inverse()*(T1 - M12*ddq2);
    const Eigen::Vector<double, 4> u = M21*ddq1 + M22*ddq2 - T2;

    return u;
}

PFL::PositionController::PositionController() {
    last_u.setZero();
    last_du.setZero();
}

PFL::PositionController::Output PFL::PositionController::operator()(const Q &q, const Q &dq, const Y &y, const Y &dy, const Y &ddy) const {
    const double psi = q(5);

    const Eigen::Matrix2d Rz {
        { std::cos(psi), std::sin(psi)},
        {-std::sin(psi), std::cos(psi)}
    };

    const Eigen::Vector<double, 2> e = y - q.segment(0, 2);
    const Eigen::Vector<double, 2> de = dy - dq.segment(0, 2);
    const Eigen::Vector<double, 2> v = Rz*(ddy + Kd*de + Kp*e);

    const double vddx = v(0);
    const double vddy = v(1);

    const double ref_phi = std::atan(-vddy/Params::g);
    const double ref_theta = std::atan(vddx*std::cos(ref_phi)/Params::g);

    Output output;
    output.u.setZero();
    output.du.setZero();
    output.ddu.setZero();

    output.u = Eigen::Vector<double, 2>(ref_phi, ref_theta);
    if(dt>0.00001) {
        output.du = (output.u  - last_u)/dt;
    }
    //output.ddu = (output.du  - last_du)/dt;

    last_u = output.u;
    last_du = output.du;

    return output;
}

PFL::PFL() : Controller{6, 4, 3, 5} {

}

Eigen::VectorX<double> PFL::calculate(const Eigen::VectorX<double> &q, const Eigen::VectorX<double> &dq, const Eigen::VectorX<double> &trajectory) const {
    const Eigen::Vector<double, 4> desired_y = trajectory.segment(0, 4);
    const Eigen::Vector<double, 4> desired_dy = trajectory.segment(4, 4);
    const Eigen::Vector<double, 4> desired_ddy = trajectory.segment(8, 4);

    const PositionController::Output output = positionController(q, dq,
        desired_y.segment(0, 2),
        desired_dy.segment(0, 2),
        desired_ddy.segment(0, 2)
    );

    HoverController::Y y;
    HoverController::Y dy;
    HoverController::Y ddy;

    y.setZero();
    dy.setZero();
    ddy.setZero();

    y.segment(0, 2) = output.u;
    dy.segment(0, 2) = output.du;
    ddy.segment(0, 2) = output.ddu;
    y.segment(2, 2) = desired_y.segment(2, 2);
    dy.segment(2, 2) = desired_dy.segment(2, 2);
    ddy.segment(2, 2) = desired_ddy.segment(2, 2);

    const Eigen::Vector<double, 4> generalized = hoverController(q, dq, y, dy, ddy);

    return Plant::decodeControls(generalized);
}
