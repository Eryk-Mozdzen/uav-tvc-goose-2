#include <cassert>

#include "Object.h"
#include "Params.h"

double Object::w_t = 0;

Object::Object() {
    q.setZero();
    dq.setZero();

    dq_last.setZero();
    ddq_last.setZero();

    assert(!q.hasNaN());
    assert(!dq.hasNaN());
    assert(!dq_last.hasNaN());
    assert(!ddq_last.hasNaN());
}

void Object::step(const double dt, const U &u) {
    Q input;
    input.setZero();
    input.block(2, 0, 4, 1) = u;

    assert(!u.hasNaN());
    assert(!q.hasNaN());
    assert(!dq.hasNaN());
    assert(!dq_last.hasNaN());
    assert(!ddq_last.hasNaN());

    const Q ddq = M(q).inverse()*(input - C(q, dq)*dq - D(q));

    dq +=dt*0.5*(ddq_last + ddq);
    q +=dt*0.5*(dq_last + dq);

    dq_last = dq;
    ddq_last = ddq;

    Object::w_t = Object::decodeControls(u).omega;

    assert(!ddq.hasNaN());
    assert(!dq.hasNaN());
    assert(!q.hasNaN());
}

Object::State Object::getState() const {
    return {q, dq};
}

Eigen::Matrix<double, Object::dimQ, Object::dimQ> Object::M(const Q &q) {
    assert(!q.hasNaN());

    const Eigen::DiagonalMatrix<double, 3, 3> J(Params::J_xx, Params::J_yy, Params::J_zz);
    const Eigen::DiagonalMatrix<double, 3, 3> L(1/Params::l, 1/Params::l, -1/Params::r);

    Eigen::Matrix<double, dimQ, dimQ> M;

    M.setZero();
    M.block(0, 0, 3, 3) = Params::m*R(q).inverse();
    M.block(3, 3, 3, 3) = J*W(q).inverse()*L;

    M.row(0) +=M.row(4);
    M.row(1) -=M.row(3);

    assert(!M.hasNaN());

    return M;
}

Eigen::Matrix<double, Object::dimQ, Object::dimQ> Object::C(const Q &q, const Q &dq) {
    assert(!q.hasNaN());
    assert(!dq.hasNaN());

    const Eigen::DiagonalMatrix<double, 3, 3> J(Params::J_xx, Params::J_yy, Params::J_zz);
    const Eigen::DiagonalMatrix<double, 3, 3> L(1/Params::l, 1/Params::l, -1/Params::r);

    const double phi = q(3);
    const double theta = q(4);

    const double d_phi = dq(3);
    const double d_theta = dq(4);
    const double d_psi = dq(5);

    const double s_phi = std::sin(phi);
    const double c_phi = std::cos(phi);
    const double s_theta = std::sin(theta);
    const double c_theta = std::cos(theta);

    const Eigen::Matrix3d Winv = W(q).inverse();
    const Eigen::Matrix3d Rinv = R(q).inverse();

    const Eigen::Matrix3d C1 {
        {0, (Params::J_zz - Params::J_yy)*(-s_phi*d_theta + c_phi*c_theta*d_psi), 0},
        {0, 0, (Params::J_xx - Params::J_zz)*(d_phi - s_theta*d_psi)},
        {(Params::J_yy - Params::J_xx)*(c_phi*d_theta + c_theta*s_phi*d_psi), 0, 0}
    };

    const Eigen::Matrix3d C2 {
        { 0,                       Params::J_r*Object::w_t, 0},
        {-Params::J_r*Object::w_t, 0,                       0},
        { 0,                       0,                       0}
    };

    Eigen::Matrix<double, dimQ, dimQ> C;

    C.setZero();
    C.block(0, 0, 3, 3) = Params::m*Rinv*dR(q, dq)*Rinv;
    C.block(3, 3, 3, 3) = J*Winv*dW(q, dq)*Winv*L + C1*Winv*L + C2*Winv*L;

    C.row(0) +=C.row(4);
    C.row(1) -=C.row(3);

    assert(!C.hasNaN());

    return C;
}

Eigen::Vector<double, Object::dimQ> Object::D(const Q &q) {
    assert(!q.hasNaN());

    Eigen::Vector<double, dimQ> D;

    D.setZero();
    D.block(0, 0, 3, 1) = Params::m*R(q).inverse()*Eigen::Vector3d(0, 0, Params::g);

    const double Ft = Params::K_w*Object::w_t*Object::w_t;
    const double Fs = Params::K_l*Ft*Params::a_s;
    const double Mr = Params::K_m*Ft;
    D(2) +=(4*Fs - Params::r*Mr);

    D.row(0) +=D.row(4);
    D.row(1) -=D.row(3);

    assert(!D.hasNaN());

    return D;
}

Eigen::Matrix3d Object::R(const Q &q) {
    assert(!q.hasNaN());

    const double phi = q(3);
    const double theta = q(4);
    const double psi = q(5);

    const double s_phi = std::sin(phi);
    const double c_phi = std::cos(phi);
    const double s_theta = std::sin(theta);
    const double c_theta = std::cos(theta);
    const double s_psi = std::sin(psi);
    const double c_psi = std::cos(psi);

    const Eigen::Matrix3d Rz {
        {c_psi, -s_psi, 0},
        {s_psi,  c_psi, 0},
        {0,      0,     1},
    };

    const Eigen::Matrix3d Ry {
        { c_theta, 0, s_theta},
        { 0,       1, 0      },
        {-s_theta, 0, c_theta},
    };

    const Eigen::Matrix3d Rx {
        {1, 0,      0    },
        {0, c_phi, -s_phi},
        {0, s_phi,  c_phi},
    };

    const Eigen::Matrix3d R = Rz*Ry*Rx;

    assert(!R.hasNaN());

    return R;
}

Eigen::Matrix3d Object::W(const Q &q) {
    assert(!q.hasNaN());

    const double phi = q(3);
    const double theta = q(4);

    const double s_phi = std::sin(phi);
    const double c_phi = std::cos(phi);
    const double c_theta = std::cos(theta);
    const double t_theta = std::tan(theta);

    const Eigen::Matrix3d W {
        {1, s_phi*t_theta,  c_phi*t_theta},
        {0, c_phi,         -s_phi        },
        {0, s_phi/c_theta,  c_phi/c_theta},
    };

    assert(!W.hasNaN());

    return W;
}

Eigen::Matrix3d Object::dR(const Q &q, const Q &dq) {
    assert(!q.hasNaN());

    const double phi = q(3);
    const double theta = q(4);
    const double psi = q(5);

    const double d_phi = dq(3);
    const double d_theta = dq(4);
    const double d_psi = dq(5);

    const double s_phi = std::sin(phi);
    const double c_phi = std::cos(phi);
    const double s_theta = std::sin(theta);
    const double c_theta = std::cos(theta);
    const double s_psi = std::sin(psi);
    const double c_psi = std::cos(psi);

    const Eigen::Matrix3d Rz {
        {c_psi, -s_psi, 0},
        {s_psi,  c_psi, 0},
        {0,      0,     1},
    };

    const Eigen::Matrix3d Ry {
        { c_theta, 0, s_theta},
        { 0,       1, 0      },
        {-s_theta, 0, c_theta},
    };

    const Eigen::Matrix3d Rx {
        {1, 0,      0    },
        {0, c_phi, -s_phi},
        {0, s_phi,  c_phi},
    };

    const Eigen::Matrix3d dRz {
        {-s_psi*d_psi, -c_psi*d_psi, 0},
        { c_psi*d_psi,  s_psi*d_psi, 0},
        { 0,            0,           0},
    };

    const Eigen::Matrix3d dRy {
        {-s_theta*d_theta, 0,  c_theta*d_theta},
        { 0,               0,  0              },
        {-c_theta*d_theta, 0, -s_theta*d_theta},
    };

    const Eigen::Matrix3d dRx {
        {0, 0,      0    },
        {0, -s_phi*d_phi, -c_phi*d_phi},
        {0,  c_phi*d_phi, -s_phi*d_phi},
    };

    const Eigen::Matrix3d dR = dRz*Ry*Rx + Rz*dRy*Rx + Rz*Ry*dRx;

    assert(!dR.hasNaN());

    return dR;
}

Eigen::Matrix3d Object::dW(const Q &q, const Q &dq) {
    assert(!q.hasNaN());

    const double phi = q(3);
    const double theta = q(4);

    const double d_phi = dq(3);
    const double d_theta = dq(4);

    const double sin_phi = std::sin(phi);
    const double cos_phi = std::cos(phi);
    const double tan_theta = std::tan(theta);
    const double sec_theta = 1./std::cos(theta);

    const Eigen::Matrix3d dW {
        {0,  sec_theta*sec_theta*sin_phi*d_theta + cos_phi*tan_theta*d_phi,  cos_phi*sec_theta*sec_theta*d_theta - sin_phi*tan_theta*d_phi},
        {0, -sin_phi*d_phi,                                                 -cos_phi*d_phi                                                },
        {0,  sec_theta*(sin_phi*tan_theta*d_theta + cos_phi*d_phi),          sec_theta*(cos_phi*tan_theta*d_theta - sin_phi*d_phi)        },
    };

    assert(!dW.hasNaN());

    return dW;
}

Object::Control Object::decodeControls(const U &general) {
    Control real;
    real.omega = std::sqrt(general(0)/Params::K_w);

    const double C31 = general(1)/(Params::K_l*general(0));
    const double C42 = general(2)/(Params::K_l*general(0));
    const double Cs = general(3)/(Params::K_l*general(0));
    const double C = (Cs - C31 - C42)/2;

    real.alpha[0] = (2*C - C31 + C42)/4;
    real.alpha[1] = C - real.alpha[0];
    real.alpha[2] = C31 + real.alpha[0];
    real.alpha[3] = C42 + C - real.alpha[0];

    return real;
}

Object::U Object::encodeControls(const Control &real) {
    const double F_t = Params::K_w*real.omega*real.omega;
    const double F_1 = Params::K_l*F_t*real.alpha[0];
    const double F_2 = Params::K_l*F_t*real.alpha[1];
    const double F_3 = Params::K_l*F_t*real.alpha[2];
    const double F_4 = Params::K_l*F_t*real.alpha[3];

    return {
        F_t,
        F_3 - F_1,
        F_4 - F_2,
        F_1 + F_2 + F_3 + F_4
    };
}