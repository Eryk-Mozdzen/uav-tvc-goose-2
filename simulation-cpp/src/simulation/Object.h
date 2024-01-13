#pragma once

#include <Eigen/Dense>

class Object {
    static constexpr int dimQ = 6;
    static constexpr int dimU = 4;

    static double w_t;

    using Q = Eigen::Vector<double, dimQ>;

    Q q;
    Q dq;

    Q dq_last;
    Q ddq_last;

    static Eigen::Matrix3d R(const Q &q);
    static Eigen::Matrix3d W(const Q &q);

    static Eigen::Matrix3d dR(const Q &q, const Q &dq);
    static Eigen::Matrix3d dW(const Q &q, const Q &dq);

public:
    using U = Eigen::Vector<double, dimU>;

    struct Control {
        double alpha[4];
        double omega;
    };

    struct State {
        Q q;
        Q dq;
    };

    Object();

    void step(const double dt, const U &u);

    State getState() const;

    static Eigen::Matrix<double, dimQ, dimQ> M(const Q &q);
    static Eigen::Matrix<double, dimQ, dimQ> C(const Q &q, const Q &dq);
    static Eigen::Vector<double, dimQ> D(const Q &q);

    static Control decodeControls(const U &general);
    static U encodeControls(const Control &real);
};
