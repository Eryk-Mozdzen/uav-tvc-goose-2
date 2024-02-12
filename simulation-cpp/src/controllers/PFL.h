#pragma once

#include "Controller.h"

class PFL : public Controller {

    class PositionController {
        using Q = Eigen::Vector<double, 6>;
        using Y = Eigen::Vector<double, 2>;
        using U = Eigen::Vector<double, 2>;

        const Eigen::Matrix<double, 2, 2> Kp = 3*Eigen::Matrix<double, 2, 2>::Identity();
        const Eigen::Matrix<double, 2, 2> Kd = 3*Eigen::Matrix<double, 2, 2>::Identity();

        static U last_u;
        static U last_du;

    public:
        struct Output {
            U u;
            U du;
            U ddu;
        };

        PositionController();

        Output operator()(const Q &q, const Q &dq, const Y &y, const Y &dy, const Y &ddy) const;
    };

    class HoverController {
    public:
        using Q = Eigen::Vector<double, 6>;
        using Y = Eigen::Vector<double, 4>;
        using U = Eigen::Vector<double, 4>;

    private:
        const Eigen::Matrix<double, 4, 4> Kp = 5*Eigen::Matrix<double, 4, 4>::Identity();
        const Eigen::Matrix<double, 4, 4> Kd = 10*Eigen::Matrix<double, 4, 4>::Identity();

    public:
        HoverController();

        U operator()(const Q &q, const Q &dq, const Y &y, const Y &dy, const Y &ddy) const;
    };

    PositionController positionController;
    HoverController hoverController;

    Eigen::VectorX<double> calculate(const Eigen::VectorX<double> &q, const Eigen::VectorX<double> &dq, const Eigen::VectorX<double> &trajectory) const;

public:
    PFL();
};
