#pragma once

#include <Eigen/Dense>

#include "Controller.h"

class PFL : public Controller<4> {

    class PositionController {
        using Y = Eigen::Vector<double, 2>;
        using U = Eigen::Vector<double, 2>;

        static constexpr double dt = 0.01;

        const Eigen::Matrix<double, 2, 2> Kp = 3*Eigen::Matrix<double, 2, 2>::Identity();
        const Eigen::Matrix<double, 2, 2> Kd = 3*Eigen::Matrix<double, 2, 2>::Identity();

        U last_u;

    public:
        struct Output {
            U u;
            U du;
        };

        PositionController();

        Output operator()(const Object::State &state, const Y &y, const Y &dy, const Y &ddy);
    };

    class HoverController {
    public:
        using Y = Eigen::Vector<double, 4>;
        using U = Eigen::Vector<double, 4>;

    private:
        const Eigen::Matrix<double, 4, 4> Kp = 5*Eigen::Matrix<double, 4, 4>::Identity();
        const Eigen::Matrix<double, 4, 4> Kd = 10*Eigen::Matrix<double, 4, 4>::Identity();

    public:
        HoverController();

        U operator()(const Object::State &state, const Y &y, const Y &dy, const Y &ddy) const;
    };

    PositionController positionController;
    HoverController hoverController;

public:
    PFL();

    Object::U operator()(const Object::State &state, const TrajectoryGenerator<4>::Trajectory &desired);
};
