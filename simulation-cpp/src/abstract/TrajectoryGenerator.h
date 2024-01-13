#pragma once

#include <Eigen/Dense>

template<int N>
class TrajectoryGenerator {
public:
    struct Trajectory {
        Eigen::Vector<double, N> y;
        Eigen::Vector<double, N> dy;
        Eigen::Vector<double, N> ddy;

        Trajectory() {
            y.setZero();
            dy.setZero();
            ddy.setZero();
        }
    };

    virtual Trajectory get(const double time) = 0;
};
