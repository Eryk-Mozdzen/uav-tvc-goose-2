#pragma once

#include <Eigen/Dense>

template<int N>
class TrajectoryGenerator {
public:
    virtual Eigen::Matrix<double, N, 3> get(const double time) = 0;
};
