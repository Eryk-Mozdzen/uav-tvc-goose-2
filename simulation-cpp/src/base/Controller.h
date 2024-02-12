#pragma once

#include <drake/systems/framework/leaf_system.h>

class Controller : public drake::systems::LeafSystem<double> {
    const int state_dim;
    const int trajectory_dim;
    const int trajectory_derivatives_num;
    const int control_dim;

    int state_input_port;
    int trajectory_input_port;

    void EvalOutput(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;

    virtual Eigen::VectorX<double> calculate(const Eigen::VectorX<double> &q, const Eigen::VectorX<double> &dq, const Eigen::VectorX<double> &trajectory) const = 0;

protected:
    static double dt;
    static double last_time;

public:
    Controller(const int state_dim, const int trajectory_dim, const int trajectory_derivatives_num, const int control_dim);
};
