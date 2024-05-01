#pragma once

#include <drake/systems/framework/diagram.h>

#include "Controller.h"

class Simple : public drake::systems::Diagram<double> {
    class PFL : public Controller {
        const Eigen::Matrix<double, 4, 4> Kp = 1*Eigen::Matrix<double, 4, 4>::Identity();
        const Eigen::Matrix<double, 4, 4> Kd = 2*Eigen::Matrix<double, 4, 4>::Identity();

        Eigen::VectorX<double> calculate(const Eigen::VectorX<double> &state, const Eigen::VectorX<double> &trajectory) const;

    public:
        PFL();
    };

public:
    Simple();

    const drake::systems::InputPort<double> & get_state_input_port() const;
    const drake::systems::InputPort<double> & get_trajectory_input_port() const;
    const drake::systems::OutputPort<double> & get_control_output_port() const;
};
