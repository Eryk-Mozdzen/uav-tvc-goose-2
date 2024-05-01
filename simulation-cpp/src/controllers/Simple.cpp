#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/discrete_derivative.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include "Simple.h"
#include "Params.h"
#include "Plant.h"
#include "Selector.h"
#include "Function.h"


Simple::PFL::PFL() : Controller{6, 4, 3, 4} {

}

Eigen::VectorX<double> Simple::PFL::calculate(const Eigen::VectorX<double> &state, const Eigen::VectorX<double> &trajectory) const {
    const Eigen::Vector<double, 6> q = state.segment(0, 6);
    const Eigen::Vector<double, 6> dq = state.segment(6, 6);
    const Eigen::Vector<double, 4> y = trajectory.segment(0, 4);
    const Eigen::Vector<double, 4> dy = trajectory.segment(4, 4);
    const Eigen::Vector<double, 4> ddy = trajectory.segment(8, 4);

    const Eigen::Matrix<double, 4, 6> H {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
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

Simple::Simple() {
    drake::systems::DiagramBuilder<double> builder;

    auto pfl = builder.AddSystem<PFL>();
    //auto conv = builder.AddSystem<Function>(4, 5, [](const Eigen::VectorX<double> &generalized) {return Plant::decodeControls(generalized);});

    builder.ExportInput(pfl->get_trajectory_input_port(), "trajectory");
    builder.ExportInput(pfl->get_state_input_port(), "state");

    //builder.Connect(pfl->get_control_output_port(), conv->get_input_port());

    //builder.ExportOutput(conv->get_output_port(), "control");
    builder.ExportOutput(pfl->get_control_output_port(), "control");

    builder.BuildInto(this);
}

const drake::systems::InputPort<double> & Simple::get_state_input_port() const {
    return GetInputPort("state");
}

const drake::systems::InputPort<double> & Simple::get_trajectory_input_port() const {
    return GetInputPort("trajectory");
}

const drake::systems::OutputPort<double> & Simple::get_control_output_port() const {
    return GetOutputPort("control");
}
