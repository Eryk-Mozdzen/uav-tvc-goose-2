#include "Controller.h"

double Controller::dt;
double Controller::last_time;

Controller::Controller(const int state_dim, const int trajectory_dim, const int trajectory_derivatives_num, const int control_dim) : state_dim{state_dim}, trajectory_dim{trajectory_dim}, trajectory_derivatives_num{trajectory_derivatives_num}, control_dim{control_dim} {
    trajectory_input_port = DeclareVectorInputPort("trajectory", trajectory_dim*trajectory_derivatives_num).get_index();
    state_input_port = DeclareVectorInputPort("state", 2*state_dim).get_index();

    DeclareVectorOutputPort("control", control_dim, &Controller::EvalOutput);
}

void Controller::EvalOutput(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorX<double> trajectory = EvalVectorInput(context, trajectory_input_port)->CopyToVector();
    const Eigen::VectorX<double> state = EvalVectorInput(context, state_input_port)->CopyToVector();

    const Eigen::VectorX<double> q = state.segment(0, state_dim);
    const Eigen::VectorX<double> dq = state.segment(state_dim, state_dim);

    dt = context.get_time() - last_time;
    last_time = context.get_time();

    const Eigen::VectorX<double> u = calculate(q, dq, trajectory);

    output->SetFromVector(u);
}
