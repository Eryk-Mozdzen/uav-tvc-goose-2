#pragma once

#include <drake/systems/framework/leaf_system.h>

template<typename MySystem>
class VisualizationSink : public drake::systems::LeafSystem<double> {
    MySystem *system;

    drake::systems::EventStatus update(const drake::systems::Context<double> &context, drake::systems::State<double> *state) const;

public:
    VisualizationSink(const double frequency, const int size, MySystem *system);
};

template<typename MySystem>
VisualizationSink<MySystem>::VisualizationSink(const double frequency, const int size, MySystem *system) : system{system} {
    const double period = 1/frequency;

    this->DeclareVectorInputPort("input", size);
    this->DeclarePeriodicUnrestrictedUpdateEvent(period, 0, &VisualizationSink<MySystem>::update);
}

template<typename MySystem>
drake::systems::EventStatus VisualizationSink<MySystem>::update(const drake::systems::Context<double> &context, drake::systems::State<double> *state) const {
    (void)state;

    const Eigen::VectorX<double> &value = this->EvalVectorInput(context, 0)->CopyToVector();

    system->callback(value);

    return drake::systems::EventStatus::Succeeded();
}
