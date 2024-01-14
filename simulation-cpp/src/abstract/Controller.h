#pragma once

#include "Object.h"
#include "TrajectoryGenerator.h"

template<int N>
class Controller {
public:
    virtual Object::U operator()(const Object::State &state, const typename TrajectoryGenerator<N>::Trajectory &desired) = 0;
};
