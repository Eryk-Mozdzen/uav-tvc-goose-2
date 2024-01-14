#pragma once

#include <thread>
#include <mutex>

#include "Object.h"
#include "Controller.h"
#include "TrajectoryGenerator.h"

class Simulation {
    static constexpr double dt = 0.01;

    bool running;
    std::thread thread;
    std::mutex mutex;

    Object object;
    Controller<4> *controller = nullptr;
    TrajectoryGenerator<4> *generator = nullptr;

    Object::U u;
    TrajectoryGenerator<4>::Trajectory desired;
    double time;

    void loop();

public:
    struct Result {
        double time;
        Object::State state;
        Object::Control control;
        TrajectoryGenerator<4>::Trajectory desired;
    };

    Simulation(Controller<4> *controller, TrajectoryGenerator<4> *generator);
    ~Simulation();

    Result read();
};
