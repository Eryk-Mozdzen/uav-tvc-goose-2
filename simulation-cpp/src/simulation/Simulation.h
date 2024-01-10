#pragma once

#include <thread>
#include <mutex>

#include "Object.h"
#include "Controller.h"
#include "TrajectoryGenerator.h"

class Simulation {
    static constexpr double dt = 0.005;

    bool running;
    std::thread thread;
    std::mutex mutex;

    Object object;
    Controller controller;
    TrajectoryGenerator<4> *generator;

    Object::U u;
    Controller::Desired desired;
    double time;

    void loop();

public:
    struct Result {
        double time;
        Object::State state;
        Object::Control control;
        Controller::Desired desired;
    };

    Simulation(TrajectoryGenerator<4> *generator);
    ~Simulation();

    Result read();
};
