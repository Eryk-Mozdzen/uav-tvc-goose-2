#pragma once

#include <thread>
#include <mutex>

#include "TrajectoryGenerator.h"
#include "Gamepad.h"

class Manual : public TrajectoryGenerator<4> {
    static constexpr double dt = 0.05;

    Gamepad gamepad;
    Trajectory trajectory;

    bool running;
    std::thread thread;
    std::mutex mutex;

    void loop();

public:
    Manual();
    ~Manual();

    Trajectory get(const double time);
};
