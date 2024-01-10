#pragma once

#include <thread>
#include <mutex>

#include "TrajectoryGenerator.h"
#include "Gamepad.h"

class ManualControl : public TrajectoryGenerator<4> {
    static constexpr double dt = 0.05;

    Gamepad gamepad;
    Eigen::Matrix<double, 4, 3> result;

    bool running;
    std::thread thread;
    std::mutex mutex;

    void loop();

public:
    ManualControl();
    ~ManualControl();

    Eigen::Matrix<double, 4, 3> get(const double time);
};
