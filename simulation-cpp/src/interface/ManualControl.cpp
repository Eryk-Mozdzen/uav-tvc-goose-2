#include "ManualControl.h"

ManualControl::ManualControl() : running{true}, thread{&ManualControl::loop, this} {
    result.setZero();
}

ManualControl::~ManualControl() {
    mutex.lock();
    running = false;
    mutex.unlock();

    if(thread.joinable()) {
        thread.join();
    }
}

void ManualControl::loop() {
    while(true) {
        const Eigen::Vector<double, 4> input {
            +gamepad.get(Gamepad::Analog::LX),
            -gamepad.get(Gamepad::Analog::LY),
            -gamepad.get(Gamepad::Analog::RY),
            +gamepad.get(Gamepad::Analog::RX)
        };

        mutex.lock();

        if(!running) {
            return;
        }

        const Eigen::Vector<double, 4> last = result.col(1);

        result.col(0) +=0.5*(input + last)*dt;
        result.col(1) = input;
        //result.col(2) = (input - last)/dt;

        mutex.unlock();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt)));
    }
}

Eigen::Matrix<double, 4, 3> ManualControl::get(const double time) {
    (void)time;

    mutex.lock();

    const Eigen::Matrix<double, 4, 3> copy = result;

    mutex.unlock();

    return copy;
};
