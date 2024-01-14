#include "Manual.h"

Manual::Manual() : running{true}, thread{&Manual::loop, this} {

}

Manual::~Manual() {
    mutex.lock();
    running = false;
    mutex.unlock();

    if(thread.joinable()) {
        thread.join();
    }
}

void Manual::loop() {
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

        const Eigen::Vector<double, 4> last = trajectory.dy;

        trajectory.y +=0.5*(input + last)*dt;
        trajectory.dy = input;
        //trajectory.ddy = (input - last)/dt;

        mutex.unlock();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt)));
    }
}

Manual::Trajectory Manual::get(const double time) {
    (void)time;

    mutex.lock();

    const Trajectory copy = trajectory;

    mutex.unlock();

    return copy;
};
