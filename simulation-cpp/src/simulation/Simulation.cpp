#include "Simulation.h"

Simulation::Simulation(Controller<4> *controller, TrajectoryGenerator<4> *generator)
        : running{true}, thread{&Simulation::loop, this}, controller{controller}, generator{generator}, time{0.001} {

}

Simulation::~Simulation() {
    mutex.lock();
    running = false;
    mutex.unlock();

    if(thread.joinable()) {
        thread.join();
    }
}

void Simulation::loop() {
    while(true) {
        mutex.lock();

        if(!running) {
            return;
        }

        if(generator!=nullptr) {
            desired = generator->get(time);
        }

        if(controller!=nullptr) {
            u = (*controller)(object.getState(), desired);
        }

        object.step(dt, u);

        time +=dt;

        mutex.unlock();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt)));
    }
}

Simulation::Result Simulation::read() {
    mutex.lock();

    Result result;
    result.time = time;
    result.desired = desired;
    result.state = object.getState();
    result.control = Object::decodeControls(u);

    mutex.unlock();

    return result;
}
