#pragma once

#include <thread>
#include <mutex>
#include <drake/systems/analysis/simulator.h>

template<typename T>
class Simulator : public drake::systems::Simulator<T> {
	std::thread thread;
	std::mutex mutex;
	bool running = true;

	void run();

public:
	Simulator(const drake::systems::System<T> &system);
	~Simulator();
	void StartAdvance();
};

template<typename T>
Simulator<T>::Simulator(const drake::systems::System<T> &system) : drake::systems::Simulator<T>{system} {

}

template<typename T>
Simulator<T>::~Simulator() {
    mutex.lock();
    running = false;
    mutex.unlock();
    thread.join();
}

template<typename T>
void Simulator<T>::StartAdvance() {
    thread = std::thread(&Simulator::run, this);
}

template<typename T>
void Simulator<T>::run() {
    mutex.lock();
    while(running) {
        mutex.unlock();
        this->AdvanceTo(this->get_context().get_time() + 0.1);
        mutex.lock();
    }
}
