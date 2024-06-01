#include <chrono>
#include <thread>
#include <random>
#include <iostream>
#include <iomanip>

#include "EKF.h"
#include "Station.h"

class System : public EKF<float, 2>::SystemModel<0> {
public:
	System() : SystemModel{1} {}

	Eigen::Vector<float, 2> f(const Eigen::Vector<float, 2> &x, const Eigen::Vector<float, 0> &u) const {
		(void)u;
		return x;
	}

	Eigen::Matrix<float, 2, 2> df(const Eigen::Vector<float, 2> &x, const Eigen::Vector<float, 0> &u) const {
		(void)x;
		(void)u;
		return Eigen::Matrix<float, 2, 2>::Identity();
	}
};

int main() {
	EKF<float, 2> ekf({0, 0});

	const std::array stations = {
		Station(+10, +10),
		Station(+10, -10),
		Station(-10, +10),
		Station(-10, -10),
	};

	float t = 0;
	constexpr float pi = 3.1415926535;
	constexpr float f = 0.01;

	std::cout << std::setprecision(3) << std::fixed;

	std::default_random_engine generator;
    std::normal_distribution<float> gaussian(0, 0.1);

	while(true) {
		const float x = std::cos(2*pi*f*t);
		const float y = std::sin(2*pi*f*t);

		ekf.predict(System(), Eigen::Vector<float, 0>());

		for(const Station &station : stations) {
			const float rssi = station.getRSSI(x, y) + gaussian(generator);

			ekf.correct(station, Eigen::Vector<float, 1>(rssi));
		}

		//std::cout << ekf.getState().transpose() << std::endl;
		std::cout << (ekf.getState() - Eigen::Vector2f(x, y)).norm() << std::endl;

		t +=0.1;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}
