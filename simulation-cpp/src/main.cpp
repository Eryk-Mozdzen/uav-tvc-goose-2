#include <iostream>
#include <iomanip>

#include <QApplication>
#include <QTimer>

#include "TrajectoryGenerator.h"
#include "ManualControl.h"
#include "Simulation.h"
#include "Client.h"
#include "Chart.h"
#include "GraphXY.h"

class LemniscateTrajectory : public TrajectoryGenerator<4> {
	const double a;
	const double w;

public:
	LemniscateTrajectory(const double a, const double f) : a{a}, w{2*3.1415*f} {

	}

	Eigen::Matrix<double, 4, 3> get(const double time) {
		const double s = std::sin(w*time);
		const double c = std::cos(w*time);

		const Eigen::Vector<double, 4> y {
			a*c,
			a*s*c,
			1,
			0//std::atan2(s*s - c*c, s) + 3.1415
		};

		const Eigen::Vector<double, 4> dy {
			-a*w*s,
			a*w*(c*c - s*s),
			0,
			0
		};

		const Eigen::Vector<double, 4> ddy {
			-a*w*w*c,
			-4*a*w*w*s*c,
			0,
			0
		};

		Eigen::Matrix<double, 4, 3> result;
		result.col(0) = y;
		result.col(1) = dy;
		result.col(2) = ddy;

		return result;
	}
};

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);

	//Simulation simulation(new ManualControl());
	Simulation simulation(new LemniscateTrajectory(3, 0.1));
    Client client;
	Chart chartAlpha("Thrust Vanes", "α [°]", "%+3.0f", -10, 10);
	Chart chartOmega("Rotor Velocity", "ω [rad/s]", "%4.0f", 0, 1600);
	GraphXY graph("XY Trajectory", "%+3.1f", 5);

	Chart::Series alpha1(chartAlpha, Qt::red);
	Chart::Series alpha2(chartAlpha, Qt::green);
	Chart::Series alpha3(chartAlpha, Qt::blue);
	Chart::Series alpha4(chartAlpha, Qt::magenta);
	Chart::Series omega(chartOmega);
	GraphXY::Series trajectory(graph, Qt::black, Qt::DashLine, 1);
	GraphXY::Series position(graph, Qt::red);

	QTimer timer;

    QObject::connect(&timer, &QTimer::timeout, [&](){

		const Simulation::Result result = simulation.read();

        client.draw(result.state);

		constexpr double rad2deg = 180/3.1415;

		alpha1.append(result.control.alpha[0]*rad2deg);
		alpha2.append(result.control.alpha[1]*rad2deg);
		alpha3.append(result.control.alpha[2]*rad2deg);
		alpha4.append(result.control.alpha[3]*rad2deg);
		omega.append(result.control.omega);

		trajectory.append(result.desired.y(0), result.desired.y(1));
		position.append(result.state.q(0), result.state.q(1));
    });

	timer.start(50);

	return app.exec();
}
