#include <iostream>
#include <iomanip>

#include <QApplication>
#include <QTimer>

#include "TrajectoryGenerator.h"
#include "Simulation.h"
#include "Client.h"
#include "Chart.h"
#include "GraphXY.h"
#include "Manual.h"
#include "Lemniscate.h"
#include "Circle.h"

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);

	//Simulation simulation(new Manual());
	Simulation simulation(new Lemniscate(2, 10));
	//Simulation simulation(new Circle(2, 0, 1.5, 10));

	Client client;
	Chart chartAlpha("Thrust Vanes Angles", "α [°]", "%+3.0f", -20, 20);
	Chart chartOmega("Rotor Angular Velocity", "ω [rad/s]", "%4.0f", 0, 1600);
	GraphXY graph("XY Trajectory", "%+3.1f", 4);

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

		constexpr double rad2deg = 180/3.1415;

		alpha1.append(result.control.alpha[0]*rad2deg);
		alpha2.append(result.control.alpha[1]*rad2deg);
		alpha3.append(result.control.alpha[2]*rad2deg);
		alpha4.append(result.control.alpha[3]*rad2deg);
		omega.append(result.control.omega);
		trajectory.append(result.desired.y(0), result.desired.y(1));
		position.append(result.state.q(0), result.state.q(1));

		client.draw(result.state);
    });

	timer.start(20);

	return app.exec();
}
