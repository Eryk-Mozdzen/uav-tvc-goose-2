#include <iostream>
#include <iomanip>
#include <sstream>

#include <QApplication>
#include <QTimer>

#include "Client.h"
#include "Chart.h"
#include "GraphXY.h"
#include "Simulation.h"
#include "PFL.h"
#include "TrajectoryGenerator.h"
#include "Manual.h"
#include "Lemniscate.h"
#include "Circle.h"

TrajectoryGenerator<4> * createGenerator(int argc, char **argv) {
	std::string contents;

    for(int i=1; i<argc; i++) {
        contents +=argv[i];

        if(i!=argc-1) {
            contents +=' ';
        }
    }

	std::istringstream stream(contents);

	std::string type;
	stream >> type;

	if(type=="manual") {
		return new Manual();
	} else if(type=="lemniscate") {
		float c, T;
		stream >> c >> T;
		return new Lemniscate(c, T);
	} else if(type=="circle") {
		float x, y, R, T;
		stream >> x >> y >> R >> T;
		return new Circle(x, y, R, T);
	}

	std::cerr << "undefined trajectory generator" << std::endl;

	return nullptr;
}

int main(int argc, char **argv) {
	QApplication app(argc, argv);

	Simulation simulation(
		new PFL(),
		createGenerator(argc, argv)
	);

	Client client;
	Chart chartAlpha("Thrust Vanes Angles", "α [°]", "%+.1f", -15, 15);
	Chart chartOmega("Rotor Angular Velocity", "ω [rad/s]", "%.0f", 0, 2000);
	GraphXY graph("XY Trajectory", "%+.1f", 4);

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
