#include <fstream>
#include <cstdlib>
#include <QApplication>
#include <QThread>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>

#include "Circle.h"
#include "Lemniscate.h"
#include "PFL.h"
#include "Plant.h"
#include "VisualClient.h"
#include "GraphXY.h"
#include "Chart.h"

class Simulation : public QObject {
    Q_OBJECT

	const drake::systems::System<double> &system;

public:
    Simulation(const drake::systems::System<double> &system) : system{system} {

	}

public slots:
    void run() {
		drake::systems::Simulator<double> simulator(system);
		simulator.set_target_realtime_rate(1);
		simulator.get_mutable_integrator().request_initial_step_size_target(0.001);
		simulator.get_mutable_integrator().set_requested_minimum_step_size(0.001);
		simulator.get_mutable_integrator().set_throw_on_minimum_step_size_violation(false);
		simulator.Initialize();
		simulator.AdvanceTo(std::numeric_limits<double>::infinity());
    }
};

void save(const drake::systems::Diagram<double> &diagram) {
	std::ofstream file("diagram.dot");
    file << diagram.GetGraphvizString();
    file.close();
    system("dot -Tsvg diagram.dot -o diagram.svg");
}

int main(int argc, char **argv) {
	QApplication app(argc, argv);

	drake::systems::DiagramBuilder<double> builder;

	//TrajectoryGenerator *generator = builder.AddSystem<Circle>(0, 0, 2, 5);
	TrajectoryGenerator *generator = builder.AddSystem<Lemniscate>(2, 10);
	Controller *controller = builder.AddSystem<PFL>();
	Plant *plant = builder.AddSystem<Plant>();
	VisualClient *client = builder.AddSystem<VisualClient>();
	GraphXY *trajectory_xy = builder.AddSystem<GraphXY>("trajectory XY", "%+4.1f", 4);
	Chart *actuators_rotor = builder.AddSystem<Chart>("rotor", "velocity [rad/s]", "%4.0f", 0, 2000);
	Chart *actuators_vanes = builder.AddSystem<Chart>("thrust vanes", "angle [deg]", "%+3.0f", -10, 10);

	trajectory_xy->AddSeries("desired", Eigen::MatrixX<double>({
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	}), Qt::black, Qt::DashLine, 1);
	trajectory_xy->AddSeries("current", Eigen::MatrixX<double>({
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	}), Qt::red, Qt::SolidLine, 2);
	actuators_rotor->AddSeries("rotor", Eigen::Vector<double, 5>({1, 0, 0, 0, 0}), Qt::black, Qt::SolidLine, 2);
	actuators_vanes->AddSeries("vane_1", Eigen::Vector<double, 5>({0, 1, 0, 0, 0})*57.2957, Qt::red, Qt::SolidLine, 2);
	actuators_vanes->AddSeries("vane_2", Eigen::Vector<double, 5>({0, 0, 1, 0, 0})*57.2957, Qt::green, Qt::SolidLine, 2);
	actuators_vanes->AddSeries("vane_3", Eigen::Vector<double, 5>({0, 0, 0, 1, 0})*57.2957, Qt::blue, Qt::SolidLine, 2);
	actuators_vanes->AddSeries("vane_4", Eigen::Vector<double, 5>({0, 0, 0, 0, 1})*57.2957, Qt::magenta, Qt::SolidLine, 2);

	builder.Connect(generator->get_output_port(), controller->GetInputPort("trajectory"));
	builder.Connect(generator->get_output_port(), trajectory_xy->GetInputPort("desired"));
	builder.Connect(plant->get_output_port(), controller->GetInputPort("state"));
	builder.Connect(controller->get_output_port(), plant->get_input_port());
	builder.Connect(controller->get_output_port(), actuators_rotor->GetInputPort("rotor"));
	builder.Connect(controller->get_output_port(), actuators_vanes->GetInputPort("vane_1"));
	builder.Connect(controller->get_output_port(), actuators_vanes->GetInputPort("vane_2"));
	builder.Connect(controller->get_output_port(), actuators_vanes->GetInputPort("vane_3"));
	builder.Connect(controller->get_output_port(), actuators_vanes->GetInputPort("vane_4"));
	builder.Connect(plant->get_output_port(), client->get_input_port());
	builder.Connect(plant->get_output_port(), trajectory_xy->GetInputPort("current"));

	std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();

	save(*diagram);

	Simulation simulation(*diagram);
    QThread simulationThread;
    simulation.moveToThread(&simulationThread);
    QObject::connect(&simulationThread, &QThread::started, &simulation, &Simulation::run);
    simulationThread.start();

    return app.exec();
}

#include "main.moc"
