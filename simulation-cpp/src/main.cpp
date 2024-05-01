#include <fstream>
#include <QApplication>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include "Function.h"

#include "Simulator.h"
#include "Square.h"
#include "Circle.h"
#include "Lemniscate.h"
#include "Manual.h"
#include "Simple.h"
#include "Simple2.h"
#include "Plant.h"
#include "VisualClient.h"
#include "GraphXY.h"
#include "Chart.h"

int main(int argc, char **argv) {
	QApplication app(argc, argv);

	drake::systems::DiagramBuilder<double> builder;

	auto generator = builder.AddSystem<drake::systems::ConstantVectorSource>(Eigen::Vector<double, 12>{0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
	//auto generator = builder.AddSystem<Lemniscate>(2, 10);
	auto controller = builder.AddSystem<Simple>();
	auto plant = builder.AddSystem<Plant>(Eigen::Vector<double, 12>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
	auto client = builder.AddSystem<VisualClient>();
	auto controls = builder.AddSystem<Chart>("controls", "", "%+4.1f", -5, 5);

	controls->AddSeries("generalized", Eigen::Matrix<double, 4, 4>::Identity());

	builder.Connect(generator->get_output_port(), controller->get_trajectory_input_port());
	builder.Connect(controller->get_control_output_port(), plant->get_input_port());
	builder.Connect(controller->get_control_output_port(), controls->GetInputPort("generalized"));
	builder.Connect(plant->get_output_port(), controller->get_state_input_port());
	builder.Connect(plant->get_output_port(), client->get_input_port());

	auto diagram = builder.Build();

	std::ofstream file("diagram.dot");
    file << diagram->GetGraphvizString();
    file.close();

	Simulator<double> simulator(*diagram);
	simulator.set_target_realtime_rate(1);
	//simulator.get_mutable_integrator().request_initial_step_size_target(0.0001);
	//simulator.get_mutable_integrator().set_requested_minimum_step_size(0.0001);
	//simulator.get_mutable_integrator().set_throw_on_minimum_step_size_violation(false);
	simulator.get_mutable_integrator().set_maximum_step_size(0.0001);
	simulator.get_mutable_integrator().set_fixed_step_mode(true);
	simulator.Initialize();
	simulator.StartAdvance();

	return app.exec();
}
