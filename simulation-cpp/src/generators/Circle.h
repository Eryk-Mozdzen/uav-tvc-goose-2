#pragma once

#include "TrajectoryGenerator.h"

class Circle : public TrajectoryGenerator<4> {
	static constexpr double pi = 3.14159265359;

	const double x;
	const double y;
	const double r;
	const double w;

public:
	Circle(const double x, const double y, const double r, const double T);

	Trajectory get(const double time);
};
