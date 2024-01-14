#pragma once

#include "TrajectoryGenerator.h"

class Lemniscate : public TrajectoryGenerator<4> {
	static constexpr double sqrt2 = 1.41421356237;
	static constexpr double pi = 3.14159265359;
	static constexpr double deg2rad = pi/180;

	const double a;
	const double w;

	static double fix(double angle);

public:
	Lemniscate(const double c, const double T);

	Trajectory get(const double time);
};
