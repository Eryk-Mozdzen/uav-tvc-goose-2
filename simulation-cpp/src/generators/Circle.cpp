#include "Circle.h"

Circle::Circle(const double x, const double y, const double r, const double T) : x{x}, y{y}, r{r}, w{2*pi/T} {

}

Circle::Trajectory Circle::get(const double time) {
    const double s = std::sin(w*time);
    const double c = std::cos(w*time);

    Trajectory trajectory;

    trajectory.y = Eigen::Vector<double, 4>{
        r*c + x,
        r*s + y,
        1,
        0//w*time + pi/2
    };

    trajectory.dy = Eigen::Vector<double, 4>{
        -r*w*s,
        r*w*c,
        0,
        0//w
    };

    trajectory.ddy = Eigen::Vector<double, 4>{
        -r*w*w*c,
        -r*w*w*s,
        0,
        0
    };

    return trajectory;
}
