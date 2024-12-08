#pragma once

#include <cmath>
#include <vector>

class Planners
{

public:
    static std::vector<std::vector<double>> TrajectoryPlan(double p1n, double p1e, double p2n, double p2e,
                                                           double r1, double r2, bool start, bool stop,
                                                           double amax = 100.0, double vmax = 75.0, double vmin = 0.0, double dt = 0.02);
};