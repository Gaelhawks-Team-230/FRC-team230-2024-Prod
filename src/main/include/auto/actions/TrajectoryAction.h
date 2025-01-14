#pragma once

#include "util/planners/TrajectoryPlan.h"
#include <cmath>
#include <vector>
#include "auto/actions/Action.h"
#include <frc/DriverStation.h>
#include "subsystems/Drivetrain.h"

typedef struct
{
    double amax;
    double vmax;
    double vmin;
} Tconstraints;

class TrajectoryAction : public Action
{
private:
    Drivetrain *m_drivetrain;
    std::vector<std::vector<double>> m_waypoints;
    Tconstraints m_constraints;
    std::vector<std::vector<double>> m_velCmds; //* vector with ALL commands
    std::vector<std::vector<double>> m_curCmds; //* vector storing temp cmds to be added to l_velCmds
    int GetVelCmdSize() { return (int)m_velCmds.size(); };

public:
    TrajectoryAction(std::vector<std::vector<double>> waypoints, Tconstraints constraints);
    // void SetConstraints(double amax, double vmax, double vmin){};

    void Periodic();
    bool IsActionComplete();
};