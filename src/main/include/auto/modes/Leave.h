#pragma once

#include "auto/modes/AutoMode.h"
#include "auto/actions/TrajectoryAction.h"

class Leave : public AutoMode
{
public:
    /**
     * @brief Solely backs up to get "Leave" points.
     *
     */
    Leave()
    {
        Tconstraints l_constraints;
        l_constraints.amax = 20.0;
        l_constraints.vmax = 75.0;
        l_constraints.vmin = 0.0;

        SetInitialRobotHeading(0.0);

        AddAction(new TrajectoryAction({{0.0, 0.0, 0.0}, {64.0, 0.0, 0.0}}, l_constraints));
    }
};