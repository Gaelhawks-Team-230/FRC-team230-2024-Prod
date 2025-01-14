#include "auto/actions/TrajectoryAction.h"

/**
 * @brief takes waypoints and compiles them all in m_velCmds vector
 *
 * @param waypoints points of travel, format {{x1, y1, psi1}, {x2, y2, psi2}, ...}
 * @param constraints constraints on speed, format aMax, vMax, vMin
 *
 * @pre waypoints should be > 2 in length
 */
TrajectoryAction::TrajectoryAction(std::vector<std::vector<double>> waypoints, Tconstraints constraints) : m_waypoints(waypoints), m_constraints(constraints)
{
    m_drivetrain = Drivetrain::GetInstance();

    for (int i = 0; i < (int)waypoints.size() - 1; i++)
    {
        if ((int)waypoints.size() < 2)
        {
            break;
        }

        // determining profile type based on stop vs start
        bool l_start = false, l_stop = false;
        if (i == 0)
        {
            l_start = true;
        }
        if (i == ((int)waypoints.size() - 2))
        {
            l_stop = true;
        }

        // plan trajectory from current waypoint to next
        m_curCmds = Planners::TrajectoryPlan(waypoints[i][0], waypoints[i][1], waypoints[i + 1][0], waypoints[i + 1][1],
                                             waypoints[i][2], waypoints[i + 1][2], l_start, l_stop, constraints.amax, constraints.vmax, constraints.vmin);
        // all vcmds together (xdot, ydot, psidot)
        m_velCmds.insert(m_velCmds.end(), m_curCmds.begin(), m_curCmds.end());
    }
}

void TrajectoryAction::Periodic()
{
    double xdot, ydot, psidot;
    // printf("traj action periodic count: %d\n", Action::GetCount());

    std::vector<double> l_sample;
    l_sample = m_velCmds[Action::GetCount()];

    //? Waypoints are taken via PathPlanner on the BLUE side with the red source as (0, 0)
    //? Logic to change cmds to the appropriate sign given alliance color
    //* xdot cmds are pos both ways.
    //* ydot cmds should be neg for blue, pos for red.
    //* psidot cmds should be pos for blue, neg for red.

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        xdot = l_sample[0];
        ydot = -l_sample[1];
        psidot = l_sample[2];
    }
    else
    {
        xdot = l_sample[0];
        ydot = l_sample[1];
        psidot = -l_sample[2];
    }

    m_drivetrain->ToFieldCoordinates(&xdot, &ydot);
    m_drivetrain->SetChassisSpeeds(xdot, ydot, psidot);
}

/**
 * @brief are there waypoints that still need to be accessed in velcmds vector?
 *
 * @return iterated through entire vector? all cmds have been run?
 */
bool TrajectoryAction::IsActionComplete()
{
    if (Action::GetCount() >= (int)m_velCmds.size())
    {
        return true;
    }
    return false;
}