#include "util/planners/TrajectoryPlan.h"

/**
 * @brief
 *
 * @param p1n pos 1 for north (in.)
 * @param p1e pos 1 for east (in.)
 * @param p2n pos 2 for north (in.)
 * @param p2e pos 2 for east (in.)
 * @param r1 heading 1 (deg)
 * @param r2 heading 2 (deg)
 * @param start does motion include starting?
 * @param stop does motion include stopping?
 * @param amax target accel (in/s^2)
 * @param vmax target vel (in/s)
 * @param vmin lowest vcmd (in/s)
 * @param dt looptime of 20 ms
 * @return std::vector<std::vector<double>> xdot, ydot, psidot
 */
std::vector<std::vector<double>> Planners::TrajectoryPlan(double p1n, double p1e, double p2n, double p2e,
                                                          double r1, double r2, bool start, bool stop,
                                                          double amax, double vmax, double vmin, double dt)
{
    // dist calculation based on coords of 2 waypoints
    double deltapn = p2n - p1n;
    double deltape = p2e - p1e;
    double deltar = r2 - r1;

    double dist = sqrt(deltapn * deltapn + deltape * deltape);

    // trajMode determines which trajectory planner to use...
    // 0: CONSTANT VELOCITY (neither starting nor stopping)
    // 1: ACCEL TO / DECEL FROM (either only starting or only stopping)
    // 2: ACCEL THEN DECEL (both starting and stopping)

    int trajMode = (int)start * 1 + (int)stop * 1;

    std::vector<std::vector<double>> res;
    std::vector<double> vel, vel_start, vel_end;
    std::vector<double> vn, ve, vr;

    int n, a, d;          // cycle counters: n = count, a = accel count, d = decel count (a + d for case 2 only)
    double dist_traveled; // actual dist traveled, actual deg rotated

    // vector methods to understand -
    // push_back: appends an element to the end of the vector

    switch (trajMode)
    {
    // constant velocity
    case 0:
        n = 0;
        dist_traveled = 0.0;

        // loop until dist traveled by robot exceeds target dist
        while (dist_traveled < dist)
        {
            // increment v cmd and counter
            n += 1;
            vel.push_back(vmax);

            dist_traveled += vel[n - 1] * dt; // d = v * t
        }

        break;

    // accel or decel
    case 1:
        n = 0;
        dist_traveled = 0.0;

        while (dist_traveled < dist)
        {
            // increment v cmd and counter, limit at vmax
            n += 1;
            vel.push_back(std::min(n * amax * dt + vmin, vmax)); // vf = vi + at

            dist_traveled += vel[n - 1] * dt;
        }

        // reverse cmds if a decel profile
        if (stop > start)
        {
            for (int i = 0; i < (int)vel.size() / 2; i++)
            {
                int j = (int)vel.size() - i - 1; // index of elements in end half
                double temp = vel[i];
                vel[i] = vel[j];
                vel[j] = temp;
            }
        }

        break;

    // accel then decel
    case 2:
        a = 0; // front end counter (accel)
        d = 0; // back end counter (decel)
        dist_traveled = 0.0;

        while (dist_traveled < dist)
        {
            // alternate between front (accel) and back (decel) end cmds to get dist_traveled
            if (a == d)
            {
                a += 1;
                vel_start.push_back(std::min(a * amax * dt + vmin, vmax));
                dist_traveled += vel_start[a - 1] * dt;
            }
            else
            {
                d += 1;
                vel_end.push_back(std::min(d * amax * dt + vmin, vmax));
                dist_traveled += vel_end[d - 1] * dt;
            }
        }

        n = a + d;

        // concatenate accel and decel vcmds
        // inserts acceleration v cmds at beginning of vel
        vel.insert(vel.end(), vel_start.begin(), vel_start.end());
        // inserts deceleration v cmds after accel in reversed order
        vel.insert(vel.end(), vel_end.rbegin(), vel_end.rend());

        break;

    default:
        break;
    }

    // adjusts v cmds to force in an int num of cycles
    for (double &v : vel)
    {
        v = v * dist / dist_traveled;
    }

    // separating north and east velocity commands
    for (int i = 0; i < (int)vel.size(); i++)
    {
        // vn.push_back(deltapn / dist * vel[i]);
        // ve.push_back(deltape / dist * vel[i]);
        // vr.push_back(deltar / (n*LOOPTIME));

        res.push_back({(deltapn / dist * vel[i]), (deltape / dist * vel[i]), (deltar / (n * dt))});
    }

    // returns 2 columns of vn + ve cmds
    // res.push_back({ vn });
    // res.push_back({ ve });
    // res.push_back({ vr });

    return res;
}