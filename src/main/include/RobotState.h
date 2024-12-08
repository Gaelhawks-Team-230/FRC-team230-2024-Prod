#pragma once

#include "Constants.h"
#include "util/geometry/Pose2d.h"
#include "util/MathUtil.h"
#include "util/control/ComplimentaryFilter.h"

#include <tuple>
#include <optional>

class RobotState
{
public:
    void Reset(Pose2d p_initialPose)
    {
        m_pose = p_initialPose;
        m_xFilter.Clear();
        m_yFilter.Clear();
        m_rotationFilter.Clear();
    };
    void Update(std::optional<Pose2d> p_visionPose, double vx, double vy, double vpsi);
    static std::tuple<double, double, double> IntegrateOdomotryVel(double vx, double vy, double vpsi, double heading);
    Pose2d GetPose() { return m_pose; };

    static RobotState *GetInstance();

private:
    RobotState();
    static RobotState *m_instance;
    ComplementaryFilter m_xFilter{0.2 * 2 * M_PI, 0.707, Constants::LOOPTIME};
    ComplementaryFilter m_yFilter{0.2 * 2 * M_PI, 0.707, Constants::LOOPTIME};
    ComplementaryFilter m_rotationFilter{0.2 * 2 * M_PI, 0.707, Constants::LOOPTIME};
    Pose2d m_pose;
};