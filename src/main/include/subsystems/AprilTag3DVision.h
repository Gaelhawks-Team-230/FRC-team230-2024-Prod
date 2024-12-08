#pragma once

#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include "Constants.h"
#include "util/Subsystem.h"
#include "subsystems/AprilTag3DVisionIO.h"

class AprilTag3DVision : public Subsystem
{
public:
    void LocalReset(void) override;
    void Analyze() override;
    void UpdateDash() override;
    void Periodic() override{};
    void Stop() override{};

    VisionUpdate GetVisionUpdate() { return VisionUpdate{m_visionPose, m_hasTarget, m_hasNewResult}; };
    // TODO Get Dist2Goal method

    static AprilTag3DVision *GetInstance();

private:
    AprilTag3DVision();
    static AprilTag3DVision *m_instance;

    AprilTag3DVisionIO *m_frontLeftCamera;

    bool m_hasTarget;
    bool m_hasNewResult;
    Pose2d m_visionPose;
};