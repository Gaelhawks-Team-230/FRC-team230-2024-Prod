#pragma once

#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include "Constants.h"
#include "util/Subsystem.h"
#include "subsystems/AprilTag2DVisionIO.h"
#include "subsystems/Drivetrain.h"

const double KPSI_SPEAKER = 4.0;
const double SPEAKER_R_VEL_LIMIT = 120.0;

class AprilTag2DVision : public Subsystem
{
public:
    void LocalReset(void) override;
    void Analyze() override;
    void UpdateDash() override;
    void Periodic() override{};
    void Stop() override{};

    void SpeakerDriveTargetting(double *psidot);
    double GetRangeToSpeaker();
    // TODO Get Dist2Goal method

    static AprilTag2DVision *GetInstance();

private:
    AprilTag2DVision();
    static AprilTag2DVision *m_instance;

    Drivetrain *m_drivetrain;

    AprilTag2DVisionIO *m_frontLeftCamera;
    Vision2DUpdate m_frontLeftVisionUpdate;

    bool m_hasTarget;
};