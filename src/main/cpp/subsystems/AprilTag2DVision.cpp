#include "subsystems/AprilTag2DVision.h"

AprilTag2DVision *AprilTag2DVision::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new AprilTag2DVision();
    }
    return m_instance;
}

AprilTag2DVision::AprilTag2DVision()
{

    m_frontLeftCamera = new AprilTag2DVisionIO(14.5, 15.0, "Gaelhawks_Left");
    m_drivetrain = Drivetrain::GetInstance();

    LocalReset();
}

void AprilTag2DVision::Analyze()
{
    m_frontLeftCamera->Analyze();

    m_frontLeftVisionUpdate = m_frontLeftCamera->GetVisionUpdate();
}
double AprilTag2DVision::GetRangeToSpeaker()
{
    m_frontLeftCamera->SetTarget(Target::kSpeaker);
    return m_frontLeftVisionUpdate.rangeToTarget;
}
void AprilTag2DVision::SpeakerDriveTargetting(double *psidot)
{
    m_frontLeftCamera->SetTarget(Target::kSpeaker);
    double vpsidot;

    if (!m_frontLeftVisionUpdate.hasTarget)
    {
        double gyroAngle = m_drivetrain->GetGyroWrappedAngle();
        double goal = 0.0;
        double error = goal - gyroAngle;
        vpsidot = KPSI_SPEAKER * error;
    }
    else
    {
        vpsidot = (KPSI_SPEAKER) * (m_frontLeftVisionUpdate.bearingToTarget);
    }

    *psidot += MathUtil::Limit(-SPEAKER_R_VEL_LIMIT, SPEAKER_R_VEL_LIMIT, vpsidot);
}
void AprilTag2DVision::UpdateDash()
{
    m_frontLeftCamera->UpdateDash();
}
void AprilTag2DVision::LocalReset()
{
    m_hasTarget = false;

    m_frontLeftCamera->LocalReset();
}

AprilTag2DVision *AprilTag2DVision::m_instance = nullptr;