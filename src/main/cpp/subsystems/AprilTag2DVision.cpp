#include "subsystems/AprilTag2DVision.h"

/**
 * @brief Get instance of AprilTag2DVision
 *
 * @return AprilTag2DVision*
 */
AprilTag2DVision *AprilTag2DVision::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new AprilTag2DVision();
    }
    return m_instance;
}
/**
 * @brief Construct a new April Tag 2D Vision object
 *
 */
AprilTag2DVision::AprilTag2DVision()
{
    m_drivetrain = Drivetrain::GetInstance();
    m_frontLeftCamera = new AprilTag2DVisionIO(0.0, 15.0, "Gaelhawks_Left");

    m_speakerTable = new Interpolator(SPEAKER_DISTANCE_TABLE);
    m_trapTable = new Interpolator(TRAP_DISTANCE_TABLE);
    m_ampTable = new Interpolator(AMP_DISTANCE_TABLE);

    LocalReset();
}

/**
 * @brief Analyze the vision data for each camera
 *
 */
void AprilTag2DVision::Analyze()
{
    m_frontLeftCamera->SetTarget(Target::kSpeakerTag);
    m_frontLeftCamera->Analyze();

    m_speakerVisionUpdate = m_frontLeftCamera->GetVisionUpdate();
}
/**
 * @brief Update the dashboard
 *
 */
void AprilTag2DVision::UpdateDash()
{
    m_frontLeftCamera->UpdateDash();
    frc::SmartDashboard::PutNumber("Vision/SpeakerDist", GetSpeakerDistance());
}

/**
 * @brief Local reset of the cameras
 *
 */
void AprilTag2DVision::LocalReset()
{
    m_frontLeftCamera->LocalReset();
}

double AprilTag2DVision::GetSpeakerDistance()
{
    return m_speakerTable->Sample(m_speakerVisionUpdate.pitchToTarget)[0];
}

/**
 * @brief Drive control for the speaker. Does not modify xdot or ydot
 *
 * @param xdot x velocity
 * @param ydot y velocity
 * @param psidot psi velocity
 */
void AprilTag2DVision::SpeakerDriveControl(double &xdot, double &ydot, double &psidot)
{
    double vpsidot;
    if (!m_speakerVisionUpdate.hasTarget)
    {
        // double gyroAngle = m_drivetrain->GetGyroWrappedAngle();
        // double error = gyroAngle - SPEAKER_GOAL_HEADING;
        // vpsidot = KPSI_SPEAKER * MathUtil::Wrap(error);
        vpsidot = 0.0;
    }
    else
    {
        double offset = 13.0 / (GetSpeakerDistance() * 12.0) * 57.2;
        vpsidot = (KPSI_SPEAKER) * (m_speakerVisionUpdate.bearingToTarget - SPEAKER_GOAL_HEADING - offset);
        psidot = MathUtil::Limit(-SPEAKER_R_VEL_LIMIT, SPEAKER_R_VEL_LIMIT, vpsidot);
    }
}
/**
 * @brief Drive control for the trap.
 *
 * @param xdot x velocity
 * @param ydot y velocity
 * @param psidot psi velocity
 */
void AprilTag2DVision::TrapDriveControl(double &xdot, double &ydot, double &psidot)
{
    //  TODO implement trap drive control
}

/**
 * @brief Drive control for the amp.
 *
 * @param xdot x velocity
 * @param ydot y velocity
 * @param psidot psi velocity
 */
void AprilTag2DVision::AmpDriveControl(double &xdot, double &ydot, double &psidot)
{
    //  TODO implement amp drive control
}

AprilTag2DVision *AprilTag2DVision::m_instance = nullptr;