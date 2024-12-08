#include "subsystems/AprilTag2dVisionIO.h"

AprilTag2DVisionIO::AprilTag2DVisionIO(const double p_cameraHeight, const double p_cameraPitch, std::string p_cameraName)
{
    m_cameraName = p_cameraName;
    m_cameraHeight = p_cameraHeight;
    m_cameraPitch = p_cameraPitch;

    APRILTAG_HEIGHT_LOOKUP = {
        {3, Constants::POI::SPEAKER_APRILTAG_HEIGHT},
        {4, Constants::POI::SPEAKER_APRILTAG_HEIGHT},
        {5, Constants::POI::AMP_APRILTAG_HEIGHT},
        {6, Constants::POI::AMP_APRILTAG_HEIGHT},
        {7, Constants::POI::SPEAKER_APRILTAG_HEIGHT},
        {8, Constants::POI::SPEAKER_APRILTAG_HEIGHT},
        {11, Constants::POI::TRAP_APRILTAG_HEIGHT},
        {12, Constants::POI::TRAP_APRILTAG_HEIGHT},
        {13, Constants::POI::TRAP_APRILTAG_HEIGHT},
        {14, Constants::POI::TRAP_APRILTAG_HEIGHT},
        {15, Constants::POI::TRAP_APRILTAG_HEIGHT},
        {16, Constants::POI::TRAP_APRILTAG_HEIGHT},
    };

    m_camera = new photon::PhotonCamera(p_cameraName);

    LocalReset();
}
void AprilTag2DVisionIO::SetTarget(Target p_target)
{
    auto alliance = frc::DriverStation::GetAlliance().value_or(Constants::DEFAULT_ALLIANCE);
    // lookup the target and figure out which ID it is
    if (alliance == frc::DriverStation::Alliance::kRed)
    {
        switch (p_target)
        {
        case kSpeaker:
            m_targetFID = 4;
            break;
        case kAmp:
            m_targetFID = 5;
            break;
        }
    }
    else
    {
        switch (p_target)
        {
        case kSpeaker:
            m_targetFID = 7;
            break;
        case kAmp:
            m_targetFID = 6;
            break;
        }
    }
    m_targetHeight = APRILTAG_HEIGHT_LOOKUP[m_targetFID];
}

void AprilTag2DVisionIO::Analyze()
{

    std::span<const photon::PhotonTrackedTarget> targets;
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    m_latency = result.GetLatency();
    units::second_t timestamp = result.GetTimestamp();
    m_hasTarget = result.HasTargets();

    if (!m_hasTarget)
    {
        m_visionUpdate = Vision2DUpdate{0.0, 0.0, false};
        return;
    }
    targets = result.GetTargets();

    bool l_targetFound = false;
    photon::PhotonTrackedTarget l_selectedTarget;

    for (photon::PhotonTrackedTarget t : targets)
    {
        if (t.GetFiducialId() == m_targetFID)
        {
            l_selectedTarget = t;
            l_targetFound = true;
            break;
        }
    }
    if (!l_targetFound)
    {
        // Short-circut logic as target is not found
        m_visionUpdate = Vision2DUpdate{0.0, 0.0, false};
        return;
    }
    m_selectedTarget = l_selectedTarget;

    double range = (m_targetHeight - m_cameraHeight) /
                   MathUtil::tand(m_cameraPitch + l_selectedTarget.GetPitch());

    double bearing = l_selectedTarget.GetYaw();

    m_visionUpdate = Vision2DUpdate{range, bearing, m_hasTarget};
}

void AprilTag2DVisionIO::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Vision/April2DTag/" + m_cameraName + "/PipelineLatency", m_latency.value());
    frc::SmartDashboard::PutBoolean("Vision/April2DTag/" + m_cameraName + "/IsValid", m_hasTarget);
    frc::SmartDashboard::PutNumber("Vision/April2DTag/" + m_cameraName + "/FID", m_targetFID);
    frc::SmartDashboard::PutNumber("Vision/April2DTag/" + m_cameraName + "/Range", m_visionUpdate.rangeToTarget);
    frc::SmartDashboard::PutNumber("Vision/April2DTag/" + m_cameraName + "/Bearing", m_visionUpdate.bearingToTarget);
}
void AprilTag2DVisionIO::LocalReset()
{
    m_camera->SetPipelineIndex(2);

    m_hasTarget = false;

    m_latency = units::time::second_t{0};

    m_visionUpdate = Vision2DUpdate{0.0, 0.0, false};
}