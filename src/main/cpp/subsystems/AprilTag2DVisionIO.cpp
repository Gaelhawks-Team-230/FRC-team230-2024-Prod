#include "subsystems/AprilTag2dVisionIO.h"

/**
 * @brief Construct a new April Tag 2D Vision Camera
 * 
 * @param p_cameraHeadingOffset heading offset in degrees
 * @param p_cameraPitchOffset pitch offset in degrees
 * @param p_cameraName name of photon camera
 */
AprilTag2DVisionIO::AprilTag2DVisionIO(const double p_cameraHeadingOffset, const double p_cameraPitchOffset, std::string p_cameraName)
{
    m_cameraName = p_cameraName;
    m_cameraHeadingOffset = p_cameraHeadingOffset;
    m_cameraPitchOffset = p_cameraPitchOffset;

    m_camera = new photon::PhotonCamera(p_cameraName);

    LocalReset();
}

/**
 * @brief Set the target for the camera to filter apriltags. Must be called before analyze
 * 
 * @param p_target either speaker, amp, or trap
 */
void AprilTag2DVisionIO::SetTarget(Target p_target)
{
    auto alliance = frc::DriverStation::GetAlliance().value_or(Constants::DEFAULT_ALLIANCE);
    // lookup the target and figure out which ID it is
    if (alliance == frc::DriverStation::Alliance::kRed)
    {
        switch (p_target)
        {
        case kSpeakerTag:
            m_targetFIDs = {4};
            break;
        case kAmpTag:
            m_targetFIDs = {5};
            break;
        case kTrapTags:
            m_targetFIDs = {11, 12, 13};
            break;
        }
    }
    else
    {
        switch (p_target)
        {
        case kSpeakerTag:
            m_targetFIDs = {7};
            break;
        case kAmpTag:
            m_targetFIDs = {6};
            break;
        case kTrapTags:
            m_targetFIDs = {14, 15, 16};
            break;
        }
    }
}

/**
 * @brief Analyze the camera's latest result and update the vision update
 * 
 */
void AprilTag2DVisionIO::Analyze()
{
    std::span<const photon::PhotonTrackedTarget> targets;
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    m_latency = result.GetLatency();
    units::second_t timestamp = result.GetTimestamp();
    bool hasTarget = result.HasTargets();

    if (!hasTarget)
    {
        m_visionUpdate = Vision2DUpdate{0.0, 0.0, false, 0};
        return;
    }

    targets = result.GetTargets();
    bool l_targetFound = false;
    photon::PhotonTrackedTarget l_selectedTarget;

    if (m_targetFIDs.size() == 1)
    {
        for (photon::PhotonTrackedTarget t : targets)
        {
            if (t.GetFiducialId() == m_targetFIDs[0])
            {
                l_selectedTarget = t;
                l_targetFound = true;
                break;
            }
        }
    }
    else if (!m_targetFIDs.empty())
    {
        double maxArea = 0.0;
        for (photon::PhotonTrackedTarget t : targets)
        {
            if (std::find(m_targetFIDs.begin(), m_targetFIDs.end(), t.GetFiducialId()) != m_targetFIDs.end())
            {
                if (t.GetArea() > maxArea)
                {
                    l_selectedTarget = t;
                    maxArea = t.GetArea();
                    l_targetFound = true;
                }
            }
        }
    }
    if (!l_targetFound)
    {
        // Short-circut logic as target is not found
        m_visionUpdate = Vision2DUpdate{0.0, 0.0, false, 0};
        return;
    }
    m_visionUpdate = Vision2DUpdate{l_selectedTarget.GetPitch(), l_selectedTarget.GetYaw(), hasTarget, l_selectedTarget.GetFiducialId()};
}

/**
 * @brief Update the dashboard with the latest vision update
 * 
 */
void AprilTag2DVisionIO::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Vision/AprilTag2D/" + m_cameraName + "/PipelineLatency", m_latency.value());
    frc::SmartDashboard::PutBoolean("Vision/AprilTag2D/" + m_cameraName + "/IsValid", m_visionUpdate.hasTarget);
    frc::SmartDashboard::PutNumber("Vision/AprilTag2D/" + m_cameraName + "/FID", m_visionUpdate.fiducialId);
    frc::SmartDashboard::PutNumber("Vision/AprilTag2D/" + m_cameraName + "/Pitch", m_visionUpdate.pitchToTarget);
    frc::SmartDashboard::PutNumber("Vision/AprilTag2D/" + m_cameraName + "/Bearing", m_visionUpdate.bearingToTarget);
}
/**
 * @brief Reset the camera's pipeline index and latency. Create default vision update
 * 
 */
void AprilTag2DVisionIO::LocalReset()
{
    m_camera->SetPipelineIndex(2);
    m_latency = units::time::second_t{0};
    m_visionUpdate = Vision2DUpdate{0.0, 0.0, false, 0};
}