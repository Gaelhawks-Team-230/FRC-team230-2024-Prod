#include "subsystems/AprilTag3DVision.h"

AprilTag3DVision *AprilTag3DVision::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new AprilTag3DVision();
    }
    return m_instance;
}

AprilTag3DVision::AprilTag3DVision()
{

    m_frontLeftCamera = new AprilTag3DVisionIO(frc::Transform3d(14.5_in, 0_in, 20.5_in, frc::Rotation3d(0_deg, 15_deg, 0_deg)), "Gaelhawks_Left");

    LocalReset();
}

void AprilTag3DVision::Analyze()
{
    m_frontLeftCamera->Analyze();

    VisionUpdate frontLeftCameraUpdate = m_frontLeftCamera->GetVisionUpdate();

    std::vector<VisionUpdate> visionUpdates;
    if (frontLeftCameraUpdate.hasNewResult && frontLeftCameraUpdate.hasTarget)
    {
        visionUpdates.push_back(frontLeftCameraUpdate);
    }
    if (visionUpdates.size() > 0)
    {
        double x = 0.0;
        double y = 0.0;
        double rotation = 0.0;
        for (VisionUpdate visionUpdate : visionUpdates)
        {
            x += visionUpdate.pose.GetX();
            y += visionUpdate.pose.GetY();
            rotation += visionUpdate.pose.GetRotation();
        }
        x /= visionUpdates.size();
        y /= visionUpdates.size();
        rotation /= visionUpdates.size();
        m_visionPose = Pose2d(x, y, rotation);
        m_hasTarget = true;
        m_hasNewResult = true;
    }
    else
    {
        m_hasTarget = false;
        m_hasNewResult = false;
    }
}

void AprilTag3DVision::UpdateDash()
{
    m_frontLeftCamera->UpdateDash();

    frc::SmartDashboard::PutBoolean("Pose/Vision/IsValid", m_hasTarget);
    frc::SmartDashboard::PutBoolean("Pose/Vision/HasUpdate", m_hasNewResult);

    frc::SmartDashboard::PutNumberArray("Pose/Vision/GlobalPose", m_visionPose.GetPoseAsArray());
}
void AprilTag3DVision::LocalReset()
{
    m_hasNewResult = false;
    m_hasTarget = false;

    m_visionPose = Pose2d();

    m_frontLeftCamera->LocalReset();
}

AprilTag3DVision *AprilTag3DVision::m_instance = nullptr;