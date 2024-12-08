#include "subsystems/AprilTag3dVisionIO.h"

AprilTag3DVisionIO::AprilTag3DVisionIO(frc::Transform3d p_robotToCam, std::string p_cameraName)
{
    m_robotToCam = p_robotToCam;
    m_cameraName = p_cameraName;

    m_photonEstimator = new photon::PhotonPoseEstimator(
        LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        photon::PhotonCamera{m_cameraName}, m_robotToCam);

    m_camera = m_photonEstimator->GetCamera();

    m_photonEstimator->SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    LocalReset();
}

void AprilTag3DVisionIO::Analyze()
{
    wpi::SmallVector<photon::PhotonTrackedTarget, 10U> targets;
    std::optional<photon::EstimatedRobotPose> currentPose = m_photonEstimator->Update();
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    m_latency = result.GetLatency();
    units::second_t timestamp = result.GetTimestamp();

    if (result.HasTargets() && currentPose.has_value())
    {
        m_hasTarget = true;
        targets = currentPose.value().targetsUsed;
    }
    else
    {
        m_hasTarget = false;
    }

    m_hasNewResult = false;

    if (!currentPose.has_value() || targets.empty())
    {
        m_lastEstTimestamp = timestamp;
        return;
    }
    if (targets.size() > 0)
    {
        double maxAmbiguity = 0.0;
        for (const photon::PhotonTrackedTarget &target : targets)
        {
            double ambiguity = target.GetPoseAmbiguity();
            if (ambiguity > maxAmbiguity)
            {
                maxAmbiguity = ambiguity;
            }
        }
        m_maxAmbiguity = maxAmbiguity;
    }
    else
    {
        m_maxAmbiguity = targets[0].GetPoseAmbiguity();
    }
    if (m_maxAmbiguity < 0.2)
    {
        if (m_lastEstTimestamp != timestamp)
        {
            m_hasNewResult = true;
            m_lastEstTimestamp = timestamp;
        }
        double x, y, rotation;
        frc::Pose3d poseNWU = currentPose.value().estimatedPose;
        frc::Pose3d poseNED = frc::CoordinateSystem::Convert(poseNWU, frc::CoordinateSystem::NWU(),frc::CoordinateSystem::NED());

        frc::Pose2d pose2dNED = poseNED.ToPose2d();

        x = pose2dNED.X().value() * 39.3701;
        y = pose2dNED.Y().value() * 39.3701;
        rotation = pose2dNED.Rotation().Degrees().value();

        if (rotation > 180)
        {
            rotation = (rotation - 360) * -1.0;
        }

        m_visionPose = Pose2d(x, y, rotation);

        // * temp testing logs
        // printf("%f, %f, %f\n", visionPoseArray[0], visionPoseArray[1], visionPoseArray[2]);
    }
    else
    {
        m_hasTarget = false;
    }
}

void AprilTag3DVisionIO::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Vision/AprilTag/" + m_cameraName + "/PipelineLatency", m_latency.value());
    frc::SmartDashboard::PutBoolean("Vision/AprilTag/" + m_cameraName + "/IsValid", m_hasTarget);
    frc::SmartDashboard::PutBoolean("Vision/AprilTag/" + m_cameraName + "/HasUpdate", m_hasNewResult);
    frc::SmartDashboard::PutNumber("Vision/AprilTag/" + m_cameraName + "/MaxAmbiguity", m_maxAmbiguity);

    frc::SmartDashboard::PutNumberArray("Vision/AprilTag/" + m_cameraName + "/Pose", m_visionPose.GetPoseAsArray());
}
void AprilTag3DVisionIO::LocalReset()
{
    m_camera->SetPipelineIndex(0);

    m_hasNewResult = false;
    m_hasTarget = false;
    m_maxAmbiguity = 0.0;

    m_lastEstTimestamp = units::time::second_t{0};
    m_latency = units::time::second_t{0};

    m_visionPose = Pose2d();
}