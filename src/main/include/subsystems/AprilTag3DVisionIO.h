#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/CoordinateSystem.h>



#include "Constants.h"
#include "util/geometry/Pose2d.h"
#include "util/Subsystem.h"

struct VisionUpdate
{
    Pose2d pose;
    bool hasTarget;
    bool hasNewResult;
};
class AprilTag3DVisionIO : public Subsystem
{
public:
    AprilTag3DVisionIO(frc::Transform3d p_robotToCam, std::string p_cameraName);
    void LocalReset() override;
    void Analyze() override;
    void UpdateDash() override;
    void Stop () override {};
    void Periodic () override {};

    VisionUpdate GetVisionUpdate() { return VisionUpdate{m_visionPose, m_hasTarget, m_hasNewResult}; };

private:
    frc::Transform3d m_robotToCam;
    std::string m_cameraName;

    photon::PhotonPoseEstimator *m_photonEstimator;

    std::shared_ptr<photon::PhotonCamera> m_camera;

    units::second_t m_latency{0_s};
    units::second_t m_lastEstTimestamp{0_s};

    bool m_hasTarget;
    bool m_hasNewResult;
    double m_maxAmbiguity;
    Pose2d m_visionPose;
};