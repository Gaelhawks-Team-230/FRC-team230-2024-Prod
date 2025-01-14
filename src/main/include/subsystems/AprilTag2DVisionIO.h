#pragma once

#include <photon/PhotonCamera.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>

#include "frc/smartdashboard/SmartDashboard.h"

#include "Constants.h"
#include "util/Subsystem.h"
#include "util/MathUtil.h"

struct Vision2DUpdate
{
    double pitchToTarget;
    double bearingToTarget;
    bool hasTarget;
    int fiducialId;
};

typedef enum
{
    kSpeakerTag = 0,
    kAmpTag = 1,
    kTrapTags = 2
} Target;

class AprilTag2DVisionIO : public Subsystem
{
public:
    AprilTag2DVisionIO(const double p_cameraHeadingOffset, const double p_cameraPitchOffset, std::string p_cameraName);
    void LocalReset() override;
    void Analyze() override;
    void UpdateDash() override;
    void Stop() override{};
    void Periodic() override{};

    void SetTarget(Target p_target);
    Vision2DUpdate GetVisionUpdate() { return m_visionUpdate; };

private:
    std::string m_cameraName;
    double m_cameraHeadingOffset;
    double m_cameraPitchOffset;

    photon::PhotonCamera *m_camera;

    std::vector<int> m_targetFIDs;
    units::second_t m_latency{0_s};
    Vision2DUpdate m_visionUpdate;
};