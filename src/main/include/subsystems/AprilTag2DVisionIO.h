#pragma once

#include <photon/PhotonUtils.h>
#include <photon/PhotonCamera.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>

#include "frc/smartdashboard/SmartDashboard.h"

#include "Constants.h"
#include "util/Subsystem.h"
#include "util/MathUtil.h"

struct Vision2DUpdate
{
    double rangeToTarget;
    double bearingToTarget;
    bool hasTarget;
};

typedef enum
{
    kSpeaker = 0,
    kAmp = 1,
} Target;

class AprilTag2DVisionIO : public Subsystem
{
public:
    AprilTag2DVisionIO(const double p_cameraHeight, const double p_cameraPitch, std::string p_cameraName);
    void LocalReset() override;
    void Analyze() override;
    void UpdateDash() override;
    void Stop() override{};
    void Periodic() override{};

    void SetTarget(Target p_target);

    Vision2DUpdate GetVisionUpdate() { return m_visionUpdate; };

private:
    std::string m_cameraName;

    double m_cameraHeight;
    double m_cameraPitch;

    photon::PhotonCamera *m_camera;

    units::second_t m_latency{0_s};

    int m_targetFID;
    double m_targetHeight;

    photon::PhotonTrackedTarget m_selectedTarget;

    bool m_hasTarget;

    Vision2DUpdate m_visionUpdate;

    std::map<int, double> APRILTAG_HEIGHT_LOOKUP;
};