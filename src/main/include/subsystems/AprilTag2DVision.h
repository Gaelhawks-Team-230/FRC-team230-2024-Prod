#pragma once

#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include "Constants.h"
#include "util/Subsystem.h"
#include "subsystems/AprilTag2DVisionIO.h"
#include "util/planners/Interpolator.h"
#include "subsystems/Drivetrain.h"

const double KPSI_SPEAKER = 2.0;
const double SPEAKER_R_VEL_LIMIT = 120.0;
const double SPEAKER_GOAL_HEADING = 0.0;

const double KPSI_TRAP = 4.0;
const double TRAP_R_VEL_LIMIT = 120.0;

const double KPSI_AMP = 4.0;
const double AMP_R_VEL_LIMIT = 120.0;

// * Pitch of camera, distance from robot
// const std::map<double, std::vector<double>> SPEAKER_DISTANCE_TABLE{{13.35, {3.0}}, {4.0, {5.0}}, {-1.8, {7.0}}, {-6.93, {10.0}}, {-9.75, {13.0}}, {-11.2, {15.0}}, {-12.81, {19.0}}};
const std::map<double, std::vector<double>> SPEAKER_DISTANCE_TABLE{{9.5, {3.0}}, {1.0, {5.0}}, {-4.3, {7.0}}, {-9.26, {10.0}}, {-11.79, {13.0}}, {-11.2, {15.0}}, {-12.81, {19.0}}};
// 3.0ft   9.5
// 4.0ft  4.66
// 5.0ft 1.0
// 7.0ft -4.3
// 10ft -9.26
// 13ft -11.79
// 15ft 
// 19ft

const std::map<double, std::vector<double>> TRAP_DISTANCE_TABLE{};
const std::map<double, std::vector<double>> AMP_DISTANCE_TABLE{};

class AprilTag2DVision : public Subsystem
{
public:
    void LocalReset(void) override;
    void Analyze() override;
    void UpdateDash() override;
    void Periodic() override{};
    void Stop() override{};

    void SpeakerDriveControl(double &xdot, double &ydot, double &psidot);
    void TrapDriveControl(double &xdot, double &ydot, double &psidot);
    void AmpDriveControl(double &xdot, double &ydot, double &psidot);

    double GetSpeakerDistance();
    bool SeesSpeakerTag() { return m_speakerVisionUpdate.hasTarget; };

    /**
     * @brief Check if robot is aligned to speaker
     *
     * @param tolerance tolerance in degrees of the heading
     * @return true
     * @return false
     */
    bool IsAlignedToSpeaker(double tolerance = 2.0)
    {
        return m_speakerVisionUpdate.hasTarget & (fabs(m_speakerVisionUpdate.bearingToTarget - SPEAKER_GOAL_HEADING) < tolerance);
    }

    /**
     * @brief Check if robot is aligned to trap
     *
     * @param tolerance
     * @return true
     * @return false
     */
    bool IsAlignedToTrap(double tolerance = 2.0)
    {
        // TODO implement check
        return false;
    };

    /**
     * @brief Check if robot is aligned to amp
     *
     * @param tolerance
     * @return true
     * @return false
     */
    bool IsAlignedToAmp(double tolerance = 2.0)
    {
        // TODO implement check
        return false;
    }

    static AprilTag2DVision *GetInstance();

private:
    AprilTag2DVision();
    static AprilTag2DVision *m_instance;

    AprilTag2DVisionIO *m_frontLeftCamera;
    Vision2DUpdate m_speakerVisionUpdate;
    Vision2DUpdate m_trapVisionUpdate;
    Vision2DUpdate m_ampVisionUpdate;

    Interpolator *m_speakerTable;
    Interpolator *m_trapTable;
    Interpolator *m_ampTable;

    Drivetrain *m_drivetrain;
};