#pragma once

#include "subsystems/Gatherer.h"
#include "subsystems/AprilTag2DVision.h"
#include "subsystems/Shooter.h"
#include "subsystems/LED.h"
#include "subsystems/Climber.h"
#include "subsystems/NoteVision.h"
#include "subsystems/ArmKinematics.h"
#include "subsystems/Drivetrain.h"

#include "util/planners/Interpolator.h"
#include "util/Loopcount.h"

#include "frc/Errors.h"

const double AUTO_SHOOTER_SPEED = 5000.0;
const double TELE_SHOOTER_SPEED = 6000.0; // RPM
const double SHOOTER_PCT_DIFF = 0.10;

const double AMP_SHOOTER_SPEED = 200.0;

const double MAX_INFRAME_SHOOTING_INDEX = 60.0;
const double MIN_INFRAME_SHOOTING_INDEX = 40.0;

const double MAX_OUTOFFRAME_SHOOTING_INDEX = 0.0;
const double MIN_OUTOFFRAME_SHOOTING_INDEX = 0.0;

const double DEFAULT_SHOOTING_DIST = 6.5;

const double HIGH_SHOOTING_INDEX = 51.0; // inframe manuel override
const double OUT_OF_FRAME_SHOOTING_INDEX = 1.0;


const double GYRO_KP = 2.0;
const double GYRO_VEL_LIMIT = 120.0;

const std::map<double, std::vector<double>> IN_FRAME_TABLE{{3.0, {57.0}}, {5.0, {53.5}}, {7.0, {51.5}}, {10.0, {49.5}}, {13.0, {49.0}}, {15.0, {46.0}}, {19.0, {46.0}}};
const std::map<double, std::vector<double>> OUT_OF_FRAME_TABLE{};
// const std::map<double, std::vector<double>> HIGH_SHOT_TABLE{};

// Target objective of the robot
enum class RobotTarget
{
    kSpeaker,
    kTrap,
    kAmp,
    kNote,
};

// State of the robot
enum class RobotMode
{
    kIdleMode,
    kGatherMode,
    kShootMode,
    kEjectMode,
};

// State of the climber
enum class ClimbMode
{
    kClimbUp,
    kClimbDown,
    kIdleClimb
};

// State of the shooter
enum class ShooterMode
{
    kInFrameShot,
    kOutOfFrameShot,
    kHighShot,
};

class RobotStateMachine
{
public:
    static RobotStateMachine *GetInstance();

    void Update(double &xdot, double &ydot, double &psidot);
    void Reset();

    /**
     * @brief If we are targeting the speaker, we will shoot the Note.
     *
     */
    void ShootAction()
    {
        m_currentMode = RobotMode::kShootMode;
    };

    /**
     * @brief Sets the shooting mode to high shooting.
     *
     */
    void HighShootingAction()
    {
        m_shooterMode = ShooterMode::kHighShot;
    }

    /**
     * @brief Sets the shooting mode to out of frame shooting.
     *
     */
    void OutOfFrameShootingAction()
    {
        m_shooterMode = ShooterMode::kOutOfFrameShot;
    }

    /**
     * @brief Set the target to Note and set the mode to gather.
     *
     */
    void GatherAction()
    {
        m_currentTarget = RobotTarget::kNote;
        m_currentMode = RobotMode::kGatherMode;
    };

    /**
     * @brief With any target, we will eject the note.
     *
     */
    void EjectAction()
    {
        m_currentMode = RobotMode::kEjectMode;
    };

    /**
     * @brief If the climb mode is not locked, we will set the climb mode to climb up.
     *
     */
    void ClimbUpAction()
    {
        if (!m_isClimbingLocked)
            m_climbMode = ClimbMode::kClimbUp;
    };

    /**
     * @brief If the climb mode is not locked, we will set the climb mode to climb down.
     *
     */
    void ClimbDownAction()
    {
        if (!m_isClimbingLocked)
            m_climbMode = ClimbMode::kClimbDown;
    };

    /**
     * @brief Set the robot to auto aim mode.
     *
     */
    void AutoAimAction()
    {
        m_autoAim = true;
    };

    /**
     * @brief Set target to trap and go to trap position.
     *
     */
    void GoToHighTrapAction()
    {
        m_currentTarget = RobotTarget::kTrap;
        m_armKin->GoToHighTrap();
    };

    void GoToStowAction()
    {
        m_armKin->GoToStow();
        m_currentMode = RobotMode::kIdleMode;
    }

    /**
     * @brief Set target to trap and go to trap position.
     *
     */
    void GoToLowTrapAction()
    {
        m_currentTarget = RobotTarget::kTrap;
        m_armKin->GoToLowTrap();
    };

    /**
     * @brief Set target to amp and go to amp position.
     *
     */
    void GoToAmpAction()
    {
        m_currentTarget = RobotTarget::kAmp;
        m_armKin->GoToAmp();
    };

    /**
     * @brief Go to inframe
     *
     */
    void GoToInframeAction()
    {
        m_armKin->GoToStow();
    };

    /**
     * @brief Go to idle mode.
     *
     */
    void IdleAction()
    {
        m_currentMode = RobotMode::kIdleMode;
    };

    void AmpAlignAction()
    {
        m_shouldAlignToAmp = true;
    }

    /**
     * @brief Toggles the climb lock. Prevents the robot from climbing.
     *
     */
    void ToggleClimbLock() { m_isClimbingLocked = !m_isClimbingLocked; };

    void SetHijackedIndex(double index) { m_hijackedIndex = index; };
    void EnableHijackMode() { hijackedMode = true; };
    void DisableHijackMode() { hijackedMode = false; };

    void AlwaysRunShooter() { m_alwaysRunShooter = true; };

    void SetShootingSolutionIndex(double index)
    {
        m_overrideShooterSolution = true;
        m_overrideShooterSolutionIndex = index;
    }

private:
    RobotStateMachine();
    static RobotStateMachine *m_robotStateMachine;

    Gatherer *m_gatherer;
    Shooter *m_shooter;
    LED *m_led;
    ArmKinematics *m_armKin;
    Loopcount *m_count;
    AprilTag2DVision *m_aprilTagVision;
    NoteVision *m_noteVision;
    Climber *m_climber;
    Drivetrain *m_drivetrain;

    Interpolator *m_inFrameTable;
    Interpolator *m_outOfFrameTable;
    Interpolator *m_highShotTable;

    bool m_hasStoredNote;
    bool m_isStoringNote;
    bool m_isClimbingLocked;

    bool m_shouldAlignToAmp;

    RobotMode m_currentMode;
    RobotTarget m_currentTarget;
    ClimbMode m_climbMode;
    ShooterMode m_shooterMode;
    bool m_autoAim;
    bool m_previousAutoAim;

    RobotMode m_previousMode;

    unsigned int m_stage;
    unsigned int m_stageCountOffset;

    std::string RobotModeToString(RobotMode mode);
    std::string RobotTargetToString(RobotTarget target);
    std::string ClimbModeToString(ClimbMode mode);
    std::string ShooterModeToString(ShooterMode mode);

    double m_hijackedIndex;
    bool hijackedMode = false;

    bool m_alwaysRunShooter = false;
    bool m_overrideShooterSolution = false;
    double m_overrideShooterSolutionIndex = 20.0;

    unsigned int m_ledStage;

     /**
     * @brief Get the Wrapped Angle error to the amp. Checks what alliance we are on.
     *
     * @return double
     */
    double GetAngleErrorToAmp()
    {
        double ampAngle;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            ampAngle = -90.0;
        }
        else
        {
            ampAngle = 90.0;
        }

        double posErr = ampAngle - m_drivetrain->GetGyroWrappedAngle();
        double wrappedPosErr = MathUtil::Wrap(posErr);
        return wrappedPosErr;
    };
};
