#include "RobotStateMachine.h"

/**
 * @brief Get instance of RobotStateMachine
 *
 * @return RobotStateMachine*
 */
RobotStateMachine *RobotStateMachine::GetInstance()
{
    if (m_robotStateMachine == nullptr)
    {
        m_robotStateMachine = new RobotStateMachine();
    }
    return m_robotStateMachine;
}
/**
 * @brief Construct a new Robot State Machine:: Robot State Machine object
 *
 */
RobotStateMachine::RobotStateMachine()
{
    m_gatherer = Gatherer::GetInstance();
    m_shooter = Shooter::GetInstance();
    m_led = LED::GetInstance();
    m_armKin = ArmKinematics::GetInstance();
    m_count = Loopcount::GetInstance();
    m_aprilTagVision = AprilTag2DVision::GetInstance();
    m_noteVision = NoteVision::GetInstance();
    m_climber = Climber::GetInstance();
    m_drivetrain = Drivetrain::GetInstance();

    m_inFrameTable = new Interpolator(IN_FRAME_TABLE);
    m_outOfFrameTable = new Interpolator(OUT_OF_FRAME_TABLE);
    // m_highShotTable = new Interpolator(HIGH_SHOT_TABLE);
}
void RobotStateMachine::Reset()
{
    m_currentMode = RobotMode::kIdleMode;
    m_previousMode = RobotMode::kIdleMode;

    m_currentTarget = RobotTarget::kSpeaker;

    m_climbMode = ClimbMode::kIdleClimb;
    m_shooterMode = ShooterMode::kInFrameShot;

    m_autoAim = false;
    m_previousAutoAim = false;

    m_hasStoredNote = false;
    m_isStoringNote = false;
    m_isClimbingLocked = true;

    m_stage = 0;
    m_stageCountOffset = 0;

    m_overrideShooterSolution = 0.0;
    m_overrideShooterSolutionIndex = 20.0;

    m_shouldAlignToAmp = false;
    m_ledStage = 0;
}

void RobotStateMachine::Update(double &xdot, double &ydot, double &psidot)
{
    m_hasStoredNote = m_gatherer->IsStorageBeamBroken();
    m_isStoringNote = !m_hasStoredNote && m_gatherer->IsEntryBeamBroken();
    bool isShooterReady = m_shooter->IsShooterReady();

    bool isNoteLeaving = m_hasStoredNote && !m_gatherer->IsEntryBeamBroken();

    if (m_aprilTagVision->SeesSpeakerTag() && m_hasStoredNote)
    {
        m_led->DisplayGamepieceWanted();
    }

    if (isNoteLeaving)
    {
        m_ledStage = 1;
    }
    if (m_ledStage)
    {
        m_ledStage++;
        m_led->DisplayAmplify();

        if (m_ledStage > 40)
        {
            m_ledStage = 0;
        }
    }

    if (m_alwaysRunShooter)
    {
        m_shooter->SetLeftShooterVel(AUTO_SHOOTER_SPEED);
        m_shooter->SetRightShooterVel(AUTO_SHOOTER_SPEED * (1.0 - SHOOTER_PCT_DIFF));
    }
    else
    {
        // * Default the shooter velocity to 0
        m_shooter->SetLeftShooterVel(0.0);
        m_shooter->SetRightShooterVel(0.0);
    }

    //  * Default the gatherer to stop
    m_gatherer->StopIntake();

    // * Robot mode state machine
    switch (m_currentMode)
    {
    case RobotMode::kIdleMode:
        m_stage = 0;
        break;
    case RobotMode::kGatherMode:
        if (m_isStoringNote)
        {
            m_led->DisplayHasGamepiece();
        }

        switch (m_stage)
        {
        case 0:
            if (m_hasStoredNote)
            {
                m_stage = 0;
                break;
            }
            // * Arm to floor pickup
            if (!Constants::SHOOT_CALIBRATION_MODE)
            {
                m_armKin->GoToPickup();
            }
            m_stage++;
            break;
        case 1:
            if (m_isStoringNote)
            {
                if (Constants::SHOOT_CALIBRATION_MODE)
                {
                    //  * Arm should not move in calibration mode
                }
                else if (m_shooterMode == ShooterMode::kInFrameShot)
                {
                    m_armKin->GoToStow();
                }
                else if (m_shooterMode == ShooterMode::kOutOfFrameShot)
                {
                    // * Arm should not move in out of frame shot mode because the arm is already in the middle of the shooting table
                }

                m_stage++;
                break;
            }
            m_led->DisplayHunting();
            m_gatherer->GatherPieceFast();
            break;
        case 2:
            if (m_hasStoredNote)
            {
                m_gatherer->StopIntake();
                m_stage = 0;
                break;
            }
            m_gatherer->GatherPieceSlow();
            break;
        }
        break;
    case RobotMode::kShootMode:
        if (!isShooterReady)
        {
            // * Do not shoot if the shooter is not ready
        }
        else
        {
            m_gatherer->ReleasePiece();
        }
        break;
    case RobotMode::kEjectMode:
        m_gatherer->EjectPiece();
        break;
    }

    // * Climbing state machine
    switch (m_climbMode)
    {
    case ClimbMode::kClimbUp:
        // * Climber up
        m_climber->ClimbUp();
        break;
    case ClimbMode::kClimbDown:
        // * Climber down
        m_climber->ClimbDown();
        break;
    case ClimbMode::kIdleClimb:
        // * Stop climber
        m_climber->StopClimb();
        break;
    }

    //  * Calculate Shooting Solution
    double distToSpeaker = m_aprilTagVision->GetSpeakerDistance();
    double armShootingSolution;
    bool l_seesSpeakerTag = m_aprilTagVision->SeesSpeakerTag();

    switch (m_shooterMode)
    {
    case ShooterMode::kInFrameShot:
        armShootingSolution = m_inFrameTable->Sample(distToSpeaker)[0];
        armShootingSolution = MathUtil::Limit(MIN_INFRAME_SHOOTING_INDEX, MAX_INFRAME_SHOOTING_INDEX, armShootingSolution);
        break;
    case ShooterMode::kOutOfFrameShot:
        // armShootingSolution = m_outOfFrameTable->Sample(distToSpeaker)[0];
        // armShootingSolution = MathUtil::Limit(MIN_OUTOFFRAME_SHOOTING_INDEX, MAX_OUTOFFRAME_SHOOTING_INDEX, armShootingSolution);
        armShootingSolution = OUT_OF_FRAME_SHOOTING_INDEX;
        m_overrideShooterSolution = true;
        break;
    case ShooterMode::kHighShot:
        l_seesSpeakerTag = true;
        armShootingSolution = HIGH_SHOOTING_INDEX;
        break;
    }

    if (m_overrideShooterSolution && m_shooterMode != ShooterMode::kOutOfFrameShot)
    {
        armShootingSolution = m_overrideShooterSolutionIndex;
    }

    // * Auto aim functionality (prepare to shoot for speaker)
    if (m_autoAim)
    {
        switch (m_currentTarget)
        {
        case RobotTarget::kNote:
            // * Auto track to note
            if (!m_hasStoredNote && !m_isStoringNote)
            {
                m_noteVision->DriveControl(xdot, ydot, psidot);
            }
            break;
        case RobotTarget::kSpeaker:
            // * Auto aim to speaker april tag, adjust arm angle, and prep shooter wheels
            if (!m_overrideShooterSolution)
            {
                m_shooter->SetLeftShooterVel(TELE_SHOOTER_SPEED);
                m_shooter->SetRightShooterVel(TELE_SHOOTER_SPEED * (1.0 - SHOOTER_PCT_DIFF));
            }

            if (l_seesSpeakerTag || m_overrideShooterSolution)
            {
                m_armKin->GoToShootingPos(armShootingSolution);
            }

            if (!m_overrideShooterSolution)
            {
                m_aprilTagVision->SpeakerDriveControl(xdot, ydot, psidot);
            }

            break;
        case RobotTarget::kAmp:
            // TODO Implement auto aim for amp
            break;
        case RobotTarget::kTrap:
            // TODO Implement trap auto aim
            break;
        }
    }

    // * Amp Alignment on flight controller
    if (m_shouldAlignToAmp)
    {
        psidot = MathUtil::Limit(-120.0, 120.0, GetAngleErrorToAmp() * GYRO_KP);
    }

    if (m_previousMode != m_currentMode)
    {
        switch (m_previousMode)
        {
        case RobotMode::kIdleMode:
            //  nothing to do
            break;
        case RobotMode::kGatherMode:
            // * After gatherering set the default mode to shoot in the speaker
            m_currentTarget = RobotTarget::kSpeaker;
            m_shooterMode = ShooterMode::kInFrameShot;
            break;
        case RobotMode::kShootMode:
            // * Automatically go to stow after shooting in speaker mode
            if (m_currentTarget == RobotTarget::kSpeaker)
            {
                m_armKin->GoToStow();
            }
            break;
        case RobotMode::kEjectMode:
            // * Automatically go to stow after ejecting in amp mode
            if (m_currentTarget == RobotTarget::kAmp)
            {
                m_armKin->GoToStow();
            }
            m_currentTarget = RobotTarget::kNote;
            m_shooterMode = ShooterMode::kInFrameShot;
            break;
        }
    }

    if (m_previousAutoAim != m_autoAim)
    {
        switch (m_currentTarget)
        {
        case RobotTarget::kNote:
            //  nothing to do
            break;
        case RobotTarget::kSpeaker:
            //  nothing to do
            // m_armKin->GoToShootingPos(m_inFrameTable->Sample(DEFAULT_SHOOTING_DIST)[0]);
            break;
        case RobotTarget::kAmp:
            //  nothing to do
            break;
        case RobotTarget::kTrap:
            //  nothing to do
            break;
        }
    }

    // * Hijack mode for calibration
    if (Constants::SHOOT_CALIBRATION_MODE && hijackedMode)
    {
        m_armKin->GoToShootingPos(m_hijackedIndex);
    }

    frc::SmartDashboard::PutString("StateMachine/RobotState", RobotModeToString(m_currentMode));
    frc::SmartDashboard::PutString("StateMachine/Target", RobotTargetToString(m_currentTarget));
    frc::SmartDashboard::PutString("StateMachine/Climbing Mode", ClimbModeToString(m_climbMode));
    frc::SmartDashboard::PutString("StateMachine/Shooter Mode", ShooterModeToString(m_shooterMode));
    frc::SmartDashboard::PutBoolean("StateMachine/HasStoredNote", m_hasStoredNote);
    frc::SmartDashboard::PutBoolean("StateMachine/ClimbLock", m_isClimbingLocked);
    frc::SmartDashboard::PutBoolean("StateMachine/IsStoringNote", m_isStoringNote);
    frc::SmartDashboard::PutBoolean("StateMachine/AutoAim", m_autoAim);

    m_previousAutoAim = m_autoAim;
    m_autoAim = false;
    m_climbMode = ClimbMode::kIdleClimb;
    m_previousMode = m_currentMode;
    m_alwaysRunShooter = false;
    m_currentMode = RobotMode::kIdleMode;
    m_overrideShooterSolution = false;
    m_shouldAlignToAmp = false;
    m_shooterMode = ShooterMode::kInFrameShot;
}
/**
 * @brief Get a string representation of the current robot target
 *
 * @param target
 * @return std::string
 */
std::string RobotStateMachine::RobotTargetToString(RobotTarget target)
{
    switch (target)
    {
    case RobotTarget::kSpeaker:
        return "Speaker";
    case RobotTarget::kTrap:
        return "Trap";
    case RobotTarget::kAmp:
        return "Amp";
    case RobotTarget::kNote:
        return "Note";
    default:
        return "Unknown Target";
    }
}

/**
 * @brief Get a string representation of the current robot mode
 *
 * @param mode
 * @return std::string
 */
std::string RobotStateMachine::RobotModeToString(RobotMode mode)
{
    switch (mode)
    {
    case RobotMode::kIdleMode:
        return "Idle";
    case RobotMode::kGatherMode:
        return "Gather";
    case RobotMode::kShootMode:
        return "Shoot";
    case RobotMode::kEjectMode:
        return "Eject";
    default:
        return "Unknown Mode";
    }
}

/**
 * @brief Get a string representation of the current climb mode
 *
 * @param mode
 * @return std::string
 */
std::string RobotStateMachine::ClimbModeToString(ClimbMode mode)
{
    switch (mode)
    {
    case ClimbMode::kClimbUp:
        return "Climb Up";
    case ClimbMode::kClimbDown:
        return "Climb Down";
    case ClimbMode::kIdleClimb:
        return "Idle";
    default:
        return "Unknown Climb Mode";
    }
}

std::string RobotStateMachine::ShooterModeToString(ShooterMode mode)
{
    switch (mode)
    {
    case ShooterMode::kInFrameShot:
        return "In Frame Shot";
    case ShooterMode::kOutOfFrameShot:
        return "Out of Frame Shot";
    case ShooterMode::kHighShot:
        return "High Shot";
    default:
        return "Unknown Shooter Mode";
    }
}

RobotStateMachine *RobotStateMachine::m_robotStateMachine = nullptr;