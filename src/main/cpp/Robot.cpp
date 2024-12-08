#include "Robot.h"

/**
 * @brief Constructs a new Timed Robot object
 *
 */
Robot::Robot() : TimedRobot(units::second_t{Constants::LOOPTIME})
{
}
/**
 * @brief Called when the robot is first started up
 *
 */
void Robot::RobotInit()
{
  m_count = Loopcount::GetInstance();

  m_pdp = new frc::PowerDistribution(Constants::CAN::PDP_MODULE, frc::PowerDistribution::ModuleType::kCTRE);
  m_subsystemManager = new SubsystemManager();

  m_drivetrain = Drivetrain::GetInstance();
  m_userInput = Joystick::GetInstance();
  m_aprilTagVision = AprilTag2DVision::GetInstance();
  m_noteVision = NoteVision::GetInstance();

  m_robotState = RobotState::GetInstance();
  m_robotState->Reset(Pose2d(582.25, 232.625, 0.0));

  m_subsystemManager->AddSubsystems({m_drivetrain, m_userInput, m_aprilTagVision, m_noteVision});

  LocalReset();
}

/**
 * @brief Resets all variables and subsystem variables to their default values
 *
 */
void Robot::LocalReset()
{
  // * Global loop count
  m_count->ResetLoopCount();

  // * If alliance is not set, set it to the default value
  m_alliance = frc::DriverStation::GetAlliance().value_or(Constants::DEFAULT_ALLIANCE);

  m_subsystemManager->ResetAll();
}

/**
 * This function is called every loop count, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
}
/**
 * @brief Called when the robot enters autonomous mode
 *
 */
void Robot::AutonomousInit()
{
  LocalReset();
}
/**
 * @brief Called every loop count during autonomous mode
 *
 */
void Robot::AutonomousPeriodic()
{
  m_count->IncrementLoopCount();
  m_subsystemManager->AnalyzeAll();
  m_subsystemManager->RunPeriodicAll();
  ServiceDash(true);
}
/**
 * @brief Called when the robot enters teleoperated mode
 *
 */
void Robot::TeleopInit()
{
  LocalReset();
}
/**
 * @brief Called every loop count during teleoperated mode
 *
 */
void Robot::TeleopPeriodic()
{
  double xdot, ydot, psidot;
  m_count->IncrementLoopCount();
  m_subsystemManager->AnalyzeAll();

  xdot = m_userInput->GetFlightCtrl_CMD_X();
  ydot = m_userInput->GetFlightCtrl_CMD_Y();
  psidot = m_userInput->GetFlightCtrl_CMD_R();

  // * Drive control convert to field right off the bat
  if (m_userInput->GetFlightCtrlButton(Constants::FlightCtrlButtons::FIELD_ROBOT_SWITCH))
  {
    m_drivetrain->SetZeroVelDebugModeOff();
  }
  else
  {
    m_drivetrain->ToFieldCoordinates(&xdot, &ydot);
    m_drivetrain->SetZeroVelDebugModeOff();
  }

  // * Vision Stuff
  if (m_userInput->GetFlightCtrlButton(Constants::GamepadButtons::PICKUP_NOTE_BUTTON))
  {
    // Set arm goal position to pickup
    m_noteVision->SetGoalXDist(6.0);
    m_noteVision->DriveTargetting(&xdot, &psidot);
    // when gamepiece acquired (beam broken), set goal to shooting pos, stop note tracking
    // continue auto gather
  }

  if (m_userInput->GetFlightCtrlButton(Constants::GamepadButtons::PREP_TO_SHOOT_BUTTON))
  {
    m_aprilTagVision->SpeakerDriveTargetting(&psidot);
    // calculate shooting position and adjust platform
  }

  // * Begin Drive control
  if (m_userInput->GetFlightCtrlButton(Constants::FlightCtrlButtons::RESET_GYRO_BUTTON))
  {
    m_drivetrain->GyroReset();
  }

  if (m_userInput->GetFlightCtrlButton(Constants::FlightCtrlButtons::GYRO_BUTTON_SWITCH))
  {
    m_drivetrain->EnableGyro();
  }
  else
  {
    m_drivetrain->DisableGyro();
  }

  // VisionUpdate visionUpdate = m_aprilTagVision->GetVisionUpdate();
  // if (visionUpdate.hasNewResult && visionUpdate.hasTarget)
  // if (false)
  // {
  //   m_robotState->Update(visionUpdate.pose, xdot, ydot, psidot);
  // }
  // else
  // {
    m_robotState->Update(std::nullopt, xdot, ydot, psidot);
  // }

  m_drivetrain->SetChassisSpeeds(xdot, ydot, psidot);

  m_subsystemManager->RunPeriodicAll();
  ServiceDash(true);
}
/**
 * @brief Called when the robot enters disabled mode
 *
 */
void Robot::DisabledInit()
{
  LocalReset();
  m_subsystemManager->StopAll();
}
/**
 * @brief Called every loop count during disabled mode
 *
 */
void Robot::DisabledPeriodic()
{
  m_count->IncrementLoopCount();
  m_subsystemManager->AnalyzeAll();
  ServiceDash(true);
}

/**
 * @brief Updates the SmartDashboard and telemetry with values
 *
 */
void Robot::ServiceDash(bool isRobotDisabled)
{
  frc::SmartDashboard::PutNumber("Loopcount", m_count->GetLoopCount());
  frc::SmartDashboard::PutNumber("CANUtilization", (frc::RobotController::GetCANStatus().percentBusUtilization * 100));
  frc::SmartDashboard::PutBoolean("Alliance", m_alliance);
  m_subsystemManager->UpdateDashAll();

  frc::SmartDashboard::PutNumberArray("Pose/RobotState", m_robotState->GetPose().GetPoseAsArray());
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
