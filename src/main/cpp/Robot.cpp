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
  printf("Running RobotInit\n");
  // * Enable logging
  frc::DataLogManager::Start();
  frc::DataLogManager::LogNetworkTables(true);

  // * Loop count
  m_count = Loopcount::GetInstance();

  // * PDP
  m_pdp = new frc::PowerDistribution(Constants::CAN::PDP_MODULE, frc::PowerDistribution::ModuleType::kRev);

  // * Subsystems
  m_subsystemManager = new SubsystemManager();

  m_drivetrain = Drivetrain::GetInstance();
  m_userInput = Joystick::GetInstance();
  m_aprilTagVision = AprilTag2DVision::GetInstance();
  m_noteVision = NoteVision::GetInstance();
  m_gatherer = Gatherer::GetInstance();
  m_shooter = Shooter::GetInstance();
  m_arm = Arm::GetInstance();
  m_armKin = ArmKinematics::GetInstance();
  m_led = LED::GetInstance();
  m_climber = Climber::GetInstance();

  m_subsystemManager->AddSubsystems({m_drivetrain, m_userInput, m_gatherer, m_shooter, m_arm, m_led, m_climber, m_noteVision, m_aprilTagVision});

  //  * Robot state machine
  m_robotStateMachine = RobotStateMachine::GetInstance();

  // * Robot pose state
  m_robotState = RobotState::GetInstance();
  m_robotState->Reset(Pose2d(582.25, 232.625, 0.0));

  m_autonomous = Autonomous::GetInstance();
  m_autonomous->UpdateAutoSelection();

  LocalReset();

  printf("RobotInit complete\n");
}

/**
 * @brief Resets all variables and subsystem variables to their default values
 *
 */
void Robot::LocalReset()
{
  printf("Running LocalReset\n");
  // * Global loop count
  m_count->ResetLoopCount();

  // * If alliance is not set, set it to the default value
  m_alliance = frc::DriverStation::GetAlliance().value_or(Constants::DEFAULT_ALLIANCE);

  m_subsystemManager->ResetAll();
  m_robotStateMachine->Reset();

  m_pdp->SetSwitchableChannel(false);

  printf("LocalReset complete\n");
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

  std::optional<double> initialRobotHeading = m_autonomous->GetAutoInitialRobotHeading();
  if (initialRobotHeading.has_value())
  {
    m_drivetrain->SetGyroHeading(initialRobotHeading.value());
  }
  else
  {
    m_drivetrain->SetGyroHeading(0.0);
  }
  m_drivetrain->GyroReset();
  m_drivetrain->EnableGyro();
  m_pdp->SetSwitchableChannel(true);
  m_drivetrain->SetIsAutonomous(true);
}
/**
 * @brief Called every loop count during autonomous mode
 *
 */
void Robot::AutonomousPeriodic()
{
  m_count->IncrementLoopCount();
  m_subsystemManager->AnalyzeAll();

  m_autonomous->RunAuto();

  double xdot, ydot, psidot;
  m_drivetrain->GetChassisSpeeds(xdot, ydot, psidot);

  m_armKin->Update();

  m_robotStateMachine->Update(xdot, ydot, psidot);

  m_drivetrain->SetChassisSpeeds(xdot, ydot, psidot);

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
  m_pdp->SetSwitchableChannel(true);
  m_drivetrain->SetIsAutonomous(false);
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

  // double armCmd = m_userInput->GetGamepad_RAW_X();
  // m_arm->IAmHijacked(armCmd);

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

  // ! Gamepad Stuff
  if (Constants::SHOOT_CALIBRATION_MODE)
  {
    // double hijackedIndex = (m_userInput->GetFlightCtrl_Slider() + 1.0) * 10.0 + 0.0;
    double hijackedIndex = (m_userInput->GetFlightCtrl_Slider() + 1.0) * 10.0 + 40.0;


    frc::SmartDashboard::PutNumber("HijackedIndex", hijackedIndex);

    if (m_userInput->FlightCtrlBtnPushed(Constants::FlightCtrlButtons::SHOOTER_CAL_BUTTON))
    {

      m_robotStateMachine->SetHijackedIndex(hijackedIndex);
      m_robotStateMachine->EnableHijackMode();
    }
    else
    {
      m_robotStateMachine->DisableHijackMode();
    }
  }

  if (m_userInput->GetDpadUpPushed())
  {
    // high shoot
    m_robotStateMachine->HighShootingAction();
  }
  else if (m_userInput->GetDpadDownPushed())
  {
    // out of frame shoot
    m_robotStateMachine->OutOfFrameShootingAction();
  }

  if (m_userInput->GetGamepadButton(Constants::GamepadButtons::PREP_TO_SHOOT_BUTTON))
  {
    // prep to shoot
    m_robotStateMachine->AutoAimAction();
  }

  if (m_userInput->GetGamepadButton(Constants::GamepadButtons::STOW_POS_BUTTON))
  {
    // go to stow
    m_robotStateMachine->GoToInframeAction();
  }

  else if (m_userInput->GetGamepadButton(Constants::GamepadButtons::AMP_POS_BUTTON))
  {
    // amp pos
    m_robotStateMachine->GoToAmpAction();
  }
  else if (m_userInput->GetGamepadButton(Constants::GamepadButtons::TRAP_POS_BUTTON))
  {
    // trap pos
    m_robotStateMachine->GoToLowTrapAction();
  }
  else if (m_userInput->GetGamepadButton(Constants::GamepadButtons::HIGH_TRAP_POS_BUTTON))
  {
    // trap pos
    m_robotStateMachine->GoToHighTrapAction();
  }

  if (m_userInput->GetGamepadButton(Constants::GamepadButtons::PICKUP_NOTE_BUTTON))
  {
    // pick up note
    m_robotStateMachine->GatherAction();
  }

  if (m_userInput->GetGamepadButton(Constants::GamepadButtons::SHOOT_BUTTON))
  {
    // shoot
    m_robotStateMachine->ShootAction();
  }

  if (m_userInput->GetGamepadButton(Constants::GamepadButtons::EJECT_NOTE_BUTTON))
  {
    // eject
    m_robotStateMachine->EjectAction();
  }

  /* Climb lock*/
  if (m_userInput->GetGamepadButton(Constants::GamepadButtons::CLIMB_LOCK_BUTTON) == kPressing)
  {
    // climb lock
    m_robotStateMachine->ToggleClimbLock();
  }

  /* LED Stuff */
  if (m_userInput->GetDpadLeftPushed())
  {
    m_led->DisplayGamepieceWanted();
  }
  else if (m_userInput->GetDpadRightPushed())
  {
    m_led->DisplayAmplify();
  }
  else
  {
    m_led->DisplayBlank();
  }

  if (m_userInput->GetGamepad_RAW_Y() < -0.1)
  {
    m_robotStateMachine->ClimbUpAction();
  }
  else if (m_userInput->GetGamepad_RAW_Y() > 0.1)
  {
    m_robotStateMachine->ClimbDownAction();
  }

  if (m_userInput->GetFlightCtrlButton(Constants::FlightCtrlButtons::AMP_ALIGN_BUTTON_1) || m_userInput->GetFlightCtrlButton(Constants::FlightCtrlButtons::AMP_ALIGN_BUTTON_2))
  {
    m_robotStateMachine->AmpAlignAction();
  }

  m_robotStateMachine->Update(xdot, ydot, psidot);

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

  m_armKin->Update();
  m_subsystemManager->RunPeriodicAll();
  // ServiceDash(true);
  m_subsystemManager->UpdateDashAll();
}
/**
 * @brief Called when the robot enters disabled mode
 *
 */
void Robot::DisabledInit()
{
  printf("Running Disabled init\n");
  LocalReset();
  m_autonomous->Reset();
  m_autonomous->UpdateAutoSelection();
  printf("Disabled init complete");
  // m_subsystemManager->StopAll();
}
/**
 * @brief Called every loop count during disabled mode
 *
 */
void Robot::DisabledPeriodic()
{
  m_count->IncrementLoopCount();
  m_subsystemManager->AnalyzeAll();
  m_autonomous->UpdateAutoSelection();

  if (frc::DriverStation::IsDSAttached())
  {
    m_led->DisplayConnected();
  }
  else
  {
    m_led->DisplayDisconnected();
  }
  m_led->Periodic();

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
