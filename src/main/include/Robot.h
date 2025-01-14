#pragma once

#include <frc/TimedRobot.h>
#include "frc/PowerDistribution.h"
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>

#include "Constants.h"
#include "SubsystemManager.h"

#include "subsystems/Arm.h"
#include "auto/Autonomous.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Joystick.h"
#include "subsystems/AprilTag2DVision.h"
#include "subsystems/NoteVision.h"
#include "subsystems/Gatherer.h"
#include "subsystems/Shooter.h"
#include "subsystems/ArmKinematics.h"
#include "subsystems/LED.h"
#include "subsystems/Climber.h"
#include "util/Loopcount.h"

#include "RobotStateMachine.h"
#include "RobotState.h"



class Robot : public frc::TimedRobot
{
public:
  Robot();

  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void ServiceDash(bool isRobotDisabled);
  void LocalReset();

  frc::DriverStation::Alliance GetAlliance() { return m_alliance; };

private:
  frc::DriverStation::Alliance m_alliance;

  Loopcount *m_count;

  frc::PowerDistribution *m_pdp;
  SubsystemManager *m_subsystemManager;

  Drivetrain *m_drivetrain;
  Joystick *m_userInput;
  Climber *m_climber;
  AprilTag2DVision *m_aprilTagVision;
  NoteVision *m_noteVision;
  Gatherer *m_gatherer;
  LED *m_led;
  RobotStateMachine *m_robotStateMachine;  Shooter *m_shooter;
  Arm *m_arm;
  ArmKinematics *m_armKin;
  Autonomous *m_autonomous;


  RobotState *m_robotState;
};
