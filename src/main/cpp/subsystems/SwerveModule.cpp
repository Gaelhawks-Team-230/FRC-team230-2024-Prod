#include "subsystems/SwerveModule.h"

/**
 * @brief Constructor for a single swerve module
 *
 * @param pRobot main robot
 * @param steerFalconID CAN ID of steer falcon
 * @param driveFalconID CAN ID of drive falcon
 * @param absoluteEncoderID CAN ID of the CANCoder ID
 * @return SwerveModule
 */
SwerveModule::SwerveModule(unsigned int steerFalconID, unsigned int driveFalconID, unsigned int absoluteEncoderID, unsigned int swerveID, double steerOffset)
{
    m_moduleID = swerveID;

    m_steer = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(steerFalconID);
    m_drive = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(driveFalconID);
    m_steerEncoder = new ctre::phoenix::sensors::CANCoder(absoluteEncoderID);

    m_steerOffset = steerOffset;

    StartingConfig();
    LocalReset();
}
/**
 * @brief Set Falcons to starting configuration
 *
 */
void SwerveModule::StartingConfig()
{
    if (Constants::CTRE_FACTORY_RESET)
    {
        printf("Swerve Module Factory Reset\n");
        m_steer->ConfigFactoryDefault();
        m_drive->ConfigFactoryDefault();
        m_steerEncoder->ConfigFactoryDefault();
    }

    if (Constants::SWERVE_CALIBRATION_MODE)
    {
        printf("Swerve Drive Calibration Mode: (MOTORS DISABLED)\n");
    }

    m_steer->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_drive->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_drive->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_drive->ConfigVelocityMeasurementWindow(DRIVE_VELOCITY_MEAS_WINDOW);

    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currLimitConfig;
    currLimitConfig.enable = DRIVE_CURRENT_LIMIT;
    currLimitConfig.currentLimit = DRIVE_STATOR_CURRENT_LIMIT;
    m_drive->ConfigStatorCurrentLimit(currLimitConfig);

    m_steer->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_steer->ConfigVelocityMeasurementWindow(STEER_VELOCITY_MEAS_WINDOW);

    m_steerEncoder->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_steerEncoder->ConfigVelocityMeasurementWindow(STEER_VELOCITY_MEAS_WINDOW);

    m_drive->ConfigNeutralDeadband(DRIVE_DEADBAND);
    m_steer->ConfigNeutralDeadband(STEER_DEADBAND);
}
/**
 * @brief Reset all variabes to default
 *
 */
void SwerveModule::LocalReset()
{
    m_driveLastPosition = m_drive->GetSelectedSensorPosition();

    m_driveLastPosition = 0.0;
    m_driveCurrentPosition = 0.0;
    m_driveRaw = 0.0;
    m_driveOffset = 0.0;
    m_drivePosition = 0.0;
    m_driveVel = 0.0;
    m_driveMotorVel = 0.0;
    m_drivePosVel = 0.0;

    m_steerMotorPosition = 0.0;
    m_steerMotorVel = 0.0;
    m_steerEncoderPosition = 0.0;
    m_steerEncoderRawPosition = 0.0;
    m_steerEncoderVel = 0.0;

    m_steer_pErr = 0.0;
    m_steer_vCmd = 0.0;
    m_steer_vErr = 0.0;
    m_steer_vErrIntegrator = 0.0;
    m_steer_cmd = 0.0;

    m_rev = 0.0;

    m_drive_pErr = 0.0;
    m_drive_vCmd = 0.0;
    m_drive_vErr = 0.0;
    m_drive_vErrIntegrator = 0.0;
    m_drive_cmd = 0.0;

    drivecmtrl_angle = 0.0;
    drivecntrl_vel = 0.0;
}
/**
 * @brief Stop all motor functions and reset variables
 *
 */
void SwerveModule::Stop()
{
    m_drive->StopMotor();
    m_steer->StopMotor();
    LocalReset();
}
/**
 * @brief Set drive falcon cmd
 *
 * @param cmd -1 to 1 falcon cmd
 * @note cmd is negated as motor direction is wrong
 */
void SwerveModule::SetDriveCmd(double cmd)
{
    if (Constants::SWERVE_CALIBRATION_MODE)
    {
        cmd = 0.0;
    }

    // * NOTE cmd was negative on PoC swerve
    m_drive->Set(-cmd);
}
/**
 * @brief Set steer falcon cmd
 *
 * @param cmd -1 to 1 falcon cmd
 * @note cmd is negated as motor direction is wrong
 */
void SwerveModule::SetSteerCmd(double cmd)
{
    if (Constants::SWERVE_CALIBRATION_MODE)
    {
        cmd = 0.0;
    }
    // * NOTE cmd was negative on PoC swerve
    m_steer->Set(cmd);
}
/**
 * @brief Update dashboard with swerve module data
 *
 */
void SwerveModule::UpdateDash()
{
    std::string l_moduleName;
    l_moduleName = "Module" + std::to_string(m_moduleID) + "/";

    // frc::SmartDashboard::PutNumber(l_moduleName + "Drive raw position: ", m_drivePosition);
    frc::SmartDashboard::PutNumber("Swerve/" + l_moduleName + "DriveVelocity", m_driveVel);
    // frc::SmartDashboard::PutNumber(l_moduleName + "Drive position velocity (inches per second): ", m_drivePosVel);
    frc::SmartDashboard::PutNumber("Swerve/" + l_moduleName + "DriveTargetVelocity", drivecntrl_vel);
    frc::SmartDashboard::PutNumber("Swerve/" + l_moduleName + "DriveTargetAngle", drivecmtrl_angle);
    // frc::SmartDashboard::PutNumber(l_moduleName + "Steer motor raw position: ", m_steerMotorPosition);
    // frc::SmartDashboard::PutNumber(l_moduleName + "Steer encoder position: ", m_steerEncoderPosition);
    frc::SmartDashboard::PutNumber("Swerve/" + l_moduleName + "SteerRawPosition", m_steerEncoderRawPosition);
    frc::SmartDashboard::PutNumber("Swerve/" + l_moduleName + "SteerAdjPosition", m_steerEncoderPosition);
    frc::SmartDashboard::PutNumber("Swerve/" + l_moduleName + "DriveCMD", m_drive_cmd);


    if (Constants::SWERVE_SYS_ID && m_moduleID == 1)
    {
        printf("%f, %f, %f, ", m_steerEncoderPosition, m_steerEncoderVel, m_steerMotorVel);
        printf("%f, %f, ", m_drivePosition, m_driveVel);
        printf("%f, %f\n", m_steer_cmd, m_drive_cmd);
    }

    // loopcount, drive position, drive motor velocity, steer motor position, steer motor velocity, steer encoder absolute position, steer encoder position, steer encoder velocity, can pct utilization
    // printf("%f, %f, %f, %f, %f, %f[], %f, %f, %f\n",mainRobot->GetLoopCount(), m_drivePosition, m_driveMotorVel,m_steerMotorPosition, m_steerMotorVel,m_steerEncoderAbsolutePosition, m_steerEncoderPosition,m_steerEncoderVel,(frc::RobotController::GetCANStatus().percentBusUtilization*100));
}
/**
 * @brief Collect sensor states
 * @note Negated sensor values according to correct motor direction
 *
 */
void SwerveModule::Analyze()
{
    // * Note pos and vel was negative on PoC swerve
    m_drivePosition = -m_drive->GetSelectedSensorPosition();
    m_driveMotorVel = -m_drive->GetSelectedSensorVelocity();

    m_drivePosVel = (m_drivePosition - m_driveLastPosition) * GetPositionVelocityScaleFactor();
    m_driveVel = m_driveMotorVel * GetVelocityScaleFactor();
    m_driveLastPosition = m_drivePosition;

    m_steerMotorPosition = -m_steer->GetSelectedSensorPosition();
    m_steerMotorVel = -m_steer->GetSelectedSensorVelocity();

    // adjust abs sensor range from 0-360 to -180 to 180
    m_steerEncoderRawPosition = -m_steerEncoder->GetAbsolutePosition() + 180.0;

    m_steerEncoderPosition = MathUtil::Wrap(m_steerEncoderRawPosition - m_steerOffset);

    m_steerEncoderVel = -m_steerEncoder->GetVelocity();
}

/**
 * @brief Control swerve module
 *
 * @param vel velocity in inches per second
 * @param angle angle cmd from -180 to 180
 */
void SwerveModule::DriveControl(double vel, double angle)
{
    drivecmtrl_angle = angle;
    drivecntrl_vel = vel;
    m_steer_pErr = angle - m_steerEncoderPosition + 180.0 * m_rev;
    if (fabs(MathUtil::Wrap(m_steer_pErr)) > 90.0)
    {
        m_rev = !m_rev;
        m_steer_pErr += 180.0;
    }
    m_steer_pErr = MathUtil::Wrap(m_steer_pErr);
    m_steer_vCmd = STEER_kP * m_steer_pErr;
    m_steer_vErr = m_steer_vCmd - m_steerEncoderVel;
    m_steer_vErrIntegrator += m_steer_vErr * Constants::LOOPTIME;
    m_steer_vErrIntegrator = MathUtil::Limit(-STEER_kR / STEER_kV, STEER_kR / STEER_kV, m_steer_vErrIntegrator);
    m_steer_cmd = (STEER_kV / STEER_kR) * (m_steer_vErr * STEER_TAU + m_steer_vErrIntegrator);

    if (m_rev)
    {
        vel *= -1.0;
    }
    SetSteerCmd(m_steer_cmd);

    m_drive_vErr = vel - m_driveVel + kAV * m_steer_vCmd;
    m_drive_vErrIntegrator += m_drive_vErr * Constants::LOOPTIME;
    m_drive_vErrIntegrator = MathUtil::Limit(-DRIVE_kR / DRIVE_kV, DRIVE_kR / DRIVE_kV, m_drive_vErrIntegrator);
    m_drive_cmd = (DRIVE_kV / DRIVE_kR) * (m_drive_vErr * DRIVE_TAU + m_drive_vErrIntegrator);
    
    SetDriveCmd(m_drive_cmd);
}