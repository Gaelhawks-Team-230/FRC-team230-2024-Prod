#include "subsystems/Arm.h"
#include "util/MathUtil.h"

using namespace std;

Arm *Arm::GetInstance()
{
    if (m_arm == nullptr)
    {
        m_arm = new Arm();
    }
    return m_arm;
}

Arm::Arm()
{
    // instance of arm and platform encoders
    m_armMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(Constants::CAN::ARM_FALCON);
    m_platformMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(Constants::CAN::PLATFORM_FALCON);

    // instance of absolute encoder (arm)
    armEncoderInput = new frc::DigitalInput(Constants::DigitalIO::ARM_ABSOLUTE_ENCODER);
    m_armEncoder = new frc::DutyCycle(armEncoderInput);

    // instance of absolute encoder (platform)
    platformEncoderInput = new frc::DigitalInput(Constants::DigitalIO::PLATFORM_ABSOLUTE_ENCODER);
    m_platformEncoder = new frc::DutyCycle(platformEncoderInput);

    StartingConfig();
    LocalReset();
}
/**
 * @brief Resets a few control system variables to 0.0
 *
 */
void Arm::LocalReset()
{
    // Arm control vars
    m_armVelErrInt = 0.0;

    // Wrist control vars
    m_platformVelErrInt = 0.0;

    // tells the arm and platform to stay where it is upon startup
    m_rawArmTheta = 360 * (m_armEncoder->GetOutput() - Constants::DMIN) / (Constants::DMAX - Constants::DMIN);
    m_armPos = m_rawArmTheta - ARM_OFFSET;
    m_armPosCmd = m_armPos;
    m_armPosCmdz = m_armPosCmd;
    commandRate = 0.0;
    m_armPosGoal = -80.0;

    m_rawPlatformTheta = 360 * (m_platformEncoder->GetOutput() - Constants::DMIN) / (Constants::DMAX - Constants::DMIN);
    m_platformPos = -(m_rawPlatformTheta - PLATFORM_OFFSET);
    m_platformPosCmd = m_platformPos;
    m_platformPosGoal = 0.0;
}

void Arm::StartingConfig()
{
    if (Constants::CTRE_FACTORY_RESET)
    {
        printf("Arm and Platform Motors Factory Reset\n");
        m_armMotor->ConfigFactoryDefault();
        m_platformMotor->ConfigFactoryDefault();
    }

    // sets the motors in brake mode
    m_armMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_platformMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    // configures velocity for arm motor
    m_armMotor->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_armMotor->ConfigVelocityMeasurementWindow(ARM_VELOCITY_MEAS_WINDOW);

    // configures velocity for platform motor
    m_platformMotor->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_platformMotor->ConfigVelocityMeasurementWindow(PLATFORM_VELOCITY_MEAS_WINDOW);

    // set deadband for motors to 0.001
    m_armMotor->ConfigNeutralDeadband(ARM_DEADBAND);
    m_platformMotor->ConfigNeutralDeadband(PLATFORM_DEADBAND);

    // configures current limit for motors
    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currLimitConfig;
    currLimitConfig.enable = true;
    currLimitConfig.currentLimit = 150.0;
    m_platformMotor->ConfigStatorCurrentLimit(currLimitConfig);

    currLimitConfig.currentLimit = 80.0;
    m_armMotor->ConfigStatorCurrentLimit(currLimitConfig);
}

void Arm::SetPosition(double p_desiredArmPos, double p_desiredPlatformPos)
{
    m_armPosGoal = p_desiredArmPos;
    m_platformPosGoal = p_desiredPlatformPos;
}

/**
 * @brief Does math for the arm and platform calculations
 *
 */
void Arm::Analyze()
{
    // FOR ARM CALCULATIONS
    m_rawArmTheta = 360 * (m_armEncoder->GetOutput() - Constants::DMIN) / (Constants::DMAX - Constants::DMIN);

    m_armPos = m_rawArmTheta - ARM_OFFSET; // because the encoder reads positive in the forward direction
    m_armPos = MathUtil::Wrap(m_armPos);

    // positive because motor is "forwards"
    m_armVel = (m_armMotor->GetSelectedSensorVelocity()) * ARM_MOTOR_VEL_SF * -1.0;

    // FOR PLATFORM CALCULATIONS
    m_rawPlatformTheta = 360 * (m_platformEncoder->GetOutput() - Constants::DMIN) / (Constants::DMAX - Constants::DMIN);

    m_platformPos = -(m_rawPlatformTheta - PLATFORM_OFFSET); // because the encoder reads negative in the forward direction
    m_platformPos = MathUtil::Wrap(m_platformPos);

    // Make it negative because motor is "backwards"
    m_platformVel = (m_platformMotor->GetSelectedSensorVelocity()) * PLATFORM_MOTOR_VEL_SF * -1.0;
}

/**
 * @brief Control system for the arm
 *
 */
void Arm::ArmControl()
{
    commandRate = (m_armPosCmd - m_armPosCmdz) / Constants::LOOPTIME;
    m_armPosCmdz = m_armPosCmd;
    m_armPosErr = m_armPosCmd - m_armPos;
    // kFF is called K feed forward, which just gives the control system a push. We needed this because the chain has slack on it
    m_armVelCmd = m_armPosErr * ARM_K_POS + kFF * commandRate;
    m_armVelErr = m_armVelCmd - m_armVel;
    m_armVelErrInt = m_armVelErrInt + m_armVelErr * Constants::LOOPTIME;

    double limit = (ARM_K_ROBOT / ARM_K_VEL) * 0.5;
    m_armVelErrInt = MathUtil::Limit(limit * -1.0, limit, m_armVelErrInt);
    m_armMotorCmd = ARM_K_VEL / ARM_K_ROBOT * (ARM_TAU_ROBOT * m_armVelErr + m_armVelErrInt);

    m_armMotorCmd = MathUtil::Limit(-0.5, 0.5, m_armMotorCmd); // Safety net
    m_armMotor->Set(m_armMotorCmd * -1.0);
}

/**
 * @brief Control system for the platform
 *
 */
void Arm::PlatformControl()
{
    commandRate = (m_platformPosCmd - m_platformPosCmdz) / Constants::LOOPTIME;
    m_platformPosCmdz = m_platformPosCmd;

    m_platformPosErr = m_platformPosCmd - m_platformPos;
    // kFF is called K feed forward, which just gives the control system a push. We needed this because the chain has slack on it
    m_platformVelCmd = m_platformPosErr * PLATFORM_K_POS + PLATFORM_kFF * commandRate;
    m_platformVelErr = m_platformVelCmd - m_platformVel;
    m_platformVelErrInt = m_platformVelErrInt + m_platformVelErr * Constants::LOOPTIME;

    double limit = (PLATFORM_K_ROBOT / PLATFORM_K_VEL) * 0.5;
    m_platformVelErrInt = MathUtil::Limit(limit * -1.0, limit, m_platformVelErrInt);
    m_platformMotorCmd = PLATFORM_K_VEL / PLATFORM_K_ROBOT * (PLATFORM_TAU_ROBOT * m_platformVelErr + m_platformVelErrInt);

    m_platformMotorCmd = MathUtil::Limit(-0.5, 0.5, m_platformMotorCmd); // Safety net
    m_platformMotor->Set(m_platformMotorCmd * -1.0);
}

/**
 * @brief This is going to get called every 20 ms in the robot.cpp class
 *
 */
void Arm::Periodic()
{
    // calculates the updated position of the arm

    // sets/limits the values that are calculated form the Update method
    double limitArm = ARM_MAX_RATE * Constants::LOOPTIME;
    m_armPosCmd = m_armPosCmd + MathUtil::Limit(limitArm * -1.0, limitArm, m_armPosGoal - m_armPosCmd);

    double limitPlatform = PLATFORM_MAX_RATE * Constants::LOOPTIME;
    m_platformPosCmd = m_platformPosCmd + MathUtil::Limit(limitPlatform * -1.0, limitPlatform, m_platformPosGoal - m_platformPosCmd);

    // call arm control and platform control
    ArmControl();
    PlatformControl();
}

void Arm::Stop()
{
    m_armMotor->StopMotor();
    m_platformMotor->StopMotor();
}

void Arm::UpdateDash()
{
    // arm pos and pos cmd
    frc::SmartDashboard::PutNumber("Arm/ArmCurrentPosition", m_armPos);
    frc::SmartDashboard::PutNumber("Arm/ArmCommand", m_armPosCmd);

    // platform pos and pos cmd
    frc::SmartDashboard::PutNumber("Arm/PlatformCurrentPosition", m_platformPos);
    frc::SmartDashboard::PutNumber("Arm/PlatformCommand", m_platformPosCmd);

    // arm vel and vel cmd
    frc::SmartDashboard::PutNumber("Arm/ArmVelocity", m_armVel);
    frc::SmartDashboard::PutNumber("Arm/ArmVelocityCommand", m_armVelCmd);

    // platform vel and vel cmd
    frc::SmartDashboard::PutNumber("Arm/PlatformVelocity", m_platformVel);
    frc::SmartDashboard::PutNumber("Arm/PlatformVelocityCommand", m_platformVelCmd);

    // arm and platform theta
    frc::SmartDashboard::PutNumber("Arm/RawArmTheta", m_rawArmTheta);
    frc::SmartDashboard::PutNumber("Arm/RawPlatformTheta", m_rawPlatformTheta);
}
Arm *Arm::m_arm = nullptr;
