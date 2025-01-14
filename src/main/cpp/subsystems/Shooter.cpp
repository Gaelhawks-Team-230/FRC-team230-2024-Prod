#include "subsystems/Shooter.h"
#include "Constants.h"
#include "util/MathUtil.h"

Shooter *Shooter::GetInstance()
{
    if (m_shooter == nullptr)
    {
        m_shooter = new Shooter();
    }
    return m_shooter;
}

/**
 * @brief Constructs a new Shooter::Shooter object
 *
 */
Shooter::Shooter()
{
    m_leftShooter = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(Constants::CAN::LEFT_SHOOTER_FALCON);
    m_rightShooter = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(Constants::CAN::RIGHT_SHOOTER_FALCON);

    StartingConfig();
    LocalReset();
}
/**
 * @brief Set falcons to starting configuration
 */
void Shooter::StartingConfig()
{
    if (Constants::CTRE_FACTORY_RESET)
    {
        printf("Shooter Factory Reset\n");
        m_leftShooter->ConfigFactoryDefault();
        m_rightShooter->ConfigFactoryDefault();
    }

    m_leftShooter->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_leftShooter->ConfigVelocityMeasurementWindow(SHOOTER_VELOCITY_MEAS_WINDOW);

    m_rightShooter->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_rightShooter->ConfigVelocityMeasurementWindow(SHOOTER_VELOCITY_MEAS_WINDOW);

    m_leftShooter->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    m_rightShooter->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

/**
 * @brief Resets all relevant member variables to a value of zero
 *
 */
void Shooter::LocalReset()
{
    m_leftVelCmd = 0.0;
    m_leftCurVel = 0.0;
    m_leftGoalVel = 0.0;
    m_leftVelErr = 0.0;
    m_leftErrInt = 0.0;

    m_rightVelCmd = 0.0;
    m_rightCurVel = 0.0;
    m_rightGoalVel = 0.0;
    m_rightVelErr = 0.0;
    m_rightErrInt = 0.0;

    m_leftMotorCmd = 0.0;
    m_rightMotorCmd = 0.0;
}

/**
 * @brief Reads velocity sensor data from the motors
 *
 */
void Shooter::Analyze()
{
    m_leftCurVel = m_leftShooter->GetSelectedSensorVelocity() * COUNTS2RPM;
    m_rightCurVel = m_rightShooter->GetSelectedSensorVelocity() * COUNTS2RPM;
}

/**
 * @brief This does the control system math for the acceleration and
 * then velocity control of both the left and right motors
 *
 */
void Shooter::Periodic()
{
    m_leftVelCmd += MathUtil::Limit(-ACCEL_LIMIT, ACCEL_LIMIT, (m_leftGoalVel - m_leftVelCmd));
    m_rightVelCmd += MathUtil::Limit(-ACCEL_LIMIT, ACCEL_LIMIT, (m_rightGoalVel - m_rightVelCmd));

    m_leftVelErr = m_leftVelCmd - m_leftCurVel;
    m_leftErrInt = MathUtil::Limit(-K / KV, K / KV, (m_leftErrInt + m_leftVelErr * Constants::LOOPTIME));
    m_leftMotorCmd = KV / K * (TAU * m_leftVelErr + m_leftErrInt);

    m_rightVelErr = m_rightVelCmd - m_rightCurVel;
    m_rightErrInt = MathUtil::Limit(-K / KV, K / KV, (m_rightErrInt + m_rightVelErr * Constants::LOOPTIME));
    m_rightMotorCmd = KV / K * (TAU * m_rightVelErr + m_rightErrInt);

    m_rightShooter->Set(m_rightMotorCmd);
    m_leftShooter->Set(m_leftMotorCmd);
}

/**
 * @brief Returns whether both motor velocities
 * are within the threshold for the velocity command
 *
 * @return true
 * @return false
 */
bool Shooter::IsShooterReady()
{
    // the threshold value is just a placeholder for now
    return (fabs(m_leftGoalVel - m_leftVelCmd) < SHOOTING_THRESH && fabs(m_rightGoalVel - m_rightVelCmd) < SHOOTING_THRESH);
}

/**
 * @brief Sends values to the dashboard
 *
 */
void Shooter::UpdateDash()
{
    frc::SmartDashboard::PutBoolean("Shooter/IsShooterReady", IsShooterReady());
    frc::SmartDashboard::PutNumber("Shooter/Left/VelGoal", m_leftGoalVel);
    frc::SmartDashboard::PutNumber("Shooter/Right/VelGoal", m_rightGoalVel);
    frc::SmartDashboard::PutNumber("Shooter/Left/CurVel", m_leftCurVel);
    frc::SmartDashboard::PutNumber("Shooter/Right/CurVel", m_rightCurVel);
    frc::SmartDashboard::PutNumber("Shooter/Left/VelCmd", m_leftVelCmd);
    frc::SmartDashboard::PutNumber("Shooter/Right/VelCmd", m_rightVelCmd);

    // printf("%f, %f, %f, %f\n", m_leftGoalVel, m_rightGoalVel, m_leftCurVel, m_rightCurVel);
}

/**
 * @brief Stops all shooter activity
 *
 */
void Shooter::Stop()
{
    m_leftShooter->StopMotor();
    m_rightShooter->StopMotor();
    LocalReset();
}

Shooter *Shooter::m_shooter = nullptr;