#include "subsystems/Climber.h"

/**
 * @brief get instance of the climber
 *
 * @return Climber*
 */
Climber *Climber::GetInstance()
{
    if (m_climber == nullptr)
    {
        m_climber = new Climber();
    }
    return m_climber;
}
/**
 * @brief create a new Climber object
 *
 */
Climber::Climber()
{
    m_leftMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(Constants::CAN::LEFT_CLIMBER_FALCON);
    m_rightMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(Constants::CAN::RIGHT_CLIMBER_FALCON);

    LocalReset();
    StartingConfig();
}
/**
 * @brief Reset member variables
 *
 */
void Climber::LocalReset()
{
    m_leftPos = 0.0;
    m_left_pCmd = 0.0;
    m_left_pErr = 0.0;
    m_leftVel = 0.0;
    m_left_vErr = 0.0;
    m_left_vErrIntegrator = 0.0;
    m_left_vCmd = 0.0;

    m_rightPos = 0.0;
    m_right_pCmd = 0.0;
    m_right_pErr = 0.0;
    m_rightVel = 0.0;
    m_right_vErr = 0.0;
    m_right_vErrIntegrator = 0.0;
    m_right_vCmd = 0.0;

    m_left_mCmd = 0.0;
    m_right_mCmd = 0.0;

    m_leftGoalVel = 0.0;
}

void Climber::StartingConfig()
{
    if (Constants::CTRE_FACTORY_RESET)
    {
        printf("Climber Factory Reset\n");
        m_rightMotor->ConfigFactoryDefault();
        m_leftMotor->ConfigFactoryDefault();
    }

    m_leftMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_rightMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_leftMotor->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_leftMotor->ConfigVelocityMeasurementWindow(CLIMBER_VEL_MEAS_WINDOW);

    m_rightMotor->ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms);
    m_rightMotor->ConfigVelocityMeasurementWindow(CLIMBER_VEL_MEAS_WINDOW);

    m_leftMotor->ConfigNeutralDeadband(CLIMBER_DEADBAND);
    m_rightMotor->ConfigNeutralDeadband(CLIMBER_DEADBAND);

    m_leftMotor->SetSelectedSensorPosition(0.0);
    m_rightMotor->SetSelectedSensorPosition(0.0);

    SetLeftCurrLimit(15.0);
    SetRightCurrLimit(15.0);

    m_leftCalstate = 0;
    m_isLeftCalDone = false;

    m_rightCalstate = 0;
    m_isRightCalDone = false;
}

/**
 * @brief Sets the current limit for the right motor
 *
 * @param curr current in amps
 */
void Climber::SetLeftCurrLimit(double curr)
{
    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currLimitConfig;
    currLimitConfig.enable = true;
    currLimitConfig.currentLimit = curr;
    m_leftMotor->ConfigStatorCurrentLimit(currLimitConfig);
}

/**
 * @brief Sets the current limit for the right motor
 *
 * @param curr current in amps
 */
void Climber::SetRightCurrLimit(double curr)
{
    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currLimitConfig;
    currLimitConfig.enable = true;
    currLimitConfig.currentLimit = curr;
    m_rightMotor->ConfigStatorCurrentLimit(currLimitConfig);
}

/**
 * @brief Read position and velocity of motors
 *
 */
void Climber::Analyze()
{
    m_leftPos = m_leftMotor->GetSelectedSensorPosition() * CLIMBER_SCALE_FACTOR;
    m_leftVel = m_leftMotor->GetSelectedSensorVelocity() * CLIMBER_SCALE_FACTOR * REVS_PER_SECOND;

    m_rightPos = -1.0 * m_rightMotor->GetSelectedSensorPosition() * CLIMBER_SCALE_FACTOR;
    m_rightVel = -1.0 * m_rightMotor->GetSelectedSensorVelocity() * CLIMBER_SCALE_FACTOR * REVS_PER_SECOND;
}
/**
 * @brief Run control loop
 *
 */
void Climber::Periodic()
{
    if (!m_isLeftCalDone)
    {
        LeftClimbCalibrate();
    }

    if (!m_isRightCalDone)
    {
        RightClimbCalibrate();
    }

    m_leftGoalVel = MathUtil::Limit(-MAX_VELOCITY, MAX_VELOCITY, m_leftGoalVel);
    m_rightGoalVel = MathUtil::Limit(-MAX_VELOCITY, MAX_VELOCITY, m_rightGoalVel);

    if (m_isLeftCalDone && m_isRightCalDone)
    {
        if (m_leftGoalVel > 0.01 && m_leftPos > HOOKS_UP_POS)
        {
            m_leftGoalVel = 0.0;
        }
        if (m_rightGoalVel > 0.01 && m_rightPos > HOOKS_UP_POS)
        {
            m_rightGoalVel = 0.0;
        }
        if (m_leftGoalVel < -0.01 && m_leftPos < HOOKS_DOWN_POS)
        {
            m_leftGoalVel = 0.0;
        }
        if (m_rightGoalVel < -0.01 && m_rightPos < HOOKS_DOWN_POS)
        {
            m_rightGoalVel = 0.0;
        }
    }

    m_left_vCmd = m_leftGoalVel;
    m_right_vCmd = m_rightGoalVel;

    m_left_vErr = m_left_vCmd - m_leftVel;
    m_right_vErr = m_rightVel - m_right_vCmd;

    m_left_vErrIntegrator += m_left_vErr * Constants::LOOPTIME;
    m_right_vErrIntegrator += m_right_vErr * Constants::LOOPTIME;

    m_left_vErrIntegrator = MathUtil::Limit((-CLIMBER_K / CLIMBER_KV) * 0.35, (CLIMBER_K / CLIMBER_KV) * 0.35, m_left_vErrIntegrator);
    m_right_vErrIntegrator = MathUtil::Limit((-CLIMBER_K / CLIMBER_KV) * 0.35, (CLIMBER_K / CLIMBER_KV) * 0.35, m_right_vErrIntegrator);

    m_left_mCmd = (CLIMBER_KV / CLIMBER_K) * (CLIMBER_TAU * m_left_vErr + m_left_vErrIntegrator);
    m_right_mCmd = (CLIMBER_KV / CLIMBER_K) * (CLIMBER_TAU * m_right_vErr + m_right_vErrIntegrator);

    // printf("%f, %f\n", m_left_vCmd, m_leftVel);

    m_leftMotor->Set(m_left_mCmd);

    m_rightMotor->Set(m_right_mCmd);

    // m_leftMotor->Set(0.0);
    // m_rightMotor->Set(0.0);
}

/**
 * @brief Calibrate the left climber
 *
 */
void Climber::LeftClimbCalibrate()
{
    switch (m_leftCalstate)
    {
    case 0:
        SetLeftCurrLimit(11.0);
        m_leftCalstate++;
        break;
    case 1:
        m_leftGoalVel = V_CAL;
        if ((m_leftMotor->GetStatorCurrent() > 10.0))
        {
            m_leftCalstate++;
        }
        break;
    case 2:
        m_leftGoalVel = 0.0;
        m_leftMotor->SetSelectedSensorPosition(INIT_POS * (1.0 / CLIMBER_SCALE_FACTOR));
        m_leftCalstate++;
        break;
    case 3:
        if (m_leftPos > MOVING_POS)
        {
            SetLeftCurrLimit(60.0);
            m_leftGoalVel = 0.0;
            m_isLeftCalDone = true;
            m_leftCalstate = 0;
            printf("Left Climber Calibration Complete\n");
            break;
        }
        m_leftGoalVel = END_CAL_VELOCITY;
        break;
    }
}

/**
 * @brief Calibrate the right climber
 *
 */
void Climber::RightClimbCalibrate()
{
    switch (m_rightCalstate)
    {
    case 0:
        SetRightCurrLimit(11.0);
        m_rightCalstate++;
        break;
    case 1:
        m_rightGoalVel = V_CAL;
        if ((m_rightMotor->GetStatorCurrent() > 10.0))
        {
            m_rightCalstate++;
        }
        break;
    case 2:
        m_rightGoalVel = 0.0;
        m_rightMotor->SetSelectedSensorPosition(-1.0 * INIT_POS * (1.0 / CLIMBER_SCALE_FACTOR));
        m_rightCalstate++;
        break;
    case 3:
        if (m_rightPos > MOVING_POS)
        {
            m_rightGoalVel = 0.0;
            m_isRightCalDone = true;
            m_rightCalstate = 0;
            SetRightCurrLimit(60.0);
            printf("Right Climber Calibration Complete\n");
            break;
        }
        m_rightGoalVel = END_CAL_VELOCITY;
        break;
    }
}

/**
 * @brief Update smart dashboard
 *
 */
void Climber::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Climber/LeftPosition", m_leftPos);
    frc::SmartDashboard::PutNumber("Climber/LeftVelocity", m_leftVel);
    frc::SmartDashboard::PutBoolean("Climber/IsLeftCalDone", m_isLeftCalDone);
    frc::SmartDashboard::PutNumber("Climber/LeftGoalVel", m_leftGoalVel);
    frc::SmartDashboard::PutNumber("Climber/LeftCurrent", m_leftMotor->GetStatorCurrent());

    frc::SmartDashboard::PutNumber("Climber/RightPosition", m_rightPos);
    frc::SmartDashboard::PutNumber("Climber/RightVelocity", m_rightVel);
    frc::SmartDashboard::PutBoolean("Climber/IsRightCalDone", m_isRightCalDone);
    frc::SmartDashboard::PutNumber("Climber/RightGoalVel", m_rightGoalVel);
    frc::SmartDashboard::PutNumber("Climber/RightCurrent", m_rightMotor->GetStatorCurrent());
}
/**
 * @brief Stop motors
 *
 */
void Climber::Stop()
{
    m_leftMotor->StopMotor();
    m_rightMotor->StopMotor();
    LocalReset();
}

Climber *Climber::m_climber = nullptr;