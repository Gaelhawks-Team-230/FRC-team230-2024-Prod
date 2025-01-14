#pragma once

#include <stdio.h>
#include "Constants.h"
#include <cmath>
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "util/Subsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

const double TAU = 0.1;
const double KV = 10.0;
const double K = 6000.0;
const double ACCEL_LIMIT = 10000.0;
const double SHOOTING_THRESH = 10.0;
const double COUNTS2RPM = 10.0 * 60.0 / 2048.0;

const double SHOOTER_VELOCITY_MEAS_WINDOW = 32.0;

class Shooter : public Subsystem
{
public:
    static Shooter *GetInstance();
    void SetLeftShooterVel(double p_velocity) { m_leftGoalVel = p_velocity; };
    void SetRightShooterVel(double p_velocity) { m_rightGoalVel = p_velocity * -1.0; };
    double GetLeftShooterCurVel() { return m_leftCurVel; };
    double GetRightShooterCurVel() { return m_rightCurVel; };
    double GetRightShooterVelCmd() { return m_rightVelCmd; };
    double GetLeftShooterVelCmd() { return m_leftVelCmd; };

    bool IsShooterReady();
    void LocalReset() override;
    void Stop() override;
    void Analyze() override;
    void Periodic() override;
    void UpdateDash() override;
    void StartingConfig();

private:
    Shooter();

    static Shooter *m_shooter;

    double m_leftGoalVel;
    double m_leftVelCmd;
    double m_leftMotorCmd;
    double m_leftCurVel;
    double m_leftVelErr;
    double m_leftErrInt;

    double m_rightGoalVel;
    double m_rightVelCmd;
    double m_rightMotorCmd;
    double m_rightCurVel;
    double m_rightVelErr;
    double m_rightErrInt;

    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_leftShooter;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_rightShooter;
};
