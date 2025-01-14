#pragma once

#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycle.h>

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/CANCoder.h"

#include <cmath>

#include "Constants.h"
#include "util/MathUtil.h"

const double STEER_kP = 8.0;
const double STEER_kV = 12.0;
const double STEER_kR = 1800.0;
const double STEER_TAU = 0.04;

const double DRIVE_kV = 10.0;
const double DRIVE_kR = 200.0;
const double DRIVE_TAU = 0.05;
const double kAV = 0.0185;

const double COUNTS_100MS = 1.0 / 100.0;
const double MILLISECONDS_SEC = 100.0 / 0.1;
const double REV_COUNTS = 1.0 / 2048.0;
const double RAD_REV = 2.0 * M_PI;
const double WHEEL_CIRCUMFERENCE = 1.95;
const double GEAR_RATIO = 1.0 / 6.75;

const double DRIVE_VELOCITY_MEAS_WINDOW = 32.0;
const double STEER_VELOCITY_MEAS_WINDOW = 16.0;

const double DRIVE_DEADBAND = 0.001;
const double STEER_DEADBAND = 0.001;

const bool DRIVE_CURRENT_LIMIT = false;
const double DRIVE_STATOR_CURRENT_LIMIT = 40.0;

class SwerveModule
{

public:
    SwerveModule(unsigned int steerFalconID, unsigned int driveFalconID, unsigned int absoluteEncoderID, unsigned int swerveID, double steerOffset);
    void LocalReset();
    void UpdateDash();
    void Analyze();
    void Stop();
    void StartingConfig();

    void SetDriveCmd(double cmd);
    void SetSteerCmd(double cmd);
    
    void DriveControl(double vel, double angle);

    double GetDriveVel() { return m_driveVel; };
    double GetSteerEncoderVel() { return m_steerEncoderVel; };
    double GetSteerEncoderAbsolutePosition() { return m_steerEncoderPosition; };
    double GetVelocityScaleFactor() { return COUNTS_100MS * MILLISECONDS_SEC * REV_COUNTS * RAD_REV * WHEEL_CIRCUMFERENCE * GEAR_RATIO; };
    double GetPositionVelocityScaleFactor() { return (1.0 / Constants::LOOPTIME) * REV_COUNTS * RAD_REV * WHEEL_CIRCUMFERENCE * GEAR_RATIO; };

private:
    unsigned int m_moduleID;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_steer;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_drive;
    ctre::phoenix::sensors::CANCoder *m_steerEncoder;

    double m_driveLastPosition;
    double m_driveCurrentPosition;
    double m_driveRaw;
    double m_driveOffset;
    double m_drivePosition;
    double m_driveVel;
    double m_driveMotorVel;
    double m_drivePosVel;

    double m_steerMotorPosition;
    double m_steerEncoderRawPosition;
    double m_steerMotorVel;
    double m_steerEncoderPosition;
    double m_steerEncoderVel;
    double m_steerOffset;

    double m_steer_pErr;
    double m_steer_vCmd;
    double m_steer_vErr;
    double m_steer_vErrIntegrator;
    double m_steer_cmd;

    bool m_rev;

    double m_drive_pErr;
    double m_drive_vCmd;
    double m_drive_vErr;
    double m_drive_vErrIntegrator;
    double m_drive_cmd;

    double drivecmtrl_angle;
    double drivecntrl_vel;
};
