#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

#include "Constants.h"
#include "util/Subsystem.h"
#include "util/MathUtil.h"
#include "util/control/CommandModel.h"
#include "subsystems/SwerveModule.h"

// Measured in inches, used in code as inches
// Point 1 coordinates:
const double P1X = (25.375);
const double P1Y = (25.375);
// Point 2 coordinates:
const double P2X = (2.625);
const double P2Y = (25.375);
// Point 3 coordinates:
const double P3X = (2.625);
const double P3Y = (2.625);
// Point 4 coordinates:
const double P4X = (25.375);
const double P4Y = (2.625);
// Center coordinates (from bottom left corner):
const double CX = (14.0);
const double CY = (14.0);
// Offset of theta for each module
// old data offsets pre 3/37
//  m1 -36.4
//  m2 153.63
//  m3 -0.8788
//  m4 147.65
//  offsets for 3/27
//  m1 -36.73
//  m2 153.28
//  m3 -0.26
//  m4 -165.75

const double P1Offset = 23.29;
const double P2Offset = -15.29;
const double P3Offset = -73.21;
const double P4Offset = 167.61;
const double PStart = 0;

const double GYRO_MAX_VEL = 360.0; // degrees per second

const double R_kV = 5.0;
const double R_kR = 1.0;
const double R_TAU = 0.075;
const double R_FF = 0.0;

const double DEFAULT_ROBOT_ORIENTATION = 180.0;

class Drivetrain : public Subsystem
{

private:
    Drivetrain();

    static Drivetrain *m_drivetrain;

    SwerveModule *m_frontLeftModule;
    SwerveModule *m_frontRightModule;
    SwerveModule *m_backLeftModule;
    SwerveModule *m_backRightModule;

    frc::ADXRS450_Gyro *m_gyro;

    CommandModel *m_commandModel;

    double m_gyroReading;
    double m_gyroVel;
    double m_gyroLastReading;
    bool m_zeroVelDebugMode;
    bool m_gyroEnabled;

    double m_gyro_offset;

    // gyro stuff
    double m_r_vCmd;
    double m_r_vErr;
    double m_r_vErrIntegrator;
    double m_r_cmd;
    double m_r_ff;

    // array with goal swerve angles and velocities
    double m_swerveModuleGoalStates[8];
    // array with actual swerve angles and velocities
    double m_swerveModuleActualStates[8];

    double m_targetXdot;
    double m_targetYdot;
    double m_targetPsidot;

public:
    static Drivetrain *GetInstance();

    void LocalReset(void) override;
    void Stop(void) override;
    void UpdateDash(void) override;
    /**
     * @brief Control drivetrain
     *
     * @param xdot joystick x
     * @param ydot joystick y
     * @param psidot joystick z
     */
    void SetChassisSpeeds(double xdot, double ydot, double psidot)
    {
        m_targetXdot = xdot;
        m_targetYdot = ydot;
        m_targetPsidot = psidot;
    };
    void Periodic(void) override;
    void Analyze(void) override;

    void GyroReset(void);
    double GetGyroReading(void) { return m_gyroReading; };
    double GetGyroVel(void) { return m_gyroVel; };
    void SetZeroVelDebugModeOn() { m_zeroVelDebugMode = true; };
    void SetZeroVelDebugModeOff() { m_zeroVelDebugMode = false; };

    void SwerveKinematics(double xdot, double ydot, double psidot,
                          double &v1c, double &v2c, double &v3c, double &v4c,
                          double &t1c, double &t2c, double &t3c, double &t4c);

    void ToFieldCoordinates(double *xdot, double *ydot);
    void EnableGyro() { m_gyroEnabled = true; };
    void DisableGyro() { m_gyroEnabled = false; };
    double GetGyroWrappedAngle() { return MathUtil::Wrap(m_gyroReading); };
};