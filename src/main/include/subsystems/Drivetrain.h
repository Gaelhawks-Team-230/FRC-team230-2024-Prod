#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

#include "Constants.h"
#include "util/Subsystem.h"
#include "util/MathUtil.h"
#include "util/control/CommandModel.h"
#include "subsystems/SwerveModule.h"
#include "auto\modes\AutoMode.h"

// Measured in inches, used in code as inches
// Point 1 coordinates:
const double P1X = (26.875);
const double P1Y = (27.375);
// Point 2 coordinates:
const double P2X = (2.625);
const double P2Y = (27.375);
// Point 3 coordinates:
const double P3X = (2.625);
const double P3Y = (2.625);
// Point 4 coordinates:
const double P4X = (26.875);
const double P4Y = (2.625);
// Center coordinates (from bottom left corner):
const double CX = (14.75);
const double CY = (15.0);
// Offset of theta for each module

// * Drive pinion on LEFT side
const double P1Offset = 55.723; //55.02
const double P2Offset = -105.996; //-83.1
const double P3Offset = 97.207; //35.771
const double P4Offset = -93.164; //-92.813
const double PStart = 0.0;

const double GYRO_MAX_VEL = 360.0; // degrees per second

const double R_kV = 5.0;
const double R_kR = 1.0;
const double R_TAU = 0.12;
const double R_FF = 0.0;

const double DEFAULT_ROBOT_ORIENTATION = 0.0;

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

    AutoMode *m_autoMode;

    double m_gyroReading;
    double m_gyroVel;
    double m_gyroLastReading;
    bool m_zeroVelDebugMode;
    bool m_gyroEnabled;

    double m_gyro_offset;
    double m_default_robot_orientation;

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


    bool m_isAutonomous;

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
    void GetChassisSpeeds(double &xdot, double &ydot, double &psidot)
    {
        xdot = m_targetXdot;
        ydot = m_targetYdot;
        psidot = m_targetPsidot;
    };
    void Periodic(void) override;
    void Analyze(void) override;

    void GyroReset(void);
    void SetGyroHeading(double p_offset) { m_default_robot_orientation = p_offset; };
    double GetGyroReading(void) { return m_gyroReading; };
    double GetGyroVel(void) { return m_gyroVel; };
    void SetZeroVelDebugModeOn() { m_zeroVelDebugMode = true; };
    void SetZeroVelDebugModeOff() { m_zeroVelDebugMode = false; };

    void SwerveKinematics(double xdot, double ydot, double psidot,
                          double &v1c, double &v2c, double &v3c, double &v4c,
                          double &t1c, double &t2c, double &t3c, double &t4c);
    //  TODO pass by reference not by pointer
    void ToFieldCoordinates(double *xdot, double *ydot);
    void EnableGyro() { m_gyroEnabled = true; };
    void DisableGyro() { m_gyroEnabled = false; };
    double GetGyroWrappedAngle() { return MathUtil::Wrap(m_gyroReading); };

    void SetIsAutonomous(bool isAuto){
        m_isAutonomous = isAuto;
    }
};
