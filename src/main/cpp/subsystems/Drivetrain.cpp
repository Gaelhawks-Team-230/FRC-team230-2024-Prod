#include "subsystems/Drivetrain.h"

/**
 * @brief Gets the instance of the drivetrain
 *
 * @return Drivetrain*
 */
Drivetrain *Drivetrain::GetInstance()
{
    if (m_drivetrain == nullptr)
    {
        m_drivetrain = new Drivetrain();
    }
    return m_drivetrain;
}

/**
 * @brief Construct a new Drivetrain object
 *
 */
Drivetrain::Drivetrain()
{
    m_frontLeftModule = new SwerveModule(Constants::CAN::FRONT_LEFT_STEER, Constants::CAN::FRONT_LEFT_DRIVE, Constants::CAN::FRONT_LEFT_ABSOLUTE_ENCODER, 4, P4Offset);
    m_frontRightModule = new SwerveModule(Constants::CAN::FRONT_RIGHT_STEER, Constants::CAN::FRONT_RIGHT_DRIVE, Constants::CAN::FRONT_RIGHT_ABSOLUTE_ENCODER, 1, P1Offset);
    m_backLeftModule = new SwerveModule(Constants::CAN::BACK_LEFT_STEER, Constants::CAN::BACK_LEFT_DRIVE, Constants::CAN::BACK_LEFT_ABSOLUTE_ENCODER, 3, P3Offset);
    m_backRightModule = new SwerveModule(Constants::CAN::BACK_RIGHT_STEER, Constants::CAN::BACK_RIGHT_DRIVE, Constants::CAN::BACK_RIGHT_ABSOLUTE_ENCODER, 2, P2Offset);

    m_commandModel = new CommandModel(5.0, 5.0, 5.0);

    m_gyro = new frc::ADXRS450_Gyro();
    m_gyro->Calibrate();

    LocalReset();
}
/**
 * @brief Reset variables and swerve submodules
 *
 */
void Drivetrain::LocalReset()
{
    m_zeroVelDebugMode = false;

    m_frontLeftModule->LocalReset();
    m_frontRightModule->LocalReset();
    m_backLeftModule->LocalReset();
    m_backRightModule->LocalReset();

    m_gyroEnabled = false;

    m_r_vCmd = 0.0;
    m_r_vErr = 0.0;
    m_r_vErrIntegrator = 0.0;
    m_r_cmd = 0.0;
    m_gyroVel = 0.0;
    m_r_ff = 0.0;

    m_gyro_offset = DEFAULT_ROBOT_ORIENTATION;

    for (int i = 0; i < 8; i++)
    {
        m_swerveModuleGoalStates[i] = 0.0;
        m_swerveModuleActualStates[i] = 0.0;
    }

    m_targetXdot = 0.0;
    m_targetYdot = 0.0;
    m_targetPsidot = 0.0;
}
/**
 * @brief Computes swerve velocity and rotational commands
 *
 * @param xdot forward velocity from joystick
 * @param ydot right velocity from joystick
 * @param psidot rotation rate around z from joystick. coming in as degrees, converted to radians.
 *
 * @param v1c velocity command for 1st module in inches/sec
 * @param v2c velocity command for 2nd module in inches/sec
 * @param v3c velocity command for 3rd module in inches/sec
 * @param v4c velocity command for 4th module in inches/sec
 *
 * @param t1c rotational command for 1st module in deg/sec
 * @param t2c rotational command for 2nd module in deg/sec
 * @param t3c rotational command for 3rd module in deg/sec
 * @param t4c rotational command for 4th module in deg/sec
 */
void Drivetrain::SwerveKinematics(double xdot, double ydot, double psidot,
                                  double &v1c, double &v2c, double &v3c, double &v4c,
                                  double &t1c, double &t2c, double &t3c, double &t4c)
{
    double l_v1x, l_v1y, l_v2x, l_v2y, l_v3x, l_v3y, l_v4x, l_v4y;
    psidot = (psidot * M_PI / 180);

    l_v1x = xdot - (psidot * (P1Y - CY));
    l_v1y = ydot + (psidot * (P1X - CX));

    l_v2x = xdot - (psidot * (P2Y - CY));
    l_v2y = ydot + (psidot * (P2X - CX));

    l_v3x = xdot - (psidot * (P3Y - CY));
    l_v3y = ydot + (psidot * (P3X - CX));

    l_v4x = xdot - (psidot * (P4Y - CY));
    l_v4y = ydot + (psidot * (P4X - CX));

    v1c = sqrt((l_v1x * l_v1x) + (l_v1y * l_v1y));
    v2c = sqrt((l_v2x * l_v2x) + (l_v2y * l_v2y));
    v3c = sqrt((l_v3x * l_v3x) + (l_v3y * l_v3y));
    v4c = sqrt((l_v4x * l_v4x) + (l_v4y * l_v4y));

    t1c = (180 / M_PI) * atan2(l_v1y, l_v1x);
    t2c = (180 / M_PI) * atan2(l_v2y, l_v2x);
    t3c = (180 / M_PI) * atan2(l_v3y, l_v3x);
    t4c = (180 / M_PI) * atan2(l_v4y, l_v4x);

    //   if (v1y==0 && v1x==0){
    //       t1c = P1Offset - PStart;
    //   }
    //    if (v2y==0 && v2x==0){
    //       t2c = P2Offset + PStart;
    //   }
    //    if (v3y==0 && v3x==0){
    //       t3c = P3Offset - PStart;
    //   }
    //    if (v4y==0 && v4x==0){
    //       t4c = P4Offset + PStart;
    //   }

    t1c = MathUtil::Wrap(t1c);
    t2c = MathUtil::Wrap(t2c);
    t3c = MathUtil::Wrap(t3c);
    t4c = MathUtil::Wrap(t4c);
}
/**
 * @brief Read sensor values
 *
 */
void Drivetrain::Analyze()
{
    m_frontLeftModule->Analyze();
    m_frontRightModule->Analyze();
    m_backRightModule->Analyze();
    m_backLeftModule->Analyze();

    m_gyroReading = m_gyro->GetAngle();
    m_gyroVel = (m_gyroReading - m_gyroLastReading) / Constants::LOOPTIME;
    m_gyroVel = MathUtil::Limit(-GYRO_MAX_VEL, GYRO_MAX_VEL, m_gyroVel);
    m_gyroLastReading = m_gyroReading;

    if (Constants::SWERVE_SYS_ID)
    {
        printf("%f, %f, ", m_gyroReading, m_gyroVel);
    }
}
/**
 * @brief Reset gyro zero
 *
 */
void Drivetrain::GyroReset()
{
    m_gyro->Reset();
    m_gyro_offset = 0.0;
    m_gyroReading = m_gyro->GetAngle();
    m_gyroVel = 0.0;
    m_gyroLastReading = m_gyroReading;
}
/**
 * @brief Runs control loop
 *
 */
void Drivetrain::Periodic()
{
    // gyro heading loop
    m_gyroVel = GetGyroVel();
    m_r_ff = m_targetPsidot;
    m_r_vErr = m_targetPsidot - m_gyroVel;
    m_r_vErrIntegrator += m_r_vErr * Constants::LOOPTIME;
    double l_bounds;
    l_bounds = (Constants::Swerve::R_SFACTOR * R_kR) / R_kV;
    m_r_vErrIntegrator = MathUtil::Limit(-l_bounds, l_bounds, m_r_vErrIntegrator);
    m_r_cmd = (R_kV / R_kR) * (R_TAU * m_r_vErr + m_r_vErrIntegrator) + (R_FF * m_r_ff);

    if (!m_gyroEnabled)
    {
        m_r_cmd = m_targetPsidot;
        m_r_vErrIntegrator = 0.0;
    }

    double v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c;
    double xdotFil, ydotFil, psidotFil;
    m_commandModel->Calculate(m_targetXdot, m_targetYdot, m_r_cmd, &xdotFil, &ydotFil, &psidotFil);
    SwerveKinematics(xdotFil, ydotFil, psidotFil, v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c);

    if (m_zeroVelDebugMode)
    {
        v1c = 0.0;
        v2c = 0.0;
        v3c = 0.0;
        v4c = 0.0;
    }

    m_frontRightModule->DriveControl(v1c, t1c);
    m_backRightModule->DriveControl(v2c, t2c);
    m_backLeftModule->DriveControl(v3c, t3c);
    m_frontLeftModule->DriveControl(v4c, t4c);

    // setting goal swerve module velocities and angles for each module into an array
    m_swerveModuleGoalStates[0] = t1c;
    m_swerveModuleGoalStates[1] = v1c;
    m_swerveModuleGoalStates[2] = t2c;
    m_swerveModuleGoalStates[3] = v2c;
    m_swerveModuleGoalStates[4] = t3c;
    m_swerveModuleGoalStates[5] = v3c;
    m_swerveModuleGoalStates[6] = t4c;
    m_swerveModuleGoalStates[7] = v4c;

    m_targetXdot = 0.0;
    m_targetYdot = 0.0;
    m_targetPsidot = 0.0;
}

/**
 * @brief Convert robot centric coordinates to field centric coordinates
 *
 * @param *xdot pointer to joystick xdot
 * @param *ydot pointer to joystick ydot
 */
void Drivetrain::ToFieldCoordinates(double *xdot, double *ydot)
{

    double l_vNorth, l_vEast, l_theta;
    l_vNorth = *xdot;
    l_vEast = *ydot;
    // field coordinates conversion
    l_theta = (GetGyroReading() + DEFAULT_ROBOT_ORIENTATION) * (M_PI / 180.0); // added 180 deg offset to orient field coords in correct direction
    l_theta = l_theta + GetGyroVel() * (M_PI / 180.0) * 0.15;
    *xdot = cos(l_theta) * l_vNorth + sin(l_theta) * l_vEast;
    *ydot = -sin(l_theta) * l_vNorth + cos(l_theta) * l_vEast;
}

/**
 * @brief Update SmartDashboard
 *
 */
void Drivetrain::UpdateDash()
{
    // setting actual swerve module velocities and angles for each module into an array
    m_swerveModuleActualStates[0] = m_frontRightModule->GetSteerEncoderAbsolutePosition();
    m_swerveModuleActualStates[1] = m_frontRightModule->GetDriveVel();
    m_swerveModuleActualStates[2] = m_backRightModule->GetSteerEncoderAbsolutePosition();
    m_swerveModuleActualStates[3] = m_backRightModule->GetDriveVel();
    m_swerveModuleActualStates[4] = m_backLeftModule->GetSteerEncoderAbsolutePosition();
    m_swerveModuleActualStates[5] = m_backLeftModule->GetDriveVel();
    m_swerveModuleActualStates[6] = m_frontLeftModule->GetSteerEncoderAbsolutePosition();
    m_swerveModuleActualStates[7] = m_frontLeftModule->GetDriveVel();

    m_frontRightModule->UpdateDash();
    m_frontLeftModule->UpdateDash();
    m_backLeftModule->UpdateDash();
    m_backRightModule->UpdateDash();

    frc::SmartDashboard::PutNumber("Swerve/GyroWrappedAngle", m_gyroReading);
    frc::SmartDashboard::PutNumber("Swerve/GyroVel", m_gyroVel);
    frc::SmartDashboard::PutNumberArray("Swerve/SwerveModuleGoalStates", m_swerveModuleGoalStates);
    frc::SmartDashboard::PutNumberArray("Swerve/SwerveModuleActualStates", m_swerveModuleActualStates);
}
/**
 * @brief Stop submodules
 *
 */
void Drivetrain::Stop()
{
    m_frontLeftModule->Stop();
    m_frontRightModule->Stop();
    m_backLeftModule->Stop();
    m_backRightModule->Stop();
    LocalReset();
}

Drivetrain *Drivetrain::m_drivetrain = nullptr;