#pragma once

#include "frc/DigitalInput.h"
#include <frc/DutyCycle.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix.h"
#include "Constants.h"
#include "util/Subsystem.h"

// control laws - arm
const double ARM_K_POS = 3.0;
const double ARM_K_VEL = 6.0;
const double kFF = 0.5;
const double ARM_TAU_ROBOT = 0.05;
const double ARM_K_ROBOT = 250.0;
const double ARM_DEADBAND = 0.001;

// degrees per second
// TODO: THIS WILL NEED TO BE CHANGED BEFORE COMPETITION.
const double ARM_MAX_RATE = 200.0;


// 22 to 72 gear ratio and 36 to 1 gear box
// converts from counts per 100 ms at motor to degrees per 1 second at the arm
const double ARM_MOTOR_VEL_SF = ((22.0 / 72.0) / 36.0) * (360.0 * 10.0 / 2048.0);
// ! Arm and Platform offsets MUST be zero to calibrate
const double ARM_OFFSET = 249.01;

// control laws - platform
const double PLATFORM_K_POS = 5.0;
const double PLATFORM_K_VEL = 10.0;
const double PLATFORM_kFF = 0.5;
const double PLATFORM_TAU_ROBOT = 0.03;
const double PLATFORM_K_ROBOT = 310.0;
const double PLATFORM_DEADBAND = 0.001;

// degrees per second
// TODO: THIS WILL NEED TO BE CHANGED BEFORE COMPETITION.
const double PLATFORM_MAX_RATE = 200.0;

// 36 to 1 gear box and 72 to 22 gear ratio
// converts from counts per 100 ms at motor to degrees per 1 second at the platform
const double PLATFORM_MOTOR_VEL_SF = ((22.0 / 72.0) / 36.0) * (360.0 * 10.0 / 2048.0);
// const double PLATFORM_OFFSET = 190.13;
// const double PLATFORM_OFFSET = 159.7;
const double PLATFORM_OFFSET = 175.9;



// falcon constants
const double ARM_VELOCITY_MEAS_WINDOW = 32.0;
const double PLATFORM_VELOCITY_MEAS_WINDOW = 32.0;

class Arm : public Subsystem
{
private:
    Arm();
    static Arm *m_arm;

    // arm control variables
    double m_rawArmTheta;
    double m_armPos;
    double m_armVel;
    double m_armPosCmd;
    double m_armPosGoal; // this is an intermediate variable
    double m_armPosErr;
    double m_armVelCmd;
    double m_armVelErr;
    double m_armVelErrInt;
    double m_armMotorCmd;
    double m_armPosCmdz;  
    double commandRate;

    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_armMotor;
    frc::DigitalInput *armEncoderInput;
    frc::DutyCycle *m_armEncoder;

    // platform control variables
    double m_rawPlatformTheta;
    double m_platformPos;
    double m_platformVel;
    double m_platformPosCmd;
    double m_platformPosGoal; // this is an intermediate variable
    double m_platformPosErr;
    double m_platformVelCmd;
    double m_platformPosCmdz;  
    double m_platformVelErr;
    double m_platformVelErrInt;
    double m_platformMotorCmd;

    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_platformMotor;
    frc::DigitalInput *platformEncoderInput;
    frc::DutyCycle *m_platformEncoder;

public:
    static Arm *GetInstance();

    void Analyze() override;
    void LocalReset() override;
    void Periodic() override;
    void Stop() override;
    void UpdateDash() override;

    void ArmControl();
    void PlatformControl();


    void SetPosition(double p_desiredArmPos, double p_desiredPlatformPos);
    
    void StartingConfig();

    double GetArmAngle(){
        return m_armPos;
    };
};
