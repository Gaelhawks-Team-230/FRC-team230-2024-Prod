#pragma once

#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

#include "Constants.h"
#include "util/Subsystem.h"
#include "util/MathUtil.h"

#include "ctre/phoenix/sensors/CANCoder.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

// Not finalized values

const double CLIMBER_TAU = 0.02;
const double CLIMBER_KV = 10.0;
const double CLIMBER_KP = 5.0;
const double CLIMBER_K = 20.0;

const double COUNTS_PER_REV = 2048.0;   // Convert between Falcon motor counts and motor revolutions
const double CLIMBER_GEAR_RATIO = 30.0; // Ratio of motor revolutions to actual revolutions
const double INCHES_PER_REV = 5.6;      // Length of string on wheel
const double CLIMBER_SCALE_FACTOR = (1 / COUNTS_PER_REV) * (1 / CLIMBER_GEAR_RATIO) * (INCHES_PER_REV);

const double REVS_PER_SECOND = 10.0; // Speed of rotation


const double MAX_VELOCITY = 18.0;

// const double MAX_POS = 36.5;
// const double MIN_POS = 8.0;

const double INIT_POS = 21.0;
// const double DROP_FORK_POS = 23.0;
const double MOVING_POS = 21.5;

const double HOOKS_UP_POS = 35.0;
const double HOOKS_DOWN_POS = 7.45;

const double V_CAL = -2.0;

const double CLIMBER_VEL_MEAS_WINDOW = 32.0;
const double CLIMBER_DEADBAND = 0.005;

const double END_CAL_VELOCITY = 1.0;

class Climber : public Subsystem
{

private:
    Climber();

    static Climber *m_climber;

    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_leftMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_rightMotor;

    double m_leftPos;
    double m_left_pCmd;
    double m_left_pErr;
    double m_leftVel;
    double m_left_vErr;
    double m_left_vErrIntegrator;
    double m_left_vCmd;

    double m_rightPos;
    double m_right_pCmd;
    double m_right_pErr;
    double m_rightVel;
    double m_right_vErr;
    double m_right_vErrIntegrator;
    double m_right_vCmd;

    double m_left_mCmd;
    double m_right_mCmd;

    int m_leftCalstate;
    double m_leftGoalVel;
    bool m_isLeftCalDone;

    int m_rightCalstate;
    double m_rightGoalVel;
    bool m_isRightCalDone;

public:
    static Climber *GetInstance();

    void LocalReset() override;
    void StartingConfig();
    void Stop() override;
    void UpdateDash() override;
    void Periodic() override;
    void Analyze() override;

    void LeftClimbCalibrate();
    void RightClimbCalibrate();


    void ClimbUp(){
        m_leftGoalVel = 8.0;
        m_rightGoalVel = 8.0;
    }
    void ClimbDown(){
        m_leftGoalVel = -5.0;
        m_rightGoalVel = -5.0;
    }
    void StopClimb(){
        m_leftGoalVel = 0.0;
        m_rightGoalVel = 0.0;
    }

    void SetLeftCurrLimit(double curr);
    void SetRightCurrLimit(double curr);
};