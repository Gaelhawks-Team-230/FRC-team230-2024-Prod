#pragma once
#include "util/planners/Interpolator.h"
#include "Constants.h"
#include "util/json/json.hpp"
#include <fstream>
#include "frc/Filesystem.h"
#include <vector>
#include "subsystems/Arm.h"
#include "util/MathUtil.h"

using namespace std;

class ArmKinematics
{
private:
    ArmKinematics();
    static ArmKinematics *m_armK;
    Interpolator *m_interpol;
    Arm *m_arm;

    map<double, vector<double>> ArmInterpolationTable;

    double keyVal = 0;
    double m_goalPos;
    double m_nextPos;
    double m_armDesiredAngle;
    double m_platformDesiredAngle;

    map<double, vector<double>> CreateMap();

    // must change these if there is a sub point that is added.
    map<double, double> rateTable{{0, 40}, {10, 40}, {20, 50}, {35, 50}, {40, 50}, {60, 60}, {80, 100}, {90, 100}, {100, 20}};




public:
    static ArmKinematics *GetInstance();

    void Update();

    void GoToStow();
    void GoToPickup();
    void GoToLowShot();
    void GoToShootingPos(double p_shooterPercentage);
    void GoToAmp();
    void GoToHighTrap();
    void GoToLowTrap();

};
