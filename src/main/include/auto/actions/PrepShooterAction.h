#pragma once

#include "auto/actions/WaitForTimeAction.h"
#include "RobotStateMachine.h"

class PrepShooterAction : public WaitForTimeAction
{
private:
    RobotStateMachine *m_robotStateMachine;
    double m_armIndex;

public:
    PrepShooterAction(double time, double armIndex);
    void Periodic(); 
};