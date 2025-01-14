#pragma once

#include "auto/actions/WaitForTimeAction.h"
#include "RobotStateMachine.h"

class ShootAction : public WaitForTimeAction
{
private:
    RobotStateMachine *m_robotStateMachine;

public:
    ShootAction(double time);
    void Periodic();
};