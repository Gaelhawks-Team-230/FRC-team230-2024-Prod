#pragma once

#include "auto/actions/WaitForTimeAction.h"
#include "RobotStateMachine.h"

class IntakeAction : public WaitForTimeAction
{
private:
    RobotStateMachine *m_robotStateMachine;

public:
    IntakeAction(double time);
    void Periodic();
};