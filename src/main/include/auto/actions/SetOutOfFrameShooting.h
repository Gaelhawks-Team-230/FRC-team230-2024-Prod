#pragma once

#include "RobotStateMachine.h"
#include "auto/actions/Action.h"
#include "auto/actions/WaitForTimeAction.h"

class SetOutOfFrameShooting : public WaitForTimeAction
{
private:
    RobotStateMachine *m_robotStateMachine;

public:
    SetOutOfFrameShooting(double time) : WaitForTimeAction(time)
    {
        m_robotStateMachine = RobotStateMachine::GetInstance();
    };
    void Periodic()
    {
        m_robotStateMachine->OutOfFrameShootingAction();
        WaitForTimeAction::Periodic();
    }
};