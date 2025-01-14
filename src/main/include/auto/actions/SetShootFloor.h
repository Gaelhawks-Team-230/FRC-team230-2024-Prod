#pragma once

#include "RobotStateMachine.h"
#include "auto/actions/Action.h"

class SetShootFloor : public Action
{
private:
    RobotStateMachine *m_robotStateMachine;

public:
    SetShootFloor()
    {
        m_robotStateMachine = RobotStateMachine::GetInstance();
    };
    void Start()
    {
        m_robotStateMachine->AutoAimAction();
        m_robotStateMachine->OutOfFrameShootingAction();
    };
    bool IsActionComplete() { return Action::GetCount() > 1; }
};