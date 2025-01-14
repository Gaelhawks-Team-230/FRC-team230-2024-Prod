#pragma once

#include "RobotStateMachine.h"
#include "auto/actions/Action.h"

class StowArmAction : public Action
{
private:
    RobotStateMachine *m_robotStateMachine;

public:
    StowArmAction(){};
    void Start()
    {
        m_robotStateMachine->GoToStowAction();
    };
    bool IsActionComplete() { return Action::GetCount() > 1; }
};