#include "auto/actions/ShootAction.h"

ShootAction::ShootAction(double time) : WaitForTimeAction(time)
{
    m_robotStateMachine = RobotStateMachine::GetInstance();
}

void ShootAction::Periodic()
{
    WaitForTimeAction::Periodic();
    m_robotStateMachine->ShootAction();
}