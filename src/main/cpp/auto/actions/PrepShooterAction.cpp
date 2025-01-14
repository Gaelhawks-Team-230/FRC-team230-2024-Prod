#include "auto/actions/PrepShooterAction.h"

PrepShooterAction::PrepShooterAction(double time, double armIndex) : WaitForTimeAction(time)
{
    m_robotStateMachine = RobotStateMachine::GetInstance();
    m_armIndex = armIndex;
}
void PrepShooterAction::Periodic()
{
    m_robotStateMachine->AutoAimAction();
    m_robotStateMachine->SetShootingSolutionIndex(m_armIndex);
    WaitForTimeAction::Periodic();
}