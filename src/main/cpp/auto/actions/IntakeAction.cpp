#include "auto/actions/IntakeAction.h"

IntakeAction::IntakeAction(double time): WaitForTimeAction(time)
{
    m_robotStateMachine = RobotStateMachine::GetInstance();
}

void IntakeAction::Periodic()
{
    // * Set the mode to shooting out of frame so that gatherer does not bring the intake in frame
    // m_robotStateMachine->OutOfFrameShootingAction();
    m_robotStateMachine->GatherAction();
    WaitForTimeAction::Periodic();
}
