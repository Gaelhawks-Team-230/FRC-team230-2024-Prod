#pragma once

#include "auto/actions/Action.h"


class WaitForTimeAction : public Action
{
public:
    WaitForTimeAction(double time) : m_time(time) {}

    void Start() override
    {
        // std::cout << "Starting wait for time action." << std::endl;
        // std::cout << "Waiting for " << m_time << " seconds." << std::endl;
    }

    void Periodic() override
    {
        m_elapsedTime = Action::GetCount() * LOOPTIME;
        // std::cout << "Time elapsed: " << m_elapsedTime << " seconds." << std::endl;
    }
    void Finish() override
    {
        // std::cout << "Wait for time " << m_elapsedTime << "action is done." << std::endl;
    }
    bool IsActionComplete() override
    {
        //  ! TODO - Is this correct?
        return (m_elapsedTime > (m_time + LOOPTIME / 2));
    }

private:
    double m_time;
    double m_elapsedTime = 0.0;
    const double LOOPTIME = 0.02;
};