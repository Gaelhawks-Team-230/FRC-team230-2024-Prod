#pragma once

#include "auto/actions/Action.h"
#include "Constants.h"

class WaitForTimeAction : public Action
{
public:
    WaitForTimeAction(double time) : m_time(time) {
        // printf("waiting for time: %f\n", m_time);
    }
    void Periodic()
    {
        m_elapsedTime = Action::GetCount() * Constants::LOOPTIME;
    }
    bool IsActionComplete()
    {   
        // printf("elapsed time: %f\n", m_elapsedTime);
        return (m_elapsedTime > (m_time + (Constants::LOOPTIME / 2.0)));
    }

private:
    double m_time;
    double m_elapsedTime = 0.0;
};