#pragma once

#include <vector>

#include "auto/actions/Action.h"

/**
 * @brief Composite action, starts all actions at once and runs them all in parallel. This takes as long as the longest action
 *
 */
class ParallelAction : public Action
{
public:
    ParallelAction(std::vector<Action *> actions) : m_actions(actions) {}

    void Periodic()
    {
        for (Action *action : m_actions)
        {
            action->Run();
        }
    }

    bool IsActionComplete()
    {
        for (Action *action : m_actions)
        {
            if (!action->IsActionComplete())
            {
                return false;
            }
        }
        return true;
    }

private:
    std::vector<Action *> m_actions;
};