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

    void Start() override
    {
    }

    void Periodic() override
    {
        for (Action *action : m_actions)
        {
            action->Run();
        }
    }

    void Finish() override
    {
    }

    bool IsActionComplete() override
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