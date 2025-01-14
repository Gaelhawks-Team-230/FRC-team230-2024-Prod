#pragma once

#include <vector>

#include "auto/actions/Action.h"

/**
 * @brief Sequential action, running all sub-actions in sequence
 *
 */
class SeriesAction : public Action
{
public:
    SeriesAction(std::vector<Action *> actions) : m_actions(actions) {}

    void Periodic()
    {
        if (m_currentActionIndex < m_actions.size())
        {
            Action *action = m_actions[m_currentActionIndex];
            if (!action->IsActionComplete())
            {
                action->Run();
            }
            else
            {
                m_currentActionIndex++;
            }
        }
    }

    bool IsActionComplete()
    {
        return m_currentActionIndex >= m_actions.size();
    }

private:
    std::vector<Action *> m_actions;
    int m_currentActionIndex = 0;
};