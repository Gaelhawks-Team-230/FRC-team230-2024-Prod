#pragma once

#include <vector>
#include "auto/actions/Action.h"

/**
 * @brief Represents a collection of actions to be run in sequence. (An auto mode!)
 *
 */
class AutoMode
{
public:
    /**
     * @brief Adds an action to the end of the list of actions to run
     *
     * @param action
     */
    void AddAction(Action *action)
    {
        m_actions.push_back(action);
    }
    /**
     * @brief Runs the current action, and moves to the next one when the current one is done
     *
     */
    void RunActions()
    {
        // Check if we are done
        if (m_actions.empty() && m_currentActionIndex >= m_actions.size())
        {
            m_isAutoComplete = true;
            return;
        }

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
    /**
     * @brief Returns true if all actions are done
     *
     */
    bool IsAutoComplete()
    {
        return m_isAutoComplete;
    }

private:
    std::vector<Action *> m_actions;
    int m_currentActionIndex = 0;
    bool m_isAutoComplete = false;
};