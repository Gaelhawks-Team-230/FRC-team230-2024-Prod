#pragma once

/**
 * @brief Simple action interface for autonomous mode
 *
 */
class Action
{
public:
    /**
     * @brief Run code once when the action is started, for setup
     *
     */
    virtual void Start() = 0;
    /**
     * @brief Run code periodically while the action is active
     *
     */
    virtual void Periodic() = 0;
    /**
     * @brief Check if the action has completed its task
     *
     */
    virtual bool IsActionComplete() { return true; };
    /**
     * @brief Run code once when the action is finished
     *
     */
    virtual void Finish() = 0;
    /**
     * @brief Get the local loop count. First call will return 0, then 1, 2, 3, etc.
     *
     */
    virtual int GetCount() { return m_count; };

    virtual void Run()
    {
        if (!m_started)
        {
            Start();
            m_started = true;
        }
        if (!IsActionComplete())
        {
            Periodic();
            m_count++;
        }
        if (IsActionComplete() && !m_finished)
        {
            Finish();
            m_finished = true;
        }
    }

private:
    int m_count = 0;
    bool m_started = false;
    bool m_finished = false;
};
