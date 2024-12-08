#pragma once

#include <vector>
#include "util/Subsystem.h"

class SubsystemManager
{
public:
    SubsystemManager() = default;

    // Accept variable amount of subsystems in one call - use vectors
    void AddSubsystems(std::vector<Subsystem *> subsystems)
    {
        m_subsystems = subsystems;
    }
    /**
     * @brief Reset all subsystems to their local default state.
     *
     */
    void ResetAll()
    {
        for (Subsystem *subsystem : m_subsystems)
        {
            subsystem->LocalReset();
        }
    }
    /**
     * @brief Analyze all subsystems.
     *
     */
    void AnalyzeAll()
    {
        for (Subsystem *subsystem : m_subsystems)
        {
            subsystem->Analyze();
        }
    }
    /**
     * @brief Update the dashboard for all subsystems.
     *
     */
    void UpdateDashAll()
    {
        for (Subsystem *subsystem : m_subsystems)
        {
            subsystem->UpdateDash();
        }
    }
    /**
     * @brief Run the periodic service for all subsystems.
     *
     */
    void RunPeriodicAll()
    {
        for (Subsystem *subsystem : m_subsystems)
        {
            subsystem->Periodic();
        }
    }
    /**
     * @brief Stop all subsystems.
     *
     */
    void StopAll()
    {
        for (Subsystem *subsystem : m_subsystems)
        {
            subsystem->Stop();
        }
    }

private:
    std::vector<Subsystem *> m_subsystems;
};
