#pragma once

/**
 * @class Subsystem
 * @brief This class represents a subsystem of a robot.
 *
 * A subsystem can be any part of a robot that can be controlled or monitored.
 */
class Subsystem
{
public:
    /**
     * @brief Method to set the starting configuration of the subsystem.
     */
    void StartingConfig();

    /**
     * @brief Analyzes sensor, motor, and other data.
     *
     * This method should be overridden by subclasses to implement specific behavior.
     */
    virtual void Analyze() = 0;

    /**
     * @brief Perform any actions that are necessary for the subsystem. Sends motor cmds, etc.
     *
     */
    virtual void Periodic() = 0;

    /**
     * @brief Reset the subsystem to its local default state.
     */
    virtual void LocalReset() = 0;

    /**
     * @brief Stop all operations of the subsystem.
     */
    virtual void Stop() = 0;

    /**
     * @brief Update the dashboard.
     *
     */
    virtual void UpdateDash() = 0;
};