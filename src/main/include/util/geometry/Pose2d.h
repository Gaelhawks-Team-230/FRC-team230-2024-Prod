#pragma once

#include <array>

class Pose2d
{
public:
    /**
     * @brief Construct a new Pose 2d object
     *
     */
    Pose2d() : m_x(0), m_y(0), m_rotation(0){};
    /**
     * @brief Construct a new Pose 2d object
     *
     * @param x x position in inches
     * @param y y position in inches
     * @param rotation rotation in degrees
     */
    Pose2d(double x, double y, double rotation)
    {
        m_x = x;
        m_y = y;
        m_rotation = rotation;
    };

    /**
     * @brief Get the X position
     *
     * @return double inches
     */
    double GetX() { return m_x; };
    /**
     * @brief Get the Y position
     *
     * @return double inches
     */
    double GetY() { return m_y; };
    /**
     * @brief Get the rotation
     *
     * @return double degrees
     */
    double GetRotation() { return m_rotation; };
    /**
     * @brief Get the Pose as an array
     *
     * @return std::array<double, 3> {x, y, rotation}
     */
    std::array<double, 3> GetPoseAsArray() { return {m_x, m_y, m_rotation}; };

private:
    double m_x;
    double m_y;
    double m_rotation;
};