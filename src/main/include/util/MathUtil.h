#pragma once

#include <cmath>

/**
 * @brief Utility class for math operations
 */
class MathUtil
{
public:
    /**
     * @brief Wrap value in degrees
     *
     * @param val value in degrees
     * @return double wrapped value in degrees
     */
    static double Wrap(double val)
    {
        double rVal = val * (M_PI / 180);
        double rWrap = atan2(sin(rVal), cos(rVal));
        return rWrap * (180 / M_PI);
    }

    /**
     * @brief Limit the value between a range
     *
     * @param min minimum value
     * @param max maximum value
     * @param val value to compare
     * @return double
     */
    static double Limit(double min, double max, double val)
    {
        if (val > max)
        {
            return max;
        }
        if (val < min)
        {
            return min;
        }
        return val;
    }

    /**
     * @brief Get the sign of a value
     *
     * @param x value to get the sign of
     * @return int sign of the value - (-1 if negative, 1 if positive)
     */
    static int Sign(double x)
    {
        return ((x) > 0) ? (1) : (-1);
    }
    /**
     * @brief Sin of an angle in degrees
     *
     * @param x value in degrees
     * @return double
     */
    static double sind(double x)
    {
        return sin(x * (M_PI / 180));
    }
    /**
     * @brief Cosine of an angle in degrees
     *
     * @param x value in degrees
     * @return double
     */
    static double cosd(double x)
    {
        return cos(x * (M_PI / 180));
    }
    /**
     * @brief Tangent of an angle in degrees
     *
     * @param x value in degrees
     * @return double
     */
    static double tand(double x)
    {
        return tan(x * (M_PI / 180));
    }
    /**
     * @brief Arctan of sides
     *
     * @param x base 1
     * @param y base 2
     * @return double angle in degrees
     */
    static double atan2d(double y, double x)
    {
        return atan2(y, x) * (180 / M_PI);
    }
};