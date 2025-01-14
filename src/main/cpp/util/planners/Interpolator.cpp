#include "util/planners/Interpolator.h"
#include <map>
#include <vector>
#include <iostream>

using namespace std;

/**
 * @brief Construct a new Interpolator:: Interpolator object
 * 
 * @param mymap The map you want to pass in
 */
Interpolator::Interpolator(std::map<double, std::vector<double>> mymap)
{
    m_map = mymap;
}

/**
 * @brief This finds the lower and upper key of the map based off the key value you pass in
 * 
 * @param m_map The map that you are checking
 * @param p_key The key you are using to check
 * @return double 
 */
double Interpolator::FindLowerKey(std::map<double, std::vector<double>> m_map, double p_key)
{
    if (m_map.size() == 0)
    {
        throw std::out_of_range("Received empty map.");
    }

    auto lower = m_map.lower_bound(p_key);
    // cout << "upper bound key: " << lower->first << endl;

    // If none found, return the last one.
    if (lower == m_map.end())
    {
        // cout << "no lower bound key found" << endl;
        return prev(lower)->first;
    }

    if (lower == m_map.begin())
    {
        // cout << "lower bound key is the first element" << endl;
        return lower->first;
    }

    // Return the previous key.
    auto previous = prev(lower);
    // cout << "previous key: " << previous->first << endl;
    return previous->first;
}

/**
 * @brief This does the actual interpolation
 * 
 * @param p_key The x value on which you want to interpolate
 * @return vector<double> 
 */
vector<double> Interpolator::Sample(double p_key)
{
    // checks if given key exists
    if (m_map.count(p_key) > 0)
    {
        // cout << "whats up" << endl;
        return m_map.at(p_key);
    }

    // get lower key
    auto lowerKey = FindLowerKey(m_map, p_key);
    // get upper key
    auto upperKey = m_map.lower_bound(p_key)->first;

    vector<double> actualVector;

    m_edgeCaseInterpolate = false;
    // checks if the user wants to interpolate values off the ends of the map
    if (m_edgeCaseInterpolate == true)
    {
        // check if upperKey is the key for the first element then return the interpolated data
        if (upperKey == m_map.begin()->first)
        {

            auto lowerKey = m_map.begin()->first;
            auto upperKey = next(m_map.begin())->first;

            for (unsigned int i = 0; i < m_map[lowerKey].size(); i++)
            {
                auto result = PointSlope(p_key, lowerKey, m_map[lowerKey][i], upperKey, m_map[upperKey][i]);
                actualVector.push_back(result);
            }
            return actualVector;
        }

        // check if lower key is the last element then returns the interpolated data
        if (lowerKey == m_map.rbegin()->first)
        {

            auto lowerKey = m_map.rbegin()->first;
            auto upperKey = prev(m_map.rbegin())->first;

            for (unsigned int i = 0; i < m_map[lowerKey].size(); i++)
            {
                auto result = PointSlope(p_key, lowerKey, m_map[lowerKey][i], upperKey, m_map[upperKey][i]);
                actualVector.push_back(result);
            }
            return actualVector;
        }
    }
    else
    {
        // check if upperKey is the key for the first element then return vector
        if (upperKey == m_map.begin()->first)
        {
            return m_map.begin()->second;
        }

        // check if lowerKey is the key for the last element then return vector
        if (lowerKey == m_map.rbegin()->first)
        {
            return m_map.rbegin()->second;
        }
    }

    // note: at this point the previous logic statements have determined p_key is somewhere in the middle of the map
    // generate interpolated data
    for (unsigned int i = 0; i < m_map[lowerKey].size(); i++)
    {
        auto result = PointSlope(p_key, lowerKey, m_map[lowerKey][i], upperKey, m_map[upperKey][i]);
        actualVector.push_back(result);
    }

    return actualVector;
}

/**
 * @brief Does the math for interpolation.
 *
 * @param xActual X value that you want to interpolate off of
 * @param x1 The x value of the lower buond
 * @param y1 The y value of the lower bound
 * @param x2 The x value of the upper bound
 * @param y2 The y value of the upper bound
 * @return double
 */
double Interpolator::PointSlope(double xActual, double x1, double y1, double x2, double y2)
{
    double interpolatorSlope = (y2 - y1) / (x2 - x1);
    double yActual = interpolatorSlope * (xActual - x1) + y1;
    return yActual;
}