#include "util/planners/Interpolator.h"


Interpolator::Interpolator(std::map<double, std::vector<double>> mymap)
{
    m_map = mymap;
}

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

std::vector<double> Interpolator::Sample(double p_key)
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

    // check if lowerkey is the key for the first element then return vector
    if (upperKey == m_map.begin()->first)
    {
        // cout << "hi" << endl;

        auto lowerKey = m_map.begin()->first;
        auto upperKey = next(m_map.begin())->first;

        std::vector<double> actualVector;

        for (unsigned int i = 0; i < m_map[lowerKey].size(); i++)
        {
            auto result = PointSlope(p_key, lowerKey, m_map[lowerKey][i], upperKey, m_map[upperKey][i]);
            actualVector.push_back(result);
        }

        return actualVector;
    }

    if (lowerKey == m_map.rbegin()->first)
    {
        // cout << "hello" << endl;

        auto lowerKey = m_map.rbegin()->first;
        auto upperKey = prev(m_map.rbegin())->first;

        std::vector<double> actualVector;

        for (unsigned int i = 0; i < m_map[lowerKey].size(); i++)
        {
            auto result = PointSlope(p_key, lowerKey, m_map[lowerKey][i], upperKey, m_map[upperKey][i]);
            actualVector.push_back(result);
        }

        return actualVector;
    }

    // note: at this point p_key is somewhere in the middle of the map

    // generate interpolated data
    std::vector<double> actualVector;

    //
    for (unsigned int i = 0; i < m_map[lowerKey].size(); i++)
    {
        auto result = PointSlope(p_key, lowerKey, m_map[lowerKey][i], upperKey, m_map[upperKey][i]);
        actualVector.push_back(result);
    }

    return actualVector;
}

double Interpolator::PointSlope(double xActual, double x1, double y1, double x2, double y2)
{
    double interpolatorSlope = (y2 - y1) / (x2 - x1);
    double yActual = interpolatorSlope * (xActual - x1) + y1;
    return yActual;
}