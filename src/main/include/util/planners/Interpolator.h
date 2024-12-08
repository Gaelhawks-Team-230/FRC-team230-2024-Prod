#pragma once

#include <array>
#include <string>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

class Interpolator
{
private:
    double PointSlope(double xActual, double x1, double y1, double x2, double y2);
    double FindLowerKey(std::map<double, std::vector<double>> m_map, double p_key);
    std::map<double, std::vector<double>> m_map;

public:
    Interpolator(std::map<double, std::vector<double>> mymap);
    std::vector<double> Sample(double p_key);
};