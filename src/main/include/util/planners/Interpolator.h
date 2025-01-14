#pragma once

#include "math.h"
#include "Constants.h"
#include "vector"
#include <string>
#include <iterator>
#include <algorithm>
#include <map>

using namespace std;
class Interpolator
{
private:
    double PointSlope(double xActual, double x1, double y1, double x2, double y2);
    map<double, vector<double>> m_map;
    bool m_edgeCaseInterpolate;


public:
    Interpolator(map<double, vector<double>> mymap);
    vector<double> Sample(double p_key);
    double FindLowerKey(map<double, vector<double>> m_map, double p_key);

};