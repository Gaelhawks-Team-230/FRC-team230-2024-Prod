#pragma once

#include <vector>
#include <cmath>
#include <tuple>
#include <cstdio>

class ComplementaryFilter
{
public:
    ComplementaryFilter(double cutoffFrequency, double dampingRatio, double sampleTime);
    double LowPassFilter(double input);
    double HighPassFilter(double input);
    void PrintDiscreteCoefficients();
    void Clear();

private:
    std::pair<std::vector<double>, std::vector<double>> CalculateDiscreteCoefficients(const std::vector<double> &numeratorCoefficients,
                                                                                      const std::vector<double> &denominatorCoefficients,
                                                                                      double samplingTime);

    double m_cutoffFrequency;
    double m_dampingRatio;
    double m_sampleTime;
    std::vector<double> m_lowPassNumerator, m_lowPassDenominator, m_highPassNumerator, m_highPassDenominator;
    double m_previousInputLowPass, m_previousPreviousInputLowPass, m_previousOutputLowPass, m_previousPreviousOutputLowPass;
    double m_previousInputHighPass, m_previousPreviousInputHighPass, m_previousOutputHighPass, m_previousPreviousOutputHighPass;
};