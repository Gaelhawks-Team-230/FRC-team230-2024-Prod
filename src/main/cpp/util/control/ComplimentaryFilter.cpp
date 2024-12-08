#include "util/control/ComplimentaryFilter.h"

ComplementaryFilter::ComplementaryFilter(double cutoffFrequency, double dampingRatio, double sampleTime)
{
    // Store parameters
    m_cutoffFrequency = cutoffFrequency;
    m_dampingRatio = dampingRatio;
    m_sampleTime = sampleTime;

    // Define continuous-time transfer function coefficients
    std::vector<double> lowPassNum = {0, 0, 1};
    std::vector<double> lowPassDen = {1 / (m_cutoffFrequency * m_cutoffFrequency), 2 * m_dampingRatio / m_cutoffFrequency, 1};
    std::vector<double> highPassNum = {1 / (m_cutoffFrequency * m_cutoffFrequency), 2 * m_dampingRatio / m_cutoffFrequency, 0};
    std::vector<double> highPassDen = {1 / (m_cutoffFrequency * m_cutoffFrequency), 2 * m_dampingRatio / m_cutoffFrequency, 1};

    // Convert to discrete-time filter coefficients using internal function
    std::tie(m_lowPassNumerator, m_lowPassDenominator) = CalculateDiscreteCoefficients(lowPassNum, lowPassDen, m_sampleTime);
    std::tie(m_highPassNumerator, m_highPassDenominator) = CalculateDiscreteCoefficients(highPassNum, highPassDen, m_sampleTime);

    // Initialize past data to zero
    m_previousInputLowPass = 0;
    m_previousPreviousInputLowPass = 0;
    m_previousOutputLowPass = 0;
    m_previousPreviousOutputLowPass = 0;
    m_previousInputHighPass = 0;
    m_previousPreviousInputHighPass = 0;
    m_previousOutputHighPass = 0;
    m_previousPreviousOutputHighPass = 0;
}

double ComplementaryFilter::LowPassFilter(double input)
{
    // Update past data
    m_previousPreviousInputLowPass = m_previousInputLowPass;
    m_previousInputLowPass = input;
    m_previousPreviousOutputLowPass = m_previousOutputLowPass;

    // Compute filter output
    double output = input * m_lowPassNumerator[0] + m_previousInputLowPass * m_lowPassNumerator[1] +
                    m_previousPreviousInputLowPass * m_lowPassNumerator[2] -
                    m_previousOutputLowPass * m_lowPassDenominator[1] - m_previousPreviousOutputLowPass * m_lowPassDenominator[2];

    m_previousOutputLowPass = output;
    return output;
}

double ComplementaryFilter::HighPassFilter(double input)
{
    // Update past data
    m_previousPreviousInputHighPass = m_previousInputHighPass;
    m_previousInputHighPass = input;
    m_previousPreviousOutputHighPass = m_previousOutputHighPass;

    // Compute filter output
    double output = input * m_highPassNumerator[0] + m_previousInputHighPass * m_highPassNumerator[1] +
                    m_previousPreviousInputHighPass * m_highPassNumerator[2] -
                    m_previousOutputHighPass * m_highPassDenominator[1] - m_previousPreviousOutputHighPass * m_highPassDenominator[2];

    m_previousOutputHighPass = output;
    return output;
}

void ComplementaryFilter::PrintDiscreteCoefficients()
{
    printf("nz_lo\n");
    for (double val : m_lowPassNumerator)
    {
        printf("%f\n", val);
    }
    printf("dz_lo\n");
    for (double val : m_lowPassDenominator)
    {
        printf("%f\n", val);
    }
    printf("nz_hi\n");
    for (double val : m_highPassNumerator)
    {
        printf("%f\n", val);
    }
    printf("dz_hi\n");
    for (double val : m_highPassDenominator)
    {
        printf("%f\n", val);
    }
}

/**
 * @brief onverts continuous-time filter coefficients to discrete-time using the bilinear transform.
 *
 * @param numeratorCoefficients
 * @param denominatorCoefficients
 * @param samplingTime delta time
 * @return std::pair<std::vector<double>, std::vector<double>>
 */
std::pair<std::vector<double>, std::vector<double>> ComplementaryFilter::CalculateDiscreteCoefficients(const std::vector<double> &numeratorCoefficients,
                                                                                                       const std::vector<double> &denominatorCoefficients,
                                                                                                       double samplingTime)
{
    // Ensure input vectors have the expected length
    if (numeratorCoefficients.size() != 3 || denominatorCoefficients.size() != 3)
    {
        throw "Input vectors must have a length of 3.";
    }

    // Extract coefficients for clarity
    double aNumerator = numeratorCoefficients[0];
    double bNumerator = numeratorCoefficients[1];
    double cNumerator = numeratorCoefficients[2];
    double aDenominator = denominatorCoefficients[0];
    double bDenominator = denominatorCoefficients[1];
    double cDenominator = denominatorCoefficients[2];

    // Calculate numerator coefficients using the bilinear transform
    std::vector<double> discreteNumerator = {
        (4.0 * aNumerator + 2.0 * samplingTime * bNumerator + samplingTime * samplingTime * cNumerator),
        (-8.0 * aNumerator + 2.0 * samplingTime * samplingTime * cNumerator),
        (4.0 * aNumerator - 2.0 * samplingTime * bNumerator + samplingTime * samplingTime * cNumerator)};

    // Calculate denominator coefficients using the bilinear transform
    std::vector<double> discreteDenominator = {
        (4.0 * aDenominator + 2.0 * samplingTime * bDenominator + samplingTime * samplingTime * cDenominator),
        (-8.0 * aDenominator + 2.0 * samplingTime * samplingTime * cDenominator),
        (4.0 * aDenominator - 2.0 * samplingTime * bDenominator + samplingTime * samplingTime * cDenominator)};

    // Normalize the coefficients by the first denominator coefficient
    double firstDenominatorValue = discreteDenominator[0];
    for (double &value : discreteNumerator)
    {
        value /= firstDenominatorValue;
    }
    for (double &value : discreteDenominator)
    {
        value /= firstDenominatorValue;
    }

    return std::make_pair(discreteNumerator, discreteDenominator);
};

void ComplementaryFilter::Clear()
{
    m_previousInputLowPass = 0;
    m_previousPreviousInputLowPass = 0;
    m_previousOutputLowPass = 0;
    m_previousPreviousOutputLowPass = 0;
    m_previousInputHighPass = 0;
    m_previousPreviousInputHighPass = 0;
    m_previousOutputHighPass = 0;
    m_previousPreviousOutputHighPass = 0;
}