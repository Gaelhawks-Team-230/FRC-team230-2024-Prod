#pragma once

#include "Constants.h"

class CommandModel
{
private:
    double m_xf, m_yf, m_zf;

public:
    /**
     * @brief Construct a new Command Model object
     *
     */
    CommandModel()
    {
        m_xf = 0.0;
        m_yf = 0.0;
        m_zf = 0.0;
    }

    void Reset()
    {
        m_xf = 0.0;
        m_yf = 0.0;
        m_zf = 0.0;
    }

    /**
     * @brief Command model filter
     *
     * @param x input xdot
     * @param y input ydot
     * @param z input psidot
     * @param pxout xdot out
     * @param pyout ydot out
     * @param pzout psidot out

     */
    void Calculate(double x, double y, double z,
                   double *pxout, double *pyout, double *pzout, double kx, double ky, double kz)
    {
        double l_xfdot, l_yfdot, l_zfdot;

        l_xfdot = (x - m_xf) * kx;
        l_yfdot = (y - m_yf) * ky;
        l_zfdot = (z - m_zf) * kz;

        m_xf = m_xf + l_xfdot * Constants::LOOPTIME;
        m_yf = m_yf + l_yfdot * Constants::LOOPTIME;
        m_zf = m_zf + l_zfdot * Constants::LOOPTIME;

        *pxout = m_xf;
        *pyout = m_yf;
        *pzout = m_zf;
    }
};