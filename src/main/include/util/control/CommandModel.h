#pragma once

#include "Constants.h"

class CommandModel
{
private:
    double m_xf, m_yf, m_zf;
    double kx, ky, kz;

public:
    /**
     * @brief Construct a new Command Model object
     *
     * @param kx x constant
     * @param ky y constant
     * @param kz z constant
     */
    CommandModel(double kx, double ky, double kz)
    {
        m_xf = 0.0;
        m_yf = 0.0;
        m_zf = 0.0;
        this->kx = kx;
        this->ky = ky;
        this->kz = kz;
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
                   double *pxout, double *pyout, double *pzout)
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