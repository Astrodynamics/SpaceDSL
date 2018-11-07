/************************************************************************
* Copyright (C) 2018 Niu ZhiYong
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Niu ZhiYong
* Date:2018-03-20
* Description:
*   SpOrbitParam.cpp
*
*   Purpose:
*
*       Related Classes and Functions of Orbital Parameters
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include <cmath>

#include "SpaceDSL/SpOrbitParam.h"
#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpUtils.h"
#include "SpaceDSL/SpConst.h"


namespace SpaceDSL{
    /*************************************************
     * Class type: Cartesian State Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Position and velocity in cartesian coordinates
     *  This Class is Thread Safe!
    **************************************************/
    CartState::CartState()
    {
        this->m_Pos.fill(0);
        this->m_Vel.fill(0);
    }

    CartState::CartState(const Vector3d &pos, const Vector3d &vel)
    {
        this->m_Pos = pos;
        this->m_Vel = vel;
    }

    CartState::CartState(double xPos, double yPos, double zPos, double xVel, double yVel, double zVel)
    {
        this->m_Pos[0] = xPos;
        this->m_Pos[1] = yPos;
        this->m_Pos[2] = zPos;

        this->m_Vel[0] = xVel;
        this->m_Vel[1] = yVel;
        this->m_Vel[2] = zVel;

    }

    CartState::~CartState()
    {

    }

    const CartState CartState::operator -() const
    {
        CartState tempState(-m_Pos, -m_Vel);

        return tempState;
    }

    const CartState CartState::operator +(const CartState &state) const
    {
        CartState tempState(m_Pos + state.Pos(), m_Vel + state.Vel());

        return tempState;
    }

    const CartState CartState::operator -(const CartState &state) const
    {
        CartState tempState(m_Pos - state.Pos(), m_Vel - state.Vel());

        return tempState;
    }

    const CartState &CartState::operator+=(const CartState &state)
    {
        m_Pos += state.Pos();
        m_Vel += state.Vel();
        return *this;
    }

    const CartState &CartState::operator-=(const CartState &state)
    {
        m_Pos -= state.Pos();
        m_Vel -= state.Vel();
        return *this;
    }

    /*************************************************
     * Class type: Geodetic Coordinate
     * Author: Niu ZhiYong
     * Date:2018-06-08
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]
     *  This Class is Thread Safe!
    **************************************************/
    GeodeticCoord::GeodeticCoord()
    {
        m_Longitude = 0;
        m_Latitude  = 0;
        m_Altitude  = 0;
    }

    GeodeticCoord::GeodeticCoord(double longitude, double latitude, double altitude)
    {
        m_Longitude = longitude;
        m_Latitude  = latitude;
        m_Altitude  = altitude;
    }

    GeodeticCoord::~GeodeticCoord()
    {

    }

    /*************************************************
     * Class type: Classic Orbit Element
     * Author: Niu ZhiYong
     * Date:2018-03-20
     *  This Class is Thread Safe!
    **************************************************/
    OrbitElem::OrbitElem()
    {
        this->m_SMajAx = 0;
        this->m_Ecc = 0;
        this->m_I = 0;
        this->m_RAAN = 0;
        this->m_ArgPeri = 0;
        this->m_TrueA = 0;
    }

    OrbitElem::OrbitElem(double sMajAx, double ecc, double i, double raan, double argPeri, double trueA)
    {
        this->m_SMajAx = sMajAx;
        this->m_Ecc = ecc;
        this->m_I = i;
        this->m_RAAN = raan;
        this->m_ArgPeri = argPeri;
        this->m_TrueA = trueA;
    }

    OrbitElem::~OrbitElem()
    {

    }
    /*************************************************
     * Class type: Modified Orbital Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  This Class is Thread Safe!
    **************************************************/
    ModOrbElem::ModOrbElem()
    {

    }

    ModOrbElem::ModOrbElem(double periRad, double ecc, double i, double raan, double argPeri, double trueA)
    {

    }

    ModOrbElem::~ModOrbElem()
    {

    }


    //
    // F : local function for use by FindEta()
    // F = 1 - eta +(m/eta**2)*W(m/eta**2-l)
    //
    double F (double eta, double m, double l)
    {
        // Constants
        const double eps = 100.0 * EPS;

        // Variables
        double  w, W, a, n, g;

        w = m / (eta * eta) - l;

        if (fabs(w) < 0.1)
        {
            // Series expansion
            W = a = 4.0 / 3.0;
            n = 0.0;
            do {
                n += 1.0;  a *= w * ( n + 2.0) / (n + 1.5);
                W += a;
            }
             while (fabs(a) >= eps);
        }
        else
        {
            if (w > 0.0)
            {
                g = 2.0 * asin(sqrt(w));
                W = (2.0 * g - sin(2.0 * g)) / pow(sin(g), 3);
            }
            else
            {
                g = 2.0 * log(sqrt(-w)+sqrt(1.0 - w));  // =2.0*arsinh(sqrt(-w))
                W = (sinh(2.0 * g) - 2.0 * g) / pow(sinh(g), 3);
            }
        }

        return ( 1.0 - eta + (w + l) * W );
    }   // End of function F

    //====================================================================
    //
    // Grouping: Parameter Conversion
    //
    //====================================================================
    bool CheakEccentricity(double eccentricity)
    {
        if (eccentricity >= 0 && eccentricity < 1)
        {
            return true;
        }
        else
            return false;

    }

    bool CheakOrbit(double sMajAx, double eccentricity, double cbRadius)
    {
        double r_min = sMajAx * ( 1 - eccentricity);
        if (CheakEccentricity(eccentricity) && r_min > cbRadius )
            return true;
        else
            return false;
    }
    //====================================================================
    double ApogeeRadToApoAlt(double apogeeRad, double cbRadius)
    {
        if (apogeeRad <= cbRadius)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Apogee Radius <= Center Body Radius!");
        }

        return (apogeeRad - cbRadius);
    }

    double ApogeeAltToApoRad(double apogeeAlt, double cbRadius)
    {
        return (apogeeAlt + cbRadius);
    }

    double ApogeeRadToMeanMotn(double apogeeRad, double eccentricity, double gm)
    {
        if (CheakEccentricity(eccentricity))
        {
            double sMajAx = apogeeRad / (1 + eccentricity);
            return sqrt(gm / pow(sMajAx, 3));
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    double MeanMotionToApoRad(double meanMotion, double eccentricity, double gm)
    {
        if (CheakEccentricity(eccentricity))
        {
            double sMajAx = pow(gm / pow(meanMotion, 2), 1/3);
            return sMajAx * (1 + eccentricity);
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    double ApogeeRadToPeriRad(double apogeeRad, double eccentricity, double cbRadius)
    {
        if (CheakEccentricity(eccentricity))
        {
            double periRad = apogeeRad * (1 - eccentricity) / (1 + eccentricity);
            if (periRad <= cbRadius)
            {
                throw SPException(__FILE__, __FUNCTION__, __LINE__, " Perigee Radius <= Center Body Radius!");
            }
            return periRad;
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");

    }

    double PerigeeRadToApoRad(double perigeeRad, double eccentricity, double cbRadius)
    {
        if (CheakEccentricity(eccentricity))
        {
            if (perigeeRad <= cbRadius)
            {
                throw SPException(__FILE__, __FUNCTION__, __LINE__, " Perigee Radius <= Center Body Radius!");
            }
            double apogeeRad =  perigeeRad * (1 + eccentricity) / (1 - eccentricity);

            return apogeeRad;
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    double ApogeeRadToPeriAlt(double apogeeRad, double eccentricity, double cbRadius)
    {
        if (CheakEccentricity(eccentricity))
        {
            double periRad = apogeeRad * (1 - eccentricity) / (1 + eccentricity);
            if (periRad <= cbRadius)
            {
                throw SPException(__FILE__, __FUNCTION__, __LINE__, " Perigee Radius <= Center Body Radius!");
            }

            return  periRad - cbRadius;
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    double PerigeeAltToApoRad(double perigeeAlt, double eccentricity, double cbRadius)
    {
        if (CheakEccentricity(eccentricity))
        {
            double perigeeRad = perigeeAlt + cbRadius;
            return perigeeRad * (1 + eccentricity) / (1 - eccentricity);
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    /*********************/


    //====================================================================
    double MeanAnomalyToEcc(double meanAnomaly, double eccentricity)
    {
        if (CheakEccentricity(eccentricity))
        {
            // Constants

            const int maxit = 50;
            const double eps = 100.0 * EPS;

            // Variables

            int    i = 0;
            double Ecc, f;

            // Starting value

            meanAnomaly = Modulo(meanAnomaly, 2.0 * PI);
            if (eccentricity < 0.8)
                Ecc = meanAnomaly;
            else
                Ecc = PI;

            // Iteration

            do {
                f = Ecc - eccentricity * sin(Ecc) - meanAnomaly;
                Ecc = Ecc - f / ( 1.0 - eccentricity * cos(Ecc) );
                ++i;
                if (i == maxit)
                {
                    throw SPException(__FILE__, __FUNCTION__, __LINE__, "Convergence Problems in MeanAnomalyToEcc!");
                }
            }
            while (fabs(f) > eps);
            return Ecc;
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");

    }

    double EccAnomalyToMean(double eccAnomaly, double eccentricity)
    {
        if (CheakEccentricity(eccentricity))
        {
            return (eccAnomaly - eccentricity * sin(eccAnomaly));
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    /*********************/
    double TrueAnomalyToEcc(double trueAnomaly, double eccentricity)
    {
        if (CheakEccentricity(eccentricity))
        {
            double temp1 = sqrt((1 - eccentricity) / (1 + eccentricity));
            return 2 * (atan(temp1 * tan(trueAnomaly/2)) + PI);
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    double EccAnomalyToTrue(double eccAnomaly, double eccentricity)
    {
        if (CheakEccentricity(eccentricity))
        {
            double temp1 = sqrt((1 + eccentricity) / (1 - eccentricity));
            return 2 * (atan(temp1 * tan(eccAnomaly/2)) + PI);
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    /*********************/
    double MeanAnomalyToTrue(double meanAnomaly, double eccentricity)
    {
        if (CheakEccentricity(eccentricity))
        {
            double ecc = MeanAnomalyToEcc(meanAnomaly, eccentricity);
            return EccAnomalyToTrue(ecc, eccentricity);
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");
    }

    double TrueAnomalyToMean(double trueAnomaly, double eccentricity)
    {
        if (CheakEccentricity(eccentricity))
        {
            double ecc = TrueAnomalyToEcc(trueAnomaly, eccentricity);
            return EccAnomalyToMean(ecc, eccentricity);
        }
        else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "This Orbit is Not an Ellipse!");


    }
    //====================================================================
    void CartToOrbitElem(const CartState &cart, double gm, OrbitElem &elem)
    {
        // Variables
        Vector3d h, r, v;
        double  H, u, R;
        double  eCosE, eSinE, e2, E, nu;
        double  a,e,i,raan,argPeri,M;
        r = cart.Pos();
        v = cart.Vel();

        h = r.cross(v);                                             // Areal velocity
        H = h.norm();

        raan = atan2 ( h(0), -h(1) );                              // Long. ascend. node
        raan = Modulo(raan, TwoPI);
        i     = atan2 ( sqrt(h(0) * h(0) + h(1) * h(1)), h(2) );    // Inclination
        u     = atan2 ( r(2) * H, -r(0) * h(1) + r(1) * h(0) );     // Arg. of latitude

        R  = r.norm();                                              // Distance

        a = 1.0 / (2.0 / R - v.dot(v) / gm);                        // Semi-major axis

        eCosE = 1.0-R/a;                                            // e*cos(E)
        eSinE = r.dot(v) / sqrt(gm * a);                            // e*sin(E)

        e2 = eCosE * eCosE + eSinE * eSinE;
        e  = sqrt(e2);                                              // Eccentricity
        E  = atan2(eSinE, eCosE);                                   // Eccentric anomaly

        M  = Modulo(E-eSinE, TwoPI);                                // Mean anomaly

        nu = atan2(sqrt(1.0 - e2) * eSinE, eCosE - e2);             // True anomaly

        argPeri = Modulo(u - nu, TwoPI);                              // Arg. of perihelion

        // Keplerian elements vector
        double trueA = MeanAnomalyToTrue(M, e);
        elem = OrbitElem(a, e, i, raan, argPeri,trueA);

    }

    void OrbitElemToCart(const OrbitElem &elem, double gm, CartState &cart)
    {
        // Variables

        double  a, e, i, raan, argPeri, T;
        double  E, cosE, sinE, fac, R, V;
        Vector3d  r, v;
        Matrix3d  PQW;

        // Keplerian elements at epoch

        a = elem.SMajAx();
        e = elem.Ecc();
        i = elem.I();
        raan = elem.RAAN();
        argPeri = elem.ArgPeri();
        T = elem.TrueA();
        // Eccentric anomaly
        E = TrueAnomalyToEcc(T, e);
        cosE = cos(E);
        sinE = sin(E);
        // Perifocal coordinates

        fac = sqrt ( (1.0 - e) * (1.0 + e) );

        R = a * (1.0 - e * cosE);   // Distance
        V = sqrt( gm * a) / R;      // Velocity

        r = Vector3d ( a * (cosE - e), a * fac * sinE , 0.0 );
        v = Vector3d ( -V * sinE   , +V * fac * cosE, 0.0 );

        // Transformation to reference system (Gaussian vectors)

        PQW = RotateZ(-raan) * RotateX(-i) * RotateZ(-argPeri);

        r = PQW * r;
        v = PQW * v;

        // State vector
        cart = CartState(r, v);
    }



}
