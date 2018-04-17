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
*   SpOrbitParam.h
*
*   Purpose:
*
*       Related Classes and Functions of Orbital Parameters
*
*
*   Last modified:
*
*   2018-03-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPORBITPARAM_H
#define SPORBITPARAM_H

#include "SpaceDSL_Global.h"

#include <Eigen/Core>


using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: Cartesian State Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Position and velocity in cartesian coordinates
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API CartState
    {
    public:
            CartState();
            CartState(const Vector3d& pos, const Vector3d& vel);
            CartState(double xPos, double yPos, double zPos, double xVel, double yVel, double zVel);
            virtual ~CartState();

            inline Vector3d     Pos() const {return m_Pos;}
            inline Vector3d     Vel() const {return m_Vel;}

            inline void         SetPos(const Vector3d& pos) { m_Pos = pos;}
            inline void         SetVel(const Vector3d& vel) { m_Vel = vel;}

            inline void         SetPos(double xPos, double yPos, double zPos)
            {
                this->m_Pos[0] = xPos;
                this->m_Pos[1] = yPos;
                this->m_Pos[2] = zPos;
            }
            inline void         SetVel(double xVel, double yVel, double zVel)
            {
                this->m_Vel[0] = xVel;
                this->m_Vel[1] = yVel;
                this->m_Vel[2] = zVel;
            }

            const CartState		operator -() const;
            const CartState		operator +(const CartState& state) const;
            const CartState		operator -(const CartState& state) const;
            const CartState&	operator+=(const CartState& state);
            const CartState&	operator-=(const CartState& state);

    protected:

            Vector3d	m_Pos;		///< position(m)
            Vector3d	m_Vel;		///< velocity(m/s)
    };
    /*************************************************
     * Class type: Classic Orbit Element
     * Author: Niu ZhiYong
     * Date:2018-03-20
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API OrbitElem
    {
    public:
            OrbitElem();
            OrbitElem(double sMajAx, double ecc, double i, double raan, double argPeri, double trueA);
            virtual ~OrbitElem();

            inline double     SMajAx()  const {return m_SMajAx;}
            inline double     Ecc()     const {return m_Ecc;}
            inline double     I()       const {return m_I;}
            inline double     RAAN()    const {return m_RAAN;}
            inline double     ArgPeri() const {return m_ArgPeri;}
            inline double     TrueA()   const {return m_TrueA;}

            inline void       SetSMajAx(double sMajAx)   { m_SMajAx = sMajAx;}
            inline void       SetEcc(double ecc)         { m_Ecc = ecc;}
            inline void       SetI(double i)             { m_I = i;}
            inline void       SetRAAN(double raan)       { m_RAAN = raan;}
            inline void       SetArgPeri(double argPeri) { m_ArgPeri = argPeri;}
            inline void       SetTrueA(double trueA)     { m_TrueA = trueA;}
    protected:
            double	m_SMajAx;	///< Semimajor axis
            double	m_Ecc;		///< Eccentricity
            double	m_I;		///< Inclination
            double	m_RAAN;		///< Right Ascension of the ascending node (RAAN)
            double	m_ArgPeri;	///< Argument of Perigee
            double	m_TrueA;	///< True Anomaly
    };

    /*************************************************
     * Class type: Modified Orbital Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API ModOrbElem
    {
    public:
            ModOrbElem();
            ModOrbElem(double periRad, double ecc, double i, double raan, double argPeri, double trueA);
            virtual ~ModOrbElem();

            inline double     PeriRad()     const {return m_PeriRad;}
            inline double     Ecc()         const {return m_Ecc;}
            inline double     I()           const {return m_I;}
            inline double     RAAN()        const {return m_RAAN;}
            inline double     ArgPeri()     const {return m_ArgPeri;}
            inline double     TrueA()       const {return m_TrueA;}

            inline void       SetPeriRad(double periRad) { m_PeriRad = periRad;}
            inline void       SetEcc(double ecc)         { m_Ecc = ecc;}
            inline void       SetI(double i)             { m_I = i;}
            inline void       SetRAAN(double raan)       { m_RAAN = raan;}
            inline void       SetArgPeri(double argPeri) { m_ArgPeri = argPeri;}
            inline void       SetTrueA(double trueA)     { m_TrueA = trueA;}

    protected:
            double	m_PeriRad;	///< periapsis radius in m
            double	m_Ecc;		///< eccentricity
            double	m_I;		///< inclination in radians
            double	m_RAAN;		///< right ascension of ascending node
            double	m_ArgPeri;	///< arg of periapsis in rad
            double	m_TrueA;	///< true anomaly in radians
    };

    /*************************************************
     * Class type: Equinoctial Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     * this is a nonsingular orbital element set that is often used
     * in operational systems since it is well behaved for small
     * eccentricities and inclinations.  The dir element of the structure
     * indicates whether the orbit is posigrade or retrograde.
     * This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API EqctlElem
    {
    protected:
            double	m_SMajAx;	///< semimajor axis length
            double	m_h;		///< e*sin(omegaBar)
            double	m_k;		///< e*cos(omegaBar)
            double	m_p;		///< tan(i/2)*sin(raan)
            double	m_q;		///< tan(i/2)*cos(raan)
            double	m_MeanLon;	///< mean longitude
            bool	m_Dir;		///< Retrograde=false, Posigrade=true
    };

    /*************************************************
     * Class type: Delaunay Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     * This is a set of canonical angle-action variables
     * that are commonly used in analytic orbit theory.
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API DelnyElem
    {
    protected:
            double          m_H;		///< z component of angle momentum, sqrt(mu*a*(1-e^2))*cosi
            double          m_G;		///< angular momentum, sqrt(mu*a*(1-e^2))
            double          m_L;		///< sqrt(mu*a)
            double          m_h;		///< right asc fo asc node (rad)
            double          m_g;		///< argument fo perigee (rad)
            double          m_l;		///< mean anomaly (rad)
    };

    /*************************************************
     * Class type: Spherical Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     * Usually used with an inertial coordinate system
     * but may be used with body fixed coordinates which
     * changes the right ascension to longitude, the declination to
     * latitude and makes the velocity angles and magitude be
     * measured from the body fixed velocity vector.
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API SphElem
    {
    protected:
            double	m_RA;		///< right ascension/longitude(fixed)
            double	m_Dec;		///< declination(inertial)/latitude(fixed)
            double	m_R;		///< radius
            double	m_FPA;		///< horiz. flight path angle
            double	m_Azi;		///< inertial flight path azimuth
            double	m_V;		///< inertial velocity
    };

    /*************************************************
     * Class type: Mixed Spherical Elements
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     * This element set represents a mixture between the inertial and
     * body fixed versions of the spherical elements.  In this case,
     * the position components are given in the "detic" body fixed
     * coordinates of latitude, longitude and altitude.  "Detic" means
     * that angular measurements are made based on a zenith direction
     * perpendicular to the surface of the central body.  The velocity
     * componets are based on the inertial velocity of the object.
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API MixSphElem
    {
    protected:
            double	m_Lon;		///< planetodetic longitude
            double	m_Lat;		///< planetodetic latitude
            double	m_Alt;		///< planetodetic altitude
            double	m_FPA;		///< horiz. flight path angle
            double	m_Azi;		///< flight path azimuth
            double	m_V;		///< velocity
    };

    //====================================================================
    //
    // Grouping: Parameter Conversion
    //
    //====================================================================
    bool SPACEDSL_API CheakEccentricity (double eccentricity);
    bool SPACEDSL_API CheakOrbit (double sMajAx, double eccentricity, double cbRadius);

    /********************************************************************/
    ///Calculate Apogee Radius to Apogee Altitude
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      apogeeRad/apogeeAlt
    /// @Param      cbRadius      Center Body Radius
    /// @Output
    /// @Return     Apogee Altitude/Apogee Radius
    /**********************************************************************/
    double SPACEDSL_API ApogeeRadToApoAlt (double apogeeRad, double cbRadius);

    double SPACEDSL_API ApogeeAltToApoRad (double apogeeAlt, double cbRadius);

    /********************************************************************/
    ///Calculate Apogee Radius to Mean Motion(rad/s)
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      apogeeRad/meanMotion
    /// @Param      eccentricity
    /// @Param      gm
    /// @Output
    /// @Return     Mean Motion/Apogee Radius
    /**********************************************************************/
    double SPACEDSL_API ApogeeRadToMeanMotn (double apogeeRad, double eccentricity, double gm);

    double SPACEDSL_API MeanMotionToApoRad (double meanMotion, double eccentricity, double gm);

    /********************************************************************/
    ///Calculate Apogee Radius to Perigee Radius.
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      apogeeRad/perigeeRad
    /// @Param      eccentricity
    /// @Param      cbRadius        Center Body Radius
    /// @Output
    /// @Return     Perigee Radius/Apogee Radius
    /**********************************************************************/
    double SPACEDSL_API ApogeeRadToPeriRad (double apogeeRad, double eccentricity, double cbRadius);

    double SPACEDSL_API PerigeeRadToApoRad (double perigeeRad, double eccentricity, double cbRadius);

    /********************************************************************/
    ///Calculate Apogee Radius to Perigee Altitude.
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      apogeeRad/perigeeAlt
    /// @Param      eccentricity
    /// @Param      cbRadius        Center Body Radius
    /// @Output
    /// @Return     Perigee Altitude/Apogee Radius
    /**********************************************************************/
    double SPACEDSL_API ApogeeRadToPeriAlt (double apogeeRad, double eccentricity, double cbRadius);

    double SPACEDSL_API PerigeeAltToApoRad (double perigeeAlt, double eccentricity, double cbRadius);

    /********************************************************************/
    ///Calculate Eccentric Anomaly in radians from Mean Anomaly
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      meanAnomaly/eccAnomaly     (rad)
    /// @Param      eccentricity
    /// @Output
    /// @Return     Ecc Anomaly/ Mean Anomaly
    /**********************************************************************/
    double SPACEDSL_API MeanAnomalyToEcc (double meanAnomaly, double eccentricity);

    double SPACEDSL_API EccAnomalyToMean(double eccAnomaly, double eccentricity);

    /********************************************************************/
    ///Calculate Eccentric Anomaly in radians from True Anomaly
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      trueAnomaly/eccAnomaly     (rad)
    /// @Param      eccentricity
    /// @Output
    /// @Return     Ecc Anomaly/True Anomaly
    /**********************************************************************/
    double SPACEDSL_API TrueAnomalyToEcc (double trueAnomaly, double eccentricity);

    double SPACEDSL_API EccAnomalyToTrue (double eccAnomaly, double eccentricity);

    /********************************************************************/
    ///Calculate True Anomaly in radians from Mean Anomaly
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      meanAnomaly/trueAnomaly     (rad)
    /// @Param      eccentricity
    /// @Output
    /// @Return     True Anomaly/Mean Anomaly
    /**********************************************************************/
    double SPACEDSL_API MeanAnomalyToTrue (double meanAnomaly, double eccentricity);

    double SPACEDSL_API TrueAnomalyToMean (double trueAnomaly, double eccentricity);

    /********************************************************************/
    ///Calculate The Position and Velocity Vector from The Orbits Elements in J2000
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      elem/cart
    /// @Param      gm
    /// @Output
    /// @Param      cart/elem
    /// @Return     void
    /**********************************************************************/
    void SPACEDSL_API OrbitElemToCart (const OrbitElem& elem, double gm, CartState& cart);

    void SPACEDSL_API CartToOrbitElem (const CartState& cart, double gm, OrbitElem& elem);
		
}
#endif //SPORBITPARAM_H
