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
*   SpPerturbation.cpp
*
*   Purpose:
*
*       Contains all Orbital Perturbation Without Gravity of Earth
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpPerturbation.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"

namespace SpaceDSL {

    /*************************************************
     * Class type: The class of The Third Body Gravity Perturbation
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    ThirdBodyGravity::ThirdBodyGravity()
    {
        m_GM = 0;
        m_ThirdBodyStarType = E_NotDefinedStarType;
        m_CenterStarType = E_NotDefinedStarType;
    }

    ThirdBodyGravity::ThirdBodyGravity(SolarSysStarType thirdBodyStarType, SolarSysStarType centerStarType)
    {
        m_CenterStarType = centerStarType;
        m_ThirdBodyStarType = thirdBodyStarType;
        switch (thirdBodyStarType)
        {
        case E_Mercury:
            m_GM = GM_Mercury;
            break;
        case E_Venus:
            m_GM = GM_Venus;
            break;
        case E_Earth:
            m_GM = GM_Earth;
            break;
        case E_Mars:
            m_GM = GM_Mars;
            break;
        case E_Jupiter:
            m_GM = GM_Jupiter;
            break;
        case E_Saturn:
            m_GM = GM_Saturn;
            break;
        case E_Uranus:
            m_GM = GM_Uranus;
            break;
        case E_Neptune:
            m_GM = GM_Neptune;
            break;
        case E_Pluto:
            m_GM = GM_Pluto;
            break;
        case E_Moon:
            m_GM = GM_Moon;
            break;
        case E_Sun:
            m_GM = GM_Sun;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "ThirdBodyGravity: SolarSysStarType Unsupport ");
        }
    }

    ThirdBodyGravity::~ThirdBodyGravity()
    {

    }

    void ThirdBodyGravity::SetThirdBodyStar(SolarSysStarType thirdBodyStarType)
    {
        m_ThirdBodyStarType = thirdBodyStarType;
        switch (thirdBodyStarType)
        {
        case E_Mercury:
            m_GM = GM_Mercury;
            break;
        case E_Venus:
            m_GM = GM_Venus;
            break;
        case E_Earth:
            m_GM = GM_Earth;
            break;
        case E_Mars:
            m_GM = GM_Mars;
            break;
        case E_Jupiter:
            m_GM = GM_Jupiter;
            break;
        case E_Saturn:
            m_GM = GM_Saturn;
            break;
        case E_Uranus:
            m_GM = GM_Uranus;
            break;
        case E_Neptune:
            m_GM = GM_Neptune;
            break;
        case E_Pluto:
            m_GM = GM_Pluto;
            break;
        case E_Moon:
            m_GM = GM_Moon;
            break;
        case E_Sun:
            m_GM = GM_Sun;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "ThirdBodyGravity: SolarSysStarType Unsupport ");
        }
    }

    void ThirdBodyGravity::SetCenterStar(SolarSysStarType centerStarType)
    {
        m_CenterStarType = centerStarType;
    }

    SolarSysStarType ThirdBodyGravity::GetThirdBodyStar()
    {
        return m_ThirdBodyStarType;
    }

    SolarSysStarType ThirdBodyGravity::GetCenterStar()
    {
        return m_CenterStarType;
    }

    Vector3d ThirdBodyGravity::AccelPointMassGravity(double Mjd_TT, const Vector3d &pos)
    {
        if (m_ThirdBodyStarType == E_NotDefinedStarType || m_CenterStarType == E_NotDefinedStarType)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "ThirdBodyGravity: StarType = E_NotDefinedStarType!");
        Vector3d relativePos;
        Vector3d bodyPos;
        //  Relative position vector of satellite w.r.t. point mass
        m_JPLEphemeris.GetJplEphemeris(Mjd_TT + MJDOffset, m_ThirdBodyStarType, m_CenterStarType, bodyPos);
        relativePos = pos - bodyPos;
        // Acceleration
        return  (-m_GM) * ( relativePos/pow(relativePos.norm(), 3) + bodyPos/pow(bodyPos.norm(), 3) );
    }

    /*************************************************
     * Class type: The class of The Atmospheric Drag. Perturbation
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    AtmosphericDrag::AtmosphericDrag()
    {
        m_AtmosphericModelType = AtmosphereModel::AtmosphereModelType::E_NotDefinedAtmosphereModel;
        m_pGeodeticSystem = nullptr;
        m_pAtmosphereModel = nullptr;
    }

    AtmosphericDrag::AtmosphericDrag(AtmosphereModel::AtmosphereModelType modelType, GeodeticCoordSystem *pGeodeticSystem)
    {
        m_AtmosphericModelType = modelType;
        m_pGeodeticSystem = pGeodeticSystem;
        m_pAtmosphereModel = new AtmosphereModel(modelType);
    }

    AtmosphericDrag::~AtmosphericDrag()
    {
        if (m_pAtmosphereModel != nullptr)
            delete m_pAtmosphereModel;
    }

    Vector3d AtmosphericDrag::AccelAtmosphericDrag(double Mjd_UTC, double Mjd_UT1, const Vector3d& pos, const Vector3d& vel, double area, double dragCoef, double mass,
                                                   double f107A, double f107, double ap[])
    {
        if (m_pAtmosphereModel == nullptr ||
                m_AtmosphericModelType == AtmosphereModel::AtmosphereModelType::E_NotDefinedAtmosphereModel)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphericDrag: m_AtmosphericModelType = E_NotDefinedAtmosphereModel!");

        // Earth angular velocity vector [rad/s]
        const Vector3d  omega( 0.0, 0.0, EarthAngVel);

        // Variables
        double      v_abs;
        Vector3d    r_ECF, v_ECF;
        Vector3d    r_TOD, v_TOD;
        Vector3d    v_rel, a_TOD;
        // Transformation matrix to ICRF/EME2000 system
        Matrix3d    J2000ToECFMtx = GeodeticCoordSystem::GetJ2000ToECFMtx(Mjd_UTC);
        Matrix3d    J2000ToTODMtx = GeodeticCoordSystem::GetJ2000ToTODMtx(Mjd_UTC);
        Matrix3d    TODToJ2000Mtx = J2000ToTODMtx.transpose();

        // Position and velocity in true-of-date system
        r_ECF = J2000ToECFMtx * pos;
        v_ECF = J2000ToECFMtx * vel;

        r_TOD = J2000ToTODMtx * pos;
        v_TOD = J2000ToTODMtx * vel;

        // Velocity relative to the Earth's atmosphere
        v_rel = v_TOD - omega.cross(r_TOD);
        v_abs = v_rel.norm();

        // Calculation lat lon and h
        GeodeticCoord lla = m_pGeodeticSystem->GetGeodeticCoord(r_ECF);
        // Atmospheric density due to AtmosphereModel::AtmosphereModelType
        double density = m_pAtmosphereModel->GetAtmosphereDensity( Mjd_UT1, lla.Altitude(), lla.Latitude() , lla.Longitude(), f107A, f107, ap);

        // Acceleration
        a_TOD = -0.5 * dragCoef * (area/mass) * density * v_abs * v_rel;

        return  TODToJ2000Mtx * a_TOD;
    }

    /*************************************************
     * Class type: The class of Solar Radiation Pressure
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    SolarRadPressure::SolarRadPressure()
    {

    }

    SolarRadPressure::~SolarRadPressure()
    {

    }

    Vector3d SolarRadPressure::AccelSolarRad(double Mjd_TT, const Vector3d &pos, double area, double solarCoef, double mass)
    {
        bool bIsInShadow = false;
        Vector3d relativePos;
        Vector3d sunPos, e_SunPos;
        Vector3d accel;
        accel.fill(0);

        // Relative position vector of spacecraft w.r.t. Sun
        m_JPLEphemeris.GetJplEphemeris(Mjd_TT + MJDOffset, E_Sun, E_Earth,sunPos);

        // Calculate the fractional illumination of a spacecraft in the
        //  vicinity of the Earth assuming a cylindrical shadow model
        e_SunPos = sunPos / sunPos.norm();      // Sun direction unit vector
        double s = pos.dot(e_SunPos);              // Projection of s/c position
        bIsInShadow = ( ( s > 0 || (pos-s*e_SunPos).norm() > EarthRadius ) ?  true : false );

        // Acceleration
        if (bIsInShadow == true)
            return accel;
        else
        {
            relativePos = pos - sunPos;
            accel = solarCoef * (area/mass) * SolarRadPreAtAU * (AU*AU) * relativePos / pow(relativePos.norm(), 3);
            return accel;
        }

    }




	
	
	
}
