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
* Date:2018-12-26
* Description:
*   SpObservation.cpp
*
*   Purpose:
*
*         Calculation of All Kinds of Observation Param.
*
*
*   Last modified:
*
*   2018-12-26  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpObservation.h"
#include "SpaceDSL/SpTarget.h"
#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

	/*************************************************
     * Class type: The class of SpaceDSL Target Observation
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     * The Observation Defined in East, North, Zenith Coordinate。
     *  This Class is Thread Safe!
    **************************************************/
    Observation::Observation()
    {
        m_Azimuth = 0;
        m_AzimuthRate = 0;
        m_Elevation = 0;
        m_ElevationRate = 0;
        m_RelativePosECF.fill(0);
        m_RelativeVelECF.fill(0);
    }

    Observation::Observation(const double azimuth, const double azimuthRate,
                             const double elevation, const double elevationRate,
                             const Vector3d &posECF, const Vector3d &velECF)
    {
        m_Azimuth = azimuth;
        m_AzimuthRate = azimuthRate;
        m_Elevation = elevation;
        m_ElevationRate = elevationRate;
        m_RelativePosECF = posECF;
        m_RelativeVelECF = velECF;
    }

    Observation::~Observation()
    {

    }

    double CalMaxObservationLat(const CartState &cart)
    {
        OrbitElem elem;
        CartToOrbitElem (cart, GM_Earth, elem);
        double apogee = elem.SMajAx()*(1.0+elem.Ecc());
        double iTemp = elem.I()*RadToDeg;
        if (iTemp >= 90.0)
            iTemp = 180.0-iTemp;
        return acos(EarthRadius/apogee) + iTemp*DegToRad;
    }

    bool CalObservation(const double Mjd, const CartState &cart, Target *target, Observation &result)
    {
        switch (target->GetTargetType())
        {
        case Target::E_NotDefindTargetType:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "CalObservation: TargetType = E_NotDefinedTargetType!");
        case Target::E_Facility:
            break;
        case Target::E_PointTarget:
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "CalObservation: Just Support Point Target!");
        }

        PointTarget *pointTarget = static_cast<PointTarget *>(target);

        if (CalMaxObservationLat(cart) < pointTarget->GetGeodeticCoord().Latitude())
            return false;

        GeodeticCoordSystem ECFSys(GeodeticCoordSystem::E_WGS84System);
        double Mjd_UT1 = Mjd + s_IERSService.GetValue(Mjd,"UT1-UTC")/DayToSec;

        GeodeticCoord LLA = pointTarget->GetGeodeticCoord();
        Vector3d objectPos = cart.Pos();
        Vector3d targetECFPos = ECFSys.GetPosition(LLA);

        Vector3d range = GMSToLTCMtx(LLA.Longitude(), LLA.Latitude()) *
                        ( GWHourAngMatrix(Mjd_UT1)*objectPos - targetECFPos ); // Topocentric position vector

        double azimuth;
        double elevation;
        GetAzEl(range, azimuth, elevation);
        result.SetAzimuth(azimuth);
        result.SetElevation(elevation);
        result.SetRelativePosECF(range);

        return true;
    }

    Observation CalSunObservation(const double Mjd, Target *target)
    {
        switch (target->GetTargetType())
        {
        case Target::E_NotDefindTargetType:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "CalObservation: TargetType = E_NotDefinedTargetType!");
        case Target::E_Facility:
            break;
        case Target::E_PointTarget:
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "CalObservation: Just Support Point Target!");
        }

        Observation result;

        PointTarget *pointTarget = static_cast<PointTarget *>(target);

        GeodeticCoordSystem ECFSys(GeodeticCoordSystem::E_WGS84System);
        double Mjd_UT1 = Mjd + s_IERSService.GetValue(Mjd,"UT1-UTC")/DayToSec;

        // Find sun position
        Vector3d        sunPos;
        Vector3d        sunVel;
        s_JPLEphemeris.GetJplEphemeris(Mjd + MJDOffset, SolarSysStarType::E_Sun, SolarSysStarType::E_Earth, sunPos);

        GeodeticCoord LLA = pointTarget->GetGeodeticCoord();
        Vector3d targetECFPos = ECFSys.GetPosition(LLA);

        Vector3d range = GMSToLTCMtx(LLA.Longitude(), LLA.Latitude()) *
                        ( GWHourAngMatrix(Mjd_UT1)*sunPos - targetECFPos ); // Topocentric position vector

        double azimuth;
        double elevation;
        GetAzEl(range, azimuth, elevation);
        result.SetAzimuth(azimuth);
        result.SetElevation(elevation);
        result.SetRelativePosECF(range);

        return result;
    }




}
