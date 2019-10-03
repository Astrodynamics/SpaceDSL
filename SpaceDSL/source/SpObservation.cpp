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

    double FMod2p(double x)
    {
        /* Returns mod 2PI of argument */

        double ret_val = fmod(x, 2*PI);

        if (ret_val < 0.0)
            ret_val += (2*PI);

        return ret_val;
    }

    double ThetaG_JD(double jd)
    {
        /* Reference:  The 1992 Astronomical Almanac, page B6. */

        double UT, TU, GMST;

        double dummy;
        UT=modf(jd+0.5, &dummy);
        jd = jd - UT;
        TU=(jd-2451545.0)/36525;
        GMST=24110.54841+TU*(8640184.812866+TU*(0.093104-TU*6.2E-6));
        GMST=fmod(GMST+DayToSec*EarthRPSDay*UT,DayToSec);

        return (2*PI*GMST/DayToSec);
    }

    double Sqr(double arg)
    {
        /* Returns square of a double */
        return (arg*arg);
    }

    bool CalObservationAll(const double Mjd, const CartState &cart, Target *target, Observation &result)
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
        /* The procedures Calculate_Obs and Calculate_RADec calculate         */
        /* the *topocentric* coordinates of the object with ECI position,     */
        /* {pos}, and velocity, {vel}, from location {geodetic} at {time}.    */
        /* The {obs_set} returned for Calculate_Obs consists of azimuth,      */
        /* elevation, range, and range rate (in that order) with units of     */
        /* radians, radians, kilometers, and kilometers/second, respectively. */
        /* The WGS '72 geoid is used and the effect of atmospheric refraction */
        /* (under standard temperature and pressure) is incorporated into the */
        /* elevation calculation; the effect of atmospheric refraction on     */
        /* range and range rate has not yet been quantified.                  */

        /* The {obs_set} for Calculate_RADec consists of right ascension and  */
        /* declination (in that order) in radians.  Again, calculations are   */
        /* based on *topocentric* position using the WGS '72 geoid and        */
        /* incorporating atmospheric refraction.                              */


        Vector3d obs_pos;
        Vector3d obs_vel;
        Vector3d range;
        Vector3d rgvel;

        double lat = pointTarget->GetGeodeticCoord().Latitude();
        double lon = pointTarget->GetGeodeticCoord().Longitude();
        double alt = pointTarget->GetGeodeticCoord().Altitude() / 1000.0;
        double theta = 0.0;

        double c, sq, achcp;
        double jd =Mjd + MJDOffset;
        theta=FMod2p(ThetaG_JD(jd)+lon); /* LMST */

        c=1/sqrt(1+EarthFlatFact*(EarthFlatFact-2)*Sqr(sin(lat)));
        sq=Sqr(1-EarthFlatFact)*c;
        achcp=(EarthRadius*c+alt)*cos(lat);
        obs_pos[0] = (achcp*cos(theta)); /* kilometers */
        obs_pos[1] = (achcp*sin(theta));
        obs_pos[2] = ((EarthRadius*sq+alt)*sin(lat));
        obs_vel[0] = (-EarthAngVel*obs_pos[1]); /* kilometers/second */
        obs_vel[1] = (EarthAngVel*obs_pos[0]);
        obs_vel[2] = (0);
        //Calculate_User_PosVel(time, &geodetic, obs_pos, obs_vel);

        //vec3_sub(pos, obs_pos, range);
        //vec3_sub(vel, obs_vel, rgvel);
        range = cart.Pos() - obs_pos;
        rgvel = cart.Vel() - obs_vel;


        double range_length = range.norm();
        double range_rate_length = range.dot(rgvel) / range_length;

        double theta_dot = 2*PI*EarthRPSDay/DayToSec;
        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);

        double top_s = sin_lat*cos_theta*range[0] + sin_lat*sin_theta*range[1] - cos_lat*range[2];
        double top_e = -sin_theta*range[0] + cos_theta*range[1];
        double top_z = cos_lat*cos_theta*range[0] + cos_lat*sin_theta*range[1] + sin_lat*range[2];


        double top_s_dot = sin_lat*(cos_theta*rgvel[0] - sin_theta*range[0]*theta_dot) +
                            sin_lat*(sin_theta*rgvel[1] + cos_theta*range[1]*theta_dot) -
                            cos_lat*rgvel[2];
        double top_e_dot = - (sin_theta*rgvel[0] + cos_theta*range[0]*theta_dot) +
                            (cos_theta*rgvel[1] - sin_theta*range[1]*theta_dot);

        double top_z_dot = cos_lat * ( cos_theta*(rgvel[0] + range[1]*theta_dot) +
                                    sin_theta*(rgvel[1] - range[0]*theta_dot) ) +
                                    sin_lat*rgvel[2];

        // Azimut
        double y = -top_e / top_s;
        double az = atan(-top_e / top_s);

        if (top_s > 0.0) az = az + PI;
        if (az < 0.0) az = az + 2*PI;

        // Azimut rate
        double y_dot = - (top_e_dot*top_s - top_s_dot*top_e) / (top_s*top_s);
        double az_dot = y_dot / (1 + y*y);

        // Elevation
        double x = top_z / range_length;
        double el = asin(x < -1.0 ? -1.0 : (x > 1.0 ? 1.0 : x));

        // Elevation rate
        double x_dot = (top_z_dot*range_length - range_rate_length*top_z) / (range_length * range_length);
        double el_dot = x_dot / sqrt( 1 - x*x );

        result.SetAzimuth(az);
        result.SetAzimuthRate(az_dot);
        result.SetElevation(el);
        result.SetElevationRate(el_dot);
        result.SetRelativePosECF(range);
        result.SetRelativeVelECF(rgvel);

        return true;
    }




}
