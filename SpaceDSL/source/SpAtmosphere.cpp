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
*   SpAtmosphere.cpp
*
*   Purpose:
*
*       Atmosphere Model of Earth
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpAtmosphere.h"
#include "SpaceDSL/SpInterpolation.h"
#include "SpaceDSL/SpTimeSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"

#include "SpaceDSL/nrlmsise00/nrlmsise00.h"

namespace SpaceDSL {

    /*************************************************
     * Class type: The class of Atmospheric Model
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    AtmosphereModel::AtmosphereModel()
    {
        m_AtmosphericModelType = E_NotDefinedAtmosphereModel;
    }

    AtmosphereModel::AtmosphereModel(AtmosphereModelType modelType)
    {
        m_AtmosphericModelType = modelType;
    }

    AtmosphereModel::~AtmosphereModel()
    {

    }

    void AtmosphereModel::SetAtmosphereModelType(AtmosphereModel::AtmosphereModelType modelType)
    {
        m_AtmosphericModelType = modelType;
    }

    AtmosphereModel::AtmosphereModelType AtmosphereModel::GetAtmosphereModelType()
    {
        if (m_AtmosphericModelType == E_NotDefinedAtmosphereModel)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");

        return m_AtmosphericModelType;
    }

    double AtmosphereModel::GetAtmosphereTemperature(double Mjd_UT1, double altitude, double latitude, double longitude,
                                                     double f107A, double f107, double ap[], bool useDailyAp)
    {
        switch (m_AtmosphericModelType) {
        case E_NotDefinedAtmosphereModel:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
            break;
        case E_NRLMSISE00Atmosphere:
            altitude /= 1000.0;//m to km
            return GetNRLMSISE2000Temperature(Mjd_UT1, altitude, latitude, longitude,
                                              f107A, f107, ap, useDailyAp);
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
        }
        return 0;
    }

    double AtmosphereModel::GetAtmospherePressure(double Mjd_UT1, double altitude, double latitude, double longitude,
                                                  double f107A, double f107, double ap[], bool useDailyAp)
    {
        switch (m_AtmosphericModelType) {
        case E_NotDefinedAtmosphereModel:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
        case E_NRLMSISE00Atmosphere:
            return 0;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
        }
        return 0;
    }

    double AtmosphereModel::GetAtmosphereDensity(double Mjd_UT1, double altitude, double latitude, double longitude,
                                                 double f107A, double f107, double ap[], bool useDailyAp)
    {
        switch (m_AtmosphericModelType) {
        case E_NotDefinedAtmosphereModel:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
            break;
        case E_NRLMSISE00Atmosphere:
            altitude /= 1000.0;//m to km
            return GetNRLMSISE2000Density(Mjd_UT1, altitude, latitude, longitude,
                                          f107A, f107, ap, useDailyAp);
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
        }
        return 0;
    }

    ///===================================================
    /// NRLMSIS-00 Empirical Atmosphere Model Calculator
    /// Reencapsulate Dominik Brodowski The C version.
    /// @Author	Niu Zhiyong
    /// @Date	2018-06-05
    ///===================================================
    double AtmosphereModel::GetNRLMSISE2000Density(double Mjd_UT1, double altitude, double latitude, double longitude,
                                                   double f107A, double f107, double ap[], bool useDailyAp)
    {
        NRLMSISE00 nrlmsise;
        struct NRLMSISE00::nrlmsise_output output;
        struct NRLMSISE00::nrlmsise_input input;
        struct NRLMSISE00::nrlmsise_flags flags;
        struct NRLMSISE00::ap_array aph;

        /* input values */
        if (ap == nullptr)
        {
            for (int i = 0; i < 7; i++)
            {
                aph.a[i] = ap[i];
            }
        }
        else
        {
            for (int i = 0; i < 7; i++)
            {
                aph.a[i] = ap[i];
            }
        }


        for (int i = 0; i < 24; i++)
        {
            flags.switches[i] = 1;
        }

        if (useDailyAp == false)
            flags.switches[9] = -1;

        //Neglecting deviation between Mjd_TT and Mjd_UTC
        UTCCalTime time;
        UTCCalTime time0;
        MjdToCalendarTime(Mjd_UT1, time);
        time0 = time;
        time0.SetMon(1);
        time0.SetDay(1);
        time0.SetHour(0);
        time0.SetMin(0);
        time0.SetSec(0);
        double Mjd_UT10 = CalendarTimeToMjd(time0);

        input.year  = time.Year();                                      // without effect
        input.doy   = int(Mjd_UT1 - Mjd_UT10) + 1;
        input.sec   = time.Hour()*3600 + time.Min()*60 + time.Sec();
        input.alt   = altitude;                                         //km
        input.g_lat = latitude*RadToDeg;                                //degree
        input.g_long= longitude*RadToDeg;                               //degree
        input.lst   = input.sec/3600 + input.g_long/15;                 //hour
        input.f107A = f107A;
        input.f107  = f107;
        input.ap    = aph.a[0];
        input.ap_a  = &aph;
        nrlmsise.gtd7d(&input, &flags, &output);

        return output.d[5];
    }

    double AtmosphereModel::GetNRLMSISE2000Temperature(double Mjd_UT1, double altitude, double latitude, double longitude,
                                                       double f107A, double f107, double ap[], bool useDailyAp)
    {
        NRLMSISE00 nrlmsise;
        struct NRLMSISE00::nrlmsise_output output;
        struct NRLMSISE00::nrlmsise_input input;
        struct NRLMSISE00::nrlmsise_flags flags;
        struct NRLMSISE00::ap_array aph;

        /* input values */
        if (ap == nullptr)
        {
            aph.a[0] = 14.9186481659685;
            for (int i=1;i<7;i++)
            {
                aph.a[i]=0;
            }
        }
        else
        {
            for (int i=0;i<7;i++)
            {
                aph.a[i]=ap[i];
            }
        }

        for (int i=0;i<24;i++)
        {
            flags.switches[i]=1;
        }

        if (useDailyAp == false)
            flags.switches[9] = -1;

        //Neglecting deviation between Mjd_TT and Mjd_UTC
        UTCCalTime time;
        UTCCalTime time0;
        MjdToCalendarTime(Mjd_UT1, time);
        time0 = time;
        time0.SetMon(1);
        time0.SetDay(1);
        time0.SetHour(0);
        time0.SetMin(0);
        time0.SetSec(0);
        double Mjd_TT0 = CalendarTimeToMjd(time0);

        input.year  = time.Year();                                      // without effect
        input.doy   = int(Mjd_UT1 - Mjd_TT0);
        input.sec   = time.Hour()*3600 + time.Min()*60 + time.Sec();
        input.alt   = altitude;                                         //km
        input.g_lat = latitude*180/PI;                                  //degree
        input.g_long= longitude*180/PI;                                 //degree
        input.lst   = input.sec/3600 + input.g_long/15;                 //hour
        input.f107A = f107A;
        input.f107  = f107;
        input.ap    = aph.a[0];
        input.ap_a  = &aph;
        nrlmsise.gtd7d(&input, &flags, &output);

        return output.t[1];
    }

	
	
}
