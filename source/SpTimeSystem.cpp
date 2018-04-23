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
*   SpTimeSystem.cpp
*
*   Purpose:
*
*       Time and date computation
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include "SpaceDSL/SpTimeSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"

#include "SpaceDSL/gsoap/soapH.h"
#include "SpaceDSL/gsoap/soapBinding.nsmap"

#include <fstream>

namespace SpaceDSL{

    /*************************************************
     * Class type: Gregorian Calendar Time
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Defined Gregorian Calendar Time Base Class
    **************************************************/
    CalendarTime::CalendarTime()
    {
        this->m_Year = 0;
        this->m_Mon = 0;
        this->m_Day = 0;
        this->m_Hour = 0;
        this->m_Min = 0;
        this->m_Sec = 0;
        this->m_pTimeStr = NULL;
    }

    CalendarTime::~CalendarTime()
    {
        if (m_pTimeStr != NULL)
            free (m_pTimeStr);
    }

    bool CalendarTime::operator==(const CalendarTime &time) const
    {
        if (this->m_Year != time.Year())
        {
            return false;
        }
        else if (this->m_Mon != time.Mon())
        {
            return false;
        }
        else if (this->m_Day != time.Day())
        {
            return false;
        }
        else if (this->m_Hour != time.Hour())
        {
            return false;
        }
        else if (this->m_Min != time.Min())
        {
            return false;
        }
        else if (this->m_Sec != time.Sec())
        {
            return false;
        }
        else
            return true;
    }

    bool CalendarTime::operator!=(const CalendarTime &time) const
    {
        if (this->m_Year != time.Year())
        {
            return true;
        }
        else if (this->m_Mon != time.Mon())
        {
            return true;
        }
        else if (this->m_Day != time.Day())
        {
            return true;
        }
        else if (this->m_Hour != time.Hour())
        {
            return true;
        }
        else if (this->m_Min != time.Min())
        {
            return true;
        }
        else if (this->m_Sec != time.Sec())
        {
            return true;
        }
        else
            return false;
    }

    bool CalendarTime::operator>(const CalendarTime &time) const
    {
        if (this->m_Year < time.Year())
        {
            return false;
        }
        else if (this->m_Mon < time.Mon())
        {
            return false;
        }
        else if (this->m_Day < time.Day())
        {
            return false;
        }
        else if (this->m_Hour < time.Hour())
        {
            return false;
        }
        else if (this->m_Min < time.Min())
        {
            return false;
        }
        else if (this->m_Sec < time.Sec())
        {
            return false;
        }
        else if ( (*this) == time )
        {
            return false;
        }
        else
            return true;
    }

    bool CalendarTime::operator>=(const CalendarTime &time) const
    {
        if (this->m_Year < time.Year())
        {
            return false;
        }
        else if (this->m_Mon < time.Mon())
        {
            return false;
        }
        else if (this->m_Day < time.Day())
        {
            return false;
        }
        else if (this->m_Hour < time.Hour())
        {
            return false;
        }
        else if (this->m_Min < time.Min())
        {
            return false;
        }
        else if (this->m_Sec < time.Sec())
        {
            return false;
        }
        else
            return true;
    }

    bool CalendarTime::operator<(const CalendarTime &time) const
    {
        if (this->m_Year > time.Year())
        {
            return false;
        }
        else if (this->m_Mon > time.Mon())
        {
            return false;
        }
        else if (this->m_Day > time.Day())
        {
            return false;
        }
        else if (this->m_Hour > time.Hour())
        {
            return false;
        }
        else if (this->m_Min > time.Min())
        {
            return false;
        }
        else if (this->m_Sec > time.Sec())
        {
            return false;
        }
        else if ( (*this) == time )
        {
            return false;
        }
        else
            return true;
    }

    bool CalendarTime::operator<=(const CalendarTime &time) const
    {
        if (this->m_Year > time.Year())
        {
            return false;
        }
        else if (this->m_Mon > time.Mon())
        {
            return false;
        }
        else if (this->m_Day > time.Day())
        {
            return false;
        }
        else if (this->m_Hour > time.Hour())
        {
            return false;
        }
        else if (this->m_Min > time.Min())
        {
            return false;
        }
        else if (this->m_Sec > time.Sec())
        {
            return false;
        }
        else
            return true;
    }

    void CalendarTime::ToCharArray(char *&pTimeStr)
    {
        this->FillTimeStr();
        pTimeStr = m_pTimeStr;
    }

    void CalendarTime::FillTimeStr()
    {
        // char form "yyyy-mm-dd HH:MM:SS"
        string tempStr = to_string(this->m_Year) + "-";
        tempStr += ( to_string(this->m_Mon) + "-" );
        tempStr += ( to_string(this->m_Day) + " " );
        tempStr += ( to_string(this->m_Hour) + ":" );
        tempStr += ( to_string(this->m_Min) + ":" );
        tempStr += ( to_string(this->m_Sec));
        if (m_pTimeStr != NULL)
            free (m_pTimeStr);
        int len = int(tempStr.size()) + 1;
        m_pTimeStr = (char *)malloc(len * sizeof(m_pTimeStr[0]));

        strcpy_s(m_pTimeStr, len, tempStr.c_str());
    }
    /*************************************************
     * Class type: Gregorian Calendar Time
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Defined Gregorian Calendar Time Property and Behavior
     *  This Class is Thread Safe!
    **************************************************/
    UTCCalTime::UTCCalTime()
    {
        this->m_Year = 0;
        this->m_Mon = 0;
        this->m_Day = 0;
        this->m_Hour = 0;
        this->m_Min = 0;
        this->m_Sec = 0;
    }

    UTCCalTime::UTCCalTime(int year, int mon, int day, int hour, int min, double sec)
    {
        this->m_Year = year;
        this->m_Mon = mon;
        this->m_Day = day;
        this->m_Hour = hour;
        this->m_Min = min;
        this->m_Sec = sec;
    }

    UTCCalTime::~UTCCalTime()
    {

    }

    /*************************************************
     * Class type: UT1 Time
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Defined UT1 Time Property and Behavior
    **************************************************/
    UT1CalTime::UT1CalTime()
    {
        this->m_Year = 0;
        this->m_Mon = 0;
        this->m_Day = 0;
        this->m_Hour = 0;
        this->m_Min = 0;
        this->m_Sec = 0;
    }

    UT1CalTime::UT1CalTime(int year, int mon, int day, int hour, int min, double sec)
    {
        this->m_Year = year;
        this->m_Mon = mon;
        this->m_Day = day;
        this->m_Hour = hour;
        this->m_Min = min;
        this->m_Sec = sec;
    }

    UT1CalTime::~UT1CalTime()
    {

    }

    //====================================================================
    //
    // Grouping: Date format conversions.
    //
    //====================================================================

    double CalendarTimeToMjd ( int year, int month, int day, int hour, int min, double sec )
    {
        // Variables
        long    MjdMidnight;
        double  FracOfDay;
        int     b;

        if (month <= 2)
        {
            month+=12;
            --year;
        }

        if ( (10000L*year+100L*month+day) <= 15821004L )
            b = -2 + ((year+4716)/4) - 1179;     // Julian calendar
        else
            b = (year/400)-(year/100)+(year/4);  // Gregorian calendar

        MjdMidnight = 365L*year - 679004L + b + int(30.6001*(month+1)) + day;
        FracOfDay   = (hour+min/60.0+sec/3600.0) / 24.0;

        return MjdMidnight + FracOfDay;
    }

    double CalendarTimeToMjd(CalendarTime &time)
    {
        int year = time.Year();
        int month = time.Mon();
        int day = time.Day();
        int hour = time.Hour();
        int min = time.Min();
        double sec = time.Sec();
        // Variables
        long    MjdMidnight;
        double  FracOfDay;
        int     b;

        if (month <= 2)
        {
            month+=12;
            --year;
        }

        if ( (10000L*year+100L*month+day) <= 15821004L )
            b = -2 + ((year+4716)/4) - 1179;     // Julian calendar
        else
            b = (year/400)-(year/100)+(year/4);  // Gregorian calendar

        MjdMidnight = 365L*year - 679004L + b + int(30.6001*(month+1)) + day;
        FracOfDay   = (hour+min/60.0+sec/3600.0) / 24.0;

        return MjdMidnight + FracOfDay;
    }

    void MjdToCalendarTime ( double Mjd,
                        int& year, int& month, int& day,
                        int& hour, int& min, double& sec )
    {
        // Variables
        long    a,b,c,d,e,f;
        double  hours,x;

        // Convert Julian day number to calendar date
        a = long(Mjd+2400001.0);

        if ( a < 2299161 )
        {
            // Julian calendar
            b = 0;
            c = a + 1524;
        }
        else
        {   // Gregorian calendar
            b = long((a-1867216.25)/36524.25);
            c = a +  b - (b/4) + 1525;
        }

        d     = long ( (c-122.1)/365.25 );
        e     = 365*d + d/4;
        f     = long ( (c-e)/30.6001 );

        day   = c - e - int(30.6001*f);
        month = f - 1 - 12*(f/14);
        year  = d - 4715 - ((7+month)/10);

        hours = 24.0*(Mjd-floor(Mjd));

        hour = int(hours);
        x = (hours-hour)*60.0; min = int(x);  sec = (x-min)*60.0;

    }

    void MjdToCalendarTime(double Mjd, CalendarTime &time)
    {
        int year, month, day, hour, min;
        double sec;
        // Variables
        long    a,b,c,d,e,f;
        double  Hours,x;

        // Convert Julian day number to calendar date
        a = long(Mjd+2400001.0);

        if ( a < 2299161 )
        {
            // Julian calendar
            b = 0;
            c = a + 1524;
        }
        else
        {   // Gregorian calendar
            b = long((a-1867216.25)/36524.25);
            c = a +  b - (b/4) + 1525;
        }

        d     = long ( (c-122.1)/365.25 );
        e     = 365*d + d/4;
        f     = long ( (c-e)/30.6001 );

        day   = c - e - int(30.6001*f);
        month = f - 1 - 12*(f/14);
        year  = d - 4715 - ((7+month)/10);

        Hours = 24.0*(Mjd-floor(Mjd));

        hour = int(Hours);
        x = (Hours-hour)*60.0; min = int(x);  sec = (x-min)*60.0;

        time.SetYear(year);
        time.SetMon(month);
        time.SetDay(day);
        time.SetHour(hour);
        time.SetMin(min);
        time.SetSec(min);
    }

    /*************************************************
     * Class type: IERS
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Get IERS Data through a File or Web Service
    **************************************************/
    IERSService::IERSService()
    {
        #ifdef WITH_OPENSSL
            m_bIsUseWebService = true;
        #else
            m_bIsUseWebService = false;
        #endif
    }

    IERSService::~IERSService()
    {

    }

    void IERSService::EnableWebService()
    {
        #ifdef WITH_OPENSSL
            m_bIsUseWebService = true;
        #endif
    }

    void IERSService::DisableWebService()
    {
        #ifdef WITH_OPENSSL
            m_bIsUseWebService = false;
        #endif
    }

    double IERSService::GetValue(double Mjd_UTC, char *param, char *series)
    {
        if (m_bIsUseWebService == true) //Use IERS Web Service
        {
            char *result = NULL;

            UTCCalTime caltime;
            MjdToCalendarTime(Mjd_UTC, caltime);
            char *datetime = NULL;
            caltime.ToCharArray(datetime);

            string tempStr;
            tempStr = to_string(Mjd_UTC);
            char *mjd = const_cast<char*> (tempStr.c_str());

            struct soap readIERS;
            soap_init(&readIERS);
            #ifdef WITH_OPENSSL
                soap_ssl_init();
                if (soap_ssl_client_context(&readIERS,
                    SOAP_SSL_NO_AUTHENTICATION, /* use SOAP_SSL_DEFAULT in production code */
                    NULL,       /* keyfile: required only when client must authenticate to
                                server (see SSL docs on how to obtain this file) */
                    NULL,       /* password to read the keyfile */
                    NULL,      /* optional cacert file to store trusted certificates */
                    NULL,      /* optional capath to directory with trusted certificates */
                    NULL      /* if randfile!=NULL: use a file with random data to seed randomness */
                ))
                {
                    throw SPException(__FILE__, __FUNCTION__, __LINE__, "soap_ssl_client_context() Running error");
                }
            #endif
            soap_set_namespaces(&readIERS, namespaces);

            if (series == NULL)
            {
                soap_call_ns2__getTimescale(&readIERS, NULL, NULL, param, datetime, result);
            }
            else
                soap_call_ns1__readEOP(&readIERS, NULL, NULL, param, series, mjd, result);

            string resultStr = result;
            return stod(resultStr);
        }
        else //Read File
        {
            string resultStr;
            return stod(resultStr);
        }

    }

}

