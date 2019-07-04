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
#include "SpaceDSL/SpInterpolation.h"
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
        this->m_pTimeStr = nullptr;
    }

    CalendarTime::~CalendarTime()
    {
        if (m_pTimeStr != nullptr)
            free (m_pTimeStr);
    }

    void CalendarTime::operator =(const CalendarTime &time)
    {
        this->m_Year = time.m_Year;
        this->m_Mon = time.m_Mon;
        this->m_Day = time.m_Day;
        this->m_Hour = time.m_Hour;
        this->m_Min = time.m_Min;
        this->m_Sec = time.m_Sec;
        this->m_pTimeStr = time.m_pTimeStr;
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
        else if (fabs(this->m_Sec - time.Sec()) > EPS)
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
        else if (fabs(this->m_Sec - time.Sec()) > EPS)
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

    double CalendarTime::operator -(const CalendarTime &time) const
    {
        double Mjd0 = CalendarTimeToMjd(*this);
        double Mjd1 = CalendarTimeToMjd(time);

        return (Mjd0 - Mjd1)*DayToSec;
    }

    void CalendarTime::ToCharArray(char *&pTimeStr)
    {
        this->FillTimeStr();
        pTimeStr = m_pTimeStr;
    }

    string CalendarTime::ToString()
    {
        // char form "yyyy-mm-dd HH:MM:SS"
        string tempStr = to_string(this->m_Year) + "-";
        tempStr += ( to_string(this->m_Mon) + "-" );
        tempStr += ( to_string(this->m_Day) + " " );
        tempStr += ( to_string(this->m_Hour) + ":" );
        tempStr += ( to_string(this->m_Min) + ":" );
        tempStr += ( to_string(this->m_Sec));

        return tempStr;
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
        if (m_pTimeStr != nullptr)
            free (m_pTimeStr);
        int len = int(tempStr.size()) + 1;
        m_pTimeStr = static_cast<char *>(malloc(static_cast<unsigned long long>(len) * sizeof(m_pTimeStr[0])));

#if defined(_WIN32)
        strcpy_s(m_pTimeStr, static_cast<unsigned long long>(len), tempStr.c_str());
#else
        strncpy(m_pTimeStr, tempStr.c_str(), len);
#endif
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

    double CalendarTimeToMjd(const CalendarTime &time)
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
        time.SetSec(sec);
    }

    /*************************************************
     * Class type: IERS
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Get IERS Data through a File or Web Service
    **************************************************/
    double IERSService::TT_TAI  = 32.184;
    double IERSService::GPS_TAI = -19.0;
    IERSService::IERSService()
    {
        #ifdef WITH_OPENSSL
            m_bIsUseWebService = true;
            m_EOPData = nullptr;
            m_LeapsecondData = nullptr;
            m_EOPDataLine = 0;
            m_LeapsecondDataLine = 0;
        #else
            m_bIsUseWebService = false;
            ifstream file_EOP, file_leapsecond;
            // Read EOP_2000A.dat
            int line = 0;
            file_EOP.open("./astrodata/IERS/EOP_2000A.dat");
            if (file_EOP.is_open() == false)
            {
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "EOP_2000A.dat File is Invalid!");
            }

            while ( !file_EOP.eof() )
            {
                char buffer[68];
                file_EOP.getline(buffer, 68);
                ++line;
            }

            m_EOPData = new double *[static_cast<unsigned long long>(line)];
            for (int i = 0; i < line; ++i)
            {
                m_EOPData[i] = new double[7];
            }
            m_EOPDataLine = line;

            file_EOP.seekg(0,ios::beg);
            line = 0;
            while ( !file_EOP.eof() )
            {
                char buffer[68];
                file_EOP.getline(buffer, 68);

                string bufferstr = buffer;
                m_EOPData[line][0] = stod(bufferstr.substr(0,8));          //MJD
                m_EOPData[line][1] = stod(bufferstr.substr(8,10));         //x_pole
                m_EOPData[line][2] = stod(bufferstr.substr(18,9));         //sigma_x_pole
                m_EOPData[line][3] = stod(bufferstr.substr(27,10));        //y_pole
                m_EOPData[line][4] = stod(bufferstr.substr(37,9));         //sigma_y_pole
                m_EOPData[line][5] = stod(bufferstr.substr(46,11));        //UT1-UTC
                m_EOPData[line][6] = stod(bufferstr.substr(57,10));        //sigma_UT1-UTC

                ++line;
            }
            file_EOP.close();
            // Read leapseconds.dat
            line = 0;
            file_leapsecond.open("./astrodata/IERS/leapseconds.dat");
            if (file_leapsecond.is_open() == false)
            {
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "leapsecond.dat File is Invalid!");
            }

            while ( !file_leapsecond.eof() )
            {
                char buffer[37];
                file_leapsecond.getline(buffer, 37);
                ++line;
            }

            m_LeapsecondDataLine = line;
            m_LeapsecondData = new double *[static_cast<unsigned long long>(line)];
            for (int i = 0; i < line; ++i)
            {
                m_LeapsecondData[i] = new double[2];
            }

            file_leapsecond.seekg(0,ios::beg);
            line = 0;
            while ( !file_leapsecond.eof() )
            {
                char buffer[37];
                file_leapsecond.getline(buffer, 37);

                string bufferstr = buffer;
                m_LeapsecondData[line][0] = stod(bufferstr.substr(0,10))/86400 + 15020;    //MJD = NTP /86400 + 15020
                m_LeapsecondData[line][1] = stod(bufferstr.substr(10,14));                 //leapseconds
                ++line;
            }
            file_leapsecond.close();

        #endif
    }

    IERSService::~IERSService()
    {
        if (m_EOPData != nullptr)
        {
            for (int i = 0; i < m_EOPDataLine; ++i)
            {
                delete[]  m_EOPData[i];
            }
            delete[] m_EOPData;
        }


        if (m_LeapsecondData != nullptr)
        {
            for (int i = 0; i < m_LeapsecondDataLine; ++i)
            {
                delete[]  m_LeapsecondData[i];
            }
            delete[] m_LeapsecondData;
        }
    }

    #ifdef WITH_OPENSSL
    void IERSService::EnableWebService()
    {

            m_bIsUseWebService = true;

    }

    void IERSService::DisableWebService()
    {

            m_bIsUseWebService = false;

    }
    #endif

    double IERSService::GetValue(double Mjd_UTC, const char *param, const char *series)
    {
        if (m_bIsUseWebService == true) //Use IERS Web Service
        {
            char *result = nullptr;

            UTCCalTime caltime;
            MjdToCalendarTime(Mjd_UTC, caltime);
            char *datetime = nullptr;
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
                    nullptr,       /* keyfile: required only when client must authenticate to
                                server (see SSL docs on how to obtain this file) */
                    nullptr,       /* password to read the keyfile */
                    nullptr,      /* optional cacert file to store trusted certificates */
                    nullptr,      /* optional capath to directory with trusted certificates */
                    nullptr      /* if randfile!=nullptr: use a file with random data to seed randomness */
                ))
                {
                    throw SPException(__FILE__, __FUNCTION__, __LINE__, "soap_ssl_client_context() Running error");
                }
            #endif
            soap_set_namespaces(&readIERS, namespaces);

            if (series == nullptr)
            {
                soap_call_ns2__getTimescale(&readIERS, nullptr, nullptr, const_cast<char *>(param), datetime, result);
            }
            else
                soap_call_ns1__readEOP(&readIERS, nullptr, nullptr, const_cast<char *>(param), const_cast<char *>(series), mjd, result);

            if (result == nullptr)
            {
                return 0;
            }
            else
            {
                string resultStr = result;
                return stod(resultStr);
            }
        }
        else //Read File
        {
            const char *x_poleStr = "x_pole";
            const char *sigma_x_poleStr = "sigma_x_pole";
            const char *y_poleStr = "y_pole";
            const char *sigma_y_poleStr = "sigma_y_pole";
            const char *UT1_UTCStr = "UT1-UTC";
            const char *sigma_UT1_UTCStr = "sigma_UT1-UTC";
            const char *leapsecondStr = "leapseconds";

            if ( strcmp(param, leapsecondStr) != 0
                 && (Mjd_UTC < m_EOPData[0][0] || Mjd_UTC > m_EOPData[m_EOPDataLine-1][0]) )
            {
                return 0;
            }
            if (strcmp(param, leapsecondStr) == 0 && Mjd_UTC > m_LeapsecondData[m_LeapsecondDataLine-1][0])
            {
                return m_LeapsecondData[m_LeapsecondDataLine-1][1];
            }
            if (strcmp(param, leapsecondStr) == 0 && Mjd_UTC < m_LeapsecondData[0][0])
            {
                return 0;
            }

            if (strcmp(param, leapsecondStr) == 0)
            {
                int line = 0;
                for (line = 1; line < m_LeapsecondDataLine; ++line)
                {
                    if (Mjd_UTC >= m_LeapsecondData[line-1][0] && Mjd_UTC <= m_LeapsecondData[line][0])
                    {
                        return m_LeapsecondData[line-1][1];
                    }
                }
            }
            else
            {
                int line = 0;
                for (line = 1; line < m_EOPDataLine; ++line)
                {
                    if (Mjd_UTC >= m_EOPData[line-1][0] && Mjd_UTC <= m_EOPData[line][0])
                    {
                        break;
                    }
                }

                if (strcmp(param, x_poleStr) == 0)
                {
                    return (m_EOPData[line-1][1] + (Mjd_UTC - m_EOPData[line-1][0])
                            *(m_EOPData[line][1] - m_EOPData[line-1][1])/(m_EOPData[line][0] - m_EOPData[line-1][0]));
                }
                else if (strcmp(param, sigma_x_poleStr) == 0)
                {
                    return (m_EOPData[line-1][2] + (Mjd_UTC - m_EOPData[line-1][0])
                            *(m_EOPData[line][2] - m_EOPData[line-1][2])/(m_EOPData[line][0] - m_EOPData[line-1][0]));
                }
                else if (strcmp(param, y_poleStr) == 0)
                {
                    return (m_EOPData[line-1][3] + (Mjd_UTC - m_EOPData[line-1][0])
                            *(m_EOPData[line][3] - m_EOPData[line-1][3])/(m_EOPData[line][0] - m_EOPData[line-1][0]));
                }
                else if (strcmp(param, sigma_y_poleStr) == 0)
                {
                    return (m_EOPData[line-1][4] + (Mjd_UTC - m_EOPData[line-1][0])
                            *(m_EOPData[line][4] - m_EOPData[line-1][4])/(m_EOPData[line][0] - m_EOPData[line-1][0]));
                }
                else if (strcmp(param, UT1_UTCStr) == 0)
                {
                    return (m_EOPData[line-1][5] + (Mjd_UTC - m_EOPData[line-1][0])
                            *(m_EOPData[line][5] - m_EOPData[line-1][5])/(m_EOPData[line][0] - m_EOPData[line-1][0]));
                }
                else if (strcmp(param, sigma_UT1_UTCStr) == 0)
                {
                    return (m_EOPData[line-1][6] + (Mjd_UTC - m_EOPData[line-1][0])
                            *(m_EOPData[line][6] - m_EOPData[line-1][6])/(m_EOPData[line][0] - m_EOPData[line-1][0]));
                }
                else
                {
                    return 0;
                }

            }

        }

        return 0;
    }

}

