/************************************************************************
* Copyright (C) 2017 Niu ZhiYong
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
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"

namespace SpaceDSL{
    /*************************************************
     * Class type: Gregorian Calendar Time
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Defined Gregorian Calendar Time Property and Behavior
     *  This Class is Thread Safe!
    **************************************************/
    CalendarTime::CalendarTime()
    {
        this->m_Year = 0;
        this->m_Mon = 0;
        this->m_Day = 0;
        this->m_Hour = 0;
        this->m_Min = 0;
        this->m_Sec = 0;
    }

    CalendarTime::CalendarTime(int year, int mon, int day, int hour, int min, double sec)
    {
        this->m_Year = year;
        this->m_Mon = mon;
        this->m_Day = day;
        this->m_Hour = hour;
        this->m_Min = min;
        this->m_Sec = sec;
    }

    CalendarTime::~CalendarTime()
    {

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

    //====================================================================
    //
    // Grouping: Date format conversions.
    //
    //====================================================================

    double CalendarTimeToMjd ( int Year, int Month, int Day, int Hour, int Min, double Sec )
    {
        // Variables
        long    MjdMidnight;
        double  FracOfDay;
        int     b;

        if (Month <= 2)
        {
            Month+=12;
            --Year;
        }

        if ( (10000L*Year+100L*Month+Day) <= 15821004L )
            b = -2 + ((Year+4716)/4) - 1179;     // Julian calendar
        else
            b = (Year/400)-(Year/100)+(Year/4);  // Gregorian calendar

        MjdMidnight = 365L*Year - 679004L + b + int(30.6001*(Month+1)) + Day;
        FracOfDay   = (Hour+Min/60.0+Sec/3600.0) / 24.0;

        return MjdMidnight + FracOfDay;
    }

    double CalendarTimeToMjd(CalendarTime &time)
    {
        int Year = time.Year();
        int Month = time.Mon();
        int Day = time.Day();
        int Hour = time.Hour();
        int Min = time.Min();
        double Sec = time.Sec();
        // Variables
        long    MjdMidnight;
        double  FracOfDay;
        int     b;

        if (Month <= 2)
        {
            Month+=12;
            --Year;
        }

        if ( (10000L*Year+100L*Month+Day) <= 15821004L )
            b = -2 + ((Year+4716)/4) - 1179;     // Julian calendar
        else
            b = (Year/400)-(Year/100)+(Year/4);  // Gregorian calendar

        MjdMidnight = 365L*Year - 679004L + b + int(30.6001*(Month+1)) + Day;
        FracOfDay   = (Hour+Min/60.0+Sec/3600.0) / 24.0;

        return MjdMidnight + FracOfDay;
    }

    void MjdToCalendarTime ( double Mjd,
                        int& Year, int& Month, int& Day,
                        int& Hour, int& Min, double& Sec )
    {
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

        Day   = c - e - int(30.6001*f);
        Month = f - 1 - 12*(f/14);
        Year  = d - 4715 - ((7+Month)/10);

        Hours = 24.0*(Mjd-floor(Mjd));

        Hour = int(Hours);
        x = (Hours-Hour)*60.0; Min = int(x);  Sec = (x-Min)*60.0;

    }

    void MjdToCalendarTime(double Mjd, CalendarTime &time)
    {
        int Year, Month, Day, Hour, Min;
        double Sec;
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

        Day   = c - e - int(30.6001*f);
        Month = f - 1 - 12*(f/14);
        Year  = d - 4715 - ((7+Month)/10);

        Hours = 24.0*(Mjd-floor(Mjd));

        Hour = int(Hours);
        x = (Hours-Hour)*60.0; Min = int(x);  Sec = (x-Min)*60.0;

        time = CalendarTime(Year, Month, Day, Hour, Min, Sec);
    }


}

