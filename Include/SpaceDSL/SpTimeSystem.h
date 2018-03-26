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
* Date:2017-03-20
* Description:
*   SpTimeSystem.h
*
*   Purpose:
*
*       Time and date computation
*
*
*   Last modified:
*
*   2017-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPTIMESYSTEM_H
#define SPTIMESYSTEM_H

#include <math.h>
#include <iostream>

#include "SpaceDSL_Global.h"

using namespace std;

///
/// Date Conversion Constants
///
const double SpCMJDOffset  = 2400000.5;
const double SpCTTMinusTAI = 32.184;


///
/// Coordinate System Epochs
///
const double SpCB1950Epoch = 2433282.4234591;
const double SpCJ2000Epoch = 2451545.0;			//2000-01-01 12:00:00 TDB


///
/// Earth_Motion_Constants
///
const double SpCEarthSiderealDay  = 86164.09054;
const double SpCEarthSiderealYear = 365.25636;

/*************************************************
 * Class type: Gregorian Calendar Time
 * Author: Niu ZhiYong
 * Date:2018-03-20
 * Description:
 *  Defined Gregorian Calendar Time Property and Behavior
 *  This Class is Thread Safe!
**************************************************/
class SPACEDSLSHARED_EXPORT SpCalendarTime
{
public:
    SpCalendarTime();
    SpCalendarTime(int year, int mon, int day, int hour, int min, double sec);
    SpCalendarTime(const SpCalendarTime& time);
    virtual ~SpCalendarTime();

    inline int			Year() const {return m_Year;}
    inline int			Mon () const {return m_Mon;}
    inline int			Day () const {return m_Day;}
    inline int			Hour() const {return m_Hour;}
    inline int			Min () const {return m_Min;}
    inline double		Sec () const {return m_Sec;}


    bool				operator==(const SpCalendarTime& time) const;
    bool				operator!=(const SpCalendarTime& time) const;
    bool				operator> (const SpCalendarTime& time) const;
    bool				operator>=(const SpCalendarTime& time) const;
    bool				operator< (const SpCalendarTime& time) const;
    bool				operator<=(const SpCalendarTime& time) const;


///
///Attribute.
///
protected:
    int		m_Year;		///< year
    int		m_Mon;		///< month,  1 - 12
    int		m_Day;		///< day,    1 - 31
    int		m_Hour;		///< hour,   0 - 23
    int		m_Min;		///< minute, 0 - 59
    double	m_Sec;		///< second, 0 - 59.99..
};

//====================================================================
//
// Grouping: Date format conversions.
//
//====================================================================

/********************************************************************
///
/// @Author     Niu Zhiyong
/// @Date       2018-03-20
/// @Input
/// @Param      Year        Calendar date components
/// @Param      Month
/// @Param      Day
/// @Param      Hour		Time components (optional)
/// @Param      Min
/// @Param      Sec
/// @Return     Modified Julian Date
**********************************************************************/
double SPACEDSLSHARED_EXPORT SpTimeToMjd ( int Year,   int Month, int Day,
             int Hour=0, int Min=0, double Sec=0.0 );

/********************************************************************
/// Calendar date and time from Modified Julian Date
/// @Author     Niu Zhiyong
/// @Date       2018-03-20
/// @Input
/// @Param      Mjd         Modified Julian Date
/// @Output
/// @Param      Year        Calendar date components
/// @Param      Month
/// @Param      Day         Time components (optional)
/// @Param      Hour
/// @Param      Min
/// @Param      Sec
/// @Return     void
**********************************************************************/
void SPACEDSLSHARED_EXPORT SpMjdToTime ( double Mjd,
              int& Year, int& Month, int& Day,
              int& Hour, int& Min, double& Sec );




#endif  //SPTIMESYSTEM_H
