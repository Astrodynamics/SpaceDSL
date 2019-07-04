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
*   SpTimeSystem.h
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

#ifndef SPTIMESYSTEM_H
#define SPTIMESYSTEM_H


#include "SpaceDSL_Global.h"

#include <string>

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: Gregorian Calendar Time
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Defined Gregorian Calendar Time Base Class
    **************************************************/
    class SPACEDSL_API CalendarTime
    {
    public:
        explicit CalendarTime();
        virtual ~CalendarTime();

        inline int			Year() const {return m_Year;}
        inline int			Mon () const {return m_Mon;}
        inline int			Day () const {return m_Day;}
        inline int			Hour() const {return m_Hour;}
        inline int			Min () const {return m_Min;}
        inline double		Sec () const {return m_Sec;}

        inline void			SetYear(int year)   { m_Year = year;}
        inline void			SetMon (int mon)    { m_Mon = mon;}
        inline void			SetDay (int day)    { m_Day = day;}
        inline void			SetHour(int hour)   { m_Hour = hour;}
        inline void			SetMin (int min)    { m_Min = min;}
        inline void         SetSec (double sec) { m_Sec = sec;}

        void                operator =(const CalendarTime& time);
        bool				operator==(const CalendarTime& time) const;
        bool				operator!=(const CalendarTime& time) const;
        bool				operator> (const CalendarTime& time) const;
        bool				operator>=(const CalendarTime& time) const;
        bool				operator< (const CalendarTime& time) const;
        bool				operator<=(const CalendarTime& time) const;

        double              operator -(const CalendarTime& time) const;

    public:
        /********************************************************************/
        /// Transform from Calendar Date to Char Array
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20
        /// @Input/Output
        /// @Param      timeChar        Char Array Point Such as "yyyy-mm-dd HH:MM:SS"
        /// @Return
        /**********************************************************************/
        void                ToCharArray(char *&pTimeStr);

        string              ToString();

    protected:
        void                FillTimeStr();

    ///
    ///Attribute.
    ///
    protected:
        int		m_Year;                 ///< year
        int		m_Mon;                  ///< month,  1 - 12
        int		m_Day;                  ///< day,    1 - 31
        int		m_Hour;                 ///< hour,   0 - 23
        int		m_Min;                  ///< minute, 0 - 59
        double	m_Sec;                  ///< second, 0 - 59.99..
        char    *m_pTimeStr;            ///< char form "yyyy-mm-dd HH:MM:SS"
    };
    /*************************************************
     * Class type: UTC Time
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Defined UTC Time Property and Behavior
    **************************************************/
    class SPACEDSL_API UTCCalTime : public CalendarTime
    {
    public:
        explicit UTCCalTime();
        UTCCalTime(int year, int mon, int day, int hour, int min, double sec);
        virtual ~UTCCalTime();

    };

    /*************************************************
     * Class type: UT1 Time
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Defined UT1 Time Property and Behavior
    **************************************************/
    class SPACEDSL_API UT1CalTime : public CalendarTime
    {
    public:
        explicit UT1CalTime();
        UT1CalTime(int year, int mon, int day, int hour, int min, double sec);
        virtual ~UT1CalTime();

    };

    //====================================================================
    //
    // Grouping: Date format conversions.
    //
    //====================================================================

    /********************************************************************/
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
    /**********************************************************************/
    double SPACEDSL_API CalendarTimeToMjd (int year, int month, int day,
                                        int hour=0, int min=0, double sec=0.0 );

    /********************************************************************/
    ///
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      time                  UTCCalTime
    /// @Return     Modified Julian Date
    /**********************************************************************/
    double SPACEDSL_API CalendarTimeToMjd (const CalendarTime &time);

    /********************************************************************/
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
    /**********************************************************************/
    void SPACEDSL_API MjdToCalendarTime ( double Mjd,
                                         int& year,  int& month,  int& day,
                                         int& hour,  int& min,  double& sec);

    /********************************************************************/
    /// Calendar date and time from Modified Julian Date
    /// @Author     Niu Zhiyong
    /// @Date       2018-03-20
    /// @Input
    /// @Param      Mjd         Modified Julian Date
    /// @Output
    /// @Param      time        UTCCalTime
    /// @Return     void
    /**********************************************************************/
    void SPACEDSL_API MjdToCalendarTime (double Mjd, CalendarTime &time);

    /*************************************************
     * Class type: IERS
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Get IERS Data through a File or Web Service
    **************************************************/
    class SPACEDSL_API IERSService
    {
    public:
        explicit IERSService();
        virtual ~IERSService();

    public:
        #ifdef WITH_OPENSSL
        void        EnableWebService();
        void        DisableWebService();
        #endif
        /********************************************************************/
        /// Get IERS Data
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20
        /// @Input
        /// @Param      Mjd_UTC         Modified Julian Date of UTC Time
        /// @Param      param           Type of Data Required
        /// @Param      series          The Series of Data
        /// @Output
        /// @Return     Value of Data   [sec]
        /// Mote:
        /// [param] Can be
        ///     "x_pole", "sigma_x_pole", "y_pole", "sigma_y_pole", "UT1-UTC", "sigma_UT1-UTC",
        ///     "LOD", "sigma_LOD", "dPsi", "sigma_dPsi", "dEpsilon", "sigma_dEpsilon",
        ///     "dX", "sigma_dX", "dY", "sigma_dY"
        ///     "leapseconds", "UT1-UTC" , "UT1", "TAI", "TT"
        /// [series] Can be
        ///     "Finals All (IAU2000)", "Finals Daily (IAU2000)", "Finals Data (IAU2000)", "Finals All (IAU1980)",
        ///     "Finals Daily (IAU1980)","Finals Data (IAU1980)","EOP 08 C04 (IAU1980)","EOP 08 C04 (IAU2000)",
        ///     "EOP 14 C04 (IAU1980)","EOP 14 C04 (IAU2000)","GPS Rapid","GPS Rapid Daily",
        ///     "Bulletin A","Bulletin B","EOP C01"
        /**********************************************************************/
        double                      GetValue(double Mjd_UTC, const char *param, const char *series = nullptr);

    public:
        static double               TT_TAI;         // TT-TAI time difference [s]
        static double               GPS_TAI;        // GPS-TAI time difference [s]

    protected:

        bool                        m_bIsUseWebService;
        double                      **m_EOPData;
        double                      **m_LeapsecondData;
        int                         m_EOPDataLine;
        int                         m_LeapsecondDataLine;




    };

}

#endif  //SPTIMESYSTEM_H
