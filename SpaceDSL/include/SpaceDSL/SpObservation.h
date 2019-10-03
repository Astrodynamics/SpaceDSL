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
*   SpObservation.h
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

#ifndef SPOBSERVATION_H
#define SPOBSERVATION_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"
#include "SpJplEph.h"
#include "SpTimeSystem.h"

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>


using namespace Eigen;
using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    class Target;
    static JplEphemeris     s_JPLEphemeris;
    static IERSService      s_IERSService;
    /*************************************************
     * Class type: The class of SpaceDSL Target Observation
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API Observation
    {
    public:
        explicit Observation();
        Observation(const double azimuth, const double azimuthRate,
                    const double elevation, const double elevationRate,
                    const Vector3d &posECF, const Vector3d &velECF);
        virtual ~Observation();

        inline double   Azimuth() const         {return m_Azimuth;}
        inline double   AzimuthRate() const     {return m_AzimuthRate;}
        inline double   Elevation() const       {return m_Elevation;}
        inline double   ElevationRate() const   {return m_ElevationRate;}
        inline Vector3d RelativePosECF() const  {return m_RelativePosECF;}
        inline Vector3d RelativeVelECF() const  {return m_RelativeVelECF;}

        inline void     SetAzimuth(const double azimuth)            { m_Azimuth = azimuth;}
        inline void     SetAzimuthRate(const double azimuthRate)    { m_AzimuthRate  = azimuthRate;}
        inline void     SetElevation(const double elevation)        { m_Elevation = elevation;}
        inline void     SetElevationRate(const double elevationRate){ m_ElevationRate  = elevationRate;}
        inline void     SetRelativePosECF(const Vector3d pos)       { m_RelativePosECF = pos;}
        inline void     SetRelativeVelECF(const Vector3d vel)       { m_RelativeVelECF  = vel;}

    protected:
        double      m_Azimuth;              ///< Azimuth angle (rad)

        double      m_AzimuthRate;          ///< Azimuth angle rate (rad/s)

        double      m_Elevation;            ///< Elevation angle (rad)

        double      m_ElevationRate;        ///< Elevation angle rate (rad/s)

        Vector3d    m_RelativePosECF;       ///< Relative Position vector in ECF(Earth Centered Fixed)(m)

        Vector3d    m_RelativeVelECF;          ///< Relative velocity vector in ECF(Earth Centered Fixed)(m/s)
    };
   
    /********************************************************************/
    /// Calculate Object Observation Param From Point Target
    /// @Author     Niu Zhiyong
    /// @Date       2019-01-04
    /// @Input
    /// @Param      cart		Observation Object Cart State in J2000
    /// @Output
    /// @Return     result      Max Observation Latitude
    /**********************************************************************/
    double SPACEDSL_API CalMaxObservationLat(const CartState &cart);

    /********************************************************************/
    /// Calculate Object Observation Param From Point Target
    /// @Author     Niu Zhiyong
    /// @Date       2019-01-04
    /// @Input
    /// @Param      Mjd         Modified Julian date of UTC
    /// @Param      cart		Observation Object Cart State in J2000
    /// @Param      target		Point Target Object Point
    /// @Output
    /// @Param      result
    /// @Return     false : The Target Can not be Seen
    /**********************************************************************/
    bool SPACEDSL_API CalObservation(const double Mjd, const CartState &cart, Target *target, Observation &result);

    bool SPACEDSL_API CalObservationAll(const double Mjd, const CartState &cart, Target *target, Observation &result);

    /********************************************************************/
    /// Calculate Sun Observation Param From Point Target
    /// @Author     Niu Zhiyong
    /// @Date       2019-01-04
    /// @Input
    /// @Param      Mjd         Modified Julian date of UTC
    /// @Param      target		Point Target Object Point
    /// @Output
    /// @Return     result      Observation Param
    /**********************************************************************/
    Observation SPACEDSL_API CalSunObservation(const double Mjd, Target *target);

}

#endif //SPOBSERVATION_H
