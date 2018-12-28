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
* Date:2019-12-26
* Description:
*   SpTarget.h
*
*   Purpose:
*
*         Definition of All Kinds of Target.
*
*
*   Last modified:
*
*   2019-12-26  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPTARGET_H
#define SPTARGET_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"


#include <string>
#include <vector>
#include <atomic>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL Point Target
     * Author: Niu ZhiYong
     * Date:2019-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API PointTarget
    {
    public:
        explicit PointTarget();
        PointTarget(const string &name, const double longitude, const double latitude, const double altitude, const double minElevation);
        PointTarget(const string &name, const GeodeticCoord &LLA, const double minElevation);
        virtual ~PointTarget();

    public:
        inline void		SetName (const string &name)                    { m_Name = name; }

        inline void		SetGeodeticCoord (const GeodeticCoord &LLA)     { m_LonLatAltitude = LLA; }

        inline void		SetGeodeticCoord (const double longitude, const double latitude, const double altitude);

        inline void     SetMinElevation (const double angle)            { m_MinElevation = angle; }

        inline int                  GetID() const                       { return PointTargetID; }

        inline const string&        GetName() const                     { return m_Name; }

        inline const GeodeticCoord& GetGeodeticCoord() const            { return m_LonLatAltitude; }

        inline double               GetMinElevation() const             { return m_MinElevation; }

    protected:

    //
    // Attribute.
    //
    protected:
        static atomic<int>      PointTargetID;              ///< Point Target ID
        string                  m_Name;                     ///< Point Target Name
        GeodeticCoord           m_LonLatAltitude;           ///< In The Coordinate System of Initialization
        double                  m_MinElevation;             ///< Minimum Observation Elevations
    };
    /*************************************************
     * Class type: The class of SpaceDSL Line Target
     * Author: Niu ZhiYong
     * Date:2019-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API LineTarget
    {
    public:
        explicit LineTarget();
        LineTarget(const string &name, const vector<GeodeticCoord> &points,
                   const int anchorPointIndex, const double minElevation);
        virtual ~LineTarget();

    public:
        inline void		SetName (const string &name)                        { m_Name = name; }

        inline void		SetPointList (const vector<GeodeticCoord> &points)  { m_PointList = points; }

        inline void     SetAnchorPointIndex (const int index)               { m_AnchorPointIndex = index; }

        inline void     SetMinElevation (const double angle)                { m_MinElevation = angle; }

        inline int                          GetID() const                   { return LineTargetID; }

        inline const string&                GetName() const                 { return m_Name; }

        inline const vector<GeodeticCoord>& GetPointList() const            { return m_PointList; }

        inline int                          GetAnchorPointIndex() const     { return m_AnchorPointIndex; }

        inline double                       GetMinElevation() const         { return m_MinElevation; }

    protected:

    //
    // Attribute.
    //
    protected:
        static atomic<int>      LineTargetID;               ///< Line Target ID
        string                  m_Name;                     ///< Line Target Name
        vector<GeodeticCoord>   m_PointList;                ///< Point List of Component The Line
        int                     m_AnchorPointIndex;         ///< Anchor Point Index In m_PointList of The Line
        double                  m_MinElevation;             ///< Minimum Observation Elevations
    };

    /*************************************************
     * Class type: The class of SpaceDSL Area Target
     * Author: Niu ZhiYong
     * Date:2019-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API AreaTarget
    {
    public:
        explicit AreaTarget();
        AreaTarget(const string &name, const vector<GeodeticCoord> &points, const double minElevation);
        virtual ~AreaTarget();

    public:
        inline void		SetName (const string &name)                        { m_Name = name; }

        inline void		SetPointList (const vector<GeodeticCoord> &points);

        inline void     SetMinElevation (const double angle)                { m_MinElevation = angle; }

        inline int                          GetID() const                   { return AreaTargetID; }

        inline const string&                GetName() const                 { return m_Name; }

        inline const vector<GeodeticCoord>& GetPointList() const            { return m_PointList; }

        inline const GeodeticCoord        & GetCenterPoint() const          { return m_CenterPoint; }

        inline double                       GetMinElevation() const         { return m_MinElevation; }

    protected:
        bool            IsSimplePolygon();

        GeodeticCoord   EstimateCenterPoint();

    //
    // Attribute.
    //
    protected:
        static atomic<int>      AreaTargetID;               ///< Line Target ID
        string                  m_Name;                     ///< Line Target Name
        vector<GeodeticCoord>   m_PointList;                ///< Point List of Component The Line
        GeodeticCoord           m_CenterPoint;              ///< Center Point In Area
        double                  m_MinElevation;             ///< Minimum Observation Elevations
    };

}

#endif //SPTARGET_H
