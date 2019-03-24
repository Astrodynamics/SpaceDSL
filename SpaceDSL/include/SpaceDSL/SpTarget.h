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
*   SpTarget.h
*
*   Purpose:
*
*         Definition of All Kinds of Target.
*
*
*   Last modified:
*
*   2018-12-26  Niu Zhiyong (1st edition)
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
     * Class type: The Base class of SpaceDSL Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API Target
    {
    public:
        explicit Target();
        virtual ~Target();

        enum TargetType
        {
            E_NotDefindTargetType = 0,
            E_PointTarget = 1,
            E_LineTarget = 2,
            E_AreaTarget = 3,

            E_Facility = 11,
        };

    protected:
        static atomic<int>      TargetID;               ///< Target ID
        string                  m_Name;                 ///< Target Name
        TargetType              m_TargetType;           ///< Target Type
        double                  m_MinElevation;         ///< Minimum Observation Elevations

    public:
        inline void             SetName (const string &name)        { m_Name = name; }

        inline void             SetMinElevation (const double angle){ m_MinElevation = angle; }

        inline const string&    GetName() const                     { return m_Name; }

        inline int              GetID() const                       { return TargetID; }

        inline TargetType       GetTargetType() const               { return m_TargetType; }

        inline double           GetMinElevation() const             { return m_MinElevation; }

    };
    /*************************************************
     * Class type: The class of SpaceDSL Point Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API PointTarget : public Target
    {
    public:
        explicit PointTarget();
        PointTarget(const string &name, const double longitude, const double latitude, const double altitude, const double minElevation);
        PointTarget(const string &name, const GeodeticCoord &LLA, const double minElevation);
        virtual ~PointTarget();

    public:
        inline void		SetGeodeticCoord (const GeodeticCoord &LLA);

        inline void		SetGeodeticCoord (const double longitude, const double latitude, const double altitude);

        inline const GeodeticCoord& GetGeodeticCoord() const            { return m_LonLatAltitude; }

        const vector<double> GetGeodeticPos() const;

    protected:

    //
    // Attribute.
    //
    protected:
        GeodeticCoord           m_LonLatAltitude;           ///< In The Coordinate System of Initialization
    };
    /*************************************************
     * Class type: The class of SpaceDSL Line Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API LineTarget : public Target
    {
    public:
        explicit LineTarget();
        LineTarget(const string &name, const vector<GeodeticCoord> &points,
                   const int anchorPointIndex, const double minElevation);
        virtual ~LineTarget();

    public:
        inline void		SetPointList (const vector<GeodeticCoord> &points)  { m_PointList = points; }

        inline void     SetAnchorPointIndex (const int index)               { m_AnchorPointIndex = index; }

        inline const vector<GeodeticCoord>& GetPointList() const            { return m_PointList; }

        inline int                          GetAnchorPointIndex() const     { return m_AnchorPointIndex; }

    protected:

    //
    // Attribute.
    //
    protected:
        vector<GeodeticCoord>   m_PointList;                ///< Point List of Component The Line
        int                     m_AnchorPointIndex;         ///< Anchor Point Index In m_PointList of The Line
    };

    /*************************************************
     * Class type: The class of SpaceDSL Area Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API AreaTarget : public Target
    {
    public:
        explicit AreaTarget();
        AreaTarget(const string &name, const vector<GeodeticCoord> &points, const double minElevation);
        virtual ~AreaTarget();

    public:
        inline void		SetPointList (const vector<GeodeticCoord> &points);

        inline const vector<GeodeticCoord>& GetPointList() const            { return m_PointList; }

        inline const GeodeticCoord        & GetCenterPoint() const          { return m_CenterPoint; }

    protected:
        bool            IsSimplePolygon();

        GeodeticCoord   EstimateCenterPoint();

    //
    // Attribute.
    //
    protected:
        vector<GeodeticCoord>   m_PointList;                ///< Point List of Component The Line
        GeodeticCoord           m_CenterPoint;              ///< Center Point In Area
    };

}

#endif //SPTARGET_H
