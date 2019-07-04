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
*   SpTarget.cpp
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
#include "SpaceDSL/SpTarget.h"
#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

    /*************************************************
     * Class type: The Base class of SpaceDSL Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  This Class is Thread Safe!
    **************************************************/
    atomic<int> Target::TargetID(0);
    Target::Target()
    {
        m_ID = ++TargetID;
        m_TargetType = E_NotDefindTargetType;
    }

    Target::~Target()
    {

    }

    /*************************************************
     * Class type: The class of SpaceDSL Point Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/

    PointTarget::PointTarget()
    {
        m_Name = "Default";
        m_TargetType = E_PointTarget;
        m_LonLatAltitude.SetLongitude(0);
        m_LonLatAltitude.SetLatitude(0);
        m_LonLatAltitude.SetAltitude(0);
        m_MinElevation = 0;
    }

    PointTarget::PointTarget(const string &name, const double longitude, const double latitude, const double altitude, const double minElevation)
    {
        m_Name = name;
        m_TargetType = E_PointTarget;
        m_LonLatAltitude.SetLongitude(longitude);
        m_LonLatAltitude.SetLatitude(latitude);
        m_LonLatAltitude.SetAltitude(altitude);
        m_MinElevation = minElevation;
    }

    PointTarget::PointTarget(const string &name, const GeodeticCoord &LLA, const double minElevation)
    {
        m_Name = name;
        m_TargetType = E_PointTarget;
        m_LonLatAltitude = LLA;
        m_MinElevation = minElevation;
    }

    PointTarget::~PointTarget()
    {

    }

    void PointTarget::SetGeodeticCoord(const GeodeticCoord &LLA)
    {
        m_LonLatAltitude = LLA;
    }

    void PointTarget::SetGeodeticCoord(const double longitude, const double latitude, const double altitude)
    {
        m_LonLatAltitude.SetLongitude(longitude);
        m_LonLatAltitude.SetLatitude(latitude);
        m_LonLatAltitude.SetAltitude(altitude);
    }

    const vector<double> PointTarget::GetGeodeticPos() const
    {
        GeodeticCoordSystem ECFSys(GeodeticCoordSystem::E_WGS84System);
        Vector3d geoPos = ECFSys.GetPosition(m_LonLatAltitude);
        vector<double> pos;
        pos.push_back(geoPos(0));
        pos.push_back(geoPos(1));
        pos.push_back(geoPos(2));
        return pos;
    }

    /*************************************************
     * Class type: The class of SpaceDSL Line Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    LineTarget::LineTarget()
    {
        m_Name = "Default";
        m_TargetType = E_LineTarget;
        m_PointList.clear();
        m_AnchorPointIndex = -1;
        m_MinElevation = 0;
    }

    LineTarget::LineTarget(const string &name, const vector<GeodeticCoord> &points,
                           const int anchorPointIndex, const double minElevation)
    {
        m_Name = name;
        m_TargetType = E_LineTarget;
        m_PointList = points;
        m_AnchorPointIndex = anchorPointIndex;
        m_MinElevation = minElevation;
    }

    LineTarget::~LineTarget()
    {

    }

    /*************************************************
     * Class type: The class of SpaceDSL Area Target
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
     *  This Class is Thread Safe!
    **************************************************/
    AreaTarget::AreaTarget()
    {
        m_Name = "Default";
        m_TargetType = E_LineTarget;
        m_PointList.clear();
        m_CenterPoint = GeodeticCoord(0,0,0);
        m_MinElevation = 0;
    }

    AreaTarget::AreaTarget(const string &name, const vector<GeodeticCoord> &points, const double minElevation)
    {
        m_Name = name;
        m_TargetType = E_LineTarget;
        m_PointList = points;
        if (this->IsSimplePolygon() == false)
             throw SPException(__FILE__, __FUNCTION__, __LINE__, "AreaTarget::AreaTarget AreaTarget is Not a Simple Polygon! ");
        m_CenterPoint = this->EstimateCenterPoint();
        m_MinElevation = minElevation;
    }

    AreaTarget::~AreaTarget()
    {

    }

    void AreaTarget::SetPointList(const vector<GeodeticCoord> &points)
    {
        m_PointList = points;
        if (this->IsSimplePolygon() == false)
             throw SPException(__FILE__, __FUNCTION__, __LINE__, "AreaTarget::AreaTarget AreaTarget is Not a Simple Polygon! ");
        m_CenterPoint = this->EstimateCenterPoint();
    }

    bool AreaTarget::IsSimplePolygon()
    {
        bool rotateDirection = true;

        for (int i = 0; i < m_PointList.size() - 3; ++i)
        {
            Vector2d p1(m_PointList[i].Longitude(), m_PointList[i].Latitude());
            Vector2d p2(m_PointList[i + 1].Longitude(), m_PointList[i + 2].Latitude());
            Vector2d p3(m_PointList[i + 2].Longitude(), m_PointList[i + 2].Latitude());

            Vector2d v1 = (p2 - p1).normalized();
            Vector2d v2 = (p3 - p2).normalized();;

            if ( i == 0 )
            {
                double temp = v1(0)*v2(1) - v1(1)*v2(0);
                if(temp >= 0)
                    rotateDirection = true;
                else
                    rotateDirection = false;
            }
            else
            {
                double temp = v1(0)*v2(1) - v1(1)*v2(0);

                if( (temp >= 0 && rotateDirection == true)
                    || (temp < 0 && rotateDirection == false))
                    continue;
                else
                {
                    return  false;
                }
            }
        }

        return true;
    }

    GeodeticCoord AreaTarget::EstimateCenterPoint()
    {
        vector<Vector2d> midpointList;
        for (auto &lla:m_PointList)
        {
            Vector2d p(lla.Longitude(), lla.Latitude());
            midpointList.push_back(p);
        }
        vector<Vector2d> tempList;
        double maxDistence = 0;
        do
        {
            midpointList = tempList;
            tempList.clear();
            for(int i = 0; i < midpointList.size() - 1; ++i)
            {
                Vector2d midp( (midpointList[i](0) + midpointList[i + 1](0))/2,
                               (midpointList[i](1) + midpointList[i + 1](1))/2);
                tempList.push_back(midp);

                double distence = sqrt(pow(midpointList[i](1) - midpointList[i + 1](1), 2) +
                                       pow(midpointList[i](0) - midpointList[i + 1](0), 2));
                if (distence > maxDistence)
                    maxDistence = distence;

            }
        }while(maxDistence > 0.1);

        return GeodeticCoord(midpointList[0](0), midpointList[0](1), 0);
    }

}
