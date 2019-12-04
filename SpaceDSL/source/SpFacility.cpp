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
*   SpFacility.cpp
*
*   Purpose:
*
*         The Base Class of All Kinds of Facilities.
*
*
*   Last modified:
*
*   2018-12-26  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpFacility.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL Facility
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]
     *  This Class is Thread Safe!
    **************************************************/
    Facility::Facility()
    {
        m_Name = "Default";
        m_TargetType = E_Facility;
        m_SensorNumber = 0;
        m_SensorList.clear();
        m_LonLatAltitude.SetLongitude(0);
        m_LonLatAltitude.SetLatitude(0);
        m_LonLatAltitude.SetAltitude(0);
        m_MinElevation = 0;
    }

    Facility::Facility(const string &name, const double longitude, const double latitude, const double altitude, const double minElevation)
    {
        m_Name = name;
        m_TargetType = E_Facility;
        m_SensorNumber = 0;
        m_SensorList.clear();
        m_LonLatAltitude.SetLongitude(longitude);
        m_LonLatAltitude.SetLatitude(latitude);
        m_LonLatAltitude.SetAltitude(altitude);
        m_MinElevation = minElevation;
    }

    Facility::Facility(const string &name, const GeodeticCoord &LLA, const double minElevation)
    {
        m_Name = name;
        m_TargetType = E_Facility;
        m_SensorNumber = 0;
        m_SensorList.clear();
        m_LonLatAltitude = LLA;
        m_MinElevation = minElevation;
    }

    Facility::~Facility()
    {
        for(auto pSensor:m_SensorList)
        {
            if (pSensor != nullptr)
                delete pSensor;
        }
        m_SensorList.clear();
    }

    void Facility::InsertSensor(const string &name, const Sensor::SensorType type, const double halfAngle1, const double halfAngle2)
    {
        if (m_SensorNumber != int(m_SensorList.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Facility::InsertSensor (SensorNumber) != (SensorList.Size)! ");
        }
        Sensor *pSensor = new Sensor(name, type, halfAngle1, halfAngle2);
        ++m_SensorNumber;
        m_SensorList.push_back(pSensor);
    }

    bool Facility::RemoveSensor(const int id)
    {
        for(auto iter = m_SensorList.begin();
            iter != m_SensorList.end();
            ++iter)
        {
            if ((*iter)->GetID() == id)
            {
                delete *iter;
                m_SensorList.erase(iter);
                --m_SensorNumber;
                return true;
            }

        }
        return false;
    }

    




}
