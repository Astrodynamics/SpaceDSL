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
* Date:2018-07-30
* Description:
*   SpSpaceVehicle.cpp
*
*   Purpose:
*
*       The Base Class of All Kinds of Space Vehicle.
*
*
*   Last modified:
*
*   2018-07-30  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include "SpaceDSL/SpSpaceVehicle.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"



namespace SpaceDSL {

    /*************************************************
     * Class type: The Base Class of All Kinds of Space Vehicle
     * Author: Niu ZhiYong
     * Date:2018-07-30
     * Description:
    **************************************************/
    atomic<int> SpaceVehicle::VehicleID(0);
    SpaceVehicle::SpaceVehicle()
    {
        m_ID = ++VehicleID;
        m_Name = "Default";
        m_InitialCartState = CartState();
        m_InitialEpoch = 0.0;
        m_InitialMass = 0.0;

        m_Epoch = m_InitialEpoch;
        m_CartState = m_InitialCartState;
        m_Mass = m_InitialMass;

        m_DragCoef = 0.0;
        m_DragArea = 0.0;
        m_SRPCoef = 0.0;
        m_SRPArea = 0.0;
    }

    SpaceVehicle::SpaceVehicle(const string &name, const CalendarTime &initialEpoch,
                               const CartState &initialState, const double initialMass,
                               const double dragCoef, const double dragArea,
                               const double SRPCoef, const double SRPArea)
    {
        m_ID = ++VehicleID;
        m_Name = name;
        m_InitialCartState = initialState;
        m_InitialEpoch = CalendarTimeToMjd(initialEpoch);
        m_InitialMass = initialMass;

        m_Epoch = m_InitialEpoch;
        m_CartState = m_InitialCartState;
        m_Mass = m_InitialMass;

        m_DragCoef = dragCoef;
        m_DragArea = dragArea;
        m_SRPCoef = SRPCoef;
        m_SRPArea = SRPArea;
    }

    SpaceVehicle::SpaceVehicle(const string &name, const double initialEpoch,
                               const CartState &initialState, const double initialMass,
                               const double dragCoef, const double dragArea,
                               const double SRPCoef, const double SRPArea)
    {
        m_ID = ++VehicleID;
        m_Name = name;
        m_InitialCartState = initialState;
        m_InitialEpoch = initialEpoch;
        m_InitialMass = initialMass;

        m_Epoch = m_InitialEpoch;
        m_CartState = m_InitialCartState;
        m_Mass = m_InitialMass;

        m_DragCoef = dragCoef;
        m_DragArea = dragArea;
        m_SRPCoef = SRPCoef;
        m_SRPArea = SRPArea;
    }

    SpaceVehicle::~SpaceVehicle()
    {

    }

    void SpaceVehicle::UpdateState(const double Mjd, const CartState &state, const double mass)
    {
        m_Epoch = Mjd;
        m_CartState = state;
        m_Mass = mass;
    }

    void SpaceVehicle::UpdateState(const double Mjd, const Vector3d &pos, const Vector3d &vel, const double mass)
    {
        m_Epoch = Mjd;
        CartState state(pos, vel);
        m_CartState = state;
        m_Mass = mass;
    }

    void SpaceVehicle::Reset()
    {
        m_Epoch = m_InitialEpoch;
        m_CartState = m_InitialCartState;
        m_Mass = m_InitialMass;
    }

    void SpaceVehicle::InsertSensor(const string &name, const Sensor::SensorType type, const double halfAngle1, const double halfAngle2)
    {
        if (m_SensorNumber != int(m_SensorList.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "SpaceVehicle::InsertSensor (SensorNumber) != (SensorList.Size)! ");
        }
        Sensor *pSensor = new Sensor(name, type, halfAngle1, halfAngle2);
        ++m_SensorNumber;
        m_SensorList.push_back(pSensor);
    }

    bool SpaceVehicle::RemoveSensor(const int id)
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

