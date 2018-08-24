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
        ++VehicleID;
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
        ++VehicleID;
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
        ++VehicleID;
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

	
	
}

