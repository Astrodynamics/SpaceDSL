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
*   SpSensor.cpp
*
*   Purpose:
*
*         The Base Class of All Kinds of Sensor.
*
*
*   Last modified:
*
*   2018-12-26  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpSensor.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {
    /*************************************************
     * Class type: The class of SpaceDSL Sensor
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Half Angle Unit is [rad]
     *  This Class is Thread Safe!
    **************************************************/
    atomic<int> Sensor::SensorID(0);
    Sensor::Sensor()
    {
        m_ID = ++SensorID;
        m_Name = "Default";
        m_SensorType = E_NotDefinedSensor;
        m_HorizontalHalfAngle = 0;
        m_VerticalHalfAngle = 0;
        m_ConeHalfAngle = 0;
    }

    Sensor::Sensor(const string &name, const SensorType type, const double halfAngle1, const double halfAngle2)
    {
        m_ID = ++SensorID;
        m_Name = name;
        m_SensorType = type;
        switch (type)
        {
        case E_NotDefinedSensor:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::Sensor() type = E_NotDefinedSensor! ");
        case E_SimpleConic:
            m_HorizontalHalfAngle = 0;
            m_VerticalHalfAngle = 0;
            m_ConeHalfAngle = halfAngle1;
            break;
        case E_Rectangular:
            m_HorizontalHalfAngle = halfAngle1;
            m_VerticalHalfAngle = halfAngle2;
            m_ConeHalfAngle = 0;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::Sensor() type = E_NotDefinedSensor! ");
        }
    }

    Sensor::~Sensor()
    {

    }

    void Sensor::SetSensorType(const Sensor::SensorType type, const double halfAngle1, const double halfAngle2)
    {
        m_SensorType = type;
        switch (type)
        {
        case E_NotDefinedSensor:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::Sensor() type = E_NotDefinedSensor! ");
        case E_SimpleConic:
            m_HorizontalHalfAngle = 0;
            m_VerticalHalfAngle = 0;
            m_ConeHalfAngle = halfAngle1;
            break;
        case E_Rectangular:
            m_HorizontalHalfAngle = halfAngle1;
            m_VerticalHalfAngle = halfAngle2;
            m_ConeHalfAngle = 0;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::Sensor() type = E_NotDefinedSensor! ");
        }
    }

    void Sensor::SetConeHalfAngle(const double angle)
    {
        if (m_SensorType == E_NotDefinedSensor)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::SetConeHalfAngle() type = E_NotDefinedSensor! ");

        if (m_SensorType == E_Rectangular)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::SetConeHalfAngle() E_Rectangular Cant Use SetConeHalfAngle()!");

        m_HorizontalHalfAngle = 0;
        m_VerticalHalfAngle = 0;
        m_ConeHalfAngle = angle;
    }

    void Sensor::SetHorizontalHalfAngle(const double angle)
    {
        if (m_SensorType == E_NotDefinedSensor)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::SetHorizontalHalfAngle() type = E_NotDefinedSensor! ");

        if (m_SensorType == E_SimpleConic)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::SetHorizontalHalfAngle() E_SimpleConic Cant Use SetHorizontalHalfAngle()!");

        m_HorizontalHalfAngle = angle;
        m_ConeHalfAngle = 0;
    }

    void Sensor::SetVerticalHalfAngle(const double angle)
    {
        if (m_SensorType == E_NotDefinedSensor)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::SetVerticalHalfAngle() type = E_NotDefinedSensor! ");

        if (m_SensorType == E_SimpleConic)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::SetVerticalHalfAngle() E_SimpleConic Cant Use SetVerticalHalfAngle()!");

        m_VerticalHalfAngle = angle;
        m_ConeHalfAngle = 0;
    }

    double Sensor::GetConeHalfAngle() const
    {
        if (m_SensorType == E_NotDefinedSensor)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::GetConeHalfAngle() type = E_NotDefinedSensor! ");

        if (m_SensorType == E_Rectangular)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::GetConeHalfAngle() E_Rectangular Cant Use GetConeHalfAngle()!");

        return m_ConeHalfAngle;
    }

    double Sensor::GetHorizontalHalfAngle() const
    {
        if (m_SensorType == E_NotDefinedSensor)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::GetHorizontalHalfAngle() type = E_NotDefinedSensor! ");

        if (m_SensorType == E_SimpleConic)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::GetHorizontalHalfAngle() E_SimpleConic Cant Use GetHorizontalHalfAngle()!");

        return m_HorizontalHalfAngle;
    }

    double Sensor::GetVerticalHalfAngle() const
    {
        if (m_SensorType == E_NotDefinedSensor)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::GetVerticalHalfAngle() type = E_NotDefinedSensor! ");

        if (m_SensorType == E_SimpleConic)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Sensor::GetVerticalHalfAngle() E_SimpleConic Cant Use GetVerticalHalfAngle()!");

        return m_VerticalHalfAngle;
    }

    




}
