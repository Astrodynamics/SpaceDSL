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
*   SpSensor.h
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

#ifndef SPSENSOR_H
#define SPSENSOR_H

#include "SpaceDSL_Global.h"


#include <string>
#include <vector>
#include <atomic>

using namespace std;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL Sensor
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Half Angle Unit is [rad]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API Sensor
    {
    public:
        enum SensorType
        {
            E_NotDefinedSensor   = 0,
            E_SimpleConic        = 1,
            E_Rectangular        = 2
        };

        explicit Sensor();
        /********************************************************************/
        /// Create a Sensor According to The Specified Type
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /// @Input
        /// @Param  halfAngle1    	Cone Half Angle/Horizontal Half Angle
        /// @Param	halfAngle2		Vertical Half Angle
        /// @Output
        /// @Param
        /**********************************************************************/
        Sensor(const string &name, const SensorType type, const double halfAngle1, const double halfAngle2 = 0);
        virtual ~Sensor();

    public:
        inline void             SetName (const string &name)        { m_Name = name; }

        inline void             SetSensorType (const SensorType type, const double halfAngle1, const double halfAngle2 = 0);

        inline void             SetConeHalfAngle (const double angle);

        inline void             SetHorizontalHalfAngle (const double angle);

        inline void             SetVerticalHalfAngle (const double angle);

        inline int              GetID() const                       { return SensorID; }

        inline const string&    GetName() const                     { return m_Name; }

        inline const SensorType GetSensorType() const               { return m_SensorType; }

        inline double           GetConeHalfAngle() const;

        inline double           GetHorizontalHalfAngle() const;

        inline double           GetVerticalHalfAngle() const;

    protected:

    //
    // Attribute.
    //
    protected:
        static atomic<int>      SensorID;               ///< Sensor Global Counter
        int                     m_ID;                   ///< Sensor ID
        string                  m_Name;                 ///< Sensor Name
        SensorType              m_SensorType;           ///< Sensor Type
        double                  m_HorizontalHalfAngle;  ///< Parallel to orbital plane , Used for Rectangular Type
        double                  m_VerticalHalfAngle;    ///< Vertical to the orbital plane, Used for Rectangular Type
        double                  m_ConeHalfAngle;        ///< Cone Half Angle, Used for SimpleConic Type

    };

}

#endif //SPSENSOR_H
