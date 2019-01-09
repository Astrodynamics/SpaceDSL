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
*   SpFacility.h
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

#ifndef SPFACILITY_H
#define SPFACILITY_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"
#include "SpTarget.h"
#include "SpSensor.h"


#include <string>
#include <vector>
#include <atomic>

using namespace std;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL Facility
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API Facility : public PointTarget
    {
    public:
        explicit Facility();
        Facility(const string &name, const double longitude, const double latitude, const double altitude, const double minElevation);
        Facility(const string &name, const GeodeticCoord &LLA, const double minElevation);
        ~Facility() override;

    public:

        /********************************************************************/
        /// Insert/Remove a Sensor Belong to The Facility
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /// @Input
        /// @Param  halfAngle1    	Cone Half Angle/Horizontal Half Angle
        /// @Param	halfAngle2		Vertical Half Angle
        /// @Output
        /// @Param
        /**********************************************************************/
        void InsertSensor(const string &name, const Sensor::SensorType type, const double halfAngle1, const double halfAngle2 = 0);

        bool RemoveSensor(const int id);

    protected:


    //
    // Attribute.
    //
    protected:
        int                     m_SensorNumber;
        vector<Sensor *>        m_SensorList;

    };


}

#endif //SPFACILITY_H
