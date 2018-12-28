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
*   SpFacility.h
*
*   Purpose:
*
*         The Base Class of All Kinds of Facilities.
*
*
*   Last modified:
*
*   2019-12-26  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPFACILITY_H
#define SPFACILITY_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"
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
     * Date:2019-12-26
     * Description:
     *  Longitude [rad] Latitude [rad] Altitude [m]
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API Facility
    {
    public:
        explicit Facility();
        Facility(const string &name, const double longitude, const double latitude, const double altitude);
        Facility(const string &name, const GeodeticCoord &LLA);
        virtual ~Facility();

    public:
        inline void		SetName (const string &name)                    { m_Name = name; }

        inline void		SetGeodeticCoord (const GeodeticCoord &LLA)     { m_LonLatAltitude = LLA; }

        inline void		SetGeodeticCoord (const double longitude, const double latitude, const double altitude);

        inline int                  GetID() const                       { return FacilityID; }

        inline const string&        GetName() const                     { return m_Name; }

        inline const GeodeticCoord& GetGeodeticCoord() const            { return m_LonLatAltitude; }

        /********************************************************************/
        /// Insert/Remove a Sensor Belong to The Facility
        /// @Author     Niu Zhiyong
        /// @Date       2019-12-26
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
        static atomic<int>      FacilityID;             ///< Facility ID
        string                  m_Name;                 ///< Facility Name
        GeodeticCoord           m_LonLatAltitude;        ///< In The Coordinate System of Initialization

        int                     m_SensorNumber;
        vector<Sensor *>        m_SensorList;

    };


}

#endif //SPFACILITY_H
