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
* Date:2018-03-20
* Description:
*   SpPerturbation.h
*
*   Purpose:
*
*       Contains all Orbital Perturbation Without Gravity of Earth
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPPERTURBATION_H
#define SPPERTURBATION_H

#include "SpaceDSL_Global.h"
#include "SpAtmosphere.h"
#include "SpCoordSystem.h"
#include "SpJplEph.h"
#include "SpMath.h"
#include "SpConst.h"
#include "SpUtils.h"

#include <Eigen/Core>

using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of The Third Body Gravity Perturbation
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API ThirdBodyGravity
    {
    public:
        explicit ThirdBodyGravity();
        ThirdBodyGravity(SolarSysStarType thirdBodyStarType, SolarSysStarType centerStarType);
        virtual ~ThirdBodyGravity();

    public:

        void                SetThirdBodyStar(SolarSysStarType thirdBodyStarType);
        void                SetCenterStar(SolarSysStarType centerStarType);

        SolarSysStarType    GetThirdBodyStar();
        SolarSysStarType    GetCenterStar();

        //********************************************************************
        /// Calculation of Three-body Gravitational Acceleration of Point Mass Model
        /// @author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	Mjd_TT          Terrestrial Time (Modified Julian Date)
        /// @Param	pos             Satellite Position Vector in the inertial system
        /// @Return Acceleration
        //********************************************************************

        Vector3d    AccelPointMassGravity(double Mjd_TT, const Vector3d& pos);

    protected:
        double              m_GM;
        SolarSysStarType    m_ThirdBodyStarType;
        SolarSysStarType    m_CenterStarType;
        JplEphemeris        m_JPLEphemeris;


    };

    /*************************************************
     * Class type: The class of The Atmospheric Drag. Perturbation
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API AtmosphericDrag
    {
    public:

        explicit AtmosphericDrag();
        AtmosphericDrag(AtmosphereModel::AtmosphereModelType modelType, GeodeticCoordSystem *pGeodeticSystem);
        virtual ~AtmosphericDrag();

    public:

        //********************************************************************
        /// Calculation of Atmospheric Drag. Acceleration
        /// @author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param  Mjd_TT          Terrestrial Time (Modified Julian Date)
        /// @Param  pos             Satellite position vector in the inertial system [m]
        /// @Param  vel             Satellite velocity vector in the inertial system [m/s]
        /// @Param  ECIToTODMtx     Transformation matrix to true-of-date inertial system
        /// @Param  area            Cross-section [m^2]
        /// @Param  mass            Spacecraft mass [kg]
        /// @Param  dragCoef        Drag coefficient
        /// @Return Acceleration
        //********************************************************************
        Vector3d    AccelAtmosphericDrag(double Mjd_UTC, double Mjd_UT1, const Vector3d& pos, const Vector3d& vel,
                                         double area, double dragCoef, double mass,
                                         double f107A = 150, double f107 = 160, double ap[] = nullptr);

    protected:

        AtmosphereModel::AtmosphereModelType     m_AtmosphericModelType;
        GeodeticCoordSystem                      *m_pGeodeticSystem;
        AtmosphereModel                          *m_pAtmosphereModel;

    };

    /*************************************************
     * Class type: The class of Solar Radiation Pressure
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API SolarRadPressure
    {
    public:

        explicit SolarRadPressure();
        virtual ~SolarRadPressure();

    public:

        //********************************************************************
        /// Calculation of Solar Radiation Pressure Acceleration
        /// @author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param  Mjd_TT          Terrestrial Time (Modified Julian Date)
        /// @Param  pos             Satellite position vector in the inertial system [m]
        /// @Param  area            Cross-section [m^2]
        /// @Param  mass            Spacecraft mass [kg]
        /// @Param  solarCoef       Solar radiation pressure coefficient
        /// @Return Acceleration
        //********************************************************************
        Vector3d    AccelSolarRad(double Mjd_TT, const Vector3d& pos,
                                  double area, double solarCoef, double mass);

    protected:
        JplEphemeris m_JPLEphemeris;


    };

}

#endif //SPPERTURBATION_H
