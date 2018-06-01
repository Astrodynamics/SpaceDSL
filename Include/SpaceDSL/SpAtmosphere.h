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
*   SpAtmosphere.h
*
*   Purpose:
*
*       Atmosphere Model of Earth
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPATMOSPHERE_H
#define SPATMOSPHERE_H

#include "SpaceDSL_Global.h"

#include <Eigen/Core>

using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of Atmospheric Model
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API AtmosphereModel
    {
    public:
        enum SPACEDSL_API AtmosphereModelType
        {
            E_NotDefinedAtmosphereModel     = 0,
            E_1976StdAtmosphere             = 1,
            E_NRLMSISE00Atmosphere          = 2,
            E_HarrisPriesterAtmosphere      = 3,
        };

        AtmosphereModel();
        AtmosphereModel(AtmosphereModelType modelType);
        virtual ~AtmosphereModel();




    public:
        //********************************************************************
        /// Get Atmosphere Temperature at Altitude
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	altitude			m
        /// @Output
        /// @Return Temperature         K
        //********************************************************************
        double      GetAtmosphereTemperature(double altitude);

        //********************************************************************
        /// Get Atmosphere Pressure at Altitude
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	altitude		 m
        /// @Output
        /// @Return Pressure         Pa
        //********************************************************************
        double      GetAtmospherePressure(double altitude);

        //********************************************************************
        /// Get Atmosphere Pressure at Altitude
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	altitude		m
        /// @Output
        /// @Return Density         kg/m^3
        //********************************************************************
        double      GetAtmosphereDensity(double altitude);

    protected:

        //********************************************************************
        /// 1976 Standard Atmosphere Calculator(USSA1976)
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	alt			altitude in km
        /// @Output
        /// @Param  density
        /// @Param  temperature
        /// @Param  pressure
        /// @Param  soundVel
        /// @Return
        //********************************************************************

        VectorXd    GetUSSA1976Composition(double altitude, int stepNum = 100);//Only Valid Between 86 km and 1000 km

        double      GetUSSA1976Density(double altitude);

        double      GetUSSA1976Temperature(double altitude);

        double      GetUSSA1976Pressure(double altitude);

        double      GetUSSA1976SoundVel(double altitude);

        double      GetUSSA1976DynamicVis(double altitude);     //Dynamic Viscosity

        double      GetUSSA1976KinematicVis(double altitude);   //Kinematic Viscosity

        double      GetUSSA1976ThermalCoeff(double altitude);   //Thermal Conductivity Coefficient


        //********************************************************************
        /// Gas Integral Program
        /// Integral values computed using 5-point Simpsons Rule
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	Z_0
        /// @Param	Z_1
        /// @Param	M
        /// @Param	sum_n
        /// @Param	n_int
        /// @Output
        /// @Param  n_int
        /// @Return
        //********************************************************************
        void    GasIntegral(double Z_0, double Z_1, VectorXd &M,
                            VectorXd &sum_n, VectorXd &n_int);

        //********************************************************************
        /// Tau Integral Computation for Hydrogen Composition Program
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	alt			altitude in meter
        /// @Output
        /// @Return TAU:    Integral Value
        ///  Note:
        ///     This program computes the value of Tau directly with the
        ///     integral done by hand and only the second integration limit
        ///     needing to be inputed
        //********************************************************************
        double      TauIntegral(double alt);

    protected:

        AtmosphereModelType   m_AtmosphericModelType;
    };

}

#endif //SPATMOSPHERE_H
