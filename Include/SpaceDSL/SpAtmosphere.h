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
        enum AtmosphereModelType
        {
            E_NotDefinedAtmosphereModel     = 0,
            E_1976StdAtmosphere             = 1,
            E_NRLMSISE00Atmosphere          = 2,
            E_HarrisPriesterAtmosphere      = 3,
        };

        AtmosphereModel();
        AtmosphereModel(AtmosphereModelType modelType);
        virtual ~AtmosphereModel();


        void                SetAtmosphereModelType(AtmosphereModelType modelType);
        AtmosphereModelType GetAtmosphereModelType();




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
        double      GetAtmosphereTemperature(double Mjd_UT1, double altitude, double latitude = 0, double longitude = 0,
                                             double f107A = 150, double f107 = 150, double ap[] = NULL, bool useDailyAp = true);

        //********************************************************************
        /// Get Atmosphere Pressure at Altitude
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	altitude		 m
        /// @Output
        /// @Return Pressure         Pa
        //********************************************************************
        double      GetAtmospherePressure(double Mjd_UT1, double altitude, double latitude = 0, double longitude = 0,
                                          double f107A = 150, double f107 = 150, double ap[] = NULL, bool useDailyAp = true);

        //********************************************************************
        /// Get Atmosphere Pressure at Altitude
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	altitude		m
        /// @Output
        /// @Return Density         kg/m^3
        //********************************************************************
        double      GetAtmosphereDensity(double Mjd_UT1, double altitude, double latitude = 0, double longitude = 0,
                                         double f107A = 150, double f107 = 150, double ap[] = NULL, bool useDailyAp = true);

    protected:
        ///=============================================
        /// 1976 Standard Atmosphere Calculator(USSA1976)
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// Note:
        ///     The Unit of Altitude is km.
        ///=============================================
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
        //********************************************************************
        void    GasIntegral(double Z_0, double Z_1, VectorXd &M,
                            VectorXd &sum_n, VectorXd &n_int);

        //********************************************************************
        /// Tau Integral Computation for Hydrogen Composition Program
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        ///  Note:
        ///     This program computes the value of Tau directly with the
        ///     integral done by hand and only the second integration limit
        ///     needing to be inputed
        //********************************************************************
        double      TauIntegral(double alt);

///-----------------------------------------------------------------------------------------------------------
///-----------------------------------------------------------------------------------------------------------

        ///===================================================
        /// NRLMSIS-00 Empirical Atmosphere Model Calculator
        /// Reencapsulate Dominik Brodowski The C version.
        /// @Author	Niu Zhiyong
        /// @Date	2018-06-05
        ///===================================================
        //********************************************************************
        /// NRLMSIS-00 Empirical Atmosphere Model Atmospheric Density and Temperature
        /// @Author	Niu Zhiyong
        /// @Date	2018-06-05
        /// @Input
        /// @Param	Mjd_UT1
        /// @Param	altitude	altitude (km)
        /// @Param	latitude	geodetic latitude (rad)
        /// @Param	longitude	geodetic longitude (rad)
        /// @Param	f107A		81 day average of F10.7 flux (centered on doy)
        /// @Param	f107		daily F10.7 flux for previous day
        /// @Param	ap			magnetic index
        //  	 	 	 		Array containing the following magnetic values:
        //  	 	 	 		  0 : daily AP
        //  	 	 	 		  1 : 3 hr AP index for current time
        //  	 	 	 		  2 : 3 hr AP index for 3 hrs before current time
        //  	 	 	 		  3 : 3 hr AP index for 6 hrs before current time
        //  	 	 	 		  4 : 3 hr AP index for 9 hrs before current time
        //  	 	 	 		  5 : Average of eight 3 hr AP indicies from 12 to 33 hrs
        //  	 	 	 		          prior to current time
        //  	 	 	 		  6 : Average of eight 3 hr AP indicies from 36 to 57 hrs
        //  	 	 	 		          prior to current time
        /// @Param	useDailyAp	true: Just Use Daily Ap, ap[0] = daily Ap, ap[1]~ap[6] Not Used
        ///                     false: Use 3 Hours Ap Mode, ap[0]~ap[6] Will be Used.
        /// @Output
        /// @Return				density (kg/m^3)
        //********************************************************************
        double	 GetNRLMSISE2000Density(double Mjd_UT1, double altitude, double latitude, double longitude,
                                        double f107A, double f107, double ap[7], bool useDailyAp);

        double	 GetNRLMSISE2000Temperature(double Mjd_UT1, double altitude, double latitude, double longitude,
                                            double f107A, double f107, double ap[7], bool useDailyAp);

    protected:

        AtmosphereModelType   m_AtmosphericModelType;
    };

}

#endif //SPATMOSPHERE_H
