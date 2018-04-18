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
*   SpOrbitPredict.h
*
*   Purpose:
*
*       Orbit Prediction Algorithm and Function
*
*
*   Last modified:
*
*   2018-03-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPORBITPREDICT_H
#define SPORBITPREDICT_H

#include "SpaceDSL_Global.h"
#include "SpAtmosphere.h"
#include "SpGravity.h"
#include "SpPerturbation.h"
#include "SpIntegration.h"

#include <Eigen/Core>


using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * struct type: Third Body Gravity Sign
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    struct ThirdBodyGravitySign
    {
        bool                m_bIsUseMercuryGrav = false;
        bool                m_bIsUseVenusGrav   = false;
        bool                m_bIsUseEarthGrav   = false;
        bool                m_bIsUseMarsGrav    = false;
        bool                m_bIsUseJupiterGrav = false;
        bool                m_bIsUseSaturnGrav  = false;
        bool                m_bIsUseUranusGrav  = false;
        bool                m_bIsUseNeptuneGrav = false;
        bool                m_bIsUsePlutoGrav   = false;
        bool                m_bIsUseMoonGrav    = false;
        bool                m_bIsUseSunGrav     = false;
    };

    /*************************************************
     * Class type: Orbit Prediction Parameters
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API OrbitPredictConfig
    {
    public:
        OrbitPredictConfig();
        virtual ~OrbitPredictConfig();

    public:
        /// Initializer() function must run before OrbitPredictConfig using!!!
        void        Initializer(SolarSysStarType centerStarType, double Mjd_UTC,
                                GravModelType gravModelType, int maxDegree, int maxOrder,
                                AtmosphereModelType atmModelType, double dragCoef, double dragArea,
                                double SRPCoef, double SRPArea, bool isUseDrag, bool isUseSRP,
                                ThirdBodyGravitySign thirdBodySign);

        bool                    IsInitialized();

        void                    SetCenterStarType(SolarSysStarType type);
        SolarSysStarType        GetCenterStarType() const;

        void                    SetMJD_UTC(double Mjd_UTC);
        double                  GetMJD_UTC() const;
        void                    SetMJD_TT(double Mjd_TT);
        double                  GetMJD_TT() const;

        void                    SetGravModelType(GravModelType type);
        GravModelType           GetGravModelType() const;
        void                    SetGravMaxDegree(int maxDegree);
        int                     GetGravMaxDegree() const;
        void                    SetGravMaxOrder(int maxOrder);
        int                     GetGravMaxOrder() const;

        void                    SetAtmosphereModelType(AtmosphereModelType type);
        AtmosphereModelType     GetAtmosphereModelType() const;
        void                    SetDragCoef(double coef);
        double                  GetDragCoef() const;
        void                    SetDragArea(double area);
        double                  GetDragArea() const;

        void                    SetSRPCoef(double coef);
        double                  GetSRPCoef() const;
        void                    SetSRPArea(double area);
        double                  GetSRPArea() const;

        void                    SetThirdBodySign(ThirdBodyGravitySign sign);
        ThirdBodyGravitySign    GetThirdBodySign() const;
        bool                    IsUseSRP() const;
        bool                    IsUseDrag() const;



    protected:
        bool                    bIsInitialized = false;
        SolarSysStarType        m_CenterStarType;
        double                  m_MJD_UTC;			///< Modified Julian Date UTC
        double                  m_MJD_TT;			///< Modified Julian Date TT

        //Gravity Parameters
        GravModelType           m_GravModelType;	///< Gravity Model
        int                     m_MaxDegree;		///< Gravity Degree[n]
        int                     m_MaxOrder;			///< Degree Order  [m]

        //Atmosphere Parameters
        AtmosphereModelType     m_AtmModelType;
        double                  m_DragCoef;			///< drag coefficient
        double                  m_DragArea;         ///< drag term m2/kg
        //double                m_F10p7;            ///< average F10.7
        //double                m_DailyF10p7;       ///< daily F10.7
        //double                m_Ap;               ///< geomagnetic index

        //Solar Radiation Parameters
        double                  m_SRPCoef;		///< Solar Radiation Pressure Coeff
        double                  m_SRPArea;          ///< Area for SRP

        //Perturbation Sign
        bool                    m_bIsUseSRP;        ///< Whether the use of Solar Radiation or not
        bool                    m_bIsUseDrag;       ///< Whether the use of Atmos. Drag. or not

        //Third Body Gravity Sign
        ThirdBodyGravitySign    m_ThirdBodySign;

    };

    /*************************************************
     * Class type: Orbit Prediction
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Orbit Prediction Algorithm and Function
    **************************************************/
    class SPACEDSL_API OrbitPredict
    {
    public:
        OrbitPredict();
        virtual ~OrbitPredict();
    public:

        //********************************************************************
        /// Using the Perturbation Model to Calculate the Orbit, One Step ,Without Maneuver
        /// @author	Niu ZhiYong
        /// @Date	2018-03-20
        /// @Input
        /// @Param  predictConfig       Orbit Prediction Parameters
        /// @Param	step                sec
        /// @Param	integType           IntegMethodType
        /// @In/Out
        /// @Param	mass                kg
        /// @Param	pos                 m
        /// @Param	vel                 m/s
        /// @Output
        /// @Param	accel               m/s^2
        /// @Return
        //********************************************************************
        void OrbitStep (OrbitPredictConfig predictConfig, double step, IntegMethodType integType,
                        double &mass, Vector3d &pos, Vector3d &vel);


    protected:

    };
    /*************************************************
     * Class type: Orbit Prediction Right Function
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class OrbitPredictRightFunc : public RightFunc
    {
    public:
        OrbitPredictRightFunc(OrbitPredictConfig config);
        ~OrbitPredictRightFunc();
    public:
        /// @Param  t                   sec
        /// @Param	step                sec
        /// @Param  x                   m,m/s
        /// @Param  result              m,m/s
        void operator() (double t, const VectorXd &x, VectorXd&result) const override;

    protected:

        OrbitPredictConfig      m_OrbitPredictConfig;
    };

	
}

#endif //SPORBITPREDICT_H
