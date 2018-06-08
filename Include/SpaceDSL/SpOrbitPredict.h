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
#include "SpConst.h"
#include "SpAtmosphere.h"
#include "SpGravity.h"
#include "SpTimeSystem.h"
#include "SpPerturbation.h"
#include "SpIntegration.h"

#include <Eigen/Core>


using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: Normalization Parameter
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API NormalizeParameter
    {
    public:
        NormalizeParameter();
        virtual ~NormalizeParameter();
    public:
        static double       GetLengthPara(SolarSysStarType centerStarType);
        static double       GetSpeedPara(SolarSysStarType centerStarType);
        static double       GetTimePara(SolarSysStarType centerStarType);
        static double       GetThrustPara(SolarSysStarType centerStarType, double mass);

    protected:

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
        /*************************************************
         * struct type: Third Body Gravity Sign
         * Author: Niu ZhiYong
         * Date:2018-03-20
         * Description:
        **************************************************/
        struct ThirdBodyGravitySign
        {
            bool                bIsUseMercuryGrav = false;
            bool                bIsUseVenusGrav   = false;
            bool                bIsUseEarthGrav   = false;
            bool                bIsUseMarsGrav    = false;
            bool                bIsUseJupiterGrav = false;
            bool                bIsUseSaturnGrav  = false;
            bool                bIsUseUranusGrav  = false;
            bool                bIsUseNeptuneGrav = false;
            bool                bIsUsePlutoGrav   = false;
            bool                bIsUseMoonGrav    = false;
            bool                bIsUseSunGrav     = false;
        };
        static ThirdBodyGravitySign DefaultThirdBodySign;

    public:
        /// Initializer() function must run before OrbitPredictConfig using!!!
        void        Initializer(double Mjd_UTC = 0, SolarSysStarType centerStarType = E_Earth, bool isUseNormalize = false,
                                GravityModel::GravModelType gravModelType = GravityModel::GravModelType::E_NotDefinedGravModel,
                                int maxDegree = 0, int maxOrder = 0,
                                ThirdBodyGravitySign thirdBodyGravSign = DefaultThirdBodySign,
                                AtmosphereModel::AtmosphereModelType atmModelType = AtmosphereModel::AtmosphereModelType::E_NotDefinedAtmosphereModel,
                                double dragCoef = 0, double dragArea = 0,
                                double SRPCoef = 0, double SRPArea = 0, bool isUseDrag = false, bool isUseSRP = false);

        bool                        IsInitialized();

        void                        SetCenterStarType(SolarSysStarType type);
        SolarSysStarType            GetCenterStarType() const;
        double                      GetCenterStarGM() const;

        void                        SetMJD_UTC(double Mjd_UTC);
        double                      GetMJD_UTC() const;
        double                      GetMJD_UT1() const;
        double                      GetMJD_TT() const;
        double                      GetMJD_TAI() const;


        double                      GetTAI_UTC() const;
        double                      GetUT1_UTC() const;
        double                      GetTT_UTC() const;

        double                      GetX_Pole() const;
        double                      GetY_Pole() const;

        void                        SetGravModelType(GravityModel::GravModelType type);
        GravityModel::GravModelType GetGravModelType() const;
        void                        SetGravMaxDegree(int maxDegree);
        int                         GetGravMaxDegree() const;
        void                        SetGravMaxOrder(int maxOrder);
        int                         GetGravMaxOrder() const;
        GravityModel                *GetGravityModel() const;

        void                        SetAtmosphereModelType(AtmosphereModel::AtmosphereModelType type);
        AtmosphereModel::AtmosphereModelType
                                    GetAtmosphereModelType() const;
        void                        SetDragCoef(double coef);
        double                      GetDragCoef() const;
        void                        SetDragArea(double area);
        double                      GetDragArea() const;

        void                        SetSRPCoef(double coef);
        double                      GetSRPCoef() const;
        void                        SetSRPArea(double area);
        double                      GetSRPArea() const;

        void                        SetThirdBodySign(OrbitPredictConfig::ThirdBodyGravitySign sign);
        OrbitPredictConfig::ThirdBodyGravitySign
                                    GetThirdBodySign() const;

        bool                        IsUseSRP() const;
        bool                        IsUseDrag() const;
        bool                        IsUseNormalize() const;



    protected:
        bool                        bIsInitialized = false;

        SolarSysStarType            m_CenterStarType;
        //Third Body Gravity Sign
        OrbitPredictConfig::ThirdBodyGravitySign
                                    m_ThirdBodySign;
        //Time Parameters
        IERSService                 m_IERSService;

        double                      m_MJD_UTC;			///< Modified Julian Date UTC(unit : day)
        double                      m_MJD_TT;			///< Modified Julian Date TT
        double                      m_MJD_TAI;
        double                      m_MJD_UT1;

        double                      m_TAI_UTC;          ///< LeapSeconds = TAI - UTC(unit : sec)
        double                      m_UT1_UTC;
        double                      m_TT_UTC;
        double                      m_X_Pole;
        double                      m_Y_Pole;


        //Gravity Parameters
        GravityModel::GravModelType m_GravModelType;	///< Gravity Model
        int                         m_MaxDegree;		///< Gravity Degree[n]
        int                         m_MaxOrder;			///< Degree Order  [m]
        GravityModel                *m_pGravityModel;

        //Atmosphere Parameters
        AtmosphereModel::AtmosphereModelType
                                    m_AtmModelType;
        double                      m_DragCoef;			///< drag coefficient
        double                      m_DragArea;         ///< drag term m2/kg
        //double                    m_F10p7;            ///< average F10.7
        //double                    m_DailyF10p7;       ///< daily F10.7
        //double                    m_Ap;               ///< geomagnetic index

        //Solar Radiation Parameters
        double                      m_SRPCoef;		///< Solar Radiation Pressure Coeff
        double                      m_SRPArea;          ///< Area for SRP

        //Perturbation Sign
        bool                        m_bIsUseSRP;        ///< Whether the use of Solar Radiation or not
        bool                        m_bIsUseDrag;       ///< Whether the use of Atmos. Drag. or not

        //Normalize Sign
        bool                        m_bIsUseNormalize;

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
        /// @Return
        //********************************************************************
        void OrbitStep (OrbitPredictConfig &predictConfig, double step, RungeKutta::IntegMethodType integType,
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
        OrbitPredictRightFunc(OrbitPredictConfig *pConfig);
        ~OrbitPredictRightFunc();
    public:
        /// @Param  t                   sec
        /// @Param	step                sec
        /// @Param  x                   m,m/s
        /// @Param  result              m,m/s
        void operator() (double t, const VectorXd &x, VectorXd&result) const override;

    protected:
        OrbitPredictConfig      *m_pOrbitPredictConfig;
        GravityModel            *m_pGravityModel;
        ThirdBodyGravity        *m_pThirdBodyGrva;
    };

    /*************************************************
     * Class type: Tow Body Orbit Prediction
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Orbit Prediction Algorithm and Function
    **************************************************/
    class SPACEDSL_API TwoBodyOrbitPredict
    {
    public:
        TwoBodyOrbitPredict();
        virtual ~TwoBodyOrbitPredict();
    public:

        //********************************************************************
        /// Using the Two Body Orbit Model to Calculate the Orbit, One Step ,Without Maneuver
        /// @author	Niu ZhiYong
        /// @Date	2018-03-20
        /// @Input
        /// @Param  centerStarType      Solar System Star Type
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
        void OrbitStep (OrbitPredictConfig &predictConfig, double step, RungeKutta::IntegMethodType integType,
                        double &mass, Vector3d &pos, Vector3d &vel);


    protected:

    };
    /*************************************************
     * Class type: Tow Body Orbit Prediction Right Function
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class TwoBodyOrbitRightFunc : public RightFunc
    {
    public:
        TwoBodyOrbitRightFunc(OrbitPredictConfig *pConfig);
        ~TwoBodyOrbitRightFunc();
    public:
        /// @Param  t                   sec
        /// @Param	step                sec
        /// @Param  x                   m,m/s
        /// @Param  result              m,m/s
        void operator() (double t, const VectorXd &x, VectorXd&result) const override;

    protected:

        OrbitPredictConfig      *m_pOrbitPredictConfig;
    };

	
}

#endif //SPORBITPREDICT_H
