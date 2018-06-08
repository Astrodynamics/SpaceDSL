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
*   SpOrbitPredict.cpp
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

#include "SpaceDSL/SpOrbitPredict.h"
#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

    /*************************************************
     * Class type: Normalization Parameter
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    NormalizeParameter::NormalizeParameter()
    {

    }

    NormalizeParameter::~NormalizeParameter()
    {

    }

    double NormalizeParameter::GetLengthPara(SolarSysStarType centerStarType)
    {
        switch (centerStarType)
        {
        case E_Earth:
            return EarthRadius;
            break;
        case E_Sun:
            return AU;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "NormalizeParameter: Center Star Type Unsupport ");
            break;
        }
    }

    double NormalizeParameter::GetSpeedPara(SolarSysStarType centerStarType)
    {
        switch (centerStarType)
        {
        case E_Earth:
            return sqrt(GM_Earth/EarthRadius);
            break;
        case E_Sun:
            return sqrt(GM_Sun/AU);
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "NormalizeParameter: Center Star Type Unsupport ");
            break;
        }
    }

    double NormalizeParameter::GetTimePara(SolarSysStarType centerStarType)
    {
        switch (centerStarType)
        {
        case E_Earth:
            return EarthRadius/sqrt(GM_Earth/EarthRadius);
            break;
        case E_Sun:
            return AU/sqrt(GM_Sun/AU);
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "NormalizeParameter: Center Star Type Unsupport ");
            break;
        }
    }

    double NormalizeParameter::GetThrustPara(SolarSysStarType centerStarType, double mass)
    {
        switch (centerStarType)
        {
        case E_Earth:
            return  (GM_Earth*mass)/pow(EarthRadius,2);
            break;
        case E_Sun:
            return (GM_Sun*mass)/pow(AU,2);
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "NormalizeParameter: Center Star Type Unsupport ");
            break;
        }
    }


    /*************************************************
     * Class type: Orbit Prediction Parameters
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Orbit Prediction Algorithm and Function
    **************************************************/
    OrbitPredictConfig::ThirdBodyGravitySign OrbitPredictConfig::DefaultThirdBodySign
                        = OrbitPredictConfig::ThirdBodyGravitySign();
    OrbitPredictConfig::OrbitPredictConfig()
    {
        m_pGravityModel = NULL;
    }

    OrbitPredictConfig::~OrbitPredictConfig()
    {
        if (m_pGravityModel != NULL)
            delete m_pGravityModel;
    }

    void OrbitPredictConfig::Initializer(double Mjd_UTC, SolarSysStarType centerStarType, bool isUseNormalize,
                                         GravityModel::GravModelType gravModelType, int maxDegree, int maxOrder,
                                         ThirdBodyGravitySign thirdBodyGravSign,
                                         AtmosphereModel::AtmosphereModelType atmModelType, double dragCoef, double dragArea,
                                         double SRPCoef, double SRPArea, bool isUseDrag, bool isUseSRP)
    {
        m_CenterStarType    = centerStarType;

        m_MJD_UTC           = Mjd_UTC;

        m_TAI_UTC           = m_IERSService.GetValue(Mjd_UTC, "leapseconds");//TAI-UTC
        m_UT1_UTC           = m_IERSService.GetValue(Mjd_UTC, "UT1-UTC");
        m_TT_UTC            = IERSService::TT_TAI + m_TAI_UTC;

        m_MJD_TT            = Mjd_UTC + m_TT_UTC/DayToSec;
        m_MJD_TAI           = m_MJD_TT - IERSService::TT_TAI/DayToSec;
        m_MJD_UT1           = Mjd_UTC + m_UT1_UTC/DayToSec;

        m_X_Pole            = m_IERSService.GetValue(Mjd_UTC, "x_pole");
        m_Y_Pole            = m_IERSService.GetValue(Mjd_UTC, "y_pole");

        m_GravModelType     = gravModelType;
        m_MaxDegree         = maxDegree;
        m_MaxOrder          = maxOrder;
        if (gravModelType != GravityModel::GravModelType::E_NotDefinedGravModel)
            m_pGravityModel     = new GravityModel(gravModelType);

        m_AtmModelType      = atmModelType;
        m_DragCoef          = dragCoef;
        m_DragArea          = dragArea;

        m_SRPCoef           = SRPCoef;
        m_SRPArea           = SRPArea;

        m_bIsUseDrag        = isUseDrag;
        m_bIsUseSRP         = isUseSRP;
        m_ThirdBodySign     = thirdBodyGravSign;
        m_bIsUseNormalize   = isUseNormalize;

        //Data Cheak
        switch (centerStarType)
        {
        case E_Mercury:
            m_ThirdBodySign.bIsUseMercuryGrav = false;
            break;
        case E_Venus:
            m_ThirdBodySign.bIsUseVenusGrav = false;
            break;
        case E_Earth:
            m_ThirdBodySign.bIsUseEarthGrav = false;
            break;
        case E_Mars:
            m_ThirdBodySign.bIsUseMarsGrav = false;
            break;
        case E_Jupiter:
            m_ThirdBodySign.bIsUseJupiterGrav = false;
            break;
        case E_Saturn:
            m_ThirdBodySign.bIsUseSaturnGrav = false;
            break;
        case E_Uranus:
            m_ThirdBodySign.bIsUseUranusGrav = false;
            break;
        case E_Neptune:
            m_ThirdBodySign.bIsUseNeptuneGrav = false;
            break;
        case E_Pluto:
            m_ThirdBodySign.bIsUsePlutoGrav = false;
            break;
        case E_Moon:
            m_ThirdBodySign.bIsUseMoonGrav = false;
            break;
        case E_Sun:
            m_ThirdBodySign.bIsUseSunGrav = false;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: SolarSysStarType Unsupport ");
            break;
        }
        bIsInitialized = true;
    }

    bool OrbitPredictConfig::IsInitialized()
    {
        return bIsInitialized;
    }

    void OrbitPredictConfig::SetCenterStarType(SolarSysStarType type)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_CenterStarType = type;
    }

    SolarSysStarType OrbitPredictConfig::GetCenterStarType() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_CenterStarType;
    }

    double OrbitPredictConfig::GetCenterStarGM() const
    {
        switch (m_CenterStarType)
        {
        case E_Mercury:
            return GM_Mercury;
            break;
        case E_Venus:
            return GM_Venus;
            break;
        case E_Earth:
            return GM_Earth;
            break;
        case E_Mars:
            return GM_Mars;
            break;
        case E_Jupiter:
            return GM_Jupiter;
            break;
        case E_Saturn:
            return GM_Saturn;
            break;
        case E_Uranus:
            return GM_Uranus;
            break;
        case E_Neptune:
            return GM_Neptune;
            break;
        case E_Pluto:
            return GM_Pluto;
            break;
        case E_Moon:
            return GM_Moon;
            break;
        case E_Sun:
            return GM_Sun;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"OrbitPredictConfig:SolarSysStarType Unsupport ");
            break;
        }
    }

    void OrbitPredictConfig::SetMJD_UTC(double Mjd_UTC)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_MJD_UTC           = Mjd_UTC;

        m_TAI_UTC           = m_IERSService.GetValue(Mjd_UTC, "leapseconds");//TAI-UTC
        m_UT1_UTC           = m_IERSService.GetValue(Mjd_UTC, "UT1-UTC");
        m_TT_UTC            = IERSService::TT_TAI + m_TAI_UTC;

        m_MJD_TT            = Mjd_UTC + m_TT_UTC/DayToSec;
        m_MJD_TAI           = m_MJD_TT - IERSService::TT_TAI/DayToSec;
        m_MJD_UT1           = Mjd_UTC + m_UT1_UTC/DayToSec;
    }

    double OrbitPredictConfig::GetMJD_UTC() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_MJD_UTC;
    }

    double OrbitPredictConfig::GetMJD_UT1() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_MJD_UT1;
    }

    double OrbitPredictConfig::GetMJD_TT() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_MJD_TT;
    }

    double OrbitPredictConfig::GetMJD_TAI() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_MJD_TAI;
    }

    double OrbitPredictConfig::GetTAI_UTC() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_TAI_UTC;
    }

    double OrbitPredictConfig::GetUT1_UTC() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_UT1_UTC;
    }

    double OrbitPredictConfig::GetTT_UTC() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_TT_UTC;
    }

    double OrbitPredictConfig::GetX_Pole() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_X_Pole;
    }

    double OrbitPredictConfig::GetY_Pole() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_Y_Pole;
    }

    void OrbitPredictConfig::SetGravModelType(GravityModel::GravModelType type)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_GravModelType = type;
    }

    GravityModel::GravModelType OrbitPredictConfig::GetGravModelType() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");

        return m_GravModelType;
    }

    void OrbitPredictConfig::SetGravMaxDegree(int maxDegree)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_MaxDegree = maxDegree;
    }

    int OrbitPredictConfig::GetGravMaxDegree() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_MaxDegree;
    }

    void OrbitPredictConfig::SetGravMaxOrder(int maxOrder)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_MaxOrder = maxOrder;
    }

    int OrbitPredictConfig::GetGravMaxOrder() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_MaxOrder;
    }

    GravityModel *OrbitPredictConfig::GetGravityModel() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_pGravityModel;
    }

    void OrbitPredictConfig::SetAtmosphereModelType(AtmosphereModel::AtmosphereModelType type)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_AtmModelType = type;
    }

    AtmosphereModel::AtmosphereModelType OrbitPredictConfig::GetAtmosphereModelType() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_AtmModelType;
    }

    void OrbitPredictConfig::SetDragCoef(double coef)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_DragCoef = coef;
    }

    double OrbitPredictConfig::GetDragCoef() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_DragCoef;
    }

    void OrbitPredictConfig::SetDragArea(double area)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_DragArea = area;
    }

    double OrbitPredictConfig::GetDragArea() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_DragArea;
    }

    void OrbitPredictConfig::SetSRPCoef(double coef)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_SRPCoef = coef;
    }

    double OrbitPredictConfig::GetSRPCoef() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_SRPCoef;
    }

    void OrbitPredictConfig::SetSRPArea(double area)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_SRPArea = area;
    }

    double OrbitPredictConfig::GetSRPArea() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        return m_SRPArea;
    }

    void OrbitPredictConfig::SetThirdBodySign(OrbitPredictConfig::ThirdBodyGravitySign sign)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");
        m_ThirdBodySign = sign;
        //Data Cheak
        switch (m_CenterStarType)
        {
        case E_Mercury:
            m_ThirdBodySign.bIsUseMercuryGrav = false;
            break;
        case E_Venus:
            m_ThirdBodySign.bIsUseVenusGrav = false;
            break;
        case E_Earth:
            m_ThirdBodySign.bIsUseEarthGrav = false;
            break;
        case E_Mars:
            m_ThirdBodySign.bIsUseMarsGrav = false;
            break;
        case E_Jupiter:
            m_ThirdBodySign.bIsUseJupiterGrav = false;
            break;
        case E_Saturn:
            m_ThirdBodySign.bIsUseSaturnGrav = false;
            break;
        case E_Uranus:
            m_ThirdBodySign.bIsUseUranusGrav = false;
            break;
        case E_Neptune:
            m_ThirdBodySign.bIsUseNeptuneGrav = false;
            break;
        case E_Pluto:
            m_ThirdBodySign.bIsUsePlutoGrav = false;
            break;
        case E_Moon:
            m_ThirdBodySign.bIsUseMoonGrav = false;
            break;
        case E_Sun:
            m_ThirdBodySign.bIsUseSunGrav = false;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: SolarSysStarType Unsupport ");
            break;
        }

    }

    OrbitPredictConfig::ThirdBodyGravitySign OrbitPredictConfig::GetThirdBodySign() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");

        return m_ThirdBodySign;
    }

    bool OrbitPredictConfig::IsUseSRP() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");

        return m_bIsUseSRP;
    }

    bool OrbitPredictConfig::IsUseDrag() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");

        return m_bIsUseDrag;
    }

    bool OrbitPredictConfig::IsUseNormalize() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: m_OrbitPredictConfig UnInitialized! ");

        return m_bIsUseNormalize;
    }

    /*************************************************
     * Class type: Orbit Prediction
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Orbit Prediction Algorithm and Function
    **************************************************/
    OrbitPredict::OrbitPredict()
    {

    }

    OrbitPredict::~OrbitPredict()
    {

    }

    void OrbitPredict::OrbitStep(OrbitPredictConfig &predictConfig, double step, RungeKutta::IntegMethodType integType,
                                 double &mass, Vector3d &pos, Vector3d &vel)
    {
        if (predictConfig.IsInitialized() == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");

        double Mjd_TT = predictConfig.GetMJD_TT();
        VectorXd x(7), result(7);
        result.fill(0);
        x(0) = pos(0);  x(1) = pos(1);  x(2) = pos(2);
        x(3) = vel(0);  x(4) = vel(1);  x(5) = vel(2);  x(6) = mass;

        OrbitPredictRightFunc rightFunc(&predictConfig);

        RungeKutta RK(integType);
        RK.OneStep(rightFunc, Mjd_TT*DayToSec ,x, step, result);
        pos(0) = result(0);     pos(1) = result(1);     pos(2) = result(2);
        vel(0) = result(3);     vel(1) = result(4);     vel(2) = result(5);
        mass   = result(6);

    }

    /*************************************************
     * Class type: Orbit Prediction Right Function
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    OrbitPredictRightFunc::OrbitPredictRightFunc(OrbitPredictConfig *pConfig)
    {
        m_pOrbitPredictConfig = pConfig;
        m_pGravityModel = m_pOrbitPredictConfig->GetGravityModel();
        m_pThirdBodyGrva = new ThirdBodyGravity();
    }

    OrbitPredictRightFunc::~OrbitPredictRightFunc()
    { 
        if (m_pThirdBodyGrva != NULL)
            delete m_pThirdBodyGrva;
    }

    void OrbitPredictRightFunc::operator()(double t, const VectorXd &x, VectorXd &result) const
    {
        double Mjd_TT = t * SecToDay;
        Vector3d acceleration;
        acceleration.fill(0);
        Vector3d pos;
        pos(0) = x(0);  pos(1) = x(1);  pos(2) = x(2);
        Vector3d vel;
        vel(0) = x(3);  vel(1) = x(4);  vel(2) = x(5);

        double Mjd_UT1 = Mjd_TT + (m_pOrbitPredictConfig->GetUT1_UTC() - m_pOrbitPredictConfig->GetTT_UTC())/DayToSec;
        double Mjd_UTC = Mjd_TT - m_pOrbitPredictConfig->GetTT_UTC()/DayToSec;
        Matrix3d ECIToTODMtx = NutationMatrix(Mjd_TT) * PrecessMatrix(MJD_J2000,Mjd_TT);
        Matrix3d ECItoBFCMtx = PoleMatrix(m_pOrbitPredictConfig->GetX_Pole()*PI/180/360, m_pOrbitPredictConfig->GetY_Pole()*PI/180/360) *
                                GWHourAngMatrix(Mjd_UT1) * ECIToTODMtx;//

        /// Harmonic Gravity
        acceleration += m_pGravityModel->AccelHarmonicGravity(pos, ECItoBFCMtx,
                                                            m_pOrbitPredictConfig->GetGravMaxDegree(),
                                                            m_pOrbitPredictConfig->GetGravMaxOrder());
        /// Atmospheric Drag
        if (m_pOrbitPredictConfig->IsUseDrag())
        {
            AtmosphericDrag atmoDrag(m_pOrbitPredictConfig->GetAtmosphereModelType());
            acceleration += atmoDrag.AccelAtmosphericDrag(Mjd_TT, pos, vel, ECIToTODMtx,
                                                            m_pOrbitPredictConfig->GetDragArea(),
                                                            m_pOrbitPredictConfig->GetDragCoef(),
                                                            x(6));
        }
        /// Solar Rad. Pressure
        if (m_pOrbitPredictConfig->IsUseSRP())
        {
            SolarRadPressure SRPPressure;
            acceleration += SRPPressure.AccelSolarRad(Mjd_TT, pos,
                                                      m_pOrbitPredictConfig->GetSRPArea(),
                                                      m_pOrbitPredictConfig->GetDragCoef(),
                                                      x(6));
        }

        /// Third Body Gravity
        m_pThirdBodyGrva->SetCenterStar(m_pOrbitPredictConfig->GetCenterStarType());
        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseEarthGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Earth);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseJupiterGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Jupiter);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseMarsGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Mars);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseMercuryGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Mercury);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseMoonGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Moon);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseNeptuneGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Neptune);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUsePlutoGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Pluto);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseSaturnGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Saturn);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseSunGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Sun);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseUranusGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Uranus);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->GetThirdBodySign().bIsUseVenusGrav)
        {
            m_pThirdBodyGrva->SetThirdBodyStar(E_Venus);
            acceleration += m_pThirdBodyGrva->AccelPointMassGravity(Mjd_TT, pos);
        }

        if (m_pOrbitPredictConfig->IsUseNormalize())
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictRightFunc: High Precision Orbit Prediction does not Support Normalize! ");
        }
        /// make the right function
        for (int i = 0; i < 3; ++i)
        {
            result(i) = vel(i);
        }
        result(3) = acceleration(0);
        result(4) = acceleration(1);
        result(5) = acceleration(2);
        result(6) = 1;
    }

    /*************************************************
     * Class type: Tow Body Orbit Prediction
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Orbit Prediction Algorithm and Function
    **************************************************/
    TwoBodyOrbitPredict::TwoBodyOrbitPredict()
    {

    }

    TwoBodyOrbitPredict::~TwoBodyOrbitPredict()
    {

    }

    void TwoBodyOrbitPredict::OrbitStep(OrbitPredictConfig &predictConfig, double step, RungeKutta::IntegMethodType integType,
                                                  double &mass, Vector3d &pos, Vector3d &vel)
    {
        if (predictConfig.IsInitialized() == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "TwoBodyOrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        if (predictConfig.IsUseNormalize())
        {
            SolarSysStarType centerStarType = predictConfig.GetCenterStarType();
            double Mjd_TT = predictConfig.GetMJD_TT()/NormalizeParameter::GetTimePara(centerStarType);
            double norm_step = step/NormalizeParameter::GetTimePara(centerStarType);
            VectorXd x(7), result(7);
            result.fill(0);
            x(0) = pos(0)/NormalizeParameter::GetLengthPara(centerStarType);
            x(1) = pos(1)/NormalizeParameter::GetLengthPara(centerStarType);
            x(2) = pos(2)/NormalizeParameter::GetLengthPara(centerStarType);

            x(3) = vel(0)/NormalizeParameter::GetSpeedPara(centerStarType);
            x(4) = vel(1)/NormalizeParameter::GetSpeedPara(centerStarType);
            x(5) = vel(2)/NormalizeParameter::GetSpeedPara(centerStarType);

            x(6) = mass;

            TwoBodyOrbitRightFunc rightFunc(&predictConfig);
            RungeKutta RK(integType);
            RK.OneStep(rightFunc, Mjd_TT*DayToSec ,x, norm_step, result);

            pos(0) = result(0)*NormalizeParameter::GetLengthPara(centerStarType);
            pos(1) = result(1)*NormalizeParameter::GetLengthPara(centerStarType);
            pos(2) = result(2)*NormalizeParameter::GetLengthPara(centerStarType);

            vel(0) = result(3)*NormalizeParameter::GetSpeedPara(centerStarType);
            vel(1) = result(4)*NormalizeParameter::GetSpeedPara(centerStarType);
            vel(2) = result(5)*NormalizeParameter::GetSpeedPara(centerStarType);

            mass   = result(6);
        }
        else
        {
            double Mjd_TT = predictConfig.GetMJD_TT();
            VectorXd x(7), result(7);
            result.fill(0);
            x(0) = pos(0);  x(1) = pos(1);  x(2) = pos(2);
            x(3) = vel(0);  x(4) = vel(1);  x(5) = vel(2);  x(6) = mass;

            TwoBodyOrbitRightFunc rightFunc(&predictConfig);
            RungeKutta RK(integType);
            RK.OneStep(rightFunc, Mjd_TT*DayToSec ,x, step, result);

            pos(0) = result(0);     pos(1) = result(1);     pos(2) = result(2);
            vel(0) = result(3);     vel(1) = result(4);     vel(2) = result(5);
            mass   = result(6);
        }

    }

    /*************************************************
     * Class type: Tow Body Orbit Prediction Right Function
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    TwoBodyOrbitRightFunc::TwoBodyOrbitRightFunc(OrbitPredictConfig *pConfig)
    {
        m_pOrbitPredictConfig = pConfig;
    }

    TwoBodyOrbitRightFunc::~TwoBodyOrbitRightFunc()
    {

    }

    void TwoBodyOrbitRightFunc::operator()(double t, const VectorXd &x, VectorXd &result) const
    {
        Vector3d pos;
        pos(0) = x(0);  pos(1) = x(1);  pos(2) = x(2);
        Vector3d vel;
        vel(0) = x(3);  vel(1) = x(4);  vel(2) = x(5);
        double r = pos.norm();
        if (m_pOrbitPredictConfig->IsUseNormalize())
        {
            /// make the right function
            for (int i = 0; i < 3; ++i)
            {
                result(i) = vel(i);
            }
            result(3) = -pos(0)/pow(r,3);
            result(4) = -pos(1)/pow(r,3);
            result(5) = -pos(2)/pow(r,3);
            result(6) = 1;
        }
        else
        {
            double GM = m_pOrbitPredictConfig->GetCenterStarGM();
            /// make the right function
            for (int i = 0; i < 3; ++i)
            {
                result(i) = vel(i);
            }
            result(3) = -GM*pos(0)/pow(r,3);
            result(4) = -GM*pos(1)/pow(r,3);
            result(5) = -GM*pos(2)/pow(r,3);
            result(6) = 1;
        }


    }

}

