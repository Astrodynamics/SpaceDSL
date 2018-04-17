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
#include "SpaceDSL/SpTimeSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

    /*************************************************
     * Class type: Orbit Prediction Parameters
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  Orbit Prediction Algorithm and Function
    **************************************************/
    OrbitPredictConfig::OrbitPredictConfig()
    {

    }

    OrbitPredictConfig::~OrbitPredictConfig()
    {

    }

    void OrbitPredictConfig::Initializer(SolarSysStarType centerStarType, double Mjd_UTC,
                                           GravModelType gravModelType, int maxDegree, int maxOrder,
                                           AtmosphereModelType atmModelType, double dragCoef, double dragArea,
                                           double SRPCoef, double SRPArea, bool isUseDrag, bool isUseSRP,
                                           ThirdBodyGravitySign thirdBodySign)
    {
        m_CenterStarType    = centerStarType;
        m_MJD_UTC           = Mjd_UTC;
        m_MJD_TT            = Mjd_UTC;

        m_GravModelType     = gravModelType;
        m_MaxDegree         = maxDegree;
        m_MaxOrder          = maxOrder;

        m_AtmModelType      = atmModelType;
        m_DragCoef          = dragCoef;
        m_DragArea          = dragArea;

        m_SRPCoef           = SRPCoef;
        m_SRPArea           = SRPArea;

        m_bIsUseDrag        = isUseDrag;
        m_bIsUseSRP         = isUseSRP;
        m_ThirdBodySign     = thirdBodySign;

        //Data Cheak
        switch (centerStarType)
        {
        case E_Mercury:
            m_ThirdBodySign.m_bIsUseMercuryGrav = false;
            break;
        case E_Venus:
            m_ThirdBodySign.m_bIsUseVenusGrav = false;
            break;
        case E_Earth:
            m_ThirdBodySign.m_bIsUseEarthGrav = false;
            break;
        case E_Mars:
            m_ThirdBodySign.m_bIsUseMarsGrav = false;
            break;
        case E_Jupiter:
            m_ThirdBodySign.m_bIsUseJupiterGrav = false;
            break;
        case E_Saturn:
            m_ThirdBodySign.m_bIsUseSaturnGrav = false;
            break;
        case E_Uranus:
            m_ThirdBodySign.m_bIsUseUranusGrav = false;
            break;
        case E_Neptune:
            m_ThirdBodySign.m_bIsUseNeptuneGrav = false;
            break;
        case E_Pluto:
            m_ThirdBodySign.m_bIsUsePlutoGrav = false;
            break;
        case E_Moon:
            m_ThirdBodySign.m_bIsUseMoonGrav = false;
            break;
        case E_Sun:
            m_ThirdBodySign.m_bIsUseSunGrav = false;
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
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_CenterStarType = type;
    }

    SolarSysStarType OrbitPredictConfig::GetCenterStarType() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_CenterStarType;
    }

    void OrbitPredictConfig::SetMJD_UTC(double Mjd_UTC)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_MJD_UTC = Mjd_UTC;
    }

    double OrbitPredictConfig::GetMJD_UTC() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_MJD_UTC;
    }

    void OrbitPredictConfig::SetMJD_TT(double Mjd_TT)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_MJD_TT = Mjd_TT;
    }

    double OrbitPredictConfig::GetMJD_TT() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_MJD_TT;
    }

    void OrbitPredictConfig::SetGravModelType(GravModelType type)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_GravModelType = type;
    }

    GravModelType OrbitPredictConfig::GetGravModelType() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");

        return m_GravModelType;
    }

    void OrbitPredictConfig::SetGravMaxDegree(int maxDegree)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_MaxDegree = maxDegree;
    }

    int OrbitPredictConfig::GetGravMaxDegree() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_MaxDegree;
    }

    void OrbitPredictConfig::SetGravMaxOrder(int maxOrder)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_MaxOrder = maxOrder;
    }

    int OrbitPredictConfig::GetGravMaxOrder() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_MaxOrder;
    }

    void OrbitPredictConfig::SetAtmosphereModelType(AtmosphereModelType type)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_AtmModelType = type;
    }

    AtmosphereModelType OrbitPredictConfig::GetAtmosphereModelType() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_AtmModelType;
    }

    void OrbitPredictConfig::SetDragCoef(double coef)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_DragCoef = coef;
    }

    double OrbitPredictConfig::GetDragCoef() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_DragCoef;
    }

    void OrbitPredictConfig::SetDragArea(double area)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_DragArea = area;
    }

    double OrbitPredictConfig::GetDragArea() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_DragArea;
    }

    void OrbitPredictConfig::SetSRPCoef(double coef)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_SRPCoef = coef;
    }

    double OrbitPredictConfig::GetSRPCoef() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_SRPCoef;
    }

    void OrbitPredictConfig::SetSRPArea(double area)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_SRPArea = area;
    }

    double OrbitPredictConfig::GetSRPArea() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        return m_SRPArea;
    }

    void OrbitPredictConfig::SetThirdBodySign(ThirdBodyGravitySign sign)
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");
        m_ThirdBodySign = sign;
        //Data Cheak
        switch (m_CenterStarType)
        {
        case E_Mercury:
            m_ThirdBodySign.m_bIsUseMercuryGrav = false;
            break;
        case E_Venus:
            m_ThirdBodySign.m_bIsUseVenusGrav = false;
            break;
        case E_Earth:
            m_ThirdBodySign.m_bIsUseEarthGrav = false;
            break;
        case E_Mars:
            m_ThirdBodySign.m_bIsUseMarsGrav = false;
            break;
        case E_Jupiter:
            m_ThirdBodySign.m_bIsUseJupiterGrav = false;
            break;
        case E_Saturn:
            m_ThirdBodySign.m_bIsUseSaturnGrav = false;
            break;
        case E_Uranus:
            m_ThirdBodySign.m_bIsUseUranusGrav = false;
            break;
        case E_Neptune:
            m_ThirdBodySign.m_bIsUseNeptuneGrav = false;
            break;
        case E_Pluto:
            m_ThirdBodySign.m_bIsUsePlutoGrav = false;
            break;
        case E_Moon:
            m_ThirdBodySign.m_bIsUseMoonGrav = false;
            break;
        case E_Sun:
            m_ThirdBodySign.m_bIsUseSunGrav = false;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredictConfig: SolarSysStarType Unsupport ");
            break;
        }

    }

    ThirdBodyGravitySign OrbitPredictConfig::GetThirdBodySign() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");

        return m_ThirdBodySign;
    }

    bool OrbitPredictConfig::IsUseSRP() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");

        return m_bIsUseSRP;
    }

    bool OrbitPredictConfig::IsUseDrag() const
    {
        if ( bIsInitialized == false)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "OrbitPredict: m_OrbitPredictConfig UnInitialized! ");

        return m_bIsUseDrag;
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

    void OrbitPredict::OrbitStep(OrbitPredictConfig predictConfig, double step, IntegMethodType integType,
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

        OrbitPredictRightFunc rightFunc(predictConfig);

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
    OrbitPredictRightFunc::OrbitPredictRightFunc(OrbitPredictConfig config)
    {
        m_OrbitPredictConfig = config;
    }

    OrbitPredictRightFunc::~OrbitPredictRightFunc()
    {

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

        Matrix3d ECIToTODMtx = NutationMatrix(Mjd_TT) * PrecessMatrix(MJD_J2000,Mjd_TT);    //t = Mjd_TT
        Matrix3d ECItoBFCMtx = GWHourAngMatrix(Mjd_TT) * ECIToTODMtx;                       //t = Mjd_UT1

        /// Harmonic Gravity
        GravityModel gravModel(m_OrbitPredictConfig.GetGravModelType());
        acceleration += gravModel.AccelHarmonicGravity(pos, ECItoBFCMtx,
                                                         m_OrbitPredictConfig.GetGravMaxDegree(),
                                                         m_OrbitPredictConfig.GetGravMaxOrder());
        /// Atmospheric Drag
        if (m_OrbitPredictConfig.IsUseDrag())
        {
            AtmosphericDrag atmoDrag(m_OrbitPredictConfig.GetAtmosphereModelType());
            acceleration += atmoDrag.AccelAtmosphericDrag(Mjd_TT, pos, vel, ECIToTODMtx,
                                                            m_OrbitPredictConfig.GetDragArea(),
                                                            m_OrbitPredictConfig.GetDragCoef(),
                                                            x(6));
        }
        /// Solar Rad. Pressure
        if (m_OrbitPredictConfig.IsUseSRP())
        {
            Matrix3d ECIToTODMtx;
            SolarRadPressure SRPPressure;
            acceleration += SRPPressure.AccelSolarRad(Mjd_TT, pos,
                                                      m_OrbitPredictConfig.GetSRPArea(),
                                                      m_OrbitPredictConfig.GetDragCoef(),
                                                      x(6));
        }

        /// Third Body Gravity
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseEarthGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Earth, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseJupiterGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Jupiter, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseMarsGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Mars, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseMercuryGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Mercury, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseMoonGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Moon, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseNeptuneGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Neptune, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUsePlutoGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Pluto, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseSaturnGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Saturn, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseSunGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Sun, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseUranusGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Uranus, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
        }
        if (m_OrbitPredictConfig.GetThirdBodySign().m_bIsUseVenusGrav)
        {
            ThirdBodyGravity thirdBodyGrva(E_Venus, m_OrbitPredictConfig.GetCenterStarType());
            acceleration += thirdBodyGrva.AccelPointMassGravity(Mjd_TT, pos);
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
    
}

