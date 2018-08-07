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
* Date:2018-07-27
* Description:
*   SpMission.cpp
*
*   Purpose:
*
*        Mission Management Class
*        
*
*   Last modified:
*
*   2018-07-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include "SpaceDSL/SpMission.h"
#include "SpaceDSL/SpConst.h"

namespace SpaceDSL {

    /*************************************************
     * Class type: Mission Thread Run in Mission Class
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    MissionThread::MissionThread()
    {

    }

    MissionThread::~MissionThread()
    {

    }

    void MissionThread::Run()
    {

    }
    /*************************************************
     * Class type: The class of SpaceDSL Mission
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/

    Mission::Mission()
    {
        m_bIsEnvironmentInitialized = false;
        m_bIsPropagatorInitialized = false;
        m_bIsOptimizeInitialized = false;
        m_bIsMultThread = false;

        m_SpaceVehicleNumber = 0;
        m_SpaceVehicleList.clear();
        m_pEnvironment = nullptr;
        m_pPropagator = nullptr;

        m_DurationDay = 0;

    }

    Mission::~Mission()
    {
        for (auto pVehicle:m_SpaceVehicleList)
        {
            if (pVehicle != nullptr)
                delete pVehicle;
        }
        m_SpaceVehicleList.clear();

        if (m_pEnvironment != nullptr)
            delete m_pEnvironment;

        if (m_pPropagator != nullptr)
            delete m_pPropagator;
    }

    void Mission::InsertSpaceVehicle(SpaceVehicle *pVehicle)
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleList.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::InsertSpaceVehicle (SpaceVehicleNumber) != (SpaceVehicleList.Size)! ");
        }
        ++m_SpaceVehicleNumber;
        m_SpaceVehicleList.push_back(pVehicle);

    }

    void Mission::SetEnvironment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                                 const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                                 const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                                 const AtmosphereModel::AtmosphereModelType atmModelType ,
                                 const double f107A , const double f107, double ap[])
    {
        if (m_pEnvironment == nullptr)
        {
            m_pEnvironment = new Environment( centerStarType,  gravModelType,
                                              maxDegree,  maxOrder,  thirdBodyGravSign,
                                              geodeticType,atmModelType, f107A,  f107,  ap);
        }
        else
        {
            m_pEnvironment->SetCenterStarType(centerStarType);
            m_pEnvironment->SetGravModelType(gravModelType);
            m_pEnvironment->SetGravMaxDegree(maxDegree);
            m_pEnvironment->SetGravMaxOrder(maxOrder);
            m_pEnvironment->SetThirdBodySign(thirdBodyGravSign);
            m_pEnvironment->SetGeodeticCoordType(geodeticType);
            m_pEnvironment->SetAtmosphereModelType(atmModelType);
            m_pEnvironment->SetAverageF107(f107A);
            m_pEnvironment->SetDailyF107(f107);
            m_pEnvironment->SetGeomagneticIndex(ap);
        }
        m_bIsEnvironmentInitialized = true;
    }

    void Mission::SetPropagator(const IntegMethodType integMethodType, const double initialStep, const double accuracy,
                                const double minStep, const double maxStep, const double maxStepAttempts,
                                const bool bStopIfAccuracyIsViolated, const bool isUseNormalize)
    {
        if (m_pPropagator == nullptr)
        {
            m_pPropagator = new Propagator( integMethodType,  initialStep,  accuracy,
                                            minStep,   maxStep,    maxStepAttempts,
                                            bStopIfAccuracyIsViolated,  isUseNormalize );
        }
        else
        {
            m_pPropagator->SetIntegMethodType(integMethodType);
            m_pPropagator->SetInitialStep(initialStep);
            m_pPropagator->SetAccuracy(accuracy);
            m_pPropagator->SetMinStep(minStep);
            m_pPropagator->SetMaxStep(maxStep) ;
            m_pPropagator->SetMaxStepAttempts(maxStepAttempts)  ;
            m_pPropagator->SetStopIfAccuracyIsViolated(bStopIfAccuracyIsViolated);
            m_pPropagator->SetIsUseNormalize(isUseNormalize) ;
        }
        m_bIsPropagatorInitialized = true;
    }

    Environment *Mission::GetEnvironment() const
    {
        return m_pEnvironment;
    }

    Propagator *SpaceDSL::Mission::GetPropagator() const
    {
        return m_pPropagator;
    }

    void Mission::Start(bool bIsMultThread)
    {
        m_bIsMultThread = bIsMultThread;
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleList.size())
           || m_SpaceVehicleNumber <= 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Space Vehicle Initialise Error)! ");
        }

        if (m_bIsEnvironmentInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Environment Uninitialised)! ");
        }

        if (m_bIsPropagatorInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Propagator Uninitialised)! ");
        }



    }

    void Mission::Reset()
    {
        m_bIsEnvironmentInitialized = false;
        m_bIsPropagatorInitialized = false;
        m_bIsOptimizeInitialized = false;
        m_bIsMultThread = false;

        m_SpaceVehicleNumber = 0;
        m_DurationDay = 0;

        for (auto pVehicle:m_SpaceVehicleList)
        {
            if (pVehicle != nullptr)
                delete pVehicle;
        }
        m_SpaceVehicleList.clear();

        if (m_pEnvironment != nullptr)
        {
            delete m_pEnvironment;
            m_pEnvironment = nullptr;
        }

        if (m_pPropagator != nullptr)
        {
            delete m_pPropagator;
            m_pPropagator = nullptr;
        }
    }

}
