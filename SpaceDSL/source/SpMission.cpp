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
#include "SpaceDSL/SpOrbitPredict.h"
#include "SpaceDSL/SpUtils.h"
#include "SpaceDSL/SpConst.h"

namespace SpaceDSL {

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
        m_bIsMissionSequenceInitialized = false;
        m_bIsMultThread = false;

        m_SpaceVehicleNumber = 0;
        m_SpaceVehicleList.clear();
        m_MissionThreadList.clear();
        m_pEnvironment = nullptr;
        m_pPropagator = nullptr;
        m_pMissionThreadPool = new SpThreadPool();

        m_DurationSec = 0;

    }

    Mission::~Mission()
    {
        for (auto pVehicle:m_SpaceVehicleList)
        {
            if (pVehicle != nullptr)
                delete pVehicle;
        }
        m_SpaceVehicleList.clear();

        for (auto thread:m_MissionThreadList)
        {
            if (thread != nullptr)
                delete thread;
        }
        m_MissionThreadList.clear();

        map<string, vector<double *> *>::iterator iter;
        for (iter = m_ProcessDataMap.begin(); iter != m_ProcessDataMap.end(); ++iter)
        {
            for(auto pData:(*(iter->second)))
            {
                delete pData;
            }
            delete iter->second;
        }
        m_ProcessDataMap.clear();

        if (m_pEnvironment != nullptr)
            delete m_pEnvironment;

        if (m_pPropagator != nullptr)
            delete m_pPropagator;

        if (m_pMissionThreadPool != nullptr)
            delete m_pMissionThreadPool;
    }

    void Mission::InsertSpaceVehicle(const string &name, const CalendarTime& initialEpoch,
                                     const CartState& initialState, const double initialMass,
                                     const double dragCoef, const double dragArea,
                                     const double SRPCoef, const double SRPArea)
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleList.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::InsertSpaceVehicle (SpaceVehicleNumber) != (SpaceVehicleList.Size)! ");
        }
        SpaceVehicle *pVehicle = new SpaceVehicle(  name,           initialEpoch,
                                                    initialState,   initialMass,
                                                    dragCoef,       dragArea,
                                                    SRPCoef,        SRPArea);
        ++m_SpaceVehicleNumber;
        m_SpaceVehicleList.push_back(pVehicle);
        vector<double *> *pOneVehicleData = new vector<double *>;
        m_ProcessDataMap.insert(pair<string, vector<double *> *>(name ,pOneVehicleData));

    }

    void Mission::SetEnvironment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                                 const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                                 const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                                 const AtmosphereModel::AtmosphereModelType atmModelType ,
                                 const double f107A , const double f107, double ap[],
                                 bool isUseDrag, bool isUseSRP)
    {
        if (m_pEnvironment == nullptr)
        {
            m_pEnvironment = new Environment( centerStarType,  gravModelType,
                                              maxDegree,  maxOrder,  thirdBodyGravSign,
                                              geodeticType,atmModelType, f107A,  f107,  ap,
                                              isUseDrag, isUseSRP);
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
            m_pEnvironment->SetIsUseDrag(isUseDrag);
            m_pEnvironment->SetIsUseSRP(isUseSRP);
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

    void Mission::SetMissionSequence(const CalendarTime& initialEpoch, const CalendarTime& terminationEpoch)
    {
        m_InitialEpoch = initialEpoch;
        m_TerminationEpoch = terminationEpoch;
        m_DurationSec = (m_TerminationEpoch - m_InitialEpoch);

        m_bIsMissionSequenceInitialized = true;
    }

    const vector<SpaceVehicle *> &Mission::GetSpaceVehicleList() const
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleList.size())
           || m_SpaceVehicleNumber <= 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Space Vehicle Initialise Error)! ");
        }
        return m_SpaceVehicleList;
    }

    int Mission::GetSpaceVehicleNumber() const
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleList.size())
           || m_SpaceVehicleNumber <= 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Space Vehicle Initialise Error)! ");
        }
        return m_SpaceVehicleNumber;
    }

    Environment *Mission::GetEnvironment() const
    {
        if (m_bIsEnvironmentInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Environment Uninitialised)! ");
        }
        return m_pEnvironment;
    }

    Propagator *SpaceDSL::Mission::GetPropagator() const
    {
        if (m_bIsPropagatorInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Propagator Uninitialised)! ");
        }
        return m_pPropagator;
    }

    const CalendarTime &Mission::GetInitialEpoch() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        return m_InitialEpoch;
    }

    const CalendarTime &Mission::GetTerminationEpoch() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        return m_TerminationEpoch;
    }

    double Mission::GetDurationTime() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        return m_DurationSec;
    }

    const map<string, vector<double *> *> *Mission::GetProcessDataMap() const
    {
        return &m_ProcessDataMap;
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


        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }

        if (m_bIsMultThread == false)
        {
            auto pMissionThread = new MissionThread();
            pMissionThread->Initializer(this, &m_SpaceVehicleList, m_pEnvironment,
                                        m_pPropagator, &m_ProcessDataMap);
            pMissionThread->Start();
            pMissionThread->Wait();
        }
        else
        {
            m_pMissionThreadPool->SetMaxThreadCount(int(GetHardwareConcurrency()));

            for (int id = 0; id < int(m_SpaceVehicleList.size()); ++id)
            {
                auto pMissionThread = new MissionThread();
                pMissionThread->Initializer(this, &m_SpaceVehicleList, m_pEnvironment,
                                            m_pPropagator, &m_ProcessDataMap, id);
                m_MissionThreadList.push_back(pMissionThread);
                m_pMissionThreadPool->Start(pMissionThread);
            }
            m_pMissionThreadPool->WaitForDone();
        }


    }

    void Mission::Reset()
    {
        m_bIsEnvironmentInitialized = false;
        m_bIsPropagatorInitialized = false;
        m_bIsOptimizeInitialized = false;
        m_bIsMultThread = false;

        m_SpaceVehicleNumber = 0;
        m_DurationSec = 0;

        for (auto pVehicle:m_SpaceVehicleList)
        {
            if (pVehicle != nullptr)
                delete pVehicle;
        }
        m_SpaceVehicleList.clear();

        for (auto thread:m_MissionThreadList)
        {
            if (thread != nullptr)
                delete thread;
        }
        m_MissionThreadList.clear();

        map<string, vector<double *> *>::iterator iter;
        for (iter = m_ProcessDataMap.begin(); iter != m_ProcessDataMap.end(); ++iter)
        {
            for(auto pData:(*(iter->second)))
            {
                delete pData;
            }
            delete iter->second;
        }
        m_ProcessDataMap.clear();

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

    /*************************************************
     * Class type: Mission Thread Run in Mission Class
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    MissionThread::MissionThread()
    {
        m_SpaceVehicleID = -1;
        m_pMission = nullptr;
        m_pSpaceVehicleList = nullptr;
        m_pEnvironment = nullptr;
        m_pPropagator = nullptr;
        m_pProcessDataMap = nullptr;
    }

    MissionThread::~MissionThread()
    {

    }

    void SpaceDSL::MissionThread::Initializer(Mission *pMission, vector<SpaceVehicle *> *spaceVehicleList,
                                              Environment *pEnvironment, Propagator *pPropagator,
                                              map<string, vector<double *> *> *pProcessDataMap,
                                              int spaceVehicleID)
    {
        m_pMission = pMission;
        m_pSpaceVehicleList = spaceVehicleList;
        m_pEnvironment = pEnvironment;
        m_pPropagator = pPropagator;
        m_pProcessDataMap = pProcessDataMap;
        m_SpaceVehicleID = spaceVehicleID;
    }

    void MissionThread::SaveProcessDataLine(SpaceVehicle *pVehicle, const double Mjd,
                                            const Vector3d &pos, const Vector3d &vel,
                                            const GeodeticCoord &LLA, const double mass)
    {
        double *processData = new double[11];
        auto processDataList = m_pProcessDataMap->find(pVehicle->GetName())->second;
        CartState currentState(pos, vel);
        pVehicle->SetTime(Mjd);
        pVehicle->SetCartState(currentState);
        pVehicle->SetMass(mass);

        processData[0] = Mjd;
        processData[1] = pos(0);   processData[2] = pos(1);   processData[3] = pos(2);
        processData[4] = vel(0);   processData[5] = vel(1);   processData[6] = vel(2);
        processData[7] = LLA.Latitude();
        processData[8] = LLA.Longitude();
        processData[9] = LLA.Altitude();
        processData[10] = mass;
        processDataList->push_back(processData);
    }

    void MissionThread::Run()
    {
        GeodeticCoordSystem GEO(GeodeticCoordSystem::GeodeticCoordType::E_WGS84System);
        GeodeticCoord LLA;
        OrbitPredictConfig predictConfig;
        double Mjd_UTC0 ;
        double Mjd_UTC;
        Vector3d pos,vel;
        double  mass;
        OrbitPredict orbit;

        if (m_SpaceVehicleID == -1)
        {
            for (auto pVehicle:(*m_pSpaceVehicleList))
            {

                Mjd_UTC0 = pVehicle->GetEpoch();
                Mjd_UTC = Mjd_UTC0;
                pos = pVehicle->GetCartState().Pos();
                vel = pVehicle->GetCartState().Vel();
                mass = pVehicle->GetMass();
                predictConfig.Initializer(Mjd_UTC0, m_pEnvironment->GetCenterStarType(),
                                          m_pPropagator->GetIsUseNormalize(),
                                          m_pEnvironment->GetGravModelType(),
                                          m_pEnvironment->GetGravMaxDegree() ,
                                          m_pEnvironment->GetGravMaxOrder(),
                                          m_pEnvironment->GetThirdBodySign(),
                                          m_pEnvironment->GetGeodeticCoordType(),
                                          m_pEnvironment->GetAtmosphereModelType(),
                                          pVehicle->GetDragCoef(),
                                          pVehicle->GetDragArea(),
                                          m_pEnvironment->GetAverageF107(),
                                          m_pEnvironment->GetDailyF107(),
                                          m_pEnvironment->GetGeomagneticIndex(),
                                          pVehicle->GetSRPCoef(),
                                          pVehicle->GetSRPArea(),
                                          m_pEnvironment->GetIsUseDrag(),
                                          m_pEnvironment->GetIsUseSRP());
                LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
                this->SaveProcessDataLine(pVehicle, Mjd_UTC, pos, vel, LLA, mass);
                double step = m_pPropagator->GetInitialStep();
                for (int i = 0; i < m_pMission->m_DurationSec/step; ++i)
                {
                    predictConfig.Update(Mjd_UTC);
                    orbit.OrbitStep(predictConfig,step, E_RungeKutta4, mass, pos, vel);
                    Mjd_UTC = Mjd_UTC0 + (i+1) * step/DayToSec;
                    LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
                    this->SaveProcessDataLine(pVehicle, Mjd_UTC, pos, vel, LLA, mass);
                }

            }

        }
        else
        {
            if (m_SpaceVehicleID >= int(m_pSpaceVehicleList->size()))
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                          "MissionThread::Run (m_SpaceVehicleID >= m_pSpaceVehicleList->size())");

            auto pVehicle = (*m_pSpaceVehicleList)[m_SpaceVehicleID];

            Mjd_UTC0 = pVehicle->GetEpoch();
            Mjd_UTC = Mjd_UTC0;
            pos = pVehicle->GetCartState().Pos();
            vel = pVehicle->GetCartState().Vel();
            mass = pVehicle->GetMass();
            predictConfig.Initializer(Mjd_UTC0, m_pEnvironment->GetCenterStarType(),
                                      m_pPropagator->GetIsUseNormalize(),
                                      m_pEnvironment->GetGravModelType(),
                                      m_pEnvironment->GetGravMaxDegree() ,
                                      m_pEnvironment->GetGravMaxOrder(),
                                      m_pEnvironment->GetThirdBodySign(),
                                      m_pEnvironment->GetGeodeticCoordType(),
                                      m_pEnvironment->GetAtmosphereModelType(),
                                      pVehicle->GetDragCoef(),
                                      pVehicle->GetDragArea(),
                                      m_pEnvironment->GetAverageF107(),
                                      m_pEnvironment->GetDailyF107(),
                                      m_pEnvironment->GetGeomagneticIndex(),
                                      pVehicle->GetSRPCoef(),
                                      pVehicle->GetSRPArea(),
                                      m_pEnvironment->GetIsUseDrag(),
                                      m_pEnvironment->GetIsUseSRP());
            LLA = GEO.GetGeodeticCoord(pos,Mjd_UTC);
            this->SaveProcessDataLine(pVehicle, Mjd_UTC, pos, vel, LLA, mass);
            double step = m_pPropagator->GetInitialStep();
            for (int i = 0; i < m_pMission->m_DurationSec/step; ++i)
            {
                predictConfig.Update(Mjd_UTC);
                orbit.OrbitStep(predictConfig,step, E_RungeKutta4, mass, pos, vel);
                Mjd_UTC = Mjd_UTC0 + (i+1) * step/DayToSec;
                LLA = GEO.GetGeodeticCoord(pos,Mjd_UTC);
                this->SaveProcessDataLine(pVehicle, Mjd_UTC, pos, vel, LLA, mass);
            }
        }
    }





}
