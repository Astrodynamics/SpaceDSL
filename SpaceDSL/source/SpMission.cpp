﻿/************************************************************************
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
#include "SpaceDSL/SpSpaceVehicle.h"
#include "SpaceDSL/SpFacility.h"
#include "SpaceDSL/SpOrbitPredict.h"
#include "SpaceDSL/SpInterpolation.h"
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
        m_FacilityNumber = 0;
        m_TargetNumber = 0;

        m_StartEpoch = 0;
        m_EndEpoch = 0;
        m_TerminationEpoch = 0;

        m_SpaceVehicleMap.clear();
        m_FacilityMap.clear();
        m_TargetMap.clear();

        m_pEnvironment = nullptr;
        m_pInitialPropagator = nullptr;
        m_pMissionThreadPool = new SpThreadPool();
        m_pAccessAnalysis = new AccessAnalysis(this);

        m_DurationSec = 0;

    }

    Mission::~Mission()
    {
        for (auto &vehiclePair:m_SpaceVehicleMap)
        {
            if (vehiclePair.second != nullptr)
                delete vehiclePair.second;
        }
        m_SpaceVehicleMap.clear();

        for (auto &targetPair:m_TargetMap)
        {
            if (targetPair.second != nullptr)
                delete targetPair.second;
        }
        m_TargetMap.clear();
        m_FacilityMap.clear();

        for (auto iter = m_ProcessDataMap.begin();
             iter != m_ProcessDataMap.end();
             ++iter)
        {
            if (iter->second != nullptr)
            {
                for(auto pData:(*(iter->second)))
                {
                    if (pData != nullptr)
                        delete pData;
                }
                delete iter->second;
            }
        }
        m_ProcessDataMap.clear();

        for (auto iter = m_AccessDataMap.begin();
             iter != m_AccessDataMap.end();
             ++iter)
        {
            iter->second.clear();
        }
        m_AccessDataMap.clear();

        if (m_pEnvironment != nullptr)
            delete m_pEnvironment;

        if (m_pInitialPropagator != nullptr)
            delete m_pInitialPropagator;

        if (m_pMissionThreadPool != nullptr)
            delete m_pMissionThreadPool;

        if (m_pAccessAnalysis != nullptr)
            delete m_pAccessAnalysis;
    }

    SpaceVehicle *Mission::InsertSpaceVehicle(const string &name, const CalendarTime& initialEpochDate,
                                     const CartState& initialState, const double initialMass,
                                     const double dragCoef, const double dragArea,
                                     const double SRPCoef, const double SRPArea)
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleMap.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::InsertSpaceVehicle (SpaceVehicleNumber) != (SpaceVehicleList.Size)! ");
        }
        SpaceVehicle *pVehicle = new SpaceVehicle(  name,           initialEpochDate,
                                                    initialState,   initialMass,
                                                    dragCoef,       dragArea,
                                                    SRPCoef,        SRPArea);
        int vehicleID = pVehicle->GetID();
        ++m_SpaceVehicleNumber;
        m_SpaceVehicleMap.insert(pair<int, SpaceVehicle *>(vehicleID, pVehicle));

        Propagator * pPropagator = new Propagator();
        m_SpaceVehiclPropagatorMap.insert(pair<int, Propagator *>(vehicleID, pPropagator));

        vector<double *> *pOneVehicleData = new vector<double *>;
        m_ProcessDataMap.insert(pair<SpaceVehicle *, vector<double *> *>(pVehicle ,pOneVehicleData));

        return pVehicle;
    }

    SpaceVehicle *Mission::InsertSpaceVehicle(const string &name, const double initialEpochMjd,
                                              const CartState &initialState, const double initialMass,
                                              const double dragCoef, const double dragArea,
                                              const double SRPCoef, const double SRPArea)
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleMap.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::InsertSpaceVehicle (SpaceVehicleNumber) != (SpaceVehicleList.Size)! ");
        }
        SpaceVehicle *pVehicle = new SpaceVehicle(  name,           initialEpochMjd,
                                                    initialState,   initialMass,
                                                    dragCoef,       dragArea,
                                                    SRPCoef,        SRPArea);
        int vehicleID = pVehicle->GetID();
        ++m_SpaceVehicleNumber;
        m_SpaceVehicleMap.insert(pair<int, SpaceVehicle *>(vehicleID, pVehicle));

        Propagator * pPropagator = new Propagator();
        m_SpaceVehiclPropagatorMap.insert(pair<int, Propagator *>(vehicleID, pPropagator));

        vector<double *> *pOneVehicleData = new vector<double *>;
        m_ProcessDataMap.insert(pair<SpaceVehicle *, vector<double *> *>(pVehicle ,pOneVehicleData));

        return pVehicle;
    }

    bool Mission::RemoveSpaceVehicle(const int id)
    {
        auto iterVehicle = m_SpaceVehicleMap.find(id);
        auto iterVehiclPropagator = m_SpaceVehiclPropagatorMap.find(id);

        if (iterVehicle != m_SpaceVehicleMap.end() && iterVehiclPropagator != m_SpaceVehiclPropagatorMap.end())
        {
            auto iterVehiclProcessData = m_ProcessDataMap.find(iterVehicle->second);
            if (iterVehiclProcessData == m_ProcessDataMap.end())
            {
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                          "Mission::RemoveSpaceVehicle iterVehiclProcessData == m_ProcessDataMap.end()");
            }
            for(auto pData:(*(iterVehiclProcessData->second)))
            {
                if (pData != nullptr)
                    delete pData;
            }
            delete iterVehiclProcessData->second;
            delete iterVehicle->second;
            delete iterVehiclPropagator->second;
            m_ProcessDataMap.erase(iterVehiclProcessData);
            m_SpaceVehicleMap.erase(iterVehicle);
            m_SpaceVehiclPropagatorMap.erase(iterVehiclPropagator);
            --m_SpaceVehicleNumber;
            return true;
        }
        else if (iterVehicle == m_SpaceVehicleMap.end() && iterVehiclPropagator != m_SpaceVehiclPropagatorMap.end())
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::RemoveSpaceVehicle iterVehicle == m_SpaceVehicleMap.end() && iterVehiclPropagator != m_SpaceVehiclPropagatorMap.end() ");
        }
        else if (iterVehicle != m_SpaceVehicleMap.end() && iterVehiclPropagator == m_SpaceVehiclPropagatorMap.end())
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::RemoveSpaceVehicle iterVehicle != m_SpaceVehicleMap.end() && iterVehiclPropagator == m_SpaceVehiclPropagatorMap.end() ");
        }

        return false;

    }

    Facility *Mission::InsertFacility(const string &name, const double longitude, const double latitude, const double altitude, const double minElevation)
    {
        if ( m_FacilityNumber != int(m_FacilityMap.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::InsertFacility (FacilityNumber) != (FacilityList.Size)! ");
        }
        Facility *pFacility = new Facility(name, longitude, latitude, altitude, minElevation);

        int targetID = pFacility->GetID();
        ++m_FacilityNumber;
        m_FacilityMap.insert(pair<int, Facility *>(targetID, pFacility));

        ++m_TargetNumber;
        m_TargetMap.insert(pair<int, Target *>(targetID, pFacility));

        return pFacility;
    }

    bool Mission::RemoveFacility(const int id)
    {
        auto iterFacility = m_FacilityMap.find(id);
        auto iterTarget = m_TargetMap.find(id);
        if (iterFacility != m_FacilityMap.end() && iterTarget != m_TargetMap.end())
        {
            delete iterFacility->second;
            m_FacilityMap.erase(iterFacility);
            m_TargetMap.erase(iterTarget);
            --m_FacilityNumber;
            --m_TargetNumber;
            return true;
        }
        else if (iterFacility == m_FacilityMap.end() && iterTarget != m_TargetMap.end())
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::RemoveFacility iterFacility == m_FacilityMap.end() && iterTarget != m_TargetMap.end() ");
        }
        else if (iterFacility != m_FacilityMap.end() && iterTarget == m_TargetMap.end())
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::RemoveFacility iterFacility != m_FacilityMap.end() && iterTarget == m_TargetMap.end() ");
        }

        return false;

    }

    Target *Mission::InsertTarget(const string &name, const Target::TargetType type)
    {
        if (m_TargetNumber != int(m_TargetMap.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::InsertTarget (SpaceVehicleNumber) != (SpaceVehicleList.Size)! ");
        }
        Target *pTarget = nullptr;
        switch (type)
        {
        case Target::E_NotDefindTargetType:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Undefined Target Type!");
        case Target::E_PointTarget:
            pTarget = new PointTarget();
            break;
        case Target::E_LineTarget:
            pTarget = new LineTarget();
            break;
        case Target::E_AreaTarget:
            pTarget = new AreaTarget();
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,"Undefined Target Type!");
        }
        pTarget->SetName(name);

        int targetID = pTarget->GetID();
        ++m_TargetNumber;
        m_TargetMap.insert(pair<int, Target *>(targetID, pTarget));

        return pTarget;
    }

    bool Mission::RemoveTarget(const int id)
    {
        auto iterFacility = m_FacilityMap.find(id);
        auto iterTarget = m_TargetMap.find(id);
        if (iterFacility != m_FacilityMap.end() && iterTarget != m_TargetMap.end())
        {
            delete iterFacility->second;
            m_FacilityMap.erase(iterFacility);
            m_TargetMap.erase(iterTarget);
            --m_FacilityNumber;
            --m_TargetNumber;
            return true;
        }
        else if (iterFacility == m_FacilityMap.end() && iterTarget != m_TargetMap.end())
        {
            delete iterTarget->second;
            m_TargetMap.erase(iterTarget);
            --m_TargetNumber;
            return true;
        }
        else if (iterFacility != m_FacilityMap.end() && iterTarget == m_TargetMap.end())
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::RemoveTarget iterFacility != m_FacilityMap.end() && iterTarget == m_TargetMap.end() ");
        }

        return false;
    }

    void Mission::SetEnvironment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                                 const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                                 const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                                 const AtmosphereModel::AtmosphereModelType atmModelType ,
                                 const double f107A , const double f107, VectorXd ap,
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
                                const double minStep, const double maxStep, const int maxStepAttempts,
                                const bool bStopIfAccuracyIsViolated, const bool isUseNormalize)
    {
        if (m_pInitialPropagator == nullptr)
        {
            m_pInitialPropagator = new Propagator( integMethodType,  initialStep,  accuracy,
                                            minStep,   maxStep,    maxStepAttempts,
                                            bStopIfAccuracyIsViolated,  isUseNormalize );
        }
        else
        {
            m_pInitialPropagator->SetIntegMethodType(integMethodType);
            m_pInitialPropagator->SetInitialStep(initialStep);
            m_pInitialPropagator->SetAccuracy(accuracy);
            m_pInitialPropagator->SetMinStep(minStep);
            m_pInitialPropagator->SetMaxStep(maxStep) ;
            m_pInitialPropagator->SetMaxStepAttempts(maxStepAttempts)  ;
            m_pInitialPropagator->SetStopIfAccuracyIsViolated(bStopIfAccuracyIsViolated);
            m_pInitialPropagator->SetIsUseNormalize(isUseNormalize) ;
        }
        m_bIsPropagatorInitialized = true;
    }

    void Mission::SetMissionSequence(const CalendarTime& startEpochDate, double durationSec)
    {
        if (durationSec <= 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::SetMissionSequence (durationSec <= 0)! ");
        }
        m_StartEpoch = CalendarTimeToMjd(startEpochDate);
        m_EndEpoch = m_StartEpoch + durationSec/DayToSec;
        m_DurationSec = durationSec;

        m_bIsMissionSequenceInitialized = true;
    }

    void Mission::SetMissionSequence(const CalendarTime &startEpochDate, const CalendarTime &endEpochDate)
    {
        m_StartEpoch = CalendarTimeToMjd(startEpochDate);
        m_EndEpoch =  CalendarTimeToMjd(endEpochDate);
        m_DurationSec = (m_EndEpoch - m_StartEpoch) * DayToSec;
        if (m_DurationSec <= 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::SetMissionSequence (m_DurationSec <= 0)! ");
        }
        m_bIsMissionSequenceInitialized = true;
    }

    void Mission::SetMissionSequence(double initialEpochMjd, double terminationEpochMjd)
    {
        m_StartEpoch = initialEpochMjd;
        m_EndEpoch = terminationEpochMjd;
        m_DurationSec = (m_EndEpoch - m_StartEpoch) * DayToSec;
        if (m_DurationSec <= 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::SetMissionSequence (m_DurationSec <= 0)! ");
        }
        m_bIsMissionSequenceInitialized = true;
    }

    const map<int, SpaceVehicle *> &Mission::GetSpaceVehicleMap() const
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleMap.size())
           || m_SpaceVehicleNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetSpaceVehicleList (Space Vehicle Initialise Error)! ");
        }
        return m_SpaceVehicleMap;
    }

    int Mission::GetSpaceVehicleNumber() const
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleMap.size())
           || m_SpaceVehicleNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetSpaceVehicleNumber (Space Vehicle Initialise Error)! ");
        }
        return m_SpaceVehicleNumber;
    }

    const map<int, Facility *> &Mission::GetFacilityMap() const
    {
        if (m_FacilityNumber != int(m_FacilityMap.size())
           || m_FacilityNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetFacilityList (Facility Initialise Error)! ");
        }
        return m_FacilityMap;
    }

    int Mission::GetFacilityNumber() const
    {
        if (m_FacilityNumber != int(m_FacilityMap.size())
           || m_FacilityNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetFacilityNumber (Facility Initialise Error)! ");
        }
        return m_FacilityNumber;
    }

    const map<int, Target *> &Mission::GetTargetMap() const
    {
        if (m_TargetNumber != int(m_TargetMap.size())
           || m_TargetNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetTargetList (Target Initialise Error)! ");
        }
        return m_TargetMap;
    }

    int Mission::GetTargetNumber() const
    {
        if (m_TargetNumber != int(m_TargetMap.size())
           || m_TargetNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetTargetNumber (Target Initialise Error)! ");
        }
        return m_TargetNumber;
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

    Propagator *SpaceDSL::Mission::GetInitialPropagator() const
    {
        if (m_bIsPropagatorInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Propagator Uninitialised)! ");
        }
        return m_pInitialPropagator;
    }

    CalendarTime Mission::GetStartEpochDate() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        CalendarTime dateTime;
        MjdToCalendarTime(m_StartEpoch, dateTime);
        return dateTime;
    }

    CalendarTime Mission::GetEndEpochDate() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        CalendarTime dateTime;
        MjdToCalendarTime(m_EndEpoch, dateTime);
        return dateTime;
    }

    CalendarTime Mission::GetTerminationEpochDate() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        CalendarTime dateTime;
        MjdToCalendarTime(m_TerminationEpoch, dateTime);
        return dateTime;
    }

    double Mission::GetStartEpoch() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        return m_StartEpoch;
    }

    double Mission::GetEndEpoch() const
    {
        if (m_bIsMissionSequenceInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::Start (Mission Sequence Uninitialised)! ");
        }
        return m_EndEpoch;
    }

    double Mission::GetTerminationEpoch() const
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

    double Mission::GetAverageOrbitalPeriod(int vehicleID) const
    {
        OrbitElem elem;
        SpaceVehicle *pVehicle = nullptr;

        auto iter = m_SpaceVehicleMap.find(vehicleID);

        if (iter == m_SpaceVehicleMap.end())
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetAverageOrbitalPeriod (Can not Find Space Vehicle in SpaceVehicleMap By ID)! ");

        pVehicle = iter->second;
        if (pVehicle == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetAverageOrbitalPeriod (Space Vehicle Ptr is NULL)! ");

        CartToOrbitElem (pVehicle->GetInitialCartState(), GM_Earth, elem);
        double T0 = 2*PI*sqrt(pow(elem.SMajAx(),3)/GM_Earth);

        CartToOrbitElem (pVehicle->GetCartState(), GM_Earth, elem);
        double Tt = 2*PI*sqrt(pow(elem.SMajAx(),3)/GM_Earth);

        return (Tt + T0)/2;
    }

    vector<pair<UTCCalTime, UTCCalTime> > Mission::CalTargetAccessData(int vehicleID, const Target *target, int order, double precision)
    {
        return m_pAccessAnalysis->CalTargetAccessData(vehicleID, target, order, precision);
    }

    map<SpaceVehicle *, vector<pair<UTCCalTime, UTCCalTime> > > Mission::CalTargetAccessData(const Target *target, int order, double precision)
    {
        return m_pAccessAnalysis->CalTargetAccessData(target, order, precision);
    }

    void Mission::CalMissionAccessData(int order, double precision)
    {
        m_pAccessAnalysis->CalMissionAccessData(order, precision);
    }

    void Mission::Start(bool bIsMultThread)
    {
        m_bIsMultThread = bIsMultThread;
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleMap.size())
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

        m_pMissionThreadPool->SetMaxThreadCount(int(GetHardwareConcurrency()));

        if (m_bIsMultThread == false)
        {
            auto pMissionThread = new MissionThread(this);
            pMissionThread->SetPriority(SpThread::Priority::HighestPriority);
            m_pMissionThreadPool->Start(pMissionThread);
        }
        else
        {
            for (auto &vehiclePari:m_SpaceVehicleMap)
            {
                auto pMissionThread = new MissionThread(this);
                pMissionThread->SetPriority(SpThread::Priority::HighestPriority);
                pMissionThread->SetSpaceVehicleIndex(vehiclePari.first);
                m_pMissionThreadPool->Start(pMissionThread);
            }
        }
        m_pMissionThreadPool->WaitForDone();
    }

    void Mission::ClearProcessData()
    {
        for (auto iter = m_ProcessDataMap.begin();
             iter != m_ProcessDataMap.end();
             ++iter)
        {
            if (iter->second != nullptr)
            {
                for(auto pData:(*(iter->second)))
                {
                    if (pData != nullptr)
                        delete pData;
                }
                iter->second->clear();
            }
        }

        for (auto iter = m_AccessDataMap.begin();
             iter != m_AccessDataMap.end();
             ++iter)
        {
            iter->second.clear();
        }
        m_AccessDataMap.clear();
    }

    void Mission::Clear()
    {
        m_bIsEnvironmentInitialized = false;
        m_bIsPropagatorInitialized = false;
        m_bIsOptimizeInitialized = false;
        m_bIsMultThread = false;

        m_SpaceVehicleNumber = 0;
        m_DurationSec = 0;

        for (auto &vehiclePair:m_SpaceVehicleMap)
        {
            if (vehiclePair.second != nullptr)
                delete vehiclePair.second;
        }
        m_SpaceVehicleMap.clear();

        for (auto &targetPair:m_TargetMap)
        {
            if (targetPair.second != nullptr)
                delete targetPair.second;
        }
        m_TargetMap.clear();

        m_FacilityMap.clear();

        for (auto iter = m_ProcessDataMap.begin();
             iter != m_ProcessDataMap.end();
             ++iter)
        {
            if (iter->second != nullptr)
            {
                for(auto pData:(*(iter->second)))
                {
                    if (pData != nullptr)
                        delete pData;
                }
                delete iter->second;
            }
        }
        m_ProcessDataMap.clear();

        if (m_pEnvironment != nullptr)
        {
            delete m_pEnvironment;
            m_pEnvironment = nullptr;
        }

        if (m_pInitialPropagator != nullptr)
        {
            delete m_pInitialPropagator;
            m_pInitialPropagator = nullptr;
        }
    }

    CartState Mission::GetCartState(SpaceVehicle *vehicle, double Mjd)
    {
        CartState cart;
        auto iterProcessData = m_ProcessDataMap.find(vehicle);
        if (iterProcessData == m_ProcessDataMap.end())
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetCartState (Can not find Process Data of The vehicle)! ");
        }
        //[Mjd_UTC, pos(3), vel(3), LLA(3), mass]
        auto pProcessData = iterProcessData->second;
        double startMjd = (*pProcessData)[0][0];
        double endMjd = (*pProcessData).back()[0];
        if (Mjd < startMjd)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetCartState (Mjd < startMjd)! ");
        }
        if (Mjd > endMjd)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetCartState (Mjd > endMjd)! ");
        }

        int order = 5;
        VectorXd mjdList;   mjdList.resize(order);  mjdList.fill(0);
        VectorXd xList;     xList.resize(order);  xList.fill(0);
        VectorXd yList;     yList.resize(order);  yList.fill(0);
        VectorXd zList;     zList.resize(order);  zList.fill(0);

        VectorXd vxList;    vxList.resize(order);  vxList.fill(0);
        VectorXd vyList;    vyList.resize(order);  vyList.fill(0);
        VectorXd vzList;    vzList.resize(order);  vzList.fill(0);

        int midNum = int(order/2);
        int dataSize = (*pProcessData).size();
        int count = 0;
        for (auto &data:(*pProcessData))
        {
            if (Mjd <= data[0])
                break;
            ++count;
        }

        if (count <= midNum)
        {
            for (int i = 0; i < order; ++i)
            {
                mjdList(i) = (*pProcessData)[i][0];
                xList(i) = (*pProcessData)[i][1];
                yList(i) = (*pProcessData)[i][2];
                zList(i) = (*pProcessData)[i][3];

                vxList(i) = (*pProcessData)[i][4];
                vyList(i) = (*pProcessData)[i][5];
                vzList(i) = (*pProcessData)[i][6];
            }
        }
        else if (count + midNum >= dataSize)
        {
            for (int i = 5; i > 0; --i)
            {
                mjdList(5-i) = (*pProcessData)[dataSize - i][0];
                xList(5-i) = (*pProcessData)[dataSize - i][1];
                yList(5-i) = (*pProcessData)[dataSize - i][2];
                zList(5-i) = (*pProcessData)[dataSize - i][3];

                vxList(5-i) = (*pProcessData)[dataSize - i][4];
                vyList(5-i) = (*pProcessData)[dataSize - i][5];
                vzList(5-i) = (*pProcessData)[dataSize - i][6];
            }
        }
        else
        {
            for (int i = midNum; i > 0; --i)
            {
                mjdList(midNum - i) = (*pProcessData)[count - i][0];
                xList(midNum - i) = (*pProcessData)[count - i][1];
                yList(midNum - i) = (*pProcessData)[count - i][2];
                zList(midNum - i) = (*pProcessData)[count - i][3];

                vxList(midNum - i) = (*pProcessData)[count - i][4];
                vyList(midNum - i) = (*pProcessData)[count - i][5];
                vzList(midNum - i) = (*pProcessData)[count - i][6];
            }

            for (int i = 0; i <= midNum; ++i)
            {
                if (midNum + i < order)
                {
                    mjdList(midNum + i) = (*pProcessData)[count + i][0];
                    xList(midNum + i) = (*pProcessData)[count + i][1];
                    yList(midNum + i) = (*pProcessData)[count + i][2];
                    zList(midNum + i) = (*pProcessData)[count + i][3];

                    vxList(midNum + i) = (*pProcessData)[count + i][4];
                    vyList(midNum + i) = (*pProcessData)[count + i][5];
                    vzList(midNum + i) = (*pProcessData)[count + i][6];
                }

            }

        }
        double x = LagrangePolynomialInterpolation(mjdList, xList, Mjd);
        double y = LagrangePolynomialInterpolation(mjdList, yList, Mjd);
        double z = LagrangePolynomialInterpolation(mjdList, zList, Mjd);

        double vx = LagrangePolynomialInterpolation(mjdList, vxList, Mjd);
        double vy = LagrangePolynomialInterpolation(mjdList, vyList, Mjd);
        double vz = LagrangePolynomialInterpolation(mjdList, vzList, Mjd);

        cart.SetPos(x,y,z);
        cart.SetVel(vx,vy,vz);
        return  cart;


    }

    const map<SpaceVehicle *, vector<double *> *> *Mission::GetProcessDataMap() const
    {
        return &m_ProcessDataMap;
    }

    const map<pair<Target *, SpaceVehicle *>, vector<pair<UTCCalTime, UTCCalTime> > > *Mission::GetAccessData() const
    {
        return &m_AccessDataMap;
    }


    /*************************************************
     * Class type: Mission Thread Run in Mission Class
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    MissionThread::MissionThread()
    {
        m_SpaceVehicleIndex = -1;
        m_pMission = nullptr;
        m_pSpaceVehicleMap = nullptr;
        m_pEnvironment = nullptr;
        m_pInitialPropagator = nullptr;
        m_pSpaceVehiclPropagatorMap = nullptr;
        m_pProcessDataMap = nullptr;
    }

    MissionThread::MissionThread(Mission *pMission)
    {
        m_SpaceVehicleIndex = -1;
        m_pMission = pMission;
        m_pSpaceVehicleMap = &(pMission->m_SpaceVehicleMap);
        m_pEnvironment = pMission->m_pEnvironment;
        m_pInitialPropagator = pMission->m_pInitialPropagator;
        m_pSpaceVehiclPropagatorMap = &(pMission->m_SpaceVehiclPropagatorMap);
        m_pProcessDataMap = &(pMission->m_ProcessDataMap);
    }

    MissionThread::~MissionThread()
    {

    }

    void MissionThread::SetMission(Mission *pMission)
    {
        m_SpaceVehicleIndex = -1;
        m_pMission = pMission;
        m_pSpaceVehicleMap = &(pMission->m_SpaceVehicleMap);
        m_pEnvironment = pMission->m_pEnvironment;
        m_pInitialPropagator = pMission->m_pInitialPropagator;
        m_pSpaceVehiclPropagatorMap = &(pMission->m_SpaceVehiclPropagatorMap);
        m_pProcessDataMap = &(pMission->m_ProcessDataMap);
    }

    void MissionThread::SetSpaceVehicleIndex(int index)
    {
        m_SpaceVehicleIndex = index;
    }

    void MissionThread::SaveProcessDataLine(vector<double *> &processDataList, const double Mjd,
                                             const Vector3d &pos, const Vector3d &vel,
                                             const GeodeticCoord &LLA, const double mass)
    {
        double *processData = new double[11];

        processData[0] = Mjd;
        processData[1] = pos(0);   processData[2] = pos(1);   processData[3] = pos(2);
        processData[4] = vel(0);   processData[5] = vel(1);   processData[6] = vel(2);
        processData[7] = LLA.Longitude();
        processData[8] = LLA.Latitude();
        processData[9] = LLA.Altitude();
        processData[10] = mass;
        processDataList.push_back(processData);
    }

    vector<double *> MissionThread::IntegralToStartEpoch(SpaceVehicle * pVehicle, double startEpoch)
    {
        vector<double *> processDataList;
        GeodeticCoordSystem GEO(GeodeticCoordSystem::GeodeticCoordType::E_WGS84System);
        GeodeticCoord LLA;
        double Mjd_UTC0 ;
        double Mjd_UTC;
        Vector3d pos,vel;
        double  mass;

        Propagator * pPropagator = m_pSpaceVehiclPropagatorMap->find(pVehicle->GetID())->second;
        *pPropagator = *m_pInitialPropagator;

        OrbitPredict orbit;
        OrbitPredictConfig predictConfig;

        Mjd_UTC0 = pVehicle->GetEpoch();
        Mjd_UTC = Mjd_UTC0;
        // If StartEpoch is earlier than Vehicle Epoch, InitialStep Should Reset to Negative.
        if(startEpoch < Mjd_UTC0)
            pPropagator->SetInitialStep(-pPropagator->GetInitialStep());

        pos = pVehicle->GetCartState().Pos();
        vel = pVehicle->GetCartState().Vel();
        mass = pVehicle->GetMass();

        auto Ap = m_pEnvironment->GetGeomagneticIndex();
        double *ap = new double[7];
        memcpy(ap, &Ap[0], 7*sizeof(double));

        predictConfig.Initializer(Mjd_UTC0, m_pEnvironment->GetCenterStarType(),
                                  pPropagator->GetIsUseNormalize(),
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
                                  ap,
                                  pVehicle->GetSRPCoef(),
                                  pVehicle->GetSRPArea(),
                                  m_pEnvironment->GetIsUseDrag(),
                                  m_pEnvironment->GetIsUseSRP());
        LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
        this->SaveProcessDataLine(processDataList, Mjd_UTC, pos, vel, LLA, mass);
        double step = 0.0;
        while (fabs(startEpoch - Mjd_UTC)*DayToSec > 0.001)
        {
            if (fabs(startEpoch - Mjd_UTC)*DayToSec < step)
            {
                step = (startEpoch - Mjd_UTC )*DayToSec;
                pPropagator->SetAdaptedStep(step);
            }
            predictConfig.Update(Mjd_UTC);
            step = orbit.OrbitStep(predictConfig, pPropagator, mass, pos, vel);
            Mjd_UTC +=  step/DayToSec;
            LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
            pVehicle->UpdateState(Mjd_UTC, pos, vel, mass);
            this->SaveProcessDataLine(processDataList, Mjd_UTC, pos, vel, LLA, mass);
            m_pMission->m_TerminationEpoch = Mjd_UTC;
        }

        delete [] ap;

        return processDataList;
    }

    vector<double *> MissionThread::IntegralToEndEpoch(SpaceVehicle *pVehicle, double endEpoch)
    {
        vector<double *> processDataList;
        GeodeticCoordSystem GEO(GeodeticCoordSystem::GeodeticCoordType::E_WGS84System);
        GeodeticCoord LLA;
        double Mjd_UTC0 ;
        double Mjd_UTC;
        Vector3d pos,vel;
        double  mass;

        Propagator * pPropagator = m_pSpaceVehiclPropagatorMap->find(pVehicle->GetID())->second;
        *pPropagator = *m_pInitialPropagator;

        OrbitPredict orbit;
        OrbitPredictConfig predictConfig;

        Mjd_UTC0 = pVehicle->GetEpoch();
        Mjd_UTC = Mjd_UTC0;
        // If EndEpoch is earlier than Vehicle Epoch, InitialStep Should Reset to Negative.
        if(endEpoch < Mjd_UTC0)
            pPropagator->SetInitialStep(-pPropagator->GetInitialStep());

        pos = pVehicle->GetCartState().Pos();
        vel = pVehicle->GetCartState().Vel();
        mass = pVehicle->GetMass();

        auto Ap = m_pEnvironment->GetGeomagneticIndex();
        double *ap = new double[7];
        memcpy(ap, &Ap[0], 7*sizeof(double));

        predictConfig.Initializer(Mjd_UTC0, m_pEnvironment->GetCenterStarType(),
                                  pPropagator->GetIsUseNormalize(),
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
                                  ap,
                                  pVehicle->GetSRPCoef(),
                                  pVehicle->GetSRPArea(),
                                  m_pEnvironment->GetIsUseDrag(),
                                  m_pEnvironment->GetIsUseSRP());
        LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
        this->SaveProcessDataLine(processDataList, Mjd_UTC, pos, vel, LLA, mass);
        double step = 0.0;
        while ( fabs(endEpoch - Mjd_UTC)*DayToSec > 0.001 )
        {
            if (fabs(endEpoch - Mjd_UTC)*DayToSec < step)
            {
                step = (endEpoch - Mjd_UTC)*DayToSec;
                pPropagator->SetAdaptedStep(step);
            }
            predictConfig.Update(Mjd_UTC);
            step = orbit.OrbitStep(predictConfig, pPropagator, mass, pos, vel);
            Mjd_UTC +=  step/DayToSec;
            LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
            pVehicle->UpdateState(Mjd_UTC, pos, vel, mass);
            this->SaveProcessDataLine(processDataList, Mjd_UTC, pos, vel, LLA, mass);
            m_pMission->m_TerminationEpoch = Mjd_UTC;
        }

        delete [] ap;

        return processDataList;
    }

    void MissionThread::Run()
    {
        if (m_pMission == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "MissionThread::Run() m_pMission == nullptr!");

        GeodeticCoordSystem GEO(GeodeticCoordSystem::GeodeticCoordType::E_WGS84System);
        GeodeticCoord LLA;
        double Mjd_UTC0 ;
        double Mjd_UTC;
        Vector3d pos,vel;
        double  mass;

        if (m_SpaceVehicleIndex == -1)
        {
            for (auto &vehiclePair:(*m_pSpaceVehicleMap))
            {
                SpaceVehicle *pVehicle = vehiclePair.second;
                double vehicleEpoch = pVehicle->GetEpoch();
                double startEpoch = m_pMission->GetStartEpoch();
                double endEpoch = m_pMission->GetEndEpoch();
                //StartTime - EndTime - VehicleEpoch
                if(vehicleEpoch > startEpoch && vehicleEpoch >= endEpoch)
                {
                    auto dataList = this->IntegralToStartEpoch(pVehicle, startEpoch);
                    reverse(dataList.begin(), dataList.end());
                    auto iter = m_pProcessDataMap->find(pVehicle);
                    if (iter == m_pProcessDataMap->end())
                        throw SPException(__FILE__, __FUNCTION__, __LINE__,
                                  "MissionThread::Run Can not Find ProcessData!");
                    auto processDataList = iter->second;
                    for(auto pData:dataList)
                    {
                        if (pData[0] >= startEpoch && pData[0] <= endEpoch)
                            processDataList->push_back(pData);
                    }
                }
                //StartTime - Epoch - EndTime
                else if (vehicleEpoch > startEpoch && vehicleEpoch < endEpoch)
                {
                    auto iter = m_pProcessDataMap->find(pVehicle);
                    if (iter == m_pProcessDataMap->end())
                        throw SPException(__FILE__, __FUNCTION__, __LINE__,
                                  "MissionThread::Run Can not Find ProcessData!");
                    auto processDataList = iter->second;

                    auto dataList1 = this->IntegralToStartEpoch(pVehicle, startEpoch);
                    reverse(dataList1.begin(), dataList1.end());
                    for(auto pData:dataList1)
                    {
                        if (pData[0] >= startEpoch && pData[0] <= vehicleEpoch)
                            processDataList->push_back(pData);
                    }

                    auto dataList2 = this->IntegralToEndEpoch(pVehicle, endEpoch);
                    for(auto pData:dataList2)
                    {
                        if (pData[0] > vehicleEpoch && pData[0] <= endEpoch)
                            processDataList->push_back(pData);
                    }
                }
                //Epoch - StartTime - EndTime
                else if (vehicleEpoch <= startEpoch && vehicleEpoch < endEpoch)
                {
                    auto dataList = this->IntegralToEndEpoch(pVehicle, endEpoch);

                    auto iter = m_pProcessDataMap->find(pVehicle);
                    if (iter == m_pProcessDataMap->end())
                        throw SPException(__FILE__, __FUNCTION__, __LINE__,
                                  "MissionThread::Run Can not Find ProcessData!");
                    auto processDataList = iter->second;
                    for(auto pData:dataList)
                    {
                        if (pData[0] >= startEpoch && pData[0] <= endEpoch)
                            processDataList->push_back(pData);
                    }
                }
                else
                {
                    throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "MissionThread::Run() Epoch Out of Condition!");
                }
            }

        }
        else
        {
            auto iter = m_pSpaceVehicleMap->find(m_SpaceVehicleIndex);

            if (iter == m_pSpaceVehicleMap->end())
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                          "MissionThread::Run (Can Find Vehicle in SpaceVehicleMap by index ID)");

            SpaceVehicle *pVehicle = iter->second;

            double vehicleEpoch = pVehicle->GetEpoch();
            double startEpoch = m_pMission->GetStartEpoch();
            double endEpoch = m_pMission->GetEndEpoch();
            //StartTime - EndTime - VehicleEpoch
            if(vehicleEpoch > startEpoch && vehicleEpoch >= endEpoch)
            {
                auto dataList = this->IntegralToStartEpoch(pVehicle, startEpoch);
                reverse(dataList.begin(), dataList.end());
                auto iter = m_pProcessDataMap->find(pVehicle);
                if (iter == m_pProcessDataMap->end())
                    throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "MissionThread::Run Can not Find ProcessData!");
                auto processDataList = iter->second;
                for(auto pData:dataList)
                {
                    if (pData[0] >= startEpoch && pData[0] <= endEpoch)
                        processDataList->push_back(pData);
                }
            }
            //StartTime - Epoch - EndTime
            else if (vehicleEpoch > startEpoch && vehicleEpoch < endEpoch)
            {
                auto iter = m_pProcessDataMap->find(pVehicle);
                if (iter == m_pProcessDataMap->end())
                    throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "MissionThread::Run Can not Find ProcessData!");
                auto processDataList = iter->second;

                auto dataList1 = this->IntegralToStartEpoch(pVehicle, startEpoch);
                reverse(dataList1.begin(), dataList1.end());
                for(auto pData:dataList1)
                {
                    if (pData[0] >= startEpoch && pData[0] <= vehicleEpoch)
                        processDataList->push_back(pData);
                }

                auto dataList2 = this->IntegralToEndEpoch(pVehicle, endEpoch);
                for(auto pData:dataList2)
                {
                    if (pData[0] > vehicleEpoch && pData[0] <= endEpoch)
                        processDataList->push_back(pData);
                }
            }
            //Epoch - StartTime - EndTime
            else if (vehicleEpoch <= startEpoch && vehicleEpoch < endEpoch)
            {
                auto dataList = this->IntegralToEndEpoch(pVehicle, endEpoch);

                auto iter = m_pProcessDataMap->find(pVehicle);
                if (iter == m_pProcessDataMap->end())
                    throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "MissionThread::Run Can not Find ProcessData!");
                auto processDataList = iter->second;
                for(auto pData:dataList)
                {
                    if (pData[0] >= startEpoch && pData[0] <= endEpoch)
                        processDataList->push_back(pData);
                }
            }
            else
            {
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                          "MissionThread::Run() Epoch Out of Condition!");
            }
        }

    }





}
