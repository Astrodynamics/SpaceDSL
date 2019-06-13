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
#include "SpaceDSL/SpSpaceVehicle.h"
#include "SpaceDSL/SpFacility.h"
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
        m_FacilityNumber = 0;
        m_TargetNumber = 0;

        m_SpaceVehicleList.clear();
        m_FacilityList.clear();
        m_TargetList.clear();

        m_pEnvironment = nullptr;
        m_pPropagator = nullptr;
        m_pMissionThreadPool = new SpThreadPool();
        m_pAccessAnalysis = new AccessAnalysis(this);

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

        for (auto pTarget:m_TargetList)
        {
            if (pTarget != nullptr)
                delete pTarget;
        }
        m_TargetList.clear();

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

        if (m_pPropagator != nullptr)
            delete m_pPropagator;

        if (m_pMissionThreadPool != nullptr)
            delete m_pMissionThreadPool;

        if (m_pAccessAnalysis != nullptr)
            delete m_pAccessAnalysis;
    }

    SpaceVehicle *Mission::InsertSpaceVehicle(const string &name, const CalendarTime& initialEpoch,
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
        m_ProcessDataMap.insert(pair<SpaceVehicle *, vector<double *> *>(pVehicle ,pOneVehicleData));

        return pVehicle;
    }

    bool Mission::RemoveSpaceVehicle(const int id)
    {
        for(auto iter = m_SpaceVehicleList.begin();
            iter != m_SpaceVehicleList.end();
            ++iter)
        {
            if ((*iter)->GetID() == id)
            {
                delete *iter;
                m_SpaceVehicleList.erase(iter);
                --m_SpaceVehicleNumber;
                return true;
            }

        }
        return false;
    }

    Facility *Mission::InsertFacility(const string &name, const double longitude, const double latitude, const double altitude, const double minElevation)
    {
        if ( m_FacilityNumber != int(m_FacilityList.size()))
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::InsertFacility (FacilityNumber) != (FacilityList.Size)! ");
        }
        Facility *pFacility = new Facility(name, longitude, latitude, altitude, minElevation);
        ++m_FacilityNumber;
        m_FacilityList.push_back(pFacility);

        ++m_TargetNumber;
        m_TargetList.push_back(pFacility);

        return pFacility;
    }

    bool Mission::RemoveFacility(const int id)
    {
        bool bRemoveFacilityList = false;
        bool bRemoveTargetList = false;

        for(auto iter = m_FacilityList.begin();
            iter != m_FacilityList.end();
            ++iter)
        {
            if ((*iter)->GetID() == id)
            {
                delete *iter;
                m_FacilityList.erase(iter);
                --m_FacilityNumber;
                bRemoveFacilityList = true;
            }

        }

        for(auto iter = m_TargetList.begin();
            iter != m_TargetList.end();
            ++iter)
        {
            if ((*iter)->GetID() == id)
            {
                delete *iter;
                m_TargetList.erase(iter);
                --m_TargetNumber;
                bRemoveTargetList = true;
            }
        }

        if (bRemoveTargetList && bRemoveFacilityList)
            return true;
        else
            return false;
    }

    Target *Mission::InsertTarget(const string &name, const Target::TargetType type)
    {
        if (m_TargetNumber != int(m_TargetList.size()))
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
        ++m_TargetNumber;
        m_TargetList.push_back(pTarget);

        return pTarget;
    }

    bool Mission::RemoveTarget(const int id)
    {
        for(auto iter = m_TargetList.begin();
            iter != m_TargetList.end();
            ++iter)
        {
            if ((*iter)->GetID() == id)
            {
                delete *iter;
                m_TargetList.erase(iter);
                --m_TargetNumber;
                return true;
            }

        }
        return false;
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
                                const double minStep, const double maxStep, const int maxStepAttempts,
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

    void Mission::SetMissionSequence(const CalendarTime& initialEpoch, double durationSec)
    {
        m_InitialEpoch = initialEpoch;
        double Mjd0 = CalendarTimeToMjd(m_InitialEpoch);
        MjdToCalendarTime(Mjd0 + durationSec/DayToSec, m_TerminationEpoch);
        m_DurationSec = durationSec;

        m_bIsMissionSequenceInitialized = true;
    }

    const vector<SpaceVehicle *> &Mission::GetSpaceVehicleList() const
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleList.size())
           || m_SpaceVehicleNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetSpaceVehicleList (Space Vehicle Initialise Error)! ");
        }
        return m_SpaceVehicleList;
    }

    int Mission::GetSpaceVehicleNumber() const
    {
        if (m_SpaceVehicleNumber != int(m_SpaceVehicleList.size())
           || m_SpaceVehicleNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetSpaceVehicleNumber (Space Vehicle Initialise Error)! ");
        }
        return m_SpaceVehicleNumber;
    }

    const vector<Facility *> &Mission::GetFacilityList() const
    {
        if (m_FacilityNumber != int(m_FacilityList.size())
           || m_FacilityNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetFacilityList (Facility Initialise Error)! ");
        }
        return m_FacilityList;
    }

    int Mission::GetFacilityNumber() const
    {
        if (m_FacilityNumber != int(m_FacilityList.size())
           || m_FacilityNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetFacilityNumber (Facility Initialise Error)! ");
        }
        return m_FacilityNumber;
    }

    const vector<Target *> &Mission::GetTargetList() const
    {
        if (m_TargetNumber != int(m_TargetList.size())
           || m_TargetNumber < 0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetTargetList (Target Initialise Error)! ");
        }
        return m_TargetList;
    }

    int Mission::GetTargetNumber() const
    {
        if (m_TargetNumber != int(m_TargetList.size())
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

    double Mission::GetAverageOrbitalPeriod(const string &name) const
    {
        OrbitElem elem;
        SpaceVehicle *pVehicle = nullptr;
        int count = 0;
        for(auto &veh:m_SpaceVehicleList)
        {
            if (veh->GetName() == name)
            {
                pVehicle = veh;
                break;
            }
            ++count;
        }

        if (pVehicle == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "Mission::GetAverageOrbitalPeriod (Can not Find  Input Space Vehicle Name)! ");

        CartToOrbitElem (pVehicle->GetInitialCartState(), GM_Earth, elem);
        double T0 = 2*PI*sqrt(pow(elem.SMajAx(),3)/GM_Earth);

        CartToOrbitElem (pVehicle->GetCartState(), GM_Earth, elem);
        double Tt = 2*PI*sqrt(pow(elem.SMajAx(),3)/GM_Earth);

        return (Tt + T0)/2;
    }

    vector<pair<UTCCalTime, UTCCalTime> > Mission::CalTargetAccessData(const string &vehicleName, const Target *target, int order, double precision)
    {
        return m_pAccessAnalysis->CalTargetAccessData(vehicleName, target, order, precision);
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

        m_pMissionThreadPool->SetMaxThreadCount(int(GetHardwareConcurrency()));

        if (m_bIsMultThread == false)
        {
            auto pMissionThread = new MissionThread(this);
            m_pMissionThreadPool->Start(pMissionThread);
        }
        else
        {
            for (int id = 0; id < int(m_SpaceVehicleList.size()); ++id)
            {
                auto pMissionThread = new MissionThread(this);
                pMissionThread->SetSpaceVehicleIndex(id);
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

        for (auto pVehicle:m_SpaceVehicleList)
        {
            if (pVehicle != nullptr)
                delete pVehicle;
        }
        m_SpaceVehicleList.clear();

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

        if (m_pPropagator != nullptr)
        {
            delete m_pPropagator;
            m_pPropagator = nullptr;
        }
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
        m_pSpaceVehicleList = nullptr;
        m_pEnvironment = nullptr;
        m_pPropagator = nullptr;
        m_pProcessDataMap = nullptr;
    }

    MissionThread::MissionThread(Mission *pMission)
    {
        m_SpaceVehicleIndex = -1;
        m_pMission = pMission;
        m_pSpaceVehicleList = &(pMission->m_SpaceVehicleList);
        m_pEnvironment = pMission->m_pEnvironment;
        m_pPropagator = pMission->m_pPropagator;
        m_pProcessDataMap = &(pMission->m_ProcessDataMap);
    }

    MissionThread::~MissionThread()
    {

    }

    void MissionThread::SetMission(Mission *pMission)
    {
        m_SpaceVehicleIndex = -1;
        m_pMission = pMission;
        m_pSpaceVehicleList = &(pMission->m_SpaceVehicleList);
        m_pEnvironment = pMission->m_pEnvironment;
        m_pPropagator = pMission->m_pPropagator;
        m_pProcessDataMap = &(pMission->m_ProcessDataMap);
    }

    void MissionThread::SetSpaceVehicleIndex(int index)
    {
        m_SpaceVehicleIndex = index;
    }

    void MissionThread::SaveProcessDataLine(SpaceVehicle *pVehicle, const double Mjd,
                                            const Vector3d &pos, const Vector3d &vel,
                                            const GeodeticCoord &LLA, const double mass)
    {
        auto iter = m_pProcessDataMap->find(pVehicle);
        if (iter == m_pProcessDataMap->end())
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "MissionThread::SaveProcessDataLine Cant Find ProcessData!");

        double *processData = new double[11];
        auto processDataList = iter->second;

        processData[0] = Mjd;
        processData[1] = pos(0);   processData[2] = pos(1);   processData[3] = pos(2);
        processData[4] = vel(0);   processData[5] = vel(1);   processData[6] = vel(2);
        processData[7] = LLA.Longitude();
        processData[8] = LLA.Latitude();
        processData[9] = LLA.Altitude();
        processData[10] = mass;
        processDataList->push_back(processData);
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
            for (auto pVehicle:(*m_pSpaceVehicleList))
            {
                m_pPropagator->ResetAdaptedStep();
                OrbitPredict orbit;
                OrbitPredictConfig predictConfig;

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
                double step = 0;
                while (m_pMission->m_DurationSec - (Mjd_UTC - Mjd_UTC0)*DayToSec > 0.001)
                {
                    if ((Mjd_UTC - Mjd_UTC0)*DayToSec + step >  m_pMission->m_DurationSec)
                    {
                        step = m_pMission->m_DurationSec - (Mjd_UTC - Mjd_UTC0)*DayToSec;
                        m_pPropagator->SetAdaptedStep(step);
                    }
                    predictConfig.Update(Mjd_UTC);
                    step = orbit.OrbitStep(predictConfig, m_pPropagator, mass, pos, vel);
                    Mjd_UTC +=  step/DayToSec;
                    LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
                    pVehicle->UpdateState(Mjd_UTC, pos, vel, mass);
                    this->SaveProcessDataLine(pVehicle, Mjd_UTC, pos, vel, LLA, mass);
                    MjdToCalendarTime(Mjd_UTC, m_pMission->m_TerminationEpoch);

                }

            }

        }
        else
        {
            if (m_SpaceVehicleIndex >= int(m_pSpaceVehicleList->size()))
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                          "MissionThread::Run (m_SpaceVehicleIndex >= m_pSpaceVehicleList->size())");

            auto pVehicle = (*m_pSpaceVehicleList)[m_SpaceVehicleIndex];

            OrbitPredict orbit;
            OrbitPredictConfig predictConfig;

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
            double step = 0;
            while (m_pMission->m_DurationSec - (Mjd_UTC - Mjd_UTC0)*DayToSec > 0.001)
            {
                if ((Mjd_UTC - Mjd_UTC0)*DayToSec + step >  m_pMission->m_DurationSec)
                {
                    step = m_pMission->m_DurationSec - (Mjd_UTC - Mjd_UTC0)*DayToSec;
                    m_pPropagator->SetAdaptedStep(step);
                }
                predictConfig.Update(Mjd_UTC);
                step = orbit.OrbitStep(predictConfig, m_pPropagator, mass, pos, vel);
                Mjd_UTC +=  step/DayToSec;
                LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
                pVehicle->UpdateState(Mjd_UTC, pos, vel, mass);
                this->SaveProcessDataLine(pVehicle, Mjd_UTC, pos, vel, LLA, mass);
                MjdToCalendarTime(Mjd_UTC, m_pMission->m_TerminationEpoch);

            }

        }
    }





}
