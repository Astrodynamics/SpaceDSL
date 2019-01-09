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
* Date:2018-12-26
* Description:
*   SpAccess.cpp
*
*   Purpose:
*
*         Calculation of All Kinds of Access.
*
*
*   Last modified:
*
*   2018-12-26  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpAccess.h"
#include "SpaceDSL/SpMission.h"
#include "SpaceDSL/SpSpaceVehicle.h"
#include "SpaceDSL/SpOrbitParam.h"
#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpObservation.h"
#include "SpaceDSL/SpInterpolation.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL Access Analysis
     * Author: Niu ZhiYong
     * Date:2019-01-02
     * Description:
     *  This Class is Thread Safe!
    **************************************************/
    AccessAnalysis::AccessAnalysis()
    {
        m_pMission = nullptr;                                 
        m_pTargetList = nullptr;
        m_pSpaceVehicleList = nullptr;
        m_pInitialEpoch = nullptr;
        m_pTerminationEpoch = nullptr;
        m_pProcessDataMap = nullptr;
        m_pAccessDataMap = nullptr;
    }

    AccessAnalysis::AccessAnalysis(Mission *pMission)
    {
        m_pMission = pMission;
        m_pTargetList = &(pMission->m_TargetList);
        m_pSpaceVehicleList = &(pMission->m_SpaceVehicleList);
        m_pInitialEpoch = &(pMission->m_InitialEpoch);
        m_pTerminationEpoch = &(pMission->m_TerminationEpoch);
        m_pProcessDataMap = &(pMission->m_ProcessDataMap);
        m_pAccessDataMap = &(pMission->m_AccessDataMap);
    }

    AccessAnalysis::~AccessAnalysis()
    {

    }

    void AccessAnalysis::SetMission(Mission *pMission)
    {
        m_pMission = pMission;
        m_pTargetList = &(pMission->m_TargetList);
        m_pSpaceVehicleList = &(pMission->m_SpaceVehicleList);
        m_pInitialEpoch = &(pMission->m_InitialEpoch);
        m_pTerminationEpoch = &(pMission->m_TerminationEpoch);
        m_pProcessDataMap = &(pMission->m_ProcessDataMap);
        m_pAccessDataMap = &(pMission->m_AccessDataMap);
    }

    const Mission *AccessAnalysis::GetMission() const
    {
        if (m_pMission == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "AccessAnalysis::GetMission There is no Mission to be setted!");

        return m_pMission;
    }

    vector<pair<UTCCalTime, UTCCalTime> > AccessAnalysis::CalTargetAccessData(const string &vehicleName, const Target *pTarget, int order, double precision)
    {
        if (m_pMission == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "AccessAnalysis::CalTargetAccessData There is no Mission to be setted!");

        vector<pair<UTCCalTime, UTCCalTime> > result;

        SpaceVehicle *pVehicle = nullptr;
        for(auto &veh:m_pMission->GetSpaceVehicleList())
        {
            if (veh->GetName() == vehicleName)
            {
                pVehicle = veh;
                break;
            }
        }
        if (pVehicle == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "AccessAnalysis::CalTargetAccessData Can not Find Vehicle With VehicleName");

        auto iterVehicle = m_pProcessDataMap->find(pVehicle);

        VectorXd startMjdList;
        VectorXd startEleList;
        startMjdList.resize(order); startMjdList.fill(0.0);
        startEleList.resize(order); startEleList.fill(0.0);

        VectorXd endMjdList;
        VectorXd endEleList;
        endMjdList.resize(order);   endMjdList.fill(0.0);
        endEleList.resize(order);   endEleList.fill(0.0);

        bool    bVisible = false;

        int midNum = int(order/2);
        int count = 0;
        for (auto &pData:*(iterVehicle->second))
        {
            double Mjd = pData[0];
            Vector3d pos(pData[1], pData[2], pData[3]);
            Vector3d vel(pData[4], pData[5], pData[6]);
            CartState cart(pos, vel);

            Observation obs;
            if(!CalObservation(Mjd, cart, const_cast<Target *>(pTarget), obs))
            {
                break;
            }

            double ele = obs.Elevation();

            if (bVisible == false && ele >= pTarget->GetMinElevation())
            {
                if (count <= midNum)
                {
                    for (int i = 0; i < order; ++i)
                    {
                        auto dataTemp = (*(iterVehicle->second))[i];
                        startMjdList(i) = dataTemp[0];
                        Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                        Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                        CartState cartTemp(posTemp, velTemp);
                        Observation obsTemp;
                        CalObservation(startMjdList(i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                        startEleList(i) = obsTemp.Elevation();
                    }
                }
                else
                {
                    for (int i = midNum; i > 0; --i)
                    {
                        auto dataTemp = (*(iterVehicle->second))[count - i];
                        startMjdList(midNum - i) = dataTemp[0];
                        Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                        Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                        CartState cartTemp(posTemp, velTemp);
                        Observation obsTemp;
                        CalObservation(startMjdList(midNum - i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                        startEleList(midNum - i) = obsTemp.Elevation();
                    }

                    for (int i = 0; i <= midNum; ++i)
                    {
                        if (midNum + i < order)
                        {
                            auto dataTemp = (*(iterVehicle->second))[count + i];
                            startMjdList(midNum + i) = dataTemp[0];
                            Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                            Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                            CartState cartTemp(posTemp, velTemp);
                            Observation obsTemp;
                            CalObservation(startMjdList(midNum + i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                            startEleList(midNum + i) = obsTemp.Elevation();
                        }

                    }

                }
                bVisible = true;
            }
            else if (bVisible == true && ele < pTarget->GetMinElevation())
            {

                for (int i = midNum; i > 0; --i)
                {
                    auto dataTemp = (*(iterVehicle->second))[count - i];
                    endMjdList(midNum - i) = dataTemp[0];
                    Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                    Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                    CartState cartTemp(posTemp, velTemp);
                    Observation obsTemp;
                    CalObservation(endMjdList(midNum - i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                    endEleList(midNum - i) = obsTemp.Elevation();
                }

                for (int i = 0; i <= midNum; ++i)
                {
                    if (midNum + i < order)
                    {
                        auto dataTemp = (*(iterVehicle->second))[count + i];
                        endMjdList(midNum + i) = dataTemp[0];
                        Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                        Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                        CartState cartTemp(posTemp, velTemp);
                        Observation obsTemp;
                        CalObservation(endMjdList(midNum + i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                        endEleList(midNum + i) = obsTemp.Elevation();
                    }
                }

                bVisible = false;
            }

            if (startMjdList(0) != 0.0 && endMjdList(0) != 0.0)
            {
                double ele0 = pTarget->GetMinElevation();
                double startMjd = CalAccessPoint(startMjdList, startEleList, ele0, precision);
                double endMjd = CalAccessPoint(endMjdList, endEleList, ele0, precision);
                UTCCalTime startTime;
                MjdToCalendarTime(startMjd, startTime);
                UTCCalTime endTime;
                MjdToCalendarTime(endMjd, endTime);
                result.push_back(pair<UTCCalTime, UTCCalTime>(startTime, endTime));

                startMjdList.fill(0.0);
                startEleList.fill(0.0);

                endMjdList.fill(0.0);
                endEleList.fill(0.0);
            }
            ++count;
        }
        return result;
    }

    map<SpaceVehicle *, vector<pair<UTCCalTime, UTCCalTime> > > AccessAnalysis::CalTargetAccessData(const Target *pTarget, int order, double precision)
    {
        if (m_pMission == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "AccessAnalysis::CalTargetAccessData There is no Mission to be setted!");

        map<SpaceVehicle *, vector<pair<UTCCalTime, UTCCalTime > > > result;

        for (auto iterVehicle = m_pProcessDataMap->begin();
             iterVehicle != m_pProcessDataMap->end();
             ++iterVehicle)
        {
            SpaceVehicle *pVehicle = iterVehicle->first;

            VectorXd startMjdList;
            VectorXd startEleList;
            startMjdList.resize(order); startMjdList.fill(0.0);
            startEleList.resize(order); startEleList.fill(0.0);

            VectorXd endMjdList;
            VectorXd endEleList;
            endMjdList.resize(order);   endMjdList.fill(0.0);
            endEleList.resize(order);   endEleList.fill(0.0);

            bool    bVisible = false;

            vector<pair<UTCCalTime, UTCCalTime > > accessDataList;
            int midNum = int(order/2);
            int count = 0;
            for (auto &pData:*(iterVehicle->second))
            {
                double Mjd = pData[0];
                Vector3d pos(pData[1], pData[2], pData[3]);
                Vector3d vel(pData[4], pData[5], pData[6]);
                CartState cart(pos, vel);

                Observation obs;
                if(!CalObservation(Mjd, cart, const_cast<Target *>(pTarget), obs))
                {
                    break;
                }

                double ele = obs.Elevation();

                if (bVisible == false && ele >= pTarget->GetMinElevation())
                {
                    if (count <= midNum)
                    {
                        for (int i = 0; i < order; ++i)
                        {
                            auto dataTemp = (*(iterVehicle->second))[i];
                            startMjdList(i) = dataTemp[0];
                            Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                            Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                            CartState cartTemp(posTemp, velTemp);
                            Observation obsTemp;
                            CalObservation(startMjdList(i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                            startEleList(i) = obsTemp.Elevation();
                        }
                    }
                    else
                    {
                        for (int i = midNum; i > 0; --i)
                        {
                            auto dataTemp = (*(iterVehicle->second))[count - i];
                            startMjdList(midNum - i) = dataTemp[0];
                            Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                            Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                            CartState cartTemp(posTemp, velTemp);
                            Observation obsTemp;
                            CalObservation(startMjdList(midNum - i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                            startEleList(midNum - i) = obsTemp.Elevation();
                        }

                        for (int i = 0; i <= midNum; ++i)
                        {
                            if (midNum + i < order)
                            {
                                auto dataTemp = (*(iterVehicle->second))[count + i];
                                startMjdList(midNum + i) = dataTemp[0];
                                Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                                Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                                CartState cartTemp(posTemp, velTemp);
                                Observation obsTemp;
                                CalObservation(startMjdList(midNum + i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                                startEleList(midNum + i) = obsTemp.Elevation();
                            }

                        }

                    }
                    bVisible = true;
                }
                else if (bVisible == true && ele < pTarget->GetMinElevation())
                {

                    for (int i = midNum; i > 0; --i)
                    {
                        auto dataTemp = (*(iterVehicle->second))[count - i];
                        endMjdList(midNum - i) = dataTemp[0];
                        Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                        Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                        CartState cartTemp(posTemp, velTemp);
                        Observation obsTemp;
                        CalObservation(endMjdList(midNum - i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                        endEleList(midNum - i) = obsTemp.Elevation();
                    }

                    for (int i = 0; i <= midNum; ++i)
                    {
                        if (midNum + i < order)
                        {
                            auto dataTemp = (*(iterVehicle->second))[count + i];
                            endMjdList(midNum + i) = dataTemp[0];
                            Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                            Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                            CartState cartTemp(posTemp, velTemp);
                            Observation obsTemp;
                            CalObservation(endMjdList(midNum + i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                            endEleList(midNum + i) = obsTemp.Elevation();
                        }
                    }

                    bVisible = false;
                }

                if (startMjdList(0) != 0.0 && endMjdList(0) != 0.0)
                {
                    double ele0 = pTarget->GetMinElevation();
                    double startMjd = CalAccessPoint(startMjdList, startEleList, ele0, precision);
                    double endMjd = CalAccessPoint(endMjdList, endEleList, ele0, precision);
                    UTCCalTime startTime;
                    MjdToCalendarTime(startMjd, startTime);
                    UTCCalTime endTime;
                    MjdToCalendarTime(endMjd, endTime);
                    accessDataList.push_back(pair<UTCCalTime, UTCCalTime>(startTime, endTime));

                    startMjdList.fill(0.0);
                    startEleList.fill(0.0);

                    endMjdList.fill(0.0);
                    endEleList.fill(0.0);
                }
                ++count;
            }

            result.insert(pair<SpaceVehicle *, vector<pair<UTCCalTime, UTCCalTime > > >(pVehicle, accessDataList));

        }

        return result;
    }

    void AccessAnalysis::CalMissionAccessData(int order, double precision)
    {
        if (m_pMission == nullptr)
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "AccessAnalysis::CalMissionAccessData There is no Mission to be setted!");

        for (auto iterVehicle = m_pProcessDataMap->begin();
             iterVehicle != m_pProcessDataMap->end();
             ++iterVehicle)
        {
            for(auto &pTarget:*m_pTargetList)
            {
                SpaceVehicle *pVehicle = iterVehicle->first;

                VectorXd startMjdList;
                VectorXd startEleList;
                startMjdList.resize(order); startMjdList.fill(0.0);
                startEleList.resize(order); startEleList.fill(0.0);

                VectorXd endMjdList;
                VectorXd endEleList;
                endMjdList.resize(order);   endMjdList.fill(0.0);
                endEleList.resize(order);   endEleList.fill(0.0);

                bool    bVisible = false;

                pair<Target *, SpaceVehicle *> PtrPair(pTarget, pVehicle);
                vector<pair<UTCCalTime, UTCCalTime > > accessDataList;

                int midNum = int(order/2);
                int count = 0;
                for (auto &pData:*(iterVehicle->second))
                {
                    double Mjd = pData[0];
                    Vector3d pos(pData[1], pData[2], pData[3]);
                    Vector3d vel(pData[4], pData[5], pData[6]);
                    CartState cart(pos, vel);

                    Observation obs;
                    if(!CalObservation(Mjd, cart, const_cast<Target *>(pTarget), obs))
                    {
                        break;
                    }

                    double ele = obs.Elevation();

                    if (bVisible == false && ele >= pTarget->GetMinElevation())
                    {
                        if (count <= midNum)
                        {
                            for (int i = 0; i < order; ++i)
                            {
                                auto dataTemp = (*(iterVehicle->second))[i];
                                startMjdList(i) = dataTemp[0];
                                Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                                Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                                CartState cartTemp(posTemp, velTemp);
                                Observation obsTemp;
                                CalObservation(startMjdList(i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                                startEleList(i) = obsTemp.Elevation();
                            }
                        }
                        else
                        {
                            for (int i = midNum; i > 0; --i)
                            {
                                auto dataTemp = (*(iterVehicle->second))[count - i];
                                startMjdList(midNum - i) = dataTemp[0];
                                Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                                Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                                CartState cartTemp(posTemp, velTemp);
                                Observation obsTemp;
                                CalObservation(startMjdList(midNum - i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                                startEleList(midNum - i) = obsTemp.Elevation();
                            }

                            for (int i = 0; i <= midNum; ++i)
                            {
                                if (midNum + i < order)
                                {
                                    auto dataTemp = (*(iterVehicle->second))[count + i];
                                    startMjdList(midNum + i) = dataTemp[0];
                                    Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                                    Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                                    CartState cartTemp(posTemp, velTemp);
                                    Observation obsTemp;
                                    CalObservation(startMjdList(midNum + i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                                    startEleList(midNum + i) = obsTemp.Elevation();
                                }

                            }

                        }
                        bVisible = true;
                    }
                    else if (bVisible == true && ele < pTarget->GetMinElevation())
                    {

                        for (int i = midNum; i > 0; --i)
                        {
                            auto dataTemp = (*(iterVehicle->second))[count - i];
                            endMjdList(midNum - i) = dataTemp[0];
                            Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                            Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                            CartState cartTemp(posTemp, velTemp);
                            Observation obsTemp;
                            CalObservation(endMjdList(midNum - i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                            endEleList(midNum - i) = obsTemp.Elevation();
                        }

                        for (int i = 0; i <= midNum; ++i)
                        {
                            if (midNum + i < order)
                            {
                                auto dataTemp = (*(iterVehicle->second))[count + i];
                                endMjdList(midNum + i) = dataTemp[0];
                                Vector3d posTemp(dataTemp[1], dataTemp[2], dataTemp[3]);
                                Vector3d velTemp(dataTemp[4], dataTemp[5], dataTemp[6]);
                                CartState cartTemp(posTemp, velTemp);
                                Observation obsTemp;
                                CalObservation(endMjdList(midNum + i), cartTemp, const_cast<Target *>(pTarget), obsTemp);
                                endEleList(midNum + i) = obsTemp.Elevation();
                            }
                        }

                        bVisible = false;
                    }

                    if (startMjdList(0) != 0.0 && endMjdList(0) != 0.0)
                    {
                        double ele0 = pTarget->GetMinElevation();
                        double startMjd = CalAccessPoint(startMjdList, startEleList, ele0, precision);
                        double endMjd = CalAccessPoint(endMjdList, endEleList, ele0, precision);
                        UTCCalTime startTime;
                        MjdToCalendarTime(startMjd, startTime);
                        UTCCalTime endTime;
                        MjdToCalendarTime(endMjd, endTime);
                        accessDataList.push_back(pair<UTCCalTime, UTCCalTime>(startTime, endTime));

                        startMjdList.fill(0.0);
                        startEleList.fill(0.0);

                        endMjdList.fill(0.0);
                        endEleList.fill(0.0);
                    }
                    ++count;
                }

                m_pAccessDataMap->insert(pair<pair<Target *, SpaceVehicle * >, vector<pair<UTCCalTime, UTCCalTime> > >
                                         (PtrPair, accessDataList));
            }
        }

    }

    void AccessAnalysis::Reset()
    {
        m_pMission = nullptr;
        m_pTargetList = nullptr;
        m_pSpaceVehicleList = nullptr;
        m_pInitialEpoch = nullptr;
        m_pTerminationEpoch = nullptr;
        m_pProcessDataMap = nullptr;
        m_pAccessDataMap = nullptr;
    }

    double AccessAnalysis::CalAccessPoint(const VectorXd &MjdList, const VectorXd &eleList, const double targetEle, const double precision)
    {
        int order = MjdList.size();
        if (order != eleList.size())
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "AccessAnalysis::CalAccessPoint Mjd List and Elevation List have Different Size!");

        double MjdSecA = MjdList(0) * DayToSec;
        double MjdSecB = MjdList(order - 1)* DayToSec;
        double MjdSecC = 0;

        int count = 0;
        do
        {
            double funA = LagrangePolynomialInterpolation(MjdList, eleList, MjdSecA*SecToDay) - targetEle;
            double funB = LagrangePolynomialInterpolation(MjdList, eleList, MjdSecB*SecToDay) - targetEle;

            MjdSecC = (MjdSecA + MjdSecB)/2;
            double funC = LagrangePolynomialInterpolation(MjdList, eleList, MjdSecC*SecToDay) - targetEle;

            if (funC * funA < 0 )
            {
                MjdSecB = MjdSecC;
            }
            else if (funC * funB < 0)
            {
                MjdSecA = MjdSecC;
            }
            else
            {
                return MjdSecC;
            }
            ++count;

            if (count > 10000)
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                          "AccessAnalysis::CalAccessPoint count > 10000");

        }while(fabs(MjdSecA - MjdSecB) > precision);

        return MjdSecC*SecToDay;
    }

}
