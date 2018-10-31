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
*   SpMission.h
*
*   Purpose:
*
*           Mission Management Class
*
*
*   Last modified:
*
*   2018-07-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPMISSION_H
#define SPMISSION_H

#include "SpaceDSL_Global.h"
#include "SpSpaceVehicle.h"
#include "SpEnvironment.h"
#include "SpPropagator.h"
#include "SpOptimize.h"
#include "SpThread.h"

#include <map>
#include <vector>

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL Mission
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    class MissionThread;// For gcc

    class SPACEDSL_API Mission
    {
    public:
        explicit        Mission();
        friend class    MissionThread;

    private:
        virtual         ~Mission();
		
	public:
        void            InsertSpaceVehicle(const string &name, const CalendarTime& initialEpoch,
                                           const CartState& initialState, const double initialMass,
                                           const double dragCoef, const double dragArea,
                                           const double SRPCoef, const double SRPArea);

        void            SetEnvironment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                                       const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                                       const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                                       const AtmosphereModel::AtmosphereModelType atmModelType ,
                                       const double f107A , const double f107, double ap[],
                                       bool isUseDrag, bool isUseSRP);

        void            SetPropagator(const IntegMethodType integMethodType, const double initialStep, const double accuracy,
                                      const double  minStep, const double  maxStep, const double   maxStepAttempts,
                                      const bool bStopIfAccuracyIsViolated, const bool isUseNormalize);

        void            SetOptimization();

        void            SetMissionSequence(const CalendarTime& initialEpoch, const CalendarTime& terminationEpoch);

        const vector<SpaceVehicle *>            &GetSpaceVehicleList() const;

        int                                     GetSpaceVehicleNumber() const;

        Environment                             *GetEnvironment() const;

        Propagator                              *GetPropagator() const;

        const CalendarTime                      &GetInitialEpoch() const;

        const CalendarTime                      &GetTerminationEpoch() const;

        double                                  GetDurationTime() const;

        const map<string, vector<double *> *>   *GetProcessDataMap() const;

        void                                    Start(bool bIsMultThread);

        void                                    Reset();

    private:

        bool                            m_bIsEnvironmentInitialized;
        bool                            m_bIsPropagatorInitialized;
        bool                            m_bIsOptimizeInitialized;
        bool                            m_bIsMissionSequenceInitialized;
        bool                            m_bIsMultThread;

        int                             m_SpaceVehicleNumber;
        vector<SpaceVehicle *>          m_SpaceVehicleList;
        Environment                     *m_pEnvironment;
        Propagator                      *m_pPropagator;

        CalendarTime                    m_InitialEpoch;
        CalendarTime                    m_TerminationEpoch;
        double                          m_DurationSec;

        vector<MissionThread *>         m_MissionThreadList;
        SpThreadPool                    *m_pMissionThreadPool;

        map<string, vector<double *> *> m_ProcessDataMap;

    };

    /*************************************************
     * Class type: Mission Thread Run in Mission Class
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    class MissionThread : public SpThread
    {
    public:
        MissionThread();
        ~MissionThread() override;

    public:
        void    Initializer(Mission *pMission, vector<SpaceVehicle *> *spaceVehicleList,
                            Environment *pEnvironment, Propagator *pPropagator,
                            map<string, vector<double *> *> *pProcessDataMap,
                            int spaceVehicleID = -1);
    protected:

        void    SaveProcessDataLine(SpaceVehicle *pVehicle, const double Mjd,
                                    const Vector3d &pos, const Vector3d &vel,
                                    const GeodeticCoord &LLA, const double mass);

        void    Run() override;


    private:
        int                                 m_SpaceVehicleID;
        Mission                             *m_pMission;
        vector<SpaceVehicle *>              *m_pSpaceVehicleList;
        Environment                         *m_pEnvironment;
        Propagator                          *m_pPropagator;
        map<string, vector<double *> *>     *m_pProcessDataMap;

    };

}
#endif //SPMISSION_H
