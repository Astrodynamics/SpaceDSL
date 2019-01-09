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
#include "SpEnvironment.h"
#include "SpPropagator.h"
#include "SpOptimize.h"
#include "SpThread.h"
#include "SpTarget.h"
#include "SpAccess.h"

#include <map>
#include <vector>

using namespace std;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL Mission
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    class SpaceVehicle;
    class Facility;
    class MissionThread;

    class SPACEDSL_API Mission
    {
    public:
        explicit        Mission();
        void            Destory()
        {
            delete this;
        }

        friend class    MissionThread;
        friend class    AccessAnalysis;

    private:
        virtual         ~Mission();
		
	public:
        /********************************************************************/
        /// Insert/Remove a Space Vehicle Belong to The Mission
        /// @Author     Niu Zhiyong
        /// @Date       2018-07-27
        /// @Input
        /// @Param  halfAngle1    	Cone Half Angle/Horizontal Half Angle
        /// @Param	halfAngle2		Vertical Half Angle
        /// @Output
        /// @Param
        /**********************************************************************/
        void            InsertSpaceVehicle(const string &name, const CalendarTime& initialEpoch,
                                           const CartState& initialState, const double initialMass,
                                           const double dragCoef, const double dragArea,
                                           const double SRPCoef, const double SRPArea);

        bool            RemoveSpaceVehicle(const int id);

        /********************************************************************/
        /// Insert/Remove a Facility Belong to The Mission
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /// @Input
        /// @Param  halfAngle1    	Cone Half Angle/Horizontal Half Angle
        /// @Param	halfAngle2		Vertical Half Angle
        /// @Output
        /// @Param
        /**********************************************************************/
        void            InsertFacility(const string &name,const double longitude, const double latitude, const double altitude, const double minElevation);

        bool            RemoveFacility(const int id);

        /********************************************************************/
        /// Insert/Remove a Target Belong to The Mission
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /// @Input
        /// @Param  halfAngle1    	Cone Half Angle/Horizontal Half Angle
        /// @Param	halfAngle2		Vertical Half Angle
        /// @Output
        /// @Param
        /**********************************************************************/
        void            InsertTarget(const string &name, const Target::TargetType type);

        bool            RemoveTarget(const int id);

        void            SetEnvironment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                                       const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                                       const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                                       const AtmosphereModel::AtmosphereModelType atmModelType ,
                                       const double f107A , const double f107, double ap[],
                                       bool isUseDrag, bool isUseSRP);

        void            SetPropagator(const IntegMethodType integMethodType, const double initialStep, const double accuracy = 0,
                                      const double  minStep = 0, const double  maxStep = 0, const int maxStepAttempts = 0,
                                      const bool bStopIfAccuracyIsViolated = true, const bool isUseNormalize = false);

        void            SetOptimization();

        void            SetMissionSequence(const CalendarTime& initialEpoch, const CalendarTime& terminationEpoch);

        const vector<SpaceVehicle *>            &GetSpaceVehicleList() const;

        int                                     GetSpaceVehicleNumber() const;

        const vector<Facility *>                &GetFacilityList() const;

        int                                     GetFacilityNumber() const;

        const vector<Target *>                  &GetTargetList() const;

        int                                     GetTargetNumber() const;

        Environment                             *GetEnvironment() const;

        Propagator                              *GetPropagator() const;

        const CalendarTime                      &GetInitialEpoch() const;

        const CalendarTime                      &GetTerminationEpoch() const;

        double                                  GetDurationTime() const;

        double                                  GetAverageOrbitalPeriod(const string &name)  const;


        /********************************************************************/
        /// Calculate a Vehicle Access Time Pair From The Point Target
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Param      vehicleName         Vehicle Name
        /// @Param      target              Point Target Object Point
        /// @Param      order               Lagrange Polynomial Interpolation Order
        /// @Param      precision           Iterative precision (unit: sec)
        /// @Output
        /// @Return     List<Start Mjd, End Mjd >
        /**********************************************************************/
        vector<pair<UTCCalTime, UTCCalTime > >                      CalTargetAccessData(const string &vehicleName, const Target *target,
                                                                                        int order = 5, double precision = 0.01);

        /********************************************************************/
        /// Calculate All Vehicle Access Time Pair From The Point Target
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Param      target              Point Target Object Point
        /// @Param      order               Lagrange Polynomial Interpolation Order
        /// @Param      precision           Iterative precision (unit: sec)
        /// @Output
        /// @Return     {Vehicle Name, List<Start Mjd, End Mjd >}
        /**********************************************************************/
        map<SpaceVehicle *, vector<pair<UTCCalTime, UTCCalTime> > > CalTargetAccessData(const Target *target,
                                                                                        int order = 5, double precision = 0.01);

        /********************************************************************/
        /// Calculate Access Time Pair Misssion
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Param      order               Lagrange Polynomial Interpolation Order
        /// @Param      precision           Iterative precision (unit: sec)
        /// @Output
        /// @Return
        /**********************************************************************/
        void                                            CalMissionAccessData(int order = 5, double precision = 0.01);



        void                                            Start(bool bIsMultThread);

        void                                            Reset();

        const map<SpaceVehicle *, vector<double *> *>                                       *GetProcessDataMap() const;

        const map<pair<Target *, SpaceVehicle *>, vector<pair<UTCCalTime, UTCCalTime> > >   *GetAccessData() const;

    private:

        bool                                    m_bIsEnvironmentInitialized;
        bool                                    m_bIsPropagatorInitialized;
        bool                                    m_bIsOptimizeInitialized;
        bool                                    m_bIsMissionSequenceInitialized;
        bool                                    m_bIsMultThread;

        int                                     m_SpaceVehicleNumber;
        vector<SpaceVehicle *>                  m_SpaceVehicleList;
        int                                     m_FacilityNumber;
        vector<Facility *>                      m_FacilityList;                     ///< m_FacilityList is Subaggregate of m_TargetList
        int                                     m_TargetNumber;
        vector<Target *>                        m_TargetList;

        Environment                             *m_pEnvironment;
        Propagator                              *m_pPropagator;

        CalendarTime                            m_InitialEpoch;
        CalendarTime                            m_TerminationEpoch;
        double                                  m_DurationSec;

        SpThreadPool                            *m_pMissionThreadPool;
        AccessAnalysis                          *m_pAccessAnalysis;

        ///< [Vehicle, [Mjd_UTC, pos(3), vel(3), LLA(3), mass]]
        map<SpaceVehicle *, vector<double *> *>                                     m_ProcessDataMap;
        ///< [pair<Target,Vehicle>, [pair<StartMjd_UTC, EndMjd_UTC>]]
        map<pair<Target *, SpaceVehicle *>, vector<pair<UTCCalTime, UTCCalTime> > > m_AccessDataMap;

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
        MissionThread(Mission *pMission);
        ~MissionThread() override;

    public:
        void    SetMission(Mission *pMission);

        void    SetSpaceVehicleIndex(int index);

    protected:
        void    SaveProcessDataLine(SpaceVehicle *pVehicle, const double Mjd,
                                    const Vector3d &pos, const Vector3d &vel,
                                    const GeodeticCoord &LLA, const double mass);

        void    Run() override;


    private:
        int                                 m_SpaceVehicleIndex;
        Mission                             *m_pMission;
        vector<SpaceVehicle *>              *m_pSpaceVehicleList;
        Environment                         *m_pEnvironment;
        Propagator                          *m_pPropagator;

        ///< [Vehicle, [Mjd_UTC, pos(3), vel(3), LLA(3), mass]]
        map<SpaceVehicle *, vector<double *> *>     *m_pProcessDataMap;

    };

}
#endif //SPMISSION_H
