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
#include <algorithm>

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
        /// @Param      initialState[m,m/s] initialMass[kg]
        /// @Output
        /// @Param
        /// @Return SpaceVehicle *  The SpaceVehicle Point Which Insert into The Mission
        /**********************************************************************/
        SpaceVehicle    *InsertSpaceVehicle(const string &name, const CalendarTime& initialEpochDate,
                                           const CartState& initialState, const double initialMass,
                                           const double dragCoef, const double dragArea,
                                           const double SRPCoef, const double SRPArea);

        SpaceVehicle    *InsertSpaceVehicle(const string &name, const double initialEpochMjd,
                                           const CartState& initialState, const double initialMass,
                                           const double dragCoef, const double dragArea,
                                           const double SRPCoef, const double SRPArea);

        bool            RemoveSpaceVehicle(const int id);

        /********************************************************************/
        /// Insert/Remove a Facility Belong to The Mission
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /// @Input
        /// @Param      Longitude [rad] Latitude [rad] Altitude [m] minElevation [rad]
        /// @Output
        /// @Param
        /// @Return Facility *      The Facility Point Which Insert into The Mission
        /**********************************************************************/
        Facility        *InsertFacility(const string &name,const double longitude, const double latitude, const double altitude, const double minElevation);

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
        /// @Return Target *        The Target Point Which Insert into The Mission
        /**********************************************************************/
        Target          *InsertTarget(const string &name, const Target::TargetType type);

        bool            RemoveTarget(const int id);

        /********************************************************************/
        /// Configure Space Environment Parameters of Mission
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /**********************************************************************/
        void            SetEnvironment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                                       const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                                       const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                                       const AtmosphereModel::AtmosphereModelType atmModelType ,
                                       const double f107A , const double f107, VectorXd ap,
                                       bool isUseDrag, bool isUseSRP);

        /********************************************************************/
        /// Configure Integration Method and Parameters of Mission
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /**********************************************************************/
        void            SetPropagator(const IntegMethodType integMethodType, const double initialStep, const double accuracy = 0.0,
                                      const double  minStep = 0.0, const double  maxStep = 0.0, const int maxStepAttempts = 0,
                                      const bool bStopIfAccuracyIsViolated = true, const bool isUseNormalize = false);

        void            SetOptimization();

        /********************************************************************/
        /// Configure Time Sequence of Mission
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /**********************************************************************/
        void            SetMissionSequence(const CalendarTime& startEpochDate,  double durationSec);

        void            SetMissionSequence(const CalendarTime& startEpochDate,  const CalendarTime& endEpochDate);

        void            SetMissionSequence(double startEpochMjd,  double endEpochMjd);

        const map<int, SpaceVehicle *>          &GetSpaceVehicleMap() const;

        int                                     GetSpaceVehicleNumber() const;

        const map<int, Facility *>              &GetFacilityMap() const;

        int                                     GetFacilityNumber() const;

        const map<int, Target *>                &GetTargetMap() const;

        int                                     GetTargetNumber() const;

        Environment                             *GetEnvironment() const;

        Propagator                              *GetInitialPropagator() const;

        CalendarTime                            GetStartEpochDate() const;

        CalendarTime                            GetEndEpochDate() const;

        CalendarTime                            GetTerminationEpochDate() const;

        double                                  GetStartEpoch() const;

        double                                  GetEndEpoch() const;

        double                                  GetTerminationEpoch() const;

        double                                  GetDurationTime() const;

        double                                  GetAverageOrbitalPeriod(int vehicleID)  const;


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
        vector<pair<UTCCalTime, UTCCalTime > >                      CalTargetAccessData(int vehicleID, const Target *target,
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
        void    CalMissionAccessData(int order = 5, double precision = 0.01);

        /********************************************************************/
        /// Start Mission Calculation
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Param      bIsMultThread       If bIsMultThread = true ,Every Vehicle has a Thread
        /// @Output
        /// @Return
        /**********************************************************************/
        void    Start(bool bIsMultThread = false);

        /********************************************************************/
        /// Clear All Process Data in Mission
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Output
        /// @Return
        /**********************************************************************/
        void    ClearProcessData();

        /********************************************************************/
        /// Clear All Data in Mission
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Output
        /// @Return
        /**********************************************************************/
        void    Clear();

        /********************************************************************/
        /// Get Cart State Data in Mission At Mjd
        /// By Fifth Order Lagrange Polynomial Interpolation
        /// @Author     Niu Zhiyong
        /// @Date       2020-01-03
        /// @Input
        /// @Output
        /// @Return
        /**********************************************************************/
        CartState   GetCartState(SpaceVehicle * vehicle, double Mjd);

        /********************************************************************/
        /// Get All Data in Mission
        /// @Author     Niu Zhiyong
        /// @Date       2020-01-03
        /// @Input
        /// @Output
        /// @Return
        /**********************************************************************/
        const map<SpaceVehicle *, vector<double *> *>                                       *GetProcessDataMap() const;

        const map<pair<Target *, SpaceVehicle *>, vector<pair<UTCCalTime, UTCCalTime> > >   *GetAccessData() const;


    private:

        bool                                    m_bIsEnvironmentInitialized;
        bool                                    m_bIsPropagatorInitialized;
        bool                                    m_bIsOptimizeInitialized;
        bool                                    m_bIsMissionSequenceInitialized;
        bool                                    m_bIsMultThread;

        int                                     m_SpaceVehicleNumber;
        map<int, SpaceVehicle *>                m_SpaceVehicleMap;
        map<int, Propagator *>                  m_SpaceVehiclPropagatorMap;        ///< Each Spacecraft has its own Propagator to
                                                                                   ///< Support Adaptive Step-Size Parallel Computing
        int                                     m_FacilityNumber;
        map<int, Facility *>                    m_FacilityMap;                     ///< m_FacilityList is Subaggregate of m_TargetList
        int                                     m_TargetNumber;
        map<int, Target *>                      m_TargetMap;

        Environment                             *m_pEnvironment;
        Propagator                              *m_pInitialPropagator;

        double                                  m_StartEpoch;                     ///< MJD Epoch of Mission Start Time
        double                                  m_EndEpoch;                         ///< MJD Epoch of Mission Start Time
        double                                  m_TerminationEpoch;                 ///< MJD Epoch of Mission Termination Time
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
        /********************************************************************/
        /// Save Data Line for a SpaceVehicle
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Param      processDataList         Data Container
        /// @Param      Mjd
        /// @Param      pos
        /// @Param      vel
        /// @Param      LLA                     Lat, Lon, Alt
        /// @Param      mass
        /// @Output
        /// @Return
        /**********************************************************************/
        void    SaveProcessDataLine(vector<double *> &processDataList, const double Mjd,
                                    const Vector3d &pos, const Vector3d &vel,
                                    const GeodeticCoord &LLA, const double mass);

        vector<double *> IntegralToStartEpoch(SpaceVehicle *pVehicle, double startEpoch);

        vector<double *> IntegralToEndEpoch(SpaceVehicle *pVehicle, double endEpoch);

        void    Run() override;


    private:
        int                                 m_SpaceVehicleIndex;
        Mission                             *m_pMission;
        map<int, SpaceVehicle *>            *m_pSpaceVehicleMap;
        Environment                         *m_pEnvironment;
        map<int, Propagator *>              *m_pSpaceVehiclPropagatorMap;
        Propagator                          *m_pInitialPropagator;

        ///< [Vehicle, [Mjd_UTC, pos(3), vel(3), LLA(3), mass]]
        map<SpaceVehicle *, vector<double *> *>     *m_pProcessDataMap;

    };

}
#endif //SPMISSION_H
