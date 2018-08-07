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
#include "SpCZMLScript.h"
#include "SpThread.h"
#include "SpUtils.h"

#include <map>
#include <vector>

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: Mission Thread Run in Mission Class
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    class SPACEDSL_API MissionThread : public SpThread
    {
    public:
        MissionThread();
        ~MissionThread() override;

    public:

        void Run() override;

    private:
        int         m_SpaceVehicleID;

    };

    /*************************************************
     * Class type: The class of SpaceDSL Mission
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    class SPACEDSL_API Mission
    {
    public:
		explicit Mission();
        virtual ~Mission();
        friend class MissionThread;
		
	public:
        void                InsertSpaceVehicle(SpaceVehicle *pVehicle);

        void                SetEnvironment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                                           const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                                           const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                                           const AtmosphereModel::AtmosphereModelType atmModelType ,
                                           const double f107A , const double f107, double ap[]);

        void                SetPropagator(const IntegMethodType  const integMethodType, const double initialStep, const double accuracy,
                                          const double  minStep, const double  maxStep, const double   maxStepAttempts,
                                          const bool bStopIfAccuracyIsViolated, const bool isUseNormalize);

        void                SetOptimization();

        void                SetMissionSequence(double durationDay);

        Environment         *GetEnvironment() const;

        Propagator          *GetPropagator() const;

        void                Start(bool bIsMultThread);

        void                Reset();

    private:

        bool                                    m_bIsEnvironmentInitialized;
        bool                                    m_bIsPropagatorInitialized;
        bool                                    m_bIsOptimizeInitialized;
        bool                                    m_bIsMultThread;

        int                                     m_SpaceVehicleNumber;
        vector<SpaceVehicle *>                  m_SpaceVehicleList;
        Environment                             *m_pEnvironment;
        Propagator                              *m_pPropagator;

        double                                  m_DurationDay;

        SpThreadPool                            m_MissionThreadPool;

        map<string, vector<double *> *>         m_ProcessDataMap;



    };


}
#endif //SPMISSION_H
