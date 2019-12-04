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
*   SpAccess.h
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

#ifndef SPACCESS_H
#define SPACCESS_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"
#include "SpTimeSystem.h"
#include "SpThread.h"

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>


using namespace Eigen;
using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    class Mission;
    class Target;
    class SpaceVehicle;
    class Observation;
    class AccessAnalysisThread;
    /*************************************************
     * Class type: The class of SpaceDSL Access Analysis
     * Author: Niu ZhiYong
     * Date:2019-01-02
     * Description:
     *  This Class is Thread Safe!
    **************************************************/
    class SPACEDSL_API AccessAnalysis
    {
    public:
        explicit AccessAnalysis();
        AccessAnalysis(Mission *pMission);
        virtual ~AccessAnalysis();

        friend class AccessAnalysisThread;

    public:
        void                            SetMission(Mission *pMission);

        const Mission                   *GetMission() const;

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
        vector<pair<UTCCalTime, UTCCalTime> > CalTargetAccessData(int vehicleID, const Target *target,
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
        /// @Return     {Vehicle, List<Start Mjd, End Mjd >}
        /**********************************************************************/
        map<SpaceVehicle *, vector<pair<UTCCalTime, UTCCalTime > > > CalTargetAccessData(const Target *target,
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
        void                            CalMissionAccessData(int order = 5, double precision = 0.01);


    protected:
        void            Reset();

        /********************************************************************/
        /// Calculate Access Point Mjd
        /// @Author     Niu Zhiyong
        /// @Date       2019-01-04
        /// @Input
        /// @Param      MjdList             Orbit Epoch
        /// @Param      eleList             Elevation List at MjdList
        /// @Param      targetEle           Target Elevation
        /// @Param      precision           Iterative precision (unit: sec)
        /// @Output
        /// @Return     Access Point Orbit Epoch
        /**********************************************************************/
        double          CalAccessPoint(const VectorXd &MjdList, const VectorXd &eleList, const double targetEle, const double precision);

    //
    // Attribute.
    //
    protected:
        Mission                                     *m_pMission;
        map<int, Target *>                          *m_pTargetMap;
        map<int, SpaceVehicle *>                    *m_pSpaceVehicleMap;
        double                                      *m_pStartEpoch;
        double                                      *m_pEndEpoch;
        double                                      *m_pTerminationEpoch;

        ///< [Vehicle, [Mjd_UTC, pos(3), vel(3), LLA(3), mass]]
        map<SpaceVehicle *, vector<double *> *>                                         *m_pProcessDataMap;
        ///< [pair<Target, Vehicle>, [pair<StartMjd_UTC, EndMjd_UTC>]]
        map<pair<Target *, SpaceVehicle *>, vector<pair<UTCCalTime, UTCCalTime> > >     *m_pAccessDataMap;

    };
}

#endif //SPACCESS_H
