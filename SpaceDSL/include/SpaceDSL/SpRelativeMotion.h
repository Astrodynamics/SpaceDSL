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
* Date:2020-01-20
* Description:
*   SpRelativeMotion.h
*
*   Purpose:
*
*       Space Vehicle Relative Motion Analysis
*
*
*   Last modified:
*
*   2020-01-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPRELATIVEMOTION_H
#define SPRELATIVEMOTION_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    enum SPACEDSL_API RelativeCoordType
    {
        E_NotDefinedRelativeCoordType       = 0,
        E_VVLH                              = 1,
        E_LVLH                              = 2
    };

    /*************************************************
     * Class type: Space Vehicle Relative Motion C-W Equation Class
     * Author: Niu ZhiYong
     * Date:2020-01-20
     * Description:
    **************************************************/
    class SPACEDSL_API CWRelMotion
    {
    public:
        explicit CWRelMotion();
        virtual ~CWRelMotion();

    public:
        void                SetRelativeCoordType(RelativeCoordType type);
        RelativeCoordType   GetRelativeCoordType();

    public:
        //********************************************************************
        /// CW State Transition Matrix
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-20
        /// @Input
        /// @Param  angVel              Reference Orbital Angular Velocity
        /// @Param  duration            Length of time
        /// @Output
        /// @Return MatrixXd(6x6)       CW State Transition Matrix
        //********************************************************************/
        virtual MatrixXd    GetStateTranMtx(double angVel, double duration);

        //********************************************************************
        /// CW State Transition Block Matrix
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-20
        /// @Input
        /// @Param  angVel              Reference Orbital Angular Velocity
        /// @Param  duration            Length of time
        /// @Output
        /// @Param  TranMtxRR           Position to Position State Transition Matrix
        /// @Param  TranMtxRV           Velocity to Position State Transition Matrix
        /// @Param  TranMtxVR           Position to Velocity State Transition Matrix
        /// @Param  TranMtxVV           Velocity to Velocity State Transition Matrix
        /// @Return
        //********************************************************************/
        virtual void        GetStateTranBlockMtx(double angVel, double duration,
                                                 Matrix3d &TranMtxRR, Matrix3d &TranMtxRV,
                                                 Matrix3d &TranMtxVR, Matrix3d &TranMtxVV);

        //********************************************************************
        /// Transfer Matrix of Impulse as Deviation
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-20
        /// @Input
        /// @Param  angVel              Reference Orbital Angular Velocity
        /// @Param  duration            Length of time
        /// @In/Out
        /// @Output
        /// @Return MatrixXd(6x3)       CW Velocity(Impulse) Deviation Transition Matrix
        //********************************************************************/
        virtual MatrixXd    GetImpluTranMtx(double angVel, double duration);

        //********************************************************************
        /// CW Relative Orbit Prediction
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-20
        /// @Input
        /// @Param  angVel          Reference Orbital Angular Velocity
        /// @Param  duration        Length of time
        /// @Param  relCart         Initial Relative State
        /// @In/Out
        /// @Output
        /// @Return CartState
        //********************************************************************/
        virtual CartState   RelOrbitPredict(double angVel, double duration, CartState &relCart);

    protected:
        RelativeCoordType       m_CoordType;
        double                  m_RadRef;                        ///<Reference Radius
        double                  m_IncRef;                        ///<Reference Inclination
        double                  m_VelRef;                        ///<Reference Velocity
        double                  m_AngVelRef;                     ///<Reference Angular Velocity

    };

    /*************************************************
     * Class type: Space Vehicle Relative Motion C-W With J2 Equation Class
     * Author: Niu ZhiYong
     * Date:2020-01-21
     * Description:
    **************************************************/
    class SPACEDSL_API CWJ2RelMotion: public CWRelMotion
    {
    public:
        explicit CWJ2RelMotion();
        virtual ~CWJ2RelMotion();

    public:
        //********************************************************************
        /// Set/Get Orbit Reference Inclination Angle
        //********************************************************************/
        void        SetIncRef(double incRef);
        double      GetIncRef();

        //********************************************************************
        /// CW With J2 State Transition Matrix
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-21
        /// @Input
        /// @Param  angVel              Reference Orbital Angular Velocity
        /// @Param  duration            Length of time
        /// @Output
        /// @Return MatrixXd(6x6)       CW State Transition Matrix
        //********************************************************************/
        virtual MatrixXd    GetStateTranMtx(double angVel, double duration) override;

        //********************************************************************
        /// CW With J2 State Transition Block Matrix
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-21
        /// @Input
        /// @Param  angVel              Reference Orbital Angular Velocity
        /// @Param  duration            Length of time
        /// @Output
        /// @Param  TranMtxRR           Position to Position State Transition Matrix
        /// @Param  TranMtxRV           Velocity to Position State Transition Matrix
        /// @Param  TranMtxVR           Position to Velocity State Transition Matrix
        /// @Param  TranMtxVV           Velocity to Velocity State Transition Matrix
        /// @Return
        //********************************************************************/
        virtual void        GetStateTranBlockMtx(double angVel, double duration,
                                                 Matrix3d &TranMtxRR, Matrix3d &TranMtxRV,
                                                 Matrix3d &TranMtxVR, Matrix3d &TranMtxVV) override;

        //********************************************************************
        /// Transfer Matrix of Impulse as Deviation With J2
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-21
        /// @Input
        /// @Param  angVel              Reference Orbital Angular Velocity
        /// @Param  duration            Length of time
        /// @In/Out
        /// @Output
        /// @Return MatrixXd(6x3)       CW Velocity(Impulse) Deviation Transition Matrix
        //********************************************************************/
        virtual MatrixXd    GetImpluTranMtx(double angVel, double duration) override;

        //********************************************************************
        /// CW With J2 Relative Orbit Prediction
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-21
        /// @Input
        /// @Param  angVel          Reference Orbital Angular Velocity
        /// @Param  duration        Length of time
        /// @Param  relCart         Initial Relative State
        /// @In/Out
        /// @Output
        /// @Return CartState
        //********************************************************************/
        virtual CartState   RelOrbitPredict(double angVel, double duration, CartState &relCart) override;
    private:
        //********************************************************************
        /// Construction of Out of Orbit Plane State Transfer Matrix
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-21
        /// @Input
        /// @Param  OrbAngVel           Orbit Angular Velocity without Considering J2
        /// @Param  duration            Length of time
        /// @In/Out
        /// @Output
        /// @Return Matrix2d            State Transfer Matrix in Z
        //********************************************************************/
        Matrix2d            StateTranMtx_Z(double angVel, double duration);

        //********************************************************************
        /// Construction of In Orbit Plane State Transfer Matrix
        /// @Author	Niu Zhiyong
        /// @Date	2020-01-21
        /// @Input
        /// @Param  OrbAngVel           Orbit Angular Velocity without Considering J2
        /// @Param  duration            Length of time
        /// @In/Out
        /// @Output
        /// @Return Matrix4d            State Transfer Matrix in XY
        //********************************************************************/
        MatrixXd            StateTranMtx_XY(double angVel, double duration);

    protected:
        bool                m_bIsInitialized = false;
        double              m_J2;           ///< J2 Oblateness Perturbation
        double              m_RCenter;      ///< Radius of Central Celestial Body
        double              m_GM;           ///< Geocentric Gravitation Constant of Central Celestial Body
        double              m_S;            ///< Intermediate Variable

    };

}

#endif //SPRELATIVEMOTION_H

