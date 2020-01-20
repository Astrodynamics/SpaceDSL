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
        MatrixXd            GetStateTranMtx(double angVel, double duration);

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
        void                GetStateTranBlockMtx(double angVel, double duration,
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
        MatrixXd            GetImpluTranMtx(double angVel, double duration);

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
        CartState           RelOrbitPredict(double angVel, double duration, CartState &relCart);

    protected:
        RelativeCoordType       m_CoordType;

    };

}

#endif //SPRELATIVEMOTION_H

