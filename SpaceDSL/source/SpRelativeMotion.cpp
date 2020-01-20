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
*   SpRelativeMotion.cpp
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

#include "SpRelativeMotion.h"
#include "SpConst.h"

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: Space Vehicle Relative Motion C-W Equation Class
     * Author: Niu ZhiYong
     * Date:2020-01-20
     * Description:
    **************************************************/
    CWRelMotion::CWRelMotion()
    {
        m_CoordType = RelativeCoordType::E_VVLH;
    }

    CWRelMotion::~CWRelMotion()
    {

    }

    void CWRelMotion::SetRelativeCoordType(RelativeCoordType type)
    {
        m_CoordType = type;
    }

    RelativeCoordType CWRelMotion::GetRelativeCoordType()
    {
        return m_CoordType;
    }

    MatrixXd CWRelMotion::GetStateTranMtx(double angVel, double duration)
    {
        MatrixXd TranMtx;    TranMtx.resize(6, 6);    TranMtx.fill(0.0);

        double detTheta = angVel*duration;
        double C = cos(detTheta);
        double S = sin(detTheta);
        double W = angVel;

        if (m_CoordType == E_VVLH)
        {
            TranMtx(0, 0) = 1.0;
            TranMtx(0, 1) = 0.0;
            TranMtx(0, 2) = 6*(detTheta-S);
            TranMtx(0, 3) = (4*S-3*detTheta)/W;
            TranMtx(0, 4) = 0.0;
            TranMtx(0, 5) = 2*(1-C)/W;

            TranMtx(1, 0) = 0.0;
            TranMtx(1, 1) = C;
            TranMtx(1, 2) = 0.0;
            TranMtx(1, 3) = 0.0;
            TranMtx(1, 4) = S/W;
            TranMtx(1, 5) = 0.0;

            TranMtx(2, 0) = 0.0;
            TranMtx(2, 1) = 0.0;
            TranMtx(2, 2) = 4-3*C;
            TranMtx(2, 3) = 2*(C-1)/W;
            TranMtx(2, 4) = 0.0;
            TranMtx(2, 5) = S/W;

            TranMtx(3, 0) = 0.0;
            TranMtx(3, 1) = 0.0;
            TranMtx(3, 2) = 6*W*(1-C);
            TranMtx(3, 3) = 4*C-3;
            TranMtx(3, 4) = 0.0;
            TranMtx(3, 5) = 2*S;

            TranMtx(4, 0) = 0.0;
            TranMtx(4, 1) = -W*S;
            TranMtx(4, 2) = 0.0;
            TranMtx(4, 3) = 0.0;
            TranMtx(4, 4) = C;
            TranMtx(4, 5) = 0.0;

            TranMtx(5, 0) = 0.0;
            TranMtx(5, 1) = 0.0;
            TranMtx(5, 2) = 3*W*S;
            TranMtx(5, 3) = -2*S;
            TranMtx(5, 4) = 0.0;
            TranMtx(5, 5) = C;
        }
        else    // E_LVLH
        {
            TranMtx(0, 0) = 4-3*C;
            TranMtx(0, 1) = 0.0;
            TranMtx(0, 2) = 0.0;
            TranMtx(0, 3) = S/W;
            TranMtx(0, 4) = 2*(1-C)/W;
            TranMtx(0, 5) = 0.0;

            TranMtx(1, 0) = 6*(S-detTheta);
            TranMtx(1, 1) = 1.0;
            TranMtx(1, 2) = 0.0;
            TranMtx(1, 3) = 2*(C-1)/W;
            TranMtx(1, 4) = (4*S-3*detTheta)/W;
            TranMtx(1, 5) = 0.0;

            TranMtx(2, 0) = 0.0;
            TranMtx(2, 1) = 0.0;
            TranMtx(2, 2) = C;
            TranMtx(2, 3) = 0.0;
            TranMtx(2, 4) = 0.0;
            TranMtx(2, 5) = S/W;

            TranMtx(3, 0) = 3*W*S;
            TranMtx(3, 1) = 0.0;
            TranMtx(3, 2) = 0.0;
            TranMtx(3, 3) = C;
            TranMtx(3, 4) = 2*S;
            TranMtx(3, 5) = 0.0;

            TranMtx(4, 0) = 6*W*(C-1);
            TranMtx(4, 1) = 0.0;
            TranMtx(4, 2) = 0.0;
            TranMtx(4, 3) = -2*S;
            TranMtx(4, 4) = 4*C-3;
            TranMtx(4, 5) = 0.0;

            TranMtx(5, 0) = 0.0;
            TranMtx(5, 1) = 0.0;
            TranMtx(5, 2) = -W*S;
            TranMtx(5, 3) = 0.0;
            TranMtx(5, 4) = 0.0;
            TranMtx(5, 5) = C;
        }

        return TranMtx;
    }

    void CWRelMotion::GetStateTranBlockMtx(double angVel, double duration,
                                           Matrix3d &TranMtxRR, Matrix3d &TranMtxRV,
                                           Matrix3d &TranMtxVR, Matrix3d &TranMtxVV)
    {
        double detTheta = angVel*duration;
        double C = cos(detTheta);
        double S = sin(detTheta);
        double W = angVel;
        if (m_CoordType == E_VVLH)
        {
            TranMtxRR(0, 0) = 1.0;                      TranMtxRV(0, 0) = (4*S-3*detTheta)/W;
            TranMtxRR(0, 1) = 0.0;                      TranMtxRV(0, 1) = 0.0;
            TranMtxRR(0, 2) = 6*(detTheta-S);           TranMtxRV(0, 2) = 2*(1-C)/W;

            TranMtxRR(1, 0) = 0.0;                      TranMtxRV(1, 0) = 0.0;
            TranMtxRR(1, 1) = C;                        TranMtxRV(1, 1) = S/W;
            TranMtxRR(1, 2) = 0.0;                      TranMtxRV(1, 2) = 0.0;

            TranMtxRR(2, 0) = 0.0;                      TranMtxRV(2, 0) = 2*(C-1)/W;
            TranMtxRR(2, 1) = 0.0;                      TranMtxRV(2, 1) = 0.0;
            TranMtxRR(2, 2) = 4-3*C;                    TranMtxRV(2, 2) = S/W;


            TranMtxVR(0, 0) = 0.0;                      TranMtxVV(0, 0) = 4*C-3;
            TranMtxVR(0, 1) = 0.0;                      TranMtxVV(0, 1) = 0.0;
            TranMtxVR(0, 2) = 6*W*(1-C);                TranMtxVV(0, 2) = 2*S;

            TranMtxVR(1, 0) = 0.0;                      TranMtxVV(1, 0) = 0.0;
            TranMtxVR(1, 1) = -W*S;                     TranMtxVV(1, 1) = C;
            TranMtxVR(1, 2) = 0.0;                      TranMtxVV(1, 2) = 0.0;

            TranMtxVR(2, 0) = 0.0;                      TranMtxVV(2, 0) = -2*S;
            TranMtxVR(2, 1) = 0.0;                   	TranMtxVV(2, 1) = 0.0;
            TranMtxVR(2, 2) = 3*W*S;                    TranMtxVV(2, 2) = C;

        }
        else     // E_LVLH
        {
            TranMtxRR(0, 0) = 4-3*C;                    TranMtxRV(0, 0) = S/W;
            TranMtxRR(0, 1) = 0.0;						TranMtxRV(0, 1) = 2*(1-C)/W;
            TranMtxRR(0, 2) = 0.0;						TranMtxRV(0, 2) = 0.0;

            TranMtxRR(1, 0) = 6*(S-detTheta);           TranMtxRV(1, 0) = 2*(C-1)/W;
            TranMtxRR(1, 1) = 1.0;                      TranMtxRV(1, 1) = (4*S-3*detTheta)/W;
            TranMtxRR(1, 2) = 0.0;                      TranMtxRV(1, 2) = 0.0;

            TranMtxRR(2, 0) = 0.0;                      TranMtxRV(2, 0) = 0.0;
            TranMtxRR(2, 1) = 0.0;                      TranMtxRV(2, 1) = 0.0;
            TranMtxRR(2, 2) = C;                        TranMtxRV(2, 2) = S/W;



            TranMtxVR(0, 0) = 3*W*S;                    TranMtxVV(0, 0) = C;
            TranMtxVR(0, 1) = 0.0;	                    TranMtxVV(0, 1) = 2*S;
            TranMtxVR(0, 2) = 0.0;                      TranMtxVV(0, 2) = 0.0;

            TranMtxVR(1, 0) = 6*W*(C-1);	            TranMtxVV(1, 0) = -2*S;
            TranMtxVR(1, 1) = 0.0;                      TranMtxVV(1, 1) = 4*C-3;
            TranMtxVR(1, 2) = 0.0;                      TranMtxVV(1, 2) = 0.0;

            TranMtxVR(2, 0) = 0.0;	                    TranMtxVV(2, 0) = 0.0;
            TranMtxVR(2, 1) = 0.0;                   	TranMtxVV(2, 1) = 0.0;
            TranMtxVR(2, 2) = -W*S;                     TranMtxVV(2, 2) = C;
        }
    }

    MatrixXd CWRelMotion::GetImpluTranMtx(double angVel, double duration)
    {
        MatrixXd TranMtxV; TranMtxV.resize(6, 3); TranMtxV.fill(0.0);

        double detTheta = angVel*duration;
        double C = cos(detTheta);
        double S = sin(detTheta);
        double W = angVel;

        if (m_CoordType == E_VVLH)
        {
            TranMtxV(0, 0) = (4*S-3*detTheta)/W;
            TranMtxV(0, 1) = 0.0;
            TranMtxV(0, 2) = 2*(1-C)/W;

            TranMtxV(1, 0) = 0.0;
            TranMtxV(1, 1) = S/W;
            TranMtxV(1, 2) = 0.0;

            TranMtxV(2, 0) = 2*(C-1)/W;
            TranMtxV(2, 1) = 0.0;
            TranMtxV(2, 2) = S/W;

            TranMtxV(3, 0) = 4*C-3;
            TranMtxV(3, 1) = 0.0;
            TranMtxV(3, 2) = 2*S;

            TranMtxV(4, 0) = 0.0;
            TranMtxV(4, 1) = C;
            TranMtxV(4, 2) = 0.0;

            TranMtxV(5, 0) = -2*S;
            TranMtxV(5, 1) = 0.0;
            TranMtxV(5, 2) = C;
        }
        else    // E_LVLH
        {
            TranMtxV(0, 0) = S/W;
            TranMtxV(0, 1) = 2*(1-C)/W;
            TranMtxV(0, 2) = 0.0;

            TranMtxV(1, 0) = 2*(C-1)/W;
            TranMtxV(1, 1) = (4*S-3*detTheta)/W;
            TranMtxV(1, 2) = 0.0;

            TranMtxV(2, 0) = 0.0;
            TranMtxV(2, 1) = 0.0;
            TranMtxV(2, 2) = S/W;

            TranMtxV(3, 0) = C;
            TranMtxV(3, 1) = 2*S;
            TranMtxV(3, 2) = 0.0;

            TranMtxV(4, 0) = -2*S;
            TranMtxV(4, 1) = 4*C-3;
            TranMtxV(4, 2) = 0.0;

            TranMtxV(5, 0) = 0.0;
            TranMtxV(5, 1) = 0.0;
            TranMtxV(5, 2) = C;
        }

        return TranMtxV;
    }

    CartState CWRelMotion::RelOrbitPredict(double angVel, double duration, CartState &relCart)
    {
        CartState newRelCart;
        MatrixXd TranMtx = this->GetStateTranMtx(angVel, duration);
        VectorXd state(6);

        for(int i = 0; i < 3; i++)
        {
            state(i)= relCart.Pos()(i);
            state(i+3) = relCart.Vel()(i);
        }
        state = TranMtx*state;

        Vector3d pos, vel;
        for(int i = 0; i < 3; i++)
        {
            pos(i) = state(i);
            vel(i) = state(i+3);
        }

        newRelCart.SetPos(pos);
        newRelCart.SetVel(vel);

        return newRelCart;
    }

	
}


