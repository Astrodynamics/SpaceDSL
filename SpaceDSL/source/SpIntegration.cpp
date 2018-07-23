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
* Date:2018-03-20
* Description:
*   SpIntegration.cpp
*
*   Purpose:
*
*         Numerical integration methods for ordinaray differential equations
*   	This module provides implemenations of the N th-order Runge-Kutta method
*
*
*   Last modified:
*
*   2018-03-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include "SpaceDSL/SpIntegration.h"
#include "SpaceDSL/SpUtils.h"
#include <iostream>

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of the N th-order Runge-Kutta method
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    RungeKutta::RungeKutta()
    {
        m_IntegMethodType = E_NotDefindIntegMethodType;
    }

    RungeKutta::RungeKutta(IntegMethodType type)
    {
        m_IntegMethodType = type;
    }

    RungeKutta::~RungeKutta()
    {

    }

    void RungeKutta::SetIntegMethodType(RungeKutta::IntegMethodType type)
    {
        m_IntegMethodType = type;
    }

    void RungeKutta::OneStep(RightFunc *rightFunc, double t, const VectorXd &x, double step, VectorXd &result)
    {
        if (m_IntegMethodType == E_RungeKutta4)
        {
            VectorXd            C(4);       C.fill(0);
            MatrixXd            A(4, 4);    A.fill(0);
            VectorXd            B(4);       B.fill(0);
            vector<VectorXd>    K;
            /************************************************************/
            C(0) = 0;       C(1) = 1.0/2.0;     C(2) = 1.0/2.0;     C(3) = 1.0;
            /************************************************************/
            A(0,0) = 0;
            A(1,0) = 1.0/2.0;
            A(2,0) = 0;     A(2,1) = 1.0/2.0;
            A(3,0) = 0;     A(3,1) = 0;         A(3,2) = 1.0;
            /************************************************************/
            B(0) = 1.0/6.0; B(1) = 1.0/3.0;     B(2) = 1.0/3.0;     B(3) = 1.0/6.0;
            /************************************************************/
            for (int i = 0; i < 4; ++i)
            {
                VectorXd Ki(x.size());      Ki.fill(0);
                VectorXd sigma(x.size());   sigma.fill(0);
                for (int j = 0; j < i; ++j)
                {
                    auto kk = K[j];
                    sigma += A(i,j)*K[j];
                }
                (*rightFunc)( t + step*C(i), x + step*sigma, Ki);
                K.push_back(Ki);
            }
            /************************************************************/
            result = x;
            for (int i = 0; i < 4; ++i)
            {
                result += step * B(i) * K[i];
            }
            return;
        }
        if (m_IntegMethodType == E_RungeKutta8)
        {
            VectorXd C(13);
            C(0) = 0;       C(1) = 1.0/18.0;    C(2) = 1.0/12.0;    C(3) = 1.0/8.0;                     C(4) = 5.0/16.0;
            C(5) = 3.0/8.0; C(6) = 59.0/400.0;  C(7) = 93.0/200.0;  C(8) = 5490023248.0/9719169821.0;   C(9) = 13.0/20.0;
            C(10) = 1201146811.0/1299019798.0;  C(11) = 1.0;        C(12) = 1.0;

        }
        else
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "IntegMethodType Is UnDefined");
        }



    }

    void RungeKutta::MultStep(RightFunc *rightFunc, double t, const VectorXd &x, double step, int stepNum, VectorXd &result)
    {
        VectorXd x_temp = x;
        if (m_IntegMethodType == E_RungeKutta4)
        {
            VectorXd            C(4);       C.fill(0);
            MatrixXd            A(4, 4);    A.fill(0);
            VectorXd            B(4);       B.fill(0);
            vector<VectorXd>    K;
            /************************************************************/
            C(0) = 0;       C(1) = 1.0/2.0;     C(2) = 1.0/2.0;     C(3) = 1.0;
            /************************************************************/
            A(0,0) = 0;
            A(1,0) = 1.0/2.0;
            A(2,0) = 0;     A(2,1) = 1.0/2.0;
            A(3,0) = 0;     A(3,1) = 0;         A(3,2) = 1.0;
            /************************************************************/
            B(0) = 1.0/6.0; B(1) = 1.0/3.0;     B(2) = 1.0/3.0;     B(3) = 1.0/6.0;
            /************************************************************/
            result = x_temp;
            for (int iStep = 0; iStep < stepNum; ++iStep)
            {
                for (int i = 0; i < 4; ++i)
                {
                    VectorXd Ki(x.size());      Ki.fill(0);
                    VectorXd sigma(x.size());   sigma.fill(0);
                    for (int j = 0; j < i; ++j)
                    {
                        sigma += A(i,j)*K[j];
                    }
                    (*rightFunc)( t + step*C(i), x_temp + step*sigma, Ki);
                    K.push_back(Ki);
                }
                /************************************************************/
                for (int i = 0; i < 4; ++i)
                {
                    result += step * B(i) * K[i];
                }
                x_temp = result;
                t += step;
                K.clear();
            }
            return;
        }
        if (m_IntegMethodType == E_RungeKutta8)
        {

        }
        else
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "IntegMethodType Is UnDefined");
        }
    }


	
}


