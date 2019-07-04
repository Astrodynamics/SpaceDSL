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
*   2018-12-25  Niu Zhiyong Add Adapted Step Integral
*
*************************************************************************/

#include "SpaceDSL/SpIntegration.h"
#include "SpaceDSL/SpUtils.h"


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
        m_Stages = 0;
        m_Order = 0;
        m_bGetAdaptedStep = false;
        m_StepAttemptCount = 0;
        m_RelativeErrorThreshold = 0.1;
    }

    RungeKutta::RungeKutta(const IntegMethodType type)
    {
        m_IntegMethodType = type;
        m_bGetAdaptedStep = false;
        m_StepAttemptCount = 0;
        m_RelativeErrorThreshold = 0.1;

        switch (m_IntegMethodType)
        {
        case E_RungeKutta4 :
            m_Stages = 4;
            m_Order = 4;
            m_IncPower = (1.0/m_Order);
            m_DecPower = (1.0/(m_Order-1));
            m_C.resize(4);          m_C.fill(0);
            m_A.resize(4, 4);       m_A.fill(0);
            m_B.resize(4);          m_B.fill(0);
            /************************************************************/
            m_C(0) = 0;         m_C(1) = 1.0/2.0;     m_C(2) = 1.0/2.0;     m_C(3) = 1.0;
            /************************************************************/
            m_A(0,0) = 0;
            m_A(1,0) = 1.0/2.0;
            m_A(2,0) = 0;       m_A(2,1) = 1.0/2.0;
            m_A(3,0) = 0;       m_A(3,1) = 0;         m_A(3,2) = 1.0;
            /************************************************************/
            m_B(0) = 1.0/6.0;   m_B(1) = 1.0/3.0;     m_B(2) = 1.0/3.0;     m_B(3) = 1.0/6.0;
            /************************************************************/
            break;
        case E_RungeKutta78 :
            m_Stages = 13;
            m_Order = 8;
            m_IncPower = (1.0/m_Order);
            m_DecPower = (1.0/(m_Order-1));
            m_C.resize(13);         m_C.fill(0);
            m_A.resize(13, 13);     m_A.fill(0);
            m_B.resize(13);         m_B.fill(0);
            m_Bbar.resize(13);      m_Bbar.fill(0);
            /************************************************************/
            m_C(0) = 0;       m_C(1) = 1.0/18.0;    m_C(2) = 1.0/12.0;    m_C(3) = 1.0/8.0;                     m_C(4) = 5.0/16.0;
            m_C(5) = 3.0/8.0; m_C(6) = 59.0/400.0;  m_C(7) = 93.0/200.0;  m_C(8) = 5490023248.0/9719169821.0;   m_C(9) = 13.0/20.0;
            m_C(10) = 1201146811.0/1299019798.0;    m_C(11) = 1.0;        m_C(12) = 1.0;
            /************************************************************/
            m_A(0,0) = 0;
            m_A(1,0) = 1.0/18.0;
            m_A(2,0) = 1.0/48.0;                    m_A(2,1) = 1.0/16.0;
            m_A(3,0) = 1.0/32.0;                    m_A(3,1) = 0;       m_A(3,2) = 3.0/32.0;
            m_A(4,0) = 5.0/16.0;                    m_A(4,1) = 0;       m_A(4,2) = -75.0/64.0;  m_A(4,3) = 75.0/64.0;
            m_A(5,0) = 3.0/80.0;                    m_A(5,1) = 0;       m_A(5,2) = 0;           m_A(5,3) = 3.0/16.0;                    m_A(5,4) = 3.0/20.0;
            m_A(6,0) = 29443841.0/614563906.0;      m_A(6,1) = 0;       m_A(6,2) = 0;           m_A(6,3) = 77736538.0/692538347.0;      m_A(6,4) = -28693883.0/1125000000.0;    m_A(6,5) = 23124283.0/1800000000.0;
            m_A(7,0) = 16016141.0/946692911.0;      m_A(7,1) = 0;       m_A(7,2) = 0;           m_A(7,3) = 61564180.0/158732637.0;      m_A(7,4) = 22789713.0/633445777.0;      m_A(7,5) = 545815736.0/2771057229.0;        m_A(7,6) = -180193667.0/1043307555.0;
            m_A(8,0) = 39632708.0/573591083.0;      m_A(8,1) = 0;       m_A(8,2) = 0;           m_A(8,3) = -433636366.0/683701615.0;    m_A(8,4) = -421739975.0/26162923001.0;  m_A(8,5) = 100302831.0/723423059.0;         m_A(8,6) = 790204164.0/839813087.0;         m_A(8,7) = 800635310.0/3783071287.0;
            m_A(9,0) = 246121993.0/1340847787.0;    m_A(9,1) = 0;       m_A(9,2) = 0;           m_A(9,3) = -37695042795.0/15268766246.0;m_A(9,4) = -309121744.0/1061227803.0;   m_A(9,5) = -12992083.0/490766935.0;         m_A(9,6) = 6005943493.0/2108947869.0;       m_A(9,7) = 393006217.0/1396673457.0;    m_A(9,8) = 123872331.0/1001029789.0;
            m_A(10,0) = -1028468189.0/846180014.0;  m_A(10,1) = 0;      m_A(10,2) = 0;          m_A(10,3) = 8478235783.0/508512852.0;   m_A(10,4) = 1311729495.0/1432422823.0;  m_A(10,5) = -10304129995.0/1701304382.0;    m_A(10,6) = -48777925059.0/3047939560.0;    m_A(10,7) = 15336726248.0/1032824649.0; m_A(10,8) = -45442868181.0/3398467696.0;m_A(10,9) = 3065993473.0/597172653.0;
            m_A(11,0) = 185892177.0/718116043.0;    m_A(11,1) = 0;      m_A(11,2) = 0;          m_A(11,3) = -3185094517.0/667107341.0;  m_A(11,4) = -477755414.0/1098053517.0;  m_A(11,5) = -703635378.0/230739211.0;       m_A(11,6) = 5731566787.0/1027545527.0;      m_A(11,7) = 5232866602.0/850066563.0;   m_A(11,8) = -4093664535.0/808688257.0;  m_A(11,9) = 3692137247.0/1805957418.0;m_A(11,10) = 65686358.0/487910083.0;
            m_A(12,0) = 403863854.0/491063109.0;    m_A(12,1) = 0;      m_A(12,2) = 0;          m_A(12,3) = -5068492393.0/434740067.0;  m_A(12,4) = -411421997.0/543043805.0;   m_A(12,5) = 652783627.0/914296604.0;        m_A(12,6) = 11173962825.0/925320556.0;      m_A(12,7) = -13158990841.0/6184727034.0;m_A(12,8) = 3936647629.0/1978049680.0;  m_A(12,9) = -160528059.0/685178525.0; m_A(12,10) = 248638103.0/1413531060.0; m_A(12,11) = 0;
            /************************************************************/
            m_B(0) = 14005451.0/335480064.0;    m_B(1) = 0;                         m_B(2) = 0;                         m_B(3) = 0;
            m_B(4) = 0;                         m_B(5) = -59238493.0/1068277825.0;  m_B(6) = 181606767.0/758867731.0;   m_B(7) = 561292985.0/797845732.0;
            m_B(8) = -1041891430.0/1371343529.0;m_B(9) = 760417239.0/1151165299.0;  m_B(10) = 118820643.0/751138087.0;  m_B(11) = -528747749.0/2220607170.0;    m_B(12) = 1.0/4.0;

            m_Bbar(0) = 13451932.0/455176623.0;   m_Bbar(1) = 0;     m_Bbar(2) = 0;     m_Bbar(3) = 0;
            m_Bbar(4) = 0;   m_Bbar(5) = -808719846.0/976000145.0;     m_Bbar(6) = 1757004468.0/5645159321.0;     m_Bbar(7) = 656045339.0/265891186.0;
            m_Bbar(8) = -3867574721.0/1518517206.0;   m_Bbar(9) = 465885868.0/322736535.0;     m_Bbar(10) = 53011238.0/667516719.0;    m_Bbar(11) = 2.0/45.0;     m_Bbar(12) = 2.0/45.0;
            /************************************************************/
            break;

        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "IntegMethodType Is UnDefined");
        }
    }

    RungeKutta::~RungeKutta()
    {

    }

    void RungeKutta::SetIntegMethodType(const IntegMethodType type)
    {
        m_IntegMethodType = type;
        switch (m_IntegMethodType)
        {
        case E_RungeKutta4 :
            m_Stages = 4;
            m_Order = 4;
            m_C.resize(4);          m_C.fill(0.0);
            m_A.resize(4, 4);       m_A.fill(0.0);
            m_B.resize(4);          m_B.fill(0.0);
            /************************************************************/
            m_C(0) = 0.0;         m_C(1) = 1.0/2.0;     m_C(2) = 1.0/2.0;     m_C(3) = 1.0;
            /************************************************************/
            m_A(0,0) = 0.0;
            m_A(1,0) = 1.0/2.0;
            m_A(2,0) = 0.0;       m_A(2,1) = 1.0/2.0;
            m_A(3,0) = 0.0;       m_A(3,1) = 0.0;         m_A(3,2) = 1.0;
            /************************************************************/
            m_B(0) = 1.0/6.0;   m_B(1) = 1.0/3.0;     m_B(2) = 1.0/3.0;     m_B(3) = 1.0/6.0;
            /************************************************************/
            break;
        case E_RungeKutta78 :
            m_Stages = 13;
            m_Order = 8;
            m_C.resize(13);         m_C.fill(0.0);
            m_A.resize(13, 13);     m_A.fill(0.0);
            m_B.resize(13);         m_B.fill(0.0);
            m_Bbar.resize(13);      m_Bbar.fill(0.0);
            /************************************************************/
            m_C(0) = 0;       m_C(1) = 1.0/18.0;    m_C(2) = 1.0/12.0;    m_C(3) = 1.0/8.0;                     m_C(4) = 5.0/16.0;
            m_C(5) = 3.0/8.0; m_C(6) = 59.0/400.0;  m_C(7) = 93.0/200.0;  m_C(8) = 5490023248.0/9719169821.0;   m_C(9) = 13.0/20.0;
            m_C(10) = 1201146811.0/1299019798.0;    m_C(11) = 1.0;        m_C(12) = 1.0;
            /************************************************************/
            m_A(0,0) = 0.0;
            m_A(1,0) = 1.0/18.0;
            m_A(2,0) = 1.0/48.0;                    m_A(2,1) = 1.0/16.0;
            m_A(3,0) = 1.0/32.0;                    m_A(3,1) = 0.0;       m_A(3,2) = 3.0/32.0;
            m_A(4,0) = 5.0/16.0;                    m_A(4,1) = 0.0;       m_A(4,2) = -75.0/64.0;  m_A(4,3) = 75.0/64.0;
            m_A(5,0) = 3.0/80.0;                    m_A(5,1) = 0.0;       m_A(5,2) = 0.0;           m_A(5,3) = 3.0/16.0;                    m_A(5,4) = 3.0/20.0;
            m_A(6,0) = 29443841.0/614563906.0;      m_A(6,1) = 0.0;       m_A(6,2) = 0.0;           m_A(6,3) = 77736538.0/692538347.0;      m_A(6,4) = -28693883.0/1125000000.0;    m_A(6,5) = 23124283.0/1800000000.0;
            m_A(7,0) = 16016141.0/946692911.0;      m_A(7,1) = 0.0;       m_A(7,2) = 0.0;           m_A(7,3) = 61564180.0/158732637.0;      m_A(7,4) = 22789713.0/633445777.0;      m_A(7,5) = 545815736.0/2771057229.0;        m_A(7,6) = -180193667.0/1043307555.0;
            m_A(8,0) = 39632708.0/573591083.0;      m_A(8,1) = 0.0;       m_A(8,2) = 0.0;           m_A(8,3) = -433636366.0/683701615.0;    m_A(8,4) = -421739975.0/26162923001.0;  m_A(8,5) = 100302831.0/723423059.0;         m_A(8,6) = 790204164.0/839813087.0;         m_A(8,7) = 800635310.0/3783071287.0;
            m_A(9,0) = 246121993.0/1340847787.0;    m_A(9,1) = 0.0;       m_A(9,2) = 0.0;           m_A(9,3) = -37695042795.0/15268766246.0;m_A(9,4) = -309121744.0/1061227803.0;   m_A(9,5) = -12992083.0/490766935.0;         m_A(9,6) = 6005943493.0/2108947869.0;       m_A(9,7) = 393006217.0/1396673457.0;    m_A(9,8) = 123872331.0/1001029789.0;
            m_A(10,0) = -1028468189.0/846180014.0;  m_A(10,1) = 0.0;      m_A(10,2) = 0.0;          m_A(10,3) = 8478235783.0/508512852.0;   m_A(10,4) = 1311729495.0/1432422823.0;  m_A(10,5) = -10304129995.0/1701304382.0;    m_A(10,6) = -48777925059.0/3047939560.0;    m_A(10,7) = 15336726248.0/1032824649.0; m_A(10,8) = -45442868181.0/3398467696.0;m_A(10,9) = 3065993473.0/597172653.0;
            m_A(11,0) = 185892177.0/718116043.0;    m_A(11,1) = 0.0;      m_A(11,2) = 0.0;          m_A(11,3) = -3185094517.0/667107341.0;  m_A(11,4) = -477755414.0/1098053517.0;  m_A(11,5) = -703635378.0/230739211.0;       m_A(11,6) = 5731566787.0/1027545527.0;      m_A(11,7) = 5232866602.0/850066563.0;   m_A(11,8) = -4093664535.0/808688257.0;  m_A(11,9) = 3692137247.0/1805957418.0;m_A(11,10) = 65686358.0/487910083.0;
            m_A(12,0) = 403863854.0/491063109.0;    m_A(12,1) = 0.0;      m_A(12,2) = 0.0;          m_A(12,3) = -5068492393.0/434740067.0;  m_A(12,4) = -411421997.0/543043805.0;   m_A(12,5) = 652783627.0/914296604.0;        m_A(12,6) = 11173962825.0/925320556.0;      m_A(12,7) = -13158990841.0/6184727034.0;m_A(12,8) = 3936647629.0/1978049680.0;  m_A(12,9) = -160528059.0/685178525.0; m_A(12,10) = 248638103.0/1413531060.0; m_A(12,11) = 0;
            /************************************************************/
            m_B(0) = 14005451.0/335480064.0;        m_B(1) = 0.0;                           m_B(2) = 0.0;                           m_B(3) = 0.0;
            m_B(4) = 0.0;                           m_B(5) = -59238493.0/1068277825.0;      m_B(6) = 181606767.0/758867731.0;       m_B(7) = 561292985.0/797845732.0;
            m_B(8) = -1041891430.0/1371343529.0;    m_B(9) = 760417239.0/1151165299.0;      m_B(10) = 118820643.0/751138087.0;      m_B(11) = -528747749.0/2220607170.0;    m_B(12) = 1.0/4.0;

            m_Bbar(0) = 13451932.0/455176623.0;     m_Bbar(1) = 0.0;                        m_Bbar(2) = 0.0;                      m_Bbar(3) = 0.0;
            m_Bbar(4) = 0.0;                        m_Bbar(5) = -808719846.0/976000145.0;   m_Bbar(6) = 1757004468.0/5645159321.0;  m_Bbar(7) = 656045339.0/265891186.0;
            m_Bbar(8) = -3867574721.0/1518517206.0; m_Bbar(9) = 465885868.0/322736535.0;    m_Bbar(10) = 53011238.0/667516719.0;    m_Bbar(11) = 2.0/45.0;                  m_Bbar(12) = 2.0/45.0;
            /************************************************************/
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "IntegMethodType Is UnDefined");
        }
    }

    void RungeKutta::SetRelativeErrorThreshold(const double relErr)
    {
        m_RelativeErrorThreshold = relErr >= 0.0 ? relErr : -relErr;
    }

    double RungeKutta::GetRelativeErrorThreshold(const double relErr) const
    {
        return m_RelativeErrorThreshold;
    }

    double RungeKutta::OneStep(RightFunc *rightFunc, double t, const VectorXd &x, double initialStep, VectorXd &result,
                               double minStep, double maxStep, int maxStepAttempts,
                               double accuracyThreshold, bool bStopIfAccuracyIsViolated)
    {
        double adaptedStep = initialStep;

        if (minStep == 0.0 && maxStep == 0.0
                && maxStepAttempts == 0 && accuracyThreshold == 0.0)
        {
            m_K.clear();
            for (int i = 0; i < m_Stages; ++i)
            {
                VectorXd Ki(x.size());      Ki.fill(0);
                VectorXd sigma(x.size());   sigma.fill(0);
                for (int j = 0; j < i; ++j)
                {
                    auto kk = m_K[j];
                    sigma += m_A(i,j)*m_K[j];
                }

                (*rightFunc)( t + initialStep*m_C(i), x + initialStep*sigma, Ki);
                m_K.push_back(Ki);
            }
            /************************************************************/
            result = x;
            for (int i = 0; i < m_Stages; ++i)
            {
                result += initialStep * m_B(i) * m_K[i];
            }
            return initialStep;
        }
        else
        {
            if (m_IntegMethodType != E_RungeKutta78)
                throw  SPException(__FILE__, __FUNCTION__, __LINE__, "Adaptive Step Integra Just Support Runge-Kutta78");

            if (initialStep < minStep)
                throw  SPException(__FILE__, __FUNCTION__, __LINE__, "Initial Step < Input minStep");

            if (initialStep > maxStep)
                throw  SPException(__FILE__, __FUNCTION__, __LINE__, "Initial Step > Input maxStep");

            if (maxStepAttempts < 1)
                throw  SPException(__FILE__, __FUNCTION__, __LINE__, "maxStepAttempts < 1");

            if (accuracyThreshold <= 0)
                throw  SPException(__FILE__, __FUNCTION__, __LINE__, "relativeErrorThreshold <= 0");

            do
            {
                m_K.clear();
                for (int i = 0; i < m_Stages; ++i)
                {
                    VectorXd Ki(x.size());      Ki.fill(0);
                    VectorXd sigma(x.size());   sigma.fill(0);
                    for (int j = 0; j < i; ++j)
                    {
                        auto kk = m_K[j];
                        sigma += m_A(i,j)*m_K[j];
                    }
                    (*rightFunc)( t + adaptedStep*m_C(i), x + adaptedStep*sigma, Ki);
                    m_K.push_back(Ki);
                }
                /************************************************************/
                m_TempResult = x;
                for (int i = 0; i < m_Stages; ++i)
                {
                    m_TempResult += adaptedStep * m_B(i) * m_K[i];
                }

                adaptedStep = GetAdaptedStep(rightFunc, t, x, adaptedStep, minStep, maxStep, maxStepAttempts,
                                             accuracyThreshold, bStopIfAccuracyIsViolated);

            }while(m_bGetAdaptedStep == false);

            result = m_TempResult;

            m_bGetAdaptedStep = false;
            m_StepAttemptCount = 0;
            return adaptedStep;
        }

    }

    void RungeKutta::MultStep(RightFunc *rightFunc, double t0, const VectorXd &x, double initialStep, double t, VectorXd &result,
                              double minStep, double maxStep, int maxStepAttempts,
                              double relativeErrorThreshold, bool bStopIfAccuracyIsViolated)
    {
        if (minStep == 0.0 && maxStep == 0.0
                && maxStepAttempts == 0 &&relativeErrorThreshold == 0.0)
        {
            double t_temp = t0;
            VectorXd x_temp = x;
            int stepNum = static_cast<int>((t - t0)/initialStep);
            for (int iStep = 0; iStep < stepNum; ++iStep)
            {
                OneStep(rightFunc, t_temp, x_temp, initialStep, result);
                x_temp = result;
                t_temp += initialStep;
            }
            double lastStep = t - stepNum * initialStep - t0;
            OneStep(rightFunc, t_temp, x_temp, lastStep, result);
        }
        else
        {

        }


    }

    double RungeKutta::GetAdaptedStep(RightFunc *rightFunc, double t, const VectorXd &x, double initialStep,
                                      double minStep, double maxStep,
                                      int maxStepAttempts, double accuracyThreshold, bool bStopIfAccuracyIsViolated)
    {
        double adaptedStep;
        VectorXd result_bar;  

        //Calculate result_bar     
        result_bar = x;
        for (int i = 0; i < m_Stages; ++i)
        {
            result_bar += initialStep * m_Bbar(i) * m_K[i];
        }
        VectorXd delta = m_TempResult - x;
        VectorXd errorEstimates = result_bar - m_TempResult;
        double relErr = 0.0, err;

        for (int i = 0; i < x.size(); ++i)
        {
            if (delta(i) > m_RelativeErrorThreshold)
                err = fabs(errorEstimates(i) / delta(i));
            else
                err = fabs(errorEstimates(i));
            if (err > relErr)
                relErr = err;
        }

        if (relErr <= accuracyThreshold)
        {
            // Increase the stepsize, but keep this step
            adaptedStep = 0.9 * initialStep * pow(accuracyThreshold/relErr, m_IncPower);
            if (adaptedStep < minStep)
                adaptedStep = minStep;

            if (adaptedStep > maxStep)
                adaptedStep = maxStep;
            m_bGetAdaptedStep = true;
        }
        else
        {
            if (m_StepAttemptCount >= maxStepAttempts )
            {
                if (bStopIfAccuracyIsViolated == true)
                    throw  SPException(__FILE__, __FUNCTION__, __LINE__, "StepAttemptCount > maxStepAttempts");
                else
                {
                    adaptedStep = initialStep;
                    if (adaptedStep < minStep)
                        adaptedStep = minStep;

                    if (adaptedStep > maxStep)
                        adaptedStep = maxStep;
                    m_bGetAdaptedStep = true;
                }
            }
            else
            {
                // Decrease the stepsize
                adaptedStep = 0.9 * initialStep * pow((accuracyThreshold/relErr), m_DecPower);
                if (adaptedStep < minStep)
                    adaptedStep = minStep;

                if (adaptedStep > maxStep)
                    adaptedStep = maxStep;

                m_bGetAdaptedStep = false;
                ++m_StepAttemptCount;
            }
        }
        return adaptedStep;
    }


	
}


