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
*   SpIntegration.h
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

#ifndef SPINTEGRATION_H
#define SPINTEGRATION_H

#include "SpaceDSL_Global.h"
#include "SpaceDSL/SpRightFunction.h"


#include <vector>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    enum SPACEDSL_API IntegMethodType
    {
        E_NotDefindIntegMethodType = 0,
        E_RungeKutta4 = 1,
        E_RungeKutta78 = 2
    };
	/*************************************************
     * Class type: The class of the N th-order Runge-Kutta method
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     * The default order is 4.
    **************************************************/
	class SPACEDSL_API RungeKutta
	{
	public:
        explicit RungeKutta();
        explicit RungeKutta(const IntegMethodType type);
        ~RungeKutta();
    public:
        void SetIntegMethodType(const IntegMethodType type);

        void SetRelativeErrorThreshold(const double relErr);

        double GetRelativeErrorThreshold(const double relErr) const;

    public:
        /********************************************************************/
        /// Runge Kutta Integral One Step(Fixed/Adapted Step Integral)
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20, 2018-12-25
        /// @Input
        /// @Param  func            Right Function of ODE
        /// @Param	t               The value of the independent variable
        /// @Param	x               Initial function value
        /// @Param	initialStep		Initial step
        /// @Param	minStep                     Mix Adapted step
        /// @Param	maxStep                     Max Adapted step
        /// @Param	maxStepAttempts             Max Attempts
        /// @Param	relativeErrorThreshold		relative Error Threshold
        /// @Param  bStopIfAccuracyIsViolated   Throw Exception or Not
        /// @Output
        /// @Param  result  Integral Step
        /**********************************************************************/

        double OneStep(RightFunc *rightFunc, double t, const VectorXd &x, double initialStep, VectorXd &result,
                       double minStep = 0.0, double maxStep =  0.0, int maxStepAttempts =  0.0,
                       double accuracyThreshold =  0.0, bool bStopIfAccuracyIsViolated = true);

        /********************************************************************/
        /// Runge Kutta Integral Mult Step(Fixed/Adapted Step Integral)
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20, 2018-12-25
        /// @Input
        /// @Param  func            Right Function of ODE
        /// @Param	t0              The Initial value of the independent variable
        /// @Param	x               Initial function value
        /// @Param	initialStep		Initial step
        /// @Param	t               The End value of the independent variable
        /// @Param	minStep                     Mix Adapted step
        /// @Param	maxStep                     Max Adapted step
        /// @Param	maxStepAttempts             Max Attempts
        /// @Param	relativeErrorThreshold		relative Error Threshold
        /// @Param  bStopIfAccuracyIsViolated   Throw Exception or Not
        /// @Output
        /// @Param  result  Integral Result
        /**********************************************************************/

        void MultStep(RightFunc *rightFunc, double t0, const VectorXd &x, double initialStep, double t, VectorXd &result,
                      double minStep = 0, double maxStep = 0, int maxStepAttempts = 0,
                      double relativeErrorThreshold = 0, bool bStopIfAccuracyIsViolated = true);

    protected:
        /********************************************************************/
        /// Runge Kutta Integral Adapted Step Get Function
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-25
        /// @Input
        /// @Param  func            Right Function of ODE
        /// @Param	t               The value of the independent variable
        /// @Param	x               Initial function value
        /// @Param	initialStep		Initial step
        /// @Param	maxStepAttempts             Max Attempts
        /// @Param	relativeErrorThreshold		relative Error Threshold
        /// @Param  bStopIfAccuracyIsViolated   Throw Exception or Not
        /// @Output
        /// @Param  return          Adapted Step
        /**********************************************************************/
        double GetAdaptedStep(RightFunc *rightFunc, double t, const VectorXd &x, double initialStep,
                              double minStep, double maxStep,
                              int maxStepAttempts, double accuracyThreshold, bool bStopIfAccuracyIsViolated);

    protected:

        IntegMethodType             m_IntegMethodType;

        int                         m_Stages;           ///< Number of stages in the specific algorithm implemented
        int                         m_Order;            ///< Order of the expansion used for the integrator
        VectorXd                    m_C;
        MatrixXd                    m_A;
        VectorXd                    m_B;
        VectorXd                    m_Bbar;
        vector<VectorXd>            m_K;
        double                      m_IncPower;
        double                      m_DecPower;

        bool                        m_bGetAdaptedStep;
        int                         m_StepAttemptCount;
        double                      m_RelativeErrorThreshold;

        VectorXd                    m_TempResult;


	};
}

#endif //SPINTEGRATION_H
