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
*
*************************************************************************/

#ifndef SPINTEGRATION_H
#define SPINTEGRATION_H

#include "SpaceDSL_Global.h"
#include "SpaceDSL/SpRightFunction.h"

#include <Eigen/Core>


using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    enum SPACEDSL_API IntegMethodType
    {
        E_NotDefindIntegMethodType = 0,
        E_RungeKutta4 = 1,
        E_RungeKutta8 = 2
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
        explicit RungeKutta(IntegMethodType type);
        ~RungeKutta();

        void SetIntegMethodType(IntegMethodType type);

    public:
        /********************************************************************/
        /// Runge Kutta Integral One Step
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20
        /// @Input
        /// @Param  func    Right Function of ODE
        /// @Param	t		The value of the independent variable
        /// @Param	x		Initial function value
        /// @Output
        /// @Param  result  Integral Result
        /**********************************************************************/

        void OneStep(RightFunc *rightFunc, double t, const VectorXd &x, double step, VectorXd &result);

        /********************************************************************/
        /// Runge Kutta Integral Mult Step
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20
        /// @Input
        /// @Param  func    Right Function of ODE
        /// @Param	t		The value of the independent variable
        /// @Param	x		Initial function value
        /// @Output
        /// @Param  result  Integral Result
        /**********************************************************************/

        void MultStep(RightFunc *rightFunc, double t, const VectorXd &x, double step, int stepNum, VectorXd &result);

    protected:

        IntegMethodType             m_IntegMethodType;


	};
}

#endif //SPINTEGRATION_H
