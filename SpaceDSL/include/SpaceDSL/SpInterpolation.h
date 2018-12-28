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
*   SpInterpolation.h
*
*   Purpose:
*
*         Numerical Interpolation Methods 
*
*
*   Last modified:
*
*   2018-03-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPINTERPOLATION_H
#define SPINTERPOLATION_H

#include "SpaceDSL_Global.h"


#include <Eigen/Core>


using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

        enum SPACEDSL_API InterpolationType
        {
            E_NotDefinedInterpolation                   = 0,
            E_LinearInterpolation                       = 1,
            E_HermitePolynomialInterpolation            = 2,
            E_LagrangePolynomialInterpolation           = 3
        };
        /********************************************************************/
        /// Unequidistant Linear Interpolation 
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20
        /// @Input
        /// @Param  x    	Independent Variable Array
        /// @Param	y		Function value array
        /// @Param	t		Interpolation Point
        /// @Output
        /// @Param  result  Interpolation Result
        /**********************************************************************/
		double LinearInterpolation(const VectorXd& x, const VectorXd& y, double t);

        /********************************************************************/
        /// Lagrange Polynomial Interpolation
        /// @Author     Niu Zhiyong
        /// @Date       2019-12-26
        /// @Input
        /// @Param  x    	Independent Variable Array
        /// @Param	y		Function value array
        /// @Param	t		Interpolation Point
        /// @Output
        /// @Param  result  Interpolation Result
        /**********************************************************************/
        double LagrangePolynomialInterpolation(const VectorXd& x, const VectorXd& y, double t);

        /********************************************************************/
        /// Hermite Polynomial Interpolation
        /// @Author     Niu Zhiyong
        /// @Date       2019-12-26
        /// @Input
        /// @Param  x    	Independent Variable Array
        /// @Param	y		Function value array
        /// @Param	v		Value derivatives array(y')
        /// @Param	t		Interpolation Point
        /// @Output
        /// @Param  result  Interpolation Result [y, y']
        /**********************************************************************/
        Vector2d HermitePolynomialInterpolation(const VectorXd& x, const VectorXd& y, const VectorXd &v, double t);

}

#endif //SPINTERPOLATION_H
