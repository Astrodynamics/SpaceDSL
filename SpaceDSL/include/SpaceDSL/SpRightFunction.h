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
*   SpRightFunction.h
*
*   Purpose:
*
*       The class of right functions is used to 
*		base classes of other right function computing classes.
*
*
*   Last modified:
*
*   2018-03-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPRIGHTFUNCTION_H
#define SPRIGHTFUNCTION_H

#include "SpaceDSL_Global.h"

#include <Eigen/Core>


using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
	/*************************************************
     * Class type: The Base Class of Right Functions of ODE
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
     *  This Function is used to Integrate
    **************************************************/
	class SPACEDSL_API RightFunc
	{
	public:
        RightFunc();
        virtual ~RightFunc();

        /********************************************************************/
        ///     Pure Virtual Function
        /// @Author     Niu Zhiyong
        /// @Date       2018-03-20
        /// @Input
        /// @Param	t		The value of the independent variable
        /// @Param	x		Initial function value
        /// @Output
        /// @Param	result	The calculated value of the function
        /**********************************************************************/
        virtual	void operator() (double t, const VectorXd &x, VectorXd&result) const = 0;

	};
}

#endif //SPRIGHTFUNCTION_H
