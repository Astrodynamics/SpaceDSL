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
*   SpInterpolation.cpp
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

#include "SpaceDSL/SpInterpolation.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

    /// Unequidistant Linear Interpolation
    double LinearInterpolation(const VectorXd &x, const VectorXd &y, double t)
    {
        int n = int(x.size());
        if(n != y.size())
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "LinearInterpolation: x.size() != y.size()!");

        int i;
        double result;
        result = 0.0;

        if (n < 1)
            return result;

        if (n == 1)
        {
            result = y(0);
            return result;
        }
        if (n == 2)
        {
            if(y(0) == y(1))
                result = y(0);
            else
                result = (y(0) * (t - x(1)) - y(1) * (t - x(0))) / (x(0) - x(1));

            return result;
        }

        if( t <= x(0) )
        {
            i = 1;
        }
        else if( t >= x(n-1) )
        {
            i = n-1;
        }
        else
        {
            i = 0;
            while( (t >= x(i)) && (i < n) )
            {
                i = i+1;
            }
        }

        if(y(i) == y(i-1))
            result = y(i-1);
        else
            result = y(i-1) + (t - x(i-1)) * (y(i) - y(i-1)) / (x(i) - x(i-1));

        return result;
    }




}

