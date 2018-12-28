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
        int length = static_cast<int>(x.size());
        if(length != y.size())
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "LinearInterpolation: x.size() != y.size()!");

        int i;
        double result;
        result = 0.0;

        if (length < 1)
            return result;

        if (length == 1)
        {
            result = y(0);
            return result;
        }
        if (length == 2)
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
        else if( t >= x(length - 1) )
        {
            i = length - 1;
        }
        else
        {
            i = 0;
            while( (t >= x(i)) && (i < length) )
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

    double LagrangePolynomialInterpolation(const VectorXd &x, const VectorXd &y, double t)
    {
        double result = 0;
        int i;
        int j;
        int length = static_cast<int>(x.size());

        if(length != static_cast<int>(y.size()))
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "LagrangePolynomialInterpolation: x.size() != y.size()!");

        for (i = 0; i < length; i++)
        {
            double coefficient = 1;
            for (j = 0; j < length; j++)
            {
                if (j != i)
                {
                    double diffX = x(i) - x(j);
                    coefficient *= (t - x(j)) / diffX;
                }
            }

            result += coefficient * y(i);
        }


        return result;
    }

    Vector2d HermitePolynomialInterpolation(const VectorXd &x, const VectorXd &y, const VectorXd &v, double t)
    {
        int length = static_cast<int>(x.size());
        if(length != y.size())
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "HermitePolynomialInterpolation: x.size() != y.size()!");

        if(length != v.size())
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "HermitePolynomialInterpolation: x.size() != v.size()!");

        Vector2d result; // [y, y']
        for( int i = 0;i < length; i++)
        {
            if(t == x(i))
            {
                result(0) = y(i);
                result(1) = v(i);
                return result;
            }
        }

        VectorXd l;     l.resize(length);   l.fill(1.0);
        VectorXd a;     a.resize(length);   a.fill(0);
        VectorXd f;     f.resize(length);   f.fill(0);
        VectorXd h1;    h1.resize(length);
        VectorXd h2;    h2.resize(length);

        for(int i = 0; i < length; i++)
        {
            for(int j = 0; j < length; j++)
            {
                if(i != j)
                {
                    l(i) *= (t - x(i)) / (x(i) - x(j));
                    a(i) += 1 / (x(i) - x(j));
                }
            }

            //遍历每个采样点,求f[i],用于求hv
            for(int j = 0; j < length; j++)
            {
                if(i != j)
                    f(i) += l(i) / (t - x(j));
            }

            h1(i) = (1 - 2*a(i) * (t-x(i)) )* l(i) * l(i);
            h2(i) = (t - x(i)) * l(i) * l(i);

            result(0) += y(i)*h1(i) + v(i)*h2(i);

            result(1) += (2*l(i)*f(i) - 2*a(i)*l(i)*l(i) - 4*a(i)*(t-x(i))*l(i)*f(i))
                        * y(i) + (l(i)*l(i) + 2*l(i)*f(i)*(t-x(i))) * v(i);
        }

        return result;
    }




}

