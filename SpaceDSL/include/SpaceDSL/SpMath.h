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
*   SpMath.h
*
*   Purpose:
*
*       Define The Math Function in Library
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/
#ifndef SPMATH_H
#define SPMATH_H

#include "SpaceDSL_Global.h"

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    //====================================================================
    //
    // Grouping: Basic Mathematical Function
    //
    //====================================================================
    //
    /// Fractional part of a number (y=x-[x])
    //
    double SPACEDSL_API Fraction (double x);

    //
    /// X Modulo Y
    //
    double SPACEDSL_API Modulo (double x, double y);

    //
    /// Kronecker Function
    // If n == m, its output value is 1, otherwise it will be 0.
    //
    double SPACEDSL_API Delta (int n, int m);

    //
    /// Factorial Function (n!)
    //
    long double SPACEDSL_API Factorial(long double n);



		
}
#endif //SPMATH_H
