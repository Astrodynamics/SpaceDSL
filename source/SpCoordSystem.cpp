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
*   SpCoordSystem.cpp
*
*   Purpose:
*
*       CoordSystem computation and Translation
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include <cmath>

#include <Eigen/Geometry>

#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpUtils.h"

namespace SpaceDSL {

    Matrix3d RotateX(double angle)
    {
        const double C = cos(angle);
        const double S = sin(angle);
        Matrix3d U;
        U(0, 0) = 1.0;  U(0, 1) = 0.0;  U(0, 2) = 0.0;
        U(1, 0) = 0.0;  U(1, 1) =  +C;  U(1, 2) =  +S;
        U(2, 0) = 0.0;  U(2, 1) =  -S;  U(2, 2) =  +C;
        return U;
    }

    Matrix3d RotateY(double angle)
    {
        const double C = cos(angle);
        const double S = sin(angle);
        Matrix3d U;
        U(0, 0) =  +C;  U(0, 1) = 0.0;  U(0, 2) =  -S;
        U(1, 0) = 0.0;  U(1, 1) = 1.0;  U(1, 2) = 0.0;
        U(2, 0) =  +S;  U(2, 1) = 0.0;  U(2, 2) =  +C;
        return U;
    }

    Matrix3d RotateZ(double angle)
    {
        const double C = cos(angle);
        const double S = sin(angle);
        Matrix3d U;
        U(0, 0) =  +C;  U(0, 1) =  +S;  U(0, 2) = 0.0;
        U(1, 0) =  -S;  U(1, 1) =  +C;  U(1, 2) = 0.0;
        U(2, 0) = 0.0;  U(2, 1) = 0.0;  U(2, 2) = 1.0;
        return U;
    }



}

