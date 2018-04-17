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
*   SpCoordSystem.h
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

#ifndef SPCOORDSYSTEM_H
#define SPCOORDSYSTEM_H

#include "SpaceDSL_Global.h"

#include <Eigen/Core>

using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
    /********************************************************************/
    /// Elementary Rotations
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	angle	Transfer angle
    /********************************************************************/
    Matrix3d SPACEDSL_API RotateX (double angle);

    Matrix3d SPACEDSL_API RotateY (double angle);

    Matrix3d SPACEDSL_API RotateZ (double angle);

    /********************************************************************/
    ///  Greenwich Mean Sidereal Time
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_UT1	Modified Julian Date (Terrestrial Time)
    /// @Return GMST in [rad]
    /********************************************************************/
    double SPACEDSL_API GMST (double mjd_UT1);

    /********************************************************************/
    /// Greenwich Apparent Sidereal Time
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_UT1	Modified Julian Date (Terrestrial Time)
    /// @Return GMST in [rad]
    /********************************************************************/
    double SPACEDSL_API GAST (double mjd_UT1);

    /********************************************************************/
    /// Computes the Mean Obliquity of the Ecliptic
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT	Modified Julian Date (Terrestrial Time)
    /// @Return Mean Obliquity of the Ecliptic
    /********************************************************************/
    double SPACEDSL_API MeanObliquity (double mjd_TT);

    /********************************************************************/
    /// Nutation in Longitude and Obliquity
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT	Modified Julian Date (Terrestrial Time)
    /// @Output
    /// @Param  dpsi
    /// @Param  deps
    /// @Return
    /********************************************************************/
    void SPACEDSL_API NutationAngles (double mjd_TT, double& dpsi, double& deps);

    /********************************************************************/
    /// Computation of the Equation of the Equinoxes
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT	Modified Julian Date (Terrestrial Time)
    /// @Return Equation of the Equinoxes
    ///  Notes:
    ///   The equation of the equinoxes dpsi*cos(eps) is the right ascension of the
    ///   mean equinox referred to the true equator and equinox and is equal to the
    ///   difference between apparent and mean sidereal time.
    /********************************************************************/
    double SPACEDSL_API EquationEquinox (double mjd_TT);

    /********************************************************************/
    /// Precession Transformation of Equatorial Coordinates
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT1     Epoch given (Modified Julian Date TT)
    /// @Param  mjd_TT2     Epoch to precess to (Modified Julian Date TT)
    /// @Return Precession Transformation Matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API PrecessMatrix (double mjd_TT1, double mjd_TT2);

    /********************************************************************/
    /// Transformation from Mean to True Equator and Equinox
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT      Modified Julian Date (Terrestrial Time)
    /// @Return Nutation matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API NutationMatrix (double mjd_TT);

    /********************************************************************/
    /// Transformation from True Equator and Equinox to
    /// Earth Equator and Greenwich Meridian System
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_UT1      Modified Julian Date UT1
    /// @Return Greenwich Hour Angle matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API GWHourAngMatrix (double mjd_UT1);

}
#endif //SPCOORDSYSTEM_H
