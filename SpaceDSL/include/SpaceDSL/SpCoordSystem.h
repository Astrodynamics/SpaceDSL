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
#include "SpTimeSystem.h"
#include "SpOrbitParam.h"
#include "SpConst.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

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
    double SPACEDSL_API GMST (double Mjd_UT1);

    /********************************************************************/
    /// Greenwich Apparent Sidereal Time
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_UT1	Modified Julian Date (Terrestrial Time)
    /// @Return GMST in [rad]
    /********************************************************************/
    double SPACEDSL_API GAST (double Mjd_UT1);

    /********************************************************************/
    /// Computes the Mean Obliquity of the Ecliptic
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT	Modified Julian Date (Terrestrial Time)
    /// @Return Mean Obliquity of the Ecliptic
    /********************************************************************/
    double SPACEDSL_API MeanObliquity (double Mjd_TT);

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
    void SPACEDSL_API NutationAngles (double Mjd_TT, double& dpsi, double& deps);

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
    double SPACEDSL_API EquationEquinox (double Mjd_TT);

    /********************************************************************/
    /// Precession Transformation of Equatorial Coordinates
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT2     Epoch given (Modified Julian Date TT)
    /// @Param  mjd_TT1     Epoch to precess to (Modified Julian Date TT)
    /// @Return Precession Transformation Matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API PrecessMatrix (double Mjd_TT2, double Mjd_TT1 = MJD_J2000);

    /********************************************************************/
    /// Transformation from Mean to True Equator and Equinox
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_TT      Modified Julian Date (Terrestrial Time)
    /// @Return Nutation matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API NutationMatrix (double Mjd_TT);

    /********************************************************************/
    /// Transformation from True Equator and Equinox to
    /// Earth Equator and Greenwich Meridian System
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	mjd_UT1      Modified Julian Date UT1
    /// @Return Greenwich Hour Angle matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API GWHourAngMatrix (double Mjd_UT1);

    /********************************************************************/
    /// Transformation from Pseudo Earth-fixed to Earth-fixed Coordinates
    /// @Author	Niu Zhiyong
    /// @Date	2018-03-20
    /// @Input
    /// @Param	Mjd_UTC      Modified Julian Date UTC
    /// @Return Pole matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API PoleMatrix (double x_pole, double y_pole);

    /********************************************************************/
    /// Transformation from Greenwich Meridian System to Local Tangent Coordinates
    /// @Author	Niu Zhiyong
    /// @Date	2019-01-05
    /// @Input
    /// @Param	longitude   Geodetic East longitude [rad]
    /// @Param	latitude    Geodetic latitude [rad]
    /// @Return Rotation matrix from the Earth equator and Greenwich meridian
    ///         to the local tangent (East-North-Zenith) coordinate system
    /********************************************************************/
    Matrix3d SPACEDSL_API GMSToLTCMtx (double longitude, double latitude);

    /********************************************************************/
    /// Computes Azimuth and Elevation from Local Tangent Coordinates
    /// @Author	Niu Zhiyong
    /// @Date	2019-01-05
    /// @Input
    /// @Param	range       Topocentric local tangent coordinates (East-North-Zenith frame)
    /// @Param	A           Azimuth [rad]
    /// @Param	E           Elevation [rad]
    /// @Return
    /********************************************************************/
    void SPACEDSL_API GetAzEl (const Vector3d &range, double& azimuth, double& elevation);

    /********************************************************************/
    /// Computes Orbit Angle Vel By Vehicle State
    /// @Author	Niu Zhiyong
    /// @Date	2019-01-20
    /// @Input
    /// @Param	range       Topocentric local tangent coordinates (East-North-Zenith frame)
    /// @Param	A           Azimuth [rad]
    /// @Param	E           Elevation [rad]
    /// @Return
    /********************************************************************/
    Vector3d CartStateToOrbAngVelVector(const CartState& state);
    double	 CartStateToOrbAngVel(const CartState& state);		//得到飞行器的轨道角速度

    /********************************************************************/
    /// Transformation VVLH(Vehicle Velocity Local Horizontal) to ICRS Each Other
    /// ICRS Can be J2000 or ECF(Earth Centered Fixed)
    /// @Author	Sun Zhenjiang, Niu Zhiyong
    /// @Date	2018-12-20
    /// @Input
    /// @Param	cart      Position and Velocity in ICRS
    /// @Return Pole matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API VVLHToICSMtx (const CartState &cart);
    Matrix3d SPACEDSL_API ICSToVVLHMtx (const CartState &cart);

    /********************************************************************/
    /// Transformation LVLH((Local Vertical Local Horizontal) to ICRS Each Other
    /// ICRS Can be J2000 or ECF(Earth Centered Fixed)
    /// @Author	Sun Zhenjiang, Niu Zhiyong
    /// @Date	2019-01-20
    /// @Input
    /// @Param	cart      Position and Velocity in ICRS
    /// @Return Pole matrix
    /********************************************************************/
    Matrix3d SPACEDSL_API LVLHToICSMtx (const CartState &cart);
    Matrix3d SPACEDSL_API ICSToLVLHMtx (const CartState &cart);

    /********************************************************************/
    /// Transformation ICRS Absolute State to VVLH Relative State Each Other
    /// ICRS Can be J2000 or ECF(Earth Centered Fixed)
    /// @Author	Niu Zhiyong
    /// @Date	2019-01-20
    /// @Input
    /// @Param	cart            Absolute Position and Velocity in ICRS
    /// @Param	tarCart         Absolute Position and Velocity in ICRS of Target Vehicle
    /// @Output
    /// @Return	relCart/Cart    Relative Position and Velocity in VVLH/Absolute Position and Velocity in ICRS
    /********************************************************************/
    CartState StateToVVLHRelState(const CartState& cart, const CartState& tarCart);
    CartState VVLHRelStateToState(const CartState& relCart, const CartState& tarCart);

    /********************************************************************/
    /// Transformation ICRS Absolute State to LVLH Relative State Each Other
    /// ICRS Can be J2000 or ECF(Earth Centered Fixed)
    /// @Author	Niu Zhiyong
    /// @Date	2019-01-20
    /// @Input
    /// @Param	Cart            Absolute Position and Velocity in ICRS
    /// @Param	tarCart         Absolute Position and Velocity in ICRS of Target Vehicle
    /// @Output
    /// @Return	relCart/Cart    Relative Position and Velocity in LVLH/Absolute Position and Velocity in ICRS
    /********************************************************************/
    CartState StateToLVLHRelState(const CartState& cart, const CartState& tarCart);
    CartState LVLHRelStateToState(const CartState& relCart, const CartState& tarCart);

    /*****************************************************************
     * Class type: Geodetic Coordingot System
     * Author: Niu ZhiYong
     * Date:2018-06-08
     * Description:
     *  Defined Geodetic Coordingot System Parameter and Transformation
    ******************************************************************/
    class SPACEDSL_API GeodeticCoordSystem
    {
    public:
        enum GeodeticCoordType
        {
            E_NotDefinedGeodeticType    = 0,
            E_WGS84System               = 1,
        };

        GeodeticCoordSystem();
        GeodeticCoordSystem(GeodeticCoordType type);
        virtual ~GeodeticCoordSystem();
    public:

        void                SetGeodeticCoordType(GeodeticCoordType type);

        GeodeticCoordType   GetGeodeticCoordType();

    public:
        //********************************************************************
        /// Get the Transformation Matrix From the J2000 ICRS to ECF(Earth Centered Fixed)
        /// @Author	Niu Zhiyong
        /// @Date	2018-06-08
        /// @Input
        /// @Param	Mjd_UTC2			Modified Julian date of UTC
        /// @Param	Mjd_UTC1			Modified Julian date of UTC
        /// @Output
        /// @Return                     the transformation matrix
        //********************************************************************
        static Matrix3d     GetJ2000ToECFMtx (double Mjd_UTC2, double Mjd_UTC1 = MJD_J2000);
        static Matrix3d     GetECFToJ2000Mtx (double Mjd_UTC2, double Mjd_UTC1 = MJD_J2000);

        //********************************************************************
        /// Get the Transformation Matrix From the J2000 ICRS to  True-Of-Date Inertial System
        /// @Author	Niu Zhiyong
        /// @Date	2018-06-08
        /// @Input
        /// @Param	Mjd_UTC2			Modified Julian date of UTC
        /// @Param	Mjd_UTC1			Modified Julian date of UTC
        /// @Output
        /// @Return                     the transformation matrix
        //********************************************************************
        static  Matrix3d    GetJ2000ToTODMtx (double Mjd_UTC2, double Mjd_UTC1 = MJD_J2000);
        static  Matrix3d    GetTODToJ2000Mtx (double Mjd_UTC2, double Mjd_UTC1 = MJD_J2000);

        //********************************************************************
        /// Conversion From the Geodetic Coordinate to ECF(Earth Centered Fixed) Position.
        /// @Author	Niu Zhiyong
        /// @Date	2018-06-08
        /// @Input
        /// @Param  pos/lla
        /// @Output
        /// @Return GeodeticCoord / Position(in ECF)(m)
        //********************************************************************
        GeodeticCoord   GetGeodeticCoord (const Vector3d &pos);
        Vector3d        GetPosition (const GeodeticCoord &lla);

        //********************************************************************
        /// Get the Ground Velocity From the Geodetic Coordinate .
        /// @Author	Niu Zhiyong
        /// @Date	2018-06-08
        /// @Input
        /// @Param  lla
        /// @Output
        /// @Return Velocity(in ECF)(m/s)
        //********************************************************************
        Vector3d        GetGroundVelocity (const GeodeticCoord &lla);

        //********************************************************************
        /// Conversion From the Geodetic Coordinate to the J2000 Position.
        /// @Author	Niu Zhiyong
        /// @Date	2018-06-08
        /// @Input
        /// @Param  pos/lla
        /// @Output
        /// @Return GeodeticCoord / Position(in J2000)(m/s)
        //********************************************************************
        GeodeticCoord   GetGeodeticCoord (const Vector3d &pos, double Mjd_UTC2, double Mjd_UTC1 = MJD_J2000);
        Vector3d        GetPosition (const GeodeticCoord &lla, double Mjd_UTC2, double Mjd_UTC1 = MJD_J2000);


    protected:

        GeodeticCoordType           m_GeodeticCoordType;

    private:
        static  IERSService         m_IERSService;

    };
}
#endif //SPCOORDSYSTEM_H
