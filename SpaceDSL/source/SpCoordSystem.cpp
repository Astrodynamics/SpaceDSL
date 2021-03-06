﻿/************************************************************************
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

#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpTimeSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
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

    double GMST(double Mjd_UT1)
    {
        // Variables

        double Mjd_0, UT1 , T_0, T, gmst;

        // Mean Sidereal Time

        Mjd_0 = floor(Mjd_UT1);
        UT1   = DayToSec*(Mjd_UT1 - Mjd_0);          // [s]
        T_0   = (Mjd_0  - MJD_J2000)/36525.0;
        T     = (Mjd_UT1 - MJD_J2000)/36525.0;

        gmst  = 24110.54841 + 8640184.812866*T_0 + 1.002737909350795*UT1
              + (0.093104-6.2e-6*T)*T*T; // [s]

        return  TwoPI*Fraction(gmst/DayToSec);       // [rad], 0..2pi
    }

    double GAST(double Mjd_UT1)
    {
        return Modulo ( GMST(Mjd_UT1) + EquationEquinox(Mjd_UT1), TwoPI );
    }

    double MeanObliquity(double Mjd_TT)
    {
        const double T = (Mjd_TT - MJD_J2000)/36525.0;

        return DegToRad*( 23.43929111-(46.8150+(0.00059-0.001813*T)*T)*T/3600.0 );
    }

    void NutationAngles(double Mjd_TT, double &dpsi, double &deps)
    {
        // Constants

        const double T  = (Mjd_TT - MJD_J2000)/36525.0;
        const double T2 = T*T;
        const double T3 = T2*T;
        const double rev = 360.0*3600.0;  // arcsec/revolution

        const int  N_coeff = 106;
        const long C[N_coeff][9] =
        {
        // IAU 1980 Nutation Theory
        // l  l' F  D Om    dpsi    *T     deps     *T       #
        //
        {  0, 0, 0, 0, 1,-1719960,-1742,  920250,   89 },   //   1
        {  0, 0, 0, 0, 2,   20620,    2,   -8950,    5 },   //   2
        { -2, 0, 2, 0, 1,     460,    0,    -240,    0 },   //   3
        {  2, 0,-2, 0, 0,     110,    0,       0,    0 },   //   4
        { -2, 0, 2, 0, 2,     -30,    0,      10,    0 },   //   5
        {  1,-1, 0,-1, 0,     -30,    0,       0,    0 },   //   6
        {  0,-2, 2,-2, 1,     -20,    0,      10,    0 },   //   7
        {  2, 0,-2, 0, 1,      10,    0,       0,    0 },   //   8
        {  0, 0, 2,-2, 2, -131870,  -16,   57360,  -31 },   //   9
        {  0, 1, 0, 0, 0,   14260,  -34,     540,   -1 },   //  10
        {  0, 1, 2,-2, 2,   -5170,   12,    2240,   -6 },   //  11
        {  0,-1, 2,-2, 2,    2170,   -5,    -950,    3 },   //  12
        {  0, 0, 2,-2, 1,    1290,    1,    -700,    0 },   //  13
        {  2, 0, 0,-2, 0,     480,    0,      10,    0 },   //  14
        {  0, 0, 2,-2, 0,    -220,    0,       0,    0 },   //  15
        {  0, 2, 0, 0, 0,     170,   -1,       0,    0 },   //  16
        {  0, 1, 0, 0, 1,    -150,    0,      90,    0 },   //  17
        {  0, 2, 2,-2, 2,    -160,    1,      70,    0 },   //  18
        {  0,-1, 0, 0, 1,    -120,    0,      60,    0 },   //  19
        { -2, 0, 0, 2, 1,     -60,    0,      30,    0 },   //  20
        {  0,-1, 2,-2, 1,     -50,    0,      30,    0 },   //  21
        {  2, 0, 0,-2, 1,      40,    0,     -20,    0 },   //  22
        {  0, 1, 2,-2, 1,      40,    0,     -20,    0 },   //  23
        {  1, 0, 0,-1, 0,     -40,    0,       0,    0 },   //  24
        {  2, 1, 0,-2, 0,      10,    0,       0,    0 },   //  25
        {  0, 0,-2, 2, 1,      10,    0,       0,    0 },   //  26
        {  0, 1,-2, 2, 0,     -10,    0,       0,    0 },   //  27
        {  0, 1, 0, 0, 2,      10,    0,       0,    0 },   //  28
        { -1, 0, 0, 1, 1,      10,    0,       0,    0 },   //  29
        {  0, 1, 2,-2, 0,     -10,    0,       0,    0 },   //  30
        {  0, 0, 2, 0, 2,  -22740,   -2,    9770,   -5 },   //  31
        {  1, 0, 0, 0, 0,    7120,    1,     -70,    0 },   //  32
        {  0, 0, 2, 0, 1,   -3860,   -4,    2000,    0 },   //  33
        {  1, 0, 2, 0, 2,   -3010,    0,    1290,   -1 },   //  34
        {  1, 0, 0,-2, 0,   -1580,    0,     -10,    0 },   //  35
        { -1, 0, 2, 0, 2,    1230,    0,    -530,    0 },   //  36
        {  0, 0, 0, 2, 0,     630,    0,     -20,    0 },   //  37
        {  1, 0, 0, 0, 1,     630,    1,    -330,    0 },   //  38
        { -1, 0, 0, 0, 1,    -580,   -1,     320,    0 },   //  39
        { -1, 0, 2, 2, 2,    -590,    0,     260,    0 },   //  40
        {  1, 0, 2, 0, 1,    -510,    0,     270,    0 },   //  41
        {  0, 0, 2, 2, 2,    -380,    0,     160,    0 },   //  42
        {  2, 0, 0, 0, 0,     290,    0,     -10,    0 },   //  43
        {  1, 0, 2,-2, 2,     290,    0,    -120,    0 },   //  44
        {  2, 0, 2, 0, 2,    -310,    0,     130,    0 },   //  45
        {  0, 0, 2, 0, 0,     260,    0,     -10,    0 },   //  46
        { -1, 0, 2, 0, 1,     210,    0,    -100,    0 },   //  47
        { -1, 0, 0, 2, 1,     160,    0,     -80,    0 },   //  48
        {  1, 0, 0,-2, 1,    -130,    0,      70,    0 },   //  49
        { -1, 0, 2, 2, 1,    -100,    0,      50,    0 },   //  50
        {  1, 1, 0,-2, 0,     -70,    0,       0,    0 },   //  51
        {  0, 1, 2, 0, 2,      70,    0,     -30,    0 },   //  52
        {  0,-1, 2, 0, 2,     -70,    0,      30,    0 },   //  53
        {  1, 0, 2, 2, 2,     -80,    0,      30,    0 },   //  54
        {  1, 0, 0, 2, 0,      60,    0,       0,    0 },   //  55
        {  2, 0, 2,-2, 2,      60,    0,     -30,    0 },   //  56
        {  0, 0, 0, 2, 1,     -60,    0,      30,    0 },   //  57
        {  0, 0, 2, 2, 1,     -70,    0,      30,    0 },   //  58
        {  1, 0, 2,-2, 1,      60,    0,     -30,    0 },   //  59
        {  0, 0, 0,-2, 1,     -50,    0,      30,    0 },   //  60
        {  1,-1, 0, 0, 0,      50,    0,       0,    0 },   //  61
        {  2, 0, 2, 0, 1,     -50,    0,      30,    0 },   //  62
        {  0, 1, 0,-2, 0,     -40,    0,       0,    0 },   //  63
        {  1, 0,-2, 0, 0,      40,    0,       0,    0 },   //  64
        {  0, 0, 0, 1, 0,     -40,    0,       0,    0 },   //  65
        {  1, 1, 0, 0, 0,     -30,    0,       0,    0 },   //  66
        {  1, 0, 2, 0, 0,      30,    0,       0,    0 },   //  67
        {  1,-1, 2, 0, 2,     -30,    0,      10,    0 },   //  68
        { -1,-1, 2, 2, 2,     -30,    0,      10,    0 },   //  69
        { -2, 0, 0, 0, 1,     -20,    0,      10,    0 },   //  70
        {  3, 0, 2, 0, 2,     -30,    0,      10,    0 },   //  71
        {  0,-1, 2, 2, 2,     -30,    0,      10,    0 },   //  72
        {  1, 1, 2, 0, 2,      20,    0,     -10,    0 },   //  73
        { -1, 0, 2,-2, 1,     -20,    0,      10,    0 },   //  74
        {  2, 0, 0, 0, 1,      20,    0,     -10,    0 },   //  75
        {  1, 0, 0, 0, 2,     -20,    0,      10,    0 },   //  76
        {  3, 0, 0, 0, 0,      20,    0,       0,    0 },   //  77
        {  0, 0, 2, 1, 2,      20,    0,     -10,    0 },   //  78
        { -1, 0, 0, 0, 2,      10,    0,     -10,    0 },   //  79
        {  1, 0, 0,-4, 0,     -10,    0,       0,    0 },   //  80
        { -2, 0, 2, 2, 2,      10,    0,     -10,    0 },   //  81
        { -1, 0, 2, 4, 2,     -20,    0,      10,    0 },   //  82
        {  2, 0, 0,-4, 0,     -10,    0,       0,    0 },   //  83
        {  1, 1, 2,-2, 2,      10,    0,     -10,    0 },   //  84
        {  1, 0, 2, 2, 1,     -10,    0,      10,    0 },   //  85
        { -2, 0, 2, 4, 2,     -10,    0,      10,    0 },   //  86
        { -1, 0, 4, 0, 2,      10,    0,       0,    0 },   //  87
        {  1,-1, 0,-2, 0,      10,    0,       0,    0 },   //  88
        {  2, 0, 2,-2, 1,      10,    0,     -10,    0 },   //  89
        {  2, 0, 2, 2, 2,     -10,    0,       0,    0 },   //  90
        {  1, 0, 0, 2, 1,     -10,    0,       0,    0 },   //  91
        {  0, 0, 4,-2, 2,      10,    0,       0,    0 },   //  92
        {  3, 0, 2,-2, 2,      10,    0,       0,    0 },   //  93
        {  1, 0, 2,-2, 0,     -10,    0,       0,    0 },   //  94
        {  0, 1, 2, 0, 1,      10,    0,       0,    0 },   //  95
        { -1,-1, 0, 2, 1,      10,    0,       0,    0 },   //  96
        {  0, 0,-2, 0, 1,     -10,    0,       0,    0 },   //  97
        {  0, 0, 2,-1, 2,     -10,    0,       0,    0 },   //  98
        {  0, 1, 0, 2, 0,     -10,    0,       0,    0 },   //  99
        {  1, 0,-2,-2, 0,     -10,    0,       0,    0 },   // 100
        {  0,-1, 2, 0, 1,     -10,    0,       0,    0 },   // 101
        {  1, 1, 0,-2, 1,     -10,    0,       0,    0 },   // 102
        {  1, 0,-2, 2, 0,     -10,    0,       0,    0 },   // 103
        {  2, 0, 0, 2, 0,      10,    0,       0,    0 },   // 104
        {  0, 0, 2, 4, 2,     -10,    0,       0,    0 },   // 105
        {  0, 1, 0, 1, 0,      10,    0,       0,    0 }    // 106
        };

        // Variables

        double  l, lp, F, D, Om;
        double  arg;


        // Mean arguments of luni-solar motion
        //
        //   l   mean anomaly of the Moon
        //   l'  mean anomaly of the Sun
        //   F   mean argument of latitude
        //   D   mean longitude elongation of the Moon from the Sun
        //   Om  mean longitude of the ascending node

        l  = Modulo (  485866.733 + (1325.0*rev +  715922.633)*T
                                     + 31.310*T2 + 0.064*T3, rev );
        lp = Modulo ( 1287099.804 + (  99.0*rev + 1292581.224)*T
                                     -  0.577*T2 - 0.012*T3, rev );
        F  = Modulo (  335778.877 + (1342.0*rev +  295263.137)*T
                                     - 13.257*T2 + 0.011*T3, rev );
        D  = Modulo ( 1072261.307 + (1236.0*rev + 1105601.328)*T
                                     -  6.891*T2 + 0.019*T3, rev );
        Om = Modulo (  450160.280 - (   5.0*rev +  482890.539)*T
                                     +  7.455*T2 + 0.008*T3, rev );

        // Nutation in longitude and obliquity [rad]

        deps = dpsi = 0.0;
        for (int i=0; i<N_coeff; i++)
        {
            arg  =  ( C[i][0]*l+C[i][1]*lp+C[i][2]*F+C[i][3]*D+C[i][4]*Om ) * ArcSecToRad;
            dpsi += ( C[i][5]+C[i][6]*T ) * sin(arg);
            deps += ( C[i][7]+C[i][8]*T ) * cos(arg);
        }

        dpsi = 1.0E-5 * dpsi * ArcSecToRad;
        deps = 1.0E-5 * deps * ArcSecToRad;
    }

    double EquationEquinox(double Mjd_TT)
    {
        // Nutation angles
        double dpsi, deps;

        // Nutation in longitude and obliquity

        NutationAngles (Mjd_TT, dpsi, deps );

        // Equation of the equinoxes

        return  dpsi * cos( MeanObliquity(Mjd_TT) );
    }

    Matrix3d PrecessMatrix(double Mjd_TT2, double Mjd_TT1)
    {

      // Constants

      const double T  = (Mjd_TT1 - MJD_J2000)/36525.0;
      const double dT = (Mjd_TT2 - Mjd_TT1)/36525.0;

      // Variables

      double zeta,z,theta;

      // Precession angles

      zeta  =  ( (2306.2181 + (1.39656 - 0.000139*T)*T)+
                    ((0.30188 - 0.000344*T) + 0.017998*dT)*dT )*dT*ArcSecToRad;
      z     =  zeta + ( (0.79280  +0.000411*T) + 0.000205*dT)*dT*dT*ArcSecToRad;
      theta =  ( (2004.3109 - (0.85330 + 0.000217*T)*T)-
                    ((0.42665 + 0.000217*T)+0.041833*dT)*dT )*dT*ArcSecToRad;

      // Precession matrix

      return RotateZ(-z) * RotateY(theta) * RotateZ(-zeta);

    }

    Matrix3d NutationMatrix(double Mjd_TT)
    {
        double dpsi, deps, eps;

        // Mean obliquity of the ecliptic

        eps = MeanObliquity(Mjd_TT);

        // Nutation in longitude and obliquity

        NutationAngles (Mjd_TT, dpsi, deps);

        // Transformation from mean to true equator and equinox

        return  RotateX(-eps-deps)*RotateZ(-dpsi)*RotateX(+eps);
    }

    Matrix3d GWHourAngMatrix(double Mjd_UT1)
    {
        return  RotateZ( GAST(Mjd_UT1) );
    }

    Matrix3d PoleMatrix( double x_pole, double y_pole)
    {
        return  RotateY(-x_pole) * RotateX(-y_pole);
    }

    Matrix3d GMSToLTCMtx(double longitude, double latitude)
    {
        Matrix3d  M;
        double  Aux;

        // Transformation to Zenith-East-North System
        M = RotateY(-latitude) * RotateZ(longitude);

        // Cyclic shift of rows 0,1,2 to 1,2,0 to obtain East-North-Zenith system
        for (int j = 0; j < 3; j++)
        {
            Aux = M(0,j);
            M(0,j)=M(1,j);
            M(1,j)=M(2,j);
            M(2,j)= Aux;
        }

        return  M;
    }

    void GetAzEl(const Vector3d &range, double &azimuth, double &elevation)
    {
        azimuth = atan2(range(0), range(1));
        azimuth = ((azimuth<0.0)? azimuth+TwoPI : azimuth);
        elevation = atan ( range(2) / sqrt(range(0)*range(0)+range(1)*range(1)) );
    }

    Vector3d CartStateToOrbAngVelVector(const CartState &cart)
    {
        Vector3d angVel;
        angVel = cart.Pos().cross(cart.Vel()) / sqrt(cart.Pos().norm());
        return angVel;
    }


    double CartStateToOrbAngVel(const CartState &cart)
    {
        Vector3d angVel = CartStateToOrbAngVelVector(cart);
        return angVel.norm();
    }

    Matrix3d VVLHToICSMtx(const CartState &cart)
    {
        auto r = cart.Pos().norm();
        auto nn = cart.Pos().cross(cart.Vel());
        auto n = nn.norm();
        auto ss = cart.Pos().cross(nn);
        auto s = ss.norm();

        Matrix3d mtx,mtxTmp;
        mtxTmp.row(0) = -ss / s;
        mtxTmp.row(1) = -nn / n;
        mtxTmp.row(2) = -cart.Pos() / r;

        mtx = mtxTmp.inverse();

        return mtx;
    }

    Matrix3d ICSToVVLHMtx(const CartState &cart)
    {
        auto r = cart.Pos().norm();
        auto nn = cart.Pos().cross(cart.Vel());
        auto n = nn.norm();
        auto ss = cart.Pos().cross(nn);
        auto s = ss.norm();

        Matrix3d mtx;
        mtx.row(0) = -ss / s;
        mtx.row(1) = -nn / n;
        mtx.row(2) = -cart.Pos() / r;

        return mtx;
    }

    Matrix3d LVLHToICSMtx(const CartState &cart)
    {
        // Radiation direction, x
        auto r = cart.Pos().norm();
        // Normal direction, z
        auto nn = cart.Pos().cross(cart.Vel());
        auto n = nn.norm();
        // Co-velocity direction, y
        auto ss = nn.cross(cart.Pos());
        auto s = ss.norm();

        Matrix3d mtx;
        Matrix3d mtxTmp;
        mtxTmp.row(0) = cart.Pos() / r;
        mtxTmp.row(1) = ss / s;
        mtxTmp.row(2) = nn / n;
        mtx = mtxTmp.inverse();

        return mtx;
    }

    Matrix3d ICSToLVLHMtx(const CartState &cart)
    {
        // Radiation direction, x
        auto r = cart.Pos().norm();
        // Normal direction, z
        auto nn = cart.Pos().cross(cart.Vel());
        auto n = nn.norm();
        // Co-velocity direction, y
        auto ss = nn.cross(cart.Pos());
        auto s = ss.norm();

        Matrix3d mtx;
        mtx.row(0) = cart.Pos() / r;
        mtx.row(1) = ss / s;
        mtx.row(2) = nn / n;

        return mtx;
    }


    CartState StateToVVLHRelState(const CartState &cart, const CartState &tarCart)
    {
        CartState   relCart;
        Vector3d    angVel;
        Matrix3d    transMtx;

        angVel = CartStateToOrbAngVelVector(tarCart);
        relCart.SetPos(cart.Pos()-tarCart.Pos());
        relCart.SetVel(cart.Vel()-tarCart.Vel()-angVel.cross(relCart.Pos()));

        transMtx = ICSToVVLHMtx(tarCart);
        relCart.SetPos(transMtx*relCart.Pos());
        relCart.SetVel(transMtx*relCart.Vel());

        return relCart;
    }

    CartState VVLHRelStateToState(const CartState &relCart, const CartState &tarCart)
    {
        CartState   cart;
        Vector3d    angVel;
        Matrix3d    transMtx;

        transMtx = VVLHToICSMtx(tarCart);
        cart.SetPos(transMtx*relCart.Pos());
        cart.SetVel(transMtx*relCart.Vel());

        angVel = CartStateToOrbAngVelVector(tarCart);
        cart.SetVel( cart.Vel() + tarCart.Vel() + angVel.cross(cart.Pos()));
        cart.SetPos( cart.Pos() + tarCart.Pos());
        return cart;
    }


    CartState StateToLVLHRelState(const CartState &cart, const CartState &tarCart)
    {
        CartState   relCart;
        Vector3d    angVel;
        Matrix3d    transMtx;

        angVel = CartStateToOrbAngVelVector(tarCart);
        relCart.SetPos(cart.Pos()-tarCart.Pos());
        relCart.SetVel(cart.Vel()-tarCart.Vel()-angVel.cross(relCart.Pos()));

        transMtx = ICSToLVLHMtx(tarCart);
        relCart.SetPos(transMtx*relCart.Pos());
        relCart.SetVel(transMtx*relCart.Vel());

        return relCart;
    }

    CartState LVLHRelStateToState(const CartState &relCart, const CartState &tarCart)
    {
        CartState   cart;
        Vector3d    angVel;
        Matrix3d    transMtx;

        transMtx = LVLHToICSMtx(tarCart);
        cart.SetPos(transMtx*relCart.Pos());
        cart.SetVel(transMtx*relCart.Vel());

        angVel = CartStateToOrbAngVelVector(tarCart);
        cart.SetVel( cart.Vel() + tarCart.Vel() + angVel.cross(cart.Pos()));
        cart.SetPos( cart.Pos() + tarCart.Pos());
        return cart;
    }

    /*****************************************************************
     * Class type: Geodetic Coordingot System
     * Author: Niu ZhiYong
     * Date:2018-06-08
     * Description:
     *  Defined Geodetic Coordingot System Parameter and Transformation
    ******************************************************************/
    IERSService GeodeticCoordSystem::m_IERSService = IERSService();
    GeodeticCoordSystem::GeodeticCoordSystem()
    {
        m_GeodeticCoordType = E_NotDefinedGeodeticType;
    }

    GeodeticCoordSystem::GeodeticCoordSystem(GeodeticCoordSystem::GeodeticCoordType type)
    {
        m_GeodeticCoordType = type;
    }

    GeodeticCoordSystem::~GeodeticCoordSystem()
    {

    }

    void GeodeticCoordSystem::SetGeodeticCoordType(GeodeticCoordSystem::GeodeticCoordType type)
    {
        m_GeodeticCoordType = type;
    }

    GeodeticCoordSystem::GeodeticCoordType GeodeticCoordSystem::GetGeodeticCoordType()
    {
        if (m_GeodeticCoordType == E_NotDefinedGeodeticType)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Geodetic Coord System Type!");

        return m_GeodeticCoordType;
    }

    Matrix3d GeodeticCoordSystem::GetJ2000ToECFMtx(double Mjd_UTC2, double Mjd_UTC1)
    {
        double Mjd_UT1 = Mjd_UTC2 + m_IERSService.GetValue(Mjd_UTC2,"UT1-UTC")/DayToSec;
        double x_pole = m_IERSService.GetValue(Mjd_UTC2,"x_pole")*PI/180/360;
        double y_pole = m_IERSService.GetValue(Mjd_UTC2,"y_pole")*PI/180/360;
        Matrix3d J2000toECFMtx = PoleMatrix(x_pole, y_pole) * GWHourAngMatrix(Mjd_UT1) *
                GetJ2000ToTODMtx(Mjd_UTC2, Mjd_UTC1);

        return J2000toECFMtx;
    }

    Matrix3d GeodeticCoordSystem::GetECFToJ2000Mtx(double Mjd_UTC2, double Mjd_UTC1)
    {
        return GetJ2000ToECFMtx(Mjd_UTC2, Mjd_UTC1).transpose();
    }

    Matrix3d GeodeticCoordSystem::GetJ2000ToTODMtx(double Mjd_UTC2, double Mjd_UTC1)
    {
        double Mjd_TT = Mjd_UTC2  + (IERSService::TT_TAI + m_IERSService.GetValue(Mjd_UTC2, "leapseconds"))/DayToSec;
        Matrix3d J2000ToTODMtx = NutationMatrix(Mjd_TT) * PrecessMatrix(Mjd_TT, Mjd_UTC1);

        return J2000ToTODMtx;
    }

    Matrix3d GeodeticCoordSystem::GetTODToJ2000Mtx(double Mjd_UTC2, double Mjd_UTC1)
    {
        return GetJ2000ToTODMtx(Mjd_UTC2, Mjd_UTC1).transpose();
    }

    GeodeticCoord GeodeticCoordSystem::GetGeodeticCoord (const Vector3d &pos)
    {
        GeodeticCoord lla;
        double R_equ;                               // Equator radius [m]
        double f;                                   // Flattening
        switch (m_GeodeticCoordType)
        {
        case E_NotDefinedGeodeticType:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Geodetic Coord System Type!");
            break;
        case E_WGS84System:
            R_equ = WGS84RE;
            f     = WGS84F;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Geodetic Coord System Type!");
        }
        const double  eps     = 100*EPS;            // Convergence criterion
        const double  epsRequ = eps*R_equ;
        const double  e2      = f*(2.0-f);          // Square of eccentricity

        const double  X = pos(0);                   // Cartesian coordinates
        const double  Y = pos(1);
        const double  Z = pos(2);
        const double  rho2 = X*X + Y*Y;             // Square of distance from z-axis

        // Check validity of input data

        if (pos.norm() == 0.0)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "GeodeticCoordSystem::GetGeodeticCoord invalid input in Geodetic constructor!");
        }
        else if (pos.norm() < EarthMinRadius)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "GeodeticCoordSystem::GetGeodeticCoord CartState is in Earth!");
        }

        // Iteration
        double  dZ = 0.0;
        double  dZ_new = 0.0;
        double  SinPhi = 0.0;
        double  ZdZ = 0.0;
        double  Nh = 0.0;
        double  N = 0.0;

        dZ = e2 * Z;
        if (isnan(dZ))
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "GeodeticCoordSystem::GetGeodeticCoord dZ is Nan!");
        do
        {
            ZdZ    =  Z + dZ;
            Nh     =  sqrt ( rho2 + ZdZ*ZdZ );
            SinPhi =  ZdZ / Nh;                    // Sine of geodetic latitude
            N      =  R_equ / sqrt(1.0-e2*SinPhi*SinPhi);
            dZ_new =  N*e2*SinPhi;
            if ( fabs(dZ-dZ_new) <= epsRequ ) break;
            dZ = dZ_new;
        }while (true);

        // Longitude, latitude, altitude
        lla.SetAltitude( Nh - N);
        lla.SetLatitude(atan2 ( ZdZ, sqrt(rho2) ));
        lla.SetLongitude(atan2 ( Y, X ));

        return lla;
    }

    Vector3d GeodeticCoordSystem::GetPosition(const GeodeticCoord &lla)
    {
        double R_equ;                               // Equator radius [m]
        double f;                                   // Flattening
        switch (m_GeodeticCoordType)
        {
        case E_NotDefinedGeodeticType:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Geodetic Coord System Type!");
            break;
        case E_WGS84System:
            R_equ = WGS84RE;
            f     = WGS84F;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Geodetic Coord System Type!");
        }
        double lat = lla.Latitude();
        double lon = lla.Longitude();
        double  h  = lla.Altitude();

        const double  e2     = f*(2.0-f);                   // Square of eccentricity
        const double  CosLat = cos(lat);                    // (Co)sine of geodetic latitude
        const double  SinLat = sin(lat);

        double      N;
        Vector3d    r;


        // Position vector

        N = R_equ / sqrt(1.0-e2*SinLat*SinLat);

        r(0) =  (         N+h)*CosLat*cos(lon);
        r(1) =  (         N+h)*CosLat*sin(lon);
        r(2) =  ((1.0-e2)*N+h)*SinLat;

        return r;
    }

    Vector3d GeodeticCoordSystem::GetGroundVelocity(const GeodeticCoord &lla)
    {
        double R_equ;                               // Equator radius [m]
        double f;                                   // Flattening
        switch (m_GeodeticCoordType)
        {
        case E_NotDefinedGeodeticType:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Geodetic Coord System Type!");
            break;
        case E_WGS84System:
            R_equ = WGS84RE;
            f     = WGS84F;
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Geodetic Coord System Type!");
        }
        double lat = lla.Latitude();
        double lon = lla.Longitude();
        double  h  = lla.Altitude();

        const double  e2     = f*(2.0-f);                   // Square of eccentricity
        const double  CosLat = cos(lat);                    // (Co)sine of geodetic latitude
        const double  SinLat = sin(lat);

        double      N;
        Vector3d    r;


        // Position vector

        N = R_equ / sqrt(1.0-e2*SinLat*SinLat);

        r(0) =  (         N+h)*CosLat*cos(lon);
        r(1) =  (         N+h)*CosLat*sin(lon);
        r(2) =  ((1.0-e2)*N+h)*SinLat;

        Vector3d    vel;

        r(0) = (-EarthAngVel * r(1));
        r(1) = (EarthAngVel * r(0));
        r(2) = (0);
        return vel;

    }

    GeodeticCoord GeodeticCoordSystem::GetGeodeticCoord(const Vector3d &pos, double Mjd_UTC2, double Mjd_UTC1)
    {
        Matrix3d J2000ToECFMtx = this->GetJ2000ToECFMtx(Mjd_UTC2, Mjd_UTC1);
        Vector3d r_ECF = J2000ToECFMtx * pos;
        GeodeticCoord lla = this->GetGeodeticCoord(r_ECF);
        return lla;
    }

    Vector3d GeodeticCoordSystem::GetPosition(const GeodeticCoord &lla, double Mjd_UTC2, double Mjd_UTC1)
    {
        Matrix3d ECFToJ2000Mtx = this->GetJ2000ToECFMtx(Mjd_UTC2, Mjd_UTC1).transpose();
        Vector3d r_ECF = this->GetPosition(lla);
        Vector3d pos = ECFToJ2000Mtx * r_ECF;
        return pos;
    }

}

