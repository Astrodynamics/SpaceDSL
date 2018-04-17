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
*   SpAtmosphere.cpp
*
*   Purpose:
*
*       Atmosphere Model of Earth
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpAtmosphere.h"
#include "SpaceDSL/SpInterpolation.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"

namespace SpaceDSL {

    /*************************************************
     * Class type: The class of Atmospheric Model
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    AtmosphereModel::AtmosphereModel()
    {
        m_AtmosphericModelType = E_NotDefinedAtmosphereModel;
    }

    AtmosphereModel::AtmosphereModel(AtmosphereModelType modelType)
    {
        m_AtmosphericModelType = modelType;
    }

    AtmosphereModel::~AtmosphereModel()
    {

    }

    double AtmosphereModel::GetAtmosphereTemperature(double altitude)
    {
        switch (m_AtmosphericModelType) {
        case E_NotDefinedAtmosphereModel:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
            break;
        case E_1976StdAtmosphere:
            altitude /= 1000;//m to km
            if (altitude > 1000)
                return 0;
            else
                return GetUSSA1976Temperature(altitude);
            break;
        default:
            break;
        }
        return 0;
    }

    double AtmosphereModel::GetAtmospherePressure(double altitude)
    {
        switch (m_AtmosphericModelType) {
        case E_NotDefinedAtmosphereModel:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
            break;
        case E_1976StdAtmosphere:
            altitude /= 1000;//m to km
            if (altitude > 1000)
                return 0;
            else
                return GetUSSA1976Pressure(altitude);
            break;
        default:
            break;
        }
        return 0;
    }

    double AtmosphereModel::GetAtmosphereDensity(double altitude)
    {
        switch (m_AtmosphericModelType) {
        case E_NotDefinedAtmosphereModel:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:E_NotDefinedAtmosphereModel!");
            break;
        case E_1976StdAtmosphere:
            altitude /= 1000;//m to km
            if (altitude > 1000)
                return 0;
            else
                return GetUSSA1976Density(altitude);
            break;
        default:
            break;
        }
        return 0;
    }


    /// Protect Function
    VectorXd AtmosphereModel::GetUSSA1976Composition(double altitude, int stepNum)
    {
        double division = 100;
        VectorXd Z_i(11);
        Z_i << 86, 91, 95, 97, 100,
              110, 115, 120, 150, 500, 1000;

        VectorXd n_i_alt(6);
        n_i_alt.fill(0);
        if (altitude < Z_i(0) && altitude >=0)
            return n_i_alt;

        if (altitude > Z_i(10))
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:USSA1976:Altitude must be 0-1000 km!");

        //   Gas coefficients
        VectorXd alpha_i(6), a_i(6), b_i(6);
        VectorXd Q_i(5), q_i(5), U_i(5), u_i(5), W_i(5), w_i(5);
        alpha_i << 0, 0, 0, 0, -0.4, -0.25;
        a_i << 0, 6.986e20, 4.863e20, 4.487e20, 1.7e21, 3.305e21;
        b_i << 0, 0.75, 0.75, 0.87, 0.691, 0.5;
        Q_i << 0, -5.809644e-4, 1.366212e-4, 9.434079e-5, -2.457369e-4;
        q_i << 0, -3.416248e-3, 0, 0, 0;
        U_i << 0, 56.90311, 86, 86, 86;
        u_i << 0, 97, 0, 0, 0;
        W_i << 0, 2.70624e-5, 8.333333e-5, 8.333333e-5, 6.666667e-4;
        w_i << 0, 5.008765e-4, 0, 0, 0;

        //  Gas Data
        const double R = 8.31432e3;
        const double phi = 7.2e11;
        const double T_7 = 186.8673;
        const double T_11 = 999.2356;
        //   Molecular Weight & Number Density based on values at 86 km & 500 km for
        //   Hydrogen
        VectorXd n_i_86(6);
        n_i_86 << 1.129794e20, 8.6e16, 3.030898e19, 1.3514e18, 7.5817e14, 8e10;
        n_i_alt = n_i_86;
        VectorXd sum_n(6);
        sum_n(0) = sum_n(1) = sum_n(2) = n_i_86(0);
        sum_n(3) = sum_n(4) = (n_i_86(0) + n_i_86(1) +n_i_86(2));
        sum_n(5) = (n_i_86(0) + n_i_86(1) +n_i_86(2) +n_i_86(3) +n_i_86(4));

        const double M_0 = 28.9644;
        VectorXd M_i(6);
        M_i << 28.0134, 15.9994, 31.9988, 39.948, 4.0026, 1.00797;


        VectorXd n_int(6);
        n_int.fill(0);
        int j = 0;
        double Z_start,Z_end;
        VectorXd M(6);

        for (int i = 0; i < (Z_i.size() -1); ++i)
        {
            if (altitude > Z_i(i))
            {
                Z_start = Z_i(i);
                if (altitude > Z_i(i+1))
                    Z_end = Z_i(i+1);
                else
                    Z_end = altitude;

                for (int k = 0; k < stepNum; ++k)//Z_0 = Z_start:step:Z_end-step;Z_1 = Z_0+step;
                {
                    double step = (Z_end - Z_start)/stepNum;
                    double Z_0 = Z_start + k*step;
                    double Z_1 = Z_0 + step;
                    if (Z_1 <= Z_i(5))
                    {
                        for (int m = 0; m < 6; ++m)
                        {
                            M(m) = M_0;
                        }
                    }
                    else
                    {
                        M(0) = n_i_alt(0)*M_i(0)/sum_n(0);
                        M(1) = n_i_alt(0)*M_i(0)/sum_n(1);
                        M(2) = n_i_alt(0)*M_i(0)/sum_n(2);
                        M(3) = (n_i_alt(0)*M_i(0) + n_i_alt(1)*M_i(1) + n_i_alt(2)*M_i(2))/sum_n(3);
                        M(4) = (n_i_alt(0)*M_i(0) + n_i_alt(1)*M_i(1) + n_i_alt(2)*M_i(2))/sum_n(4);
                        M(5) = (n_i_alt(0)*M_i(0) + n_i_alt(1)*M_i(1) + n_i_alt(2)*M_i(2) +
                                n_i_alt(3)*M_i(3) + n_i_alt(4)*M_i(4))/sum_n(5);

                    }
                    sum_n(0) = sum_n(1) = sum_n(2) = n_i_alt(0);
                    sum_n(3) = sum_n(4) = (n_i_alt(0) + n_i_alt(1) + n_i_alt(2));
                    sum_n(5) = (n_i_alt(0) + n_i_alt(1) +n_i_alt(2) +n_i_alt(3) +n_i_alt(4));

                    GasIntegral(Z_0, Z_1, M, sum_n, n_int);
                    for (int m = 0; m < 5; ++m)
                    {
                        n_i_alt(m) = n_i_86(m)*T_7/GetUSSA1976Temperature(Z_1)*exp(-n_int(m));
                    }
                    if (Z_1 < Z_i(8))
                        n_i_alt(5) = 0;
                    else
                    {
                        double tau = TauIntegral(altitude);
                        n_i_alt(5) = pow( T_11/GetUSSA1976Temperature(Z_1), 1+alpha_i(5) )*
                            (n_i_86(5)*exp(-tau)-n_int(5));
                    }

                }
            }
        }

        return n_i_alt;
    }

    double AtmosphereModel::GetUSSA1976Density(double altitude)
    {
        if (altitude < 0 || altitude > 1000)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:USSA1976:Altitude must be 0-1000 km!");

        double density = 0;
        //   Constants
        const double M_0 = 28.9644;
        const double R = 8.31432e3;
        const double N_A = 6.022169e26;
        VectorXd M_i(6);
        M_i << 28.0134, 15.9994, 31.9988, 39.948, 4.0026, 1.00797;
        if (altitude <= 86)
            density = M_0*GetUSSA1976Pressure(altitude)/(R*GetUSSA1976Temperature(altitude));
        else
        {
            VectorXd ComposMat = GetUSSA1976Composition(altitude);
            density = ComposMat.dot(M_i)/N_A;
        }

        return density;
    }

    double AtmosphereModel::GetUSSA1976Temperature(double altitude)
    {
        double tempe = 0;
        //   Constants
        const double r_E = 6.356766e3;

        //   Geopotential/Geometric Altitudes used for Geometric Altitudes < 86 km
        VectorXd H(7),Z(12);
        H << 0, 11, 20, 32, 47, 51, 71;
        for (int i = 0; i < H.size(); ++i)
        {
           Z(i) = r_E * H(i) / (r_E - H(i));
        }
        Z(7) = 86;      Z(8) = 91;      Z(9) = 110;
        Z(10) = 120;    Z(11) = 1000;
        //   Geometric Altitudes used for Altitudes >86 km
        if (altitude < Z(0) || altitude > Z(11))
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:USSA1976:Altitude must be 0-1000 km!");

        //   Defined temperatures at each layer
        VectorXd T(12),L(11);
        T << 288.15, 216.65, 216.65, 228.65, 270.65,
                270.65,214.65,186.95,186.8673,240,360,1000;
        L << -6.5, 0, 1, 2.8, 0, -2.8, -2, 0, 0, 12, 0;

        //   Temperature Calculation with Molecular Temperature below 86 km and
        //   Kinetic Temperature above
        VectorXd Z_M(13), M_M_0(13);
        for (int i = 0; i < 13; ++i)
            Z_M(i) = 80 + i * 0.5;
        M_M_0 << 1, 0.999996, 0.999989, 0.999971, 0.999941, 0.999909
           , 0.999870, 0.999829, 0.999786, 0.999741, 0.999694, 0.999641, 0.999579;
        if (altitude >= Z(0) && altitude <= 80)
            tempe = LinearInterpolation(Z, T, altitude);
        else if (altitude > 80 && altitude <= Z(7))
            tempe = LinearInterpolation(Z, T, altitude)*LinearInterpolation(Z_M,M_M_0,altitude);
        else if (altitude > Z(7) && altitude <= Z(8))
            tempe = T(8);
        else if (altitude > Z(8) && altitude <= Z(9))
        {
            double a = 19.9429;
            double A = -76.3232;
            double T_c = 263.1905;
            tempe = T_c + A * sqrt(1- pow( (altitude-Z(8))/a, 2 ));
        }
        else if (altitude > Z(9) && altitude <= Z(10))
            tempe = LinearInterpolation(Z, T, altitude);
        else if (altitude > Z(10))
        {
            double lambda = L(9)/(T(11) - T(10));
            double xi = (altitude - Z(10))*(r_E + Z(10)) / (r_E + altitude);
            tempe = T(11) - (T(11) - T(10)) * exp(-lambda*xi);
        }

        return tempe;
    }

    double AtmosphereModel::GetUSSA1976Pressure(double altitude)
    {
        double pressure = 0;
        //   Constants
        const double N_A = 6.022169e26;
        const double g_0 = 9.80665;
        const double M_0 = 28.9644;
        const double R = 8.31432e3;
        const double r_E = 6.356766e3;
        //   Geopotential/Geometric Altitudes used for Geometric Altitudes < 86 km
        VectorXd H(8),Z(8);
        H << 0, 11, 20, 32, 47, 51, 71, 84.852;
        for (int i = 0; i < H.size(); ++i)
        {
           Z(i) = r_E * H(i) / (r_E - H(i));
        }
        Z(7) = 86;

        //   Defined temperatures/lapse rates/pressures/density at each layer
        VectorXd T_M_B(7), L(7), P_ref(7);
        T_M_B << 288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65;
        L << -6.5, 0, 1, 2.8, 0, -2.8, -2;
        L /=1e3;
        P_ref << 1.01325e5, 2.2632e4, 5.4748e3, 8.6801e2, 1.1090e2, 6.6938e1, 3.9564;

        VectorXd ComposMat = GetUSSA1976Composition(altitude);
        double sum_n = ComposMat.sum();

        int index = 0;
        if (sum_n == 0)
        {
            for (int i = 0; i < 8; ++i)
            {
                if(altitude >= Z(i) && altitude <  Z(i+1))
                {
                    index = i;
                    break;
                }
            }

            double Z_H = r_E*altitude/(r_E+altitude);
            if (L(index) == 0)
            {
                pressure = P_ref(index)*exp(-g_0*M_0*(Z_H-H(index))*1e3/(R*T_M_B(index)));
            }
            else
            {
                pressure = P_ref(index)*
                        pow(T_M_B(index)/(T_M_B(index)+L(index)*(Z_H-H(index))*1e3), g_0*M_0/(R*L(index)));
            }
        }
        else
            pressure = sum_n * R * GetUSSA1976Temperature(altitude) / N_A;


        return pressure;
    }

    double AtmosphereModel::GetUSSA1976SoundVel(double altitude)
    {
        if (altitude < 0 || altitude > 1000)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:USSA1976:Altitude must be 0-1000 km!");

        double soundVel = 0;
        //   Constants
        const double gamma = 1.4;
        const double M_0 = 28.9644;
        const double R = 8.31432e3;
        if (altitude <= 86)
        {
            soundVel = sqrt(gamma*R*GetUSSA1976Temperature(altitude)/M_0);
        }
        return soundVel;
    }

    double AtmosphereModel::GetUSSA1976DynamicVis(double altitude)
    {
        if (altitude < 0 || altitude > 1000)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:USSA1976:Altitude must be 0-1000 km!");
        //   Constants
        const double beta = 1.458e-6;
        const double S = 110.4;
        //   Dynamic Viscosity
        double mu = 0;
        if (altitude <= 86)
        {
             mu = beta*pow(GetUSSA1976Temperature(altitude), 1.5)/(GetUSSA1976Temperature(altitude)+S);
        }

        return mu;
    }

    double AtmosphereModel::GetUSSA1976KinematicVis(double altitude)
    {
        if (altitude < 0 || altitude > 1000)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:USSA1976:Altitude must be 0-1000 km!");
        //   Kinematic Viscosity
        double nu = 0;
        if (altitude <= 86)
        {
              nu = GetUSSA1976DynamicVis(altitude)/GetUSSA1976Density(altitude);
        }
        return nu;

    }

    double AtmosphereModel::GetUSSA1976ThermalCoeff(double altitude)
    {
        if (altitude < 0 || altitude > 1000)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "AtmosphereModel:USSA1976:Altitude must be 0-1000 km!");
        //   Thermal Conductivity Coefficient
        double k = 0;
        if (altitude <= 86)
        {
            double T_L = GetUSSA1976Temperature(altitude);
            k = 2.64638e-3*pow(T_L,1.5)/(T_L+pow(245*10,-12/T_L));
        }
        return k;
    }

    void AtmosphereModel::GasIntegral(double Z_0, double Z_1, VectorXd &M, VectorXd &sum_n, VectorXd &n_int)
    {
        VectorXd Z_i(11);
        Z_i << 86, 91, 95, 97, 100,
              110, 115, 120, 150, 500, 1000;
        VectorXd M_i(6);
        M_i << 28.0134, 15.9994, 31.9988, 39.948, 4.0026, 1.00797;
        //   Gas coefficients
        VectorXd alpha_i(6), a_i(6), b_i(6);
        VectorXd Q_i(5), q_i(5), U_i(5), u_i(5), W_i(5), w_i(5);
        alpha_i << 0, 0, 0, 0, -0.4, -0.25;
        a_i << 0, 6.986e20, 4.863e20, 4.487e20, 1.7e21, 3.305e21;
        b_i << 0, 0.75, 0.75, 0.87, 0.691, 0.5;
        Q_i << 0, -5.809644e-4, 1.366212e-4, 9.434079e-5, -2.457369e-4;
        q_i << 0, -3.416248e-3, 0, 0, 0;
        U_i << 0, 56.90311, 86, 86, 86;
        u_i << 0, 97, 0, 0, 0;
        W_i << 0, 2.70624e-5, 8.333333e-5, 8.333333e-5, 6.666667e-4;
        w_i << 0, 5.008765e-4, 0, 0, 0;

        //  Gas Data
        const double R = 8.31432e3;
        const double phi = 7.2e11;
        const double g_0 = 9.80665;
        const double r_E = 6.356766e3;
        const double T_c = 263.1905;
        const double A_8 = -76.3232;
        const double a_8 = 19.9429;
        const double L_K_9 = 12;
        const double T_7 = 186.8673;
        const double T_9 = 240;
        const double T_10 = 360;
        const double T_11 = 999.2356;
        const double T_inf = 1000;

        //   Molecular Diffusion Coeffiecients
        const double K_7 = 1.2e2;

        VectorXd alt_j(5);
        for (int i = 0; i < 5; i++)
        {
            double step = (Z_1 - Z_0)/4;
            alt_j(i) = Z_0 + i*step;
        }

        MatrixXd n_i(6, 5); n_i.fill(0);
        double Z, T, dT_dZ, lambda, xi, K, M_N2, g;
        for (int j = 0; j < 5; ++j)
        {
            Z = alt_j(j);
            //     Temperature Values
            if (Z < Z_i(1))
            {
                T = T_7;
                dT_dZ = 0;
            }
            else if (Z < Z_i(5))
            {
                T = T_c+A_8*sqrt(1-pow((Z-Z_i(1))/a_8, 2));
                dT_dZ = -A_8/pow(a_8,2) * (Z-Z_i(1))*pow( (1-pow((Z-Z_i(1))/a_8, 2)), -0.5);
            }
            else if (Z < Z_i(7))
            {
                T = T_9+L_K_9*(Z-Z_i(5));
                dT_dZ = L_K_9;
            }
            else if (Z >= Z_i(7))
            {
                lambda = L_K_9/(T_inf-T_10);
                xi = (Z-Z_i(7))*(r_E+Z_i(7))/(r_E+Z);
                T = T_inf-(T_inf-T_10)*exp(-lambda*xi);
                dT_dZ = lambda*(T_inf-T_10) * pow((r_E+Z_i(7))/(r_E+Z), 2)*exp(-lambda*xi);
            }
            //     K Values
            if (Z < Z_i(2))
                K = K_7;
            else if (Z < Z_i(6))
                K = K_7*exp( 1 - 400/(400-pow(Z-95,2)) );
            else if (Z >= Z_i(6))
                K = 0;
            //     Gravity
            g = g_0*pow(r_E/(r_E+Z), 2);

            //     N
            if (Z <= Z_i(4))
                M_N2 = M(0);
            else
                M_N2 = M_i(0);

            n_i(0,j) = M_N2*g/(T*R)*1e3;
            //     O O2 Ar He
            VectorXd D(4), f_Z(4), vdk(4);
            for (int m = 0; m < 4; ++m)
            {
                D(m) = a_i(m+1)*exp(b_i(m+1)*log(T/273.15))/sum_n(m+1);
            }
            if (K != 0)
            {
                for (int m = 0; m < 4; ++m)
                {
                    f_Z(m) = (g/(R*T)*D(m)/(D(m)+K)*(M_i(m+1)+M(m+1)*K/D(m)+
                        alpha_i(m+1)*R/g*dT_dZ/1e3))*1e3;
                }
            }
            else
            {
                for (int m = 0; m < 4; ++m)
                {
                    f_Z(m) = (g/(R*T)*(M_i(m+1)+alpha_i(m+1)*R/g*dT_dZ/1e3))*1e3;
                }
            }
            if (Z <= Z_i(3))
            {
                for (int m = 0; m < 4; ++m)
                {
                    vdk(m) = Q_i(m+1)*pow(Z-U_i(m+1), 2)*exp(-W_i(m+1)*pow(Z-U_i(m+1), 3))+
                            q_i(m+1)*pow(u_i(m+1)-Z, 2)*
                            exp(-w_i(m+1)*pow(u_i(m+1)-Z, 3));
                }
            }
            else
            {
                for (int m = 0; m < 4; ++m)
                {
                    vdk(m) = Q_i(m+1)*pow(Z-U_i(m+1), 2)*exp(-W_i(m+1)*
                        pow(Z-U_i(m+1), 3));
                }
            }
            for (int m = 0; m < 4; ++m)
            {
                n_i(m+1,j) = f_Z(m) + vdk(m);
            }
            //     H
            if (Z_1 < 150 || Z_1 > 500)
                n_i(5,j) = 0;
            else
            {
                double D_H = a_i(5)*exp(b_i(5)*log(T/273.15))/sum_n(5);
                n_i(5,j) = pow(phi/D_H*(T/T_11),1+alpha_i(5));
            }

        }
        double h = alt_j(1)-alt_j(0);
        n_int = n_int+(2*h/45*(7*n_i.col(0)+32*n_i.col(1)+12*n_i.col(2)+32*n_i.col(3)+7*n_i.col(4)));

    }

    double AtmosphereModel::TauIntegral(double altitude)
    {
        const double L_K_9 = 12;
        const double T_10 = 360;
        const double T_inf = 1000;
        const double Z_10 = 120;
        const double g_0 = 9.80665;
        const double r_E = 6.356766e3;
        const double R = 8.31432e3;
        const double lambda = L_K_9/(T_inf-T_10);
        const double M_H = 1.00797;

        //Value of Integration limit computed previously
        double tau_11 = 8.329503912749350e-4;

        double tau_Z = M_H * g_0 * r_E * r_E / R *
                log((exp(lambda * (altitude - Z_10) * (r_E+Z_10) / (r_E + altitude)) - 1) * T_inf + T_10) /
                (lambda * T_inf * (r_E + Z_10) * (r_E + Z_10));

        return tau_Z-tau_11;
    }

	
	
	
}
