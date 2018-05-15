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
*   SpGravity.cpp
*
*   Purpose:
*
*       Gravity Computation
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include "SpaceDSL/SpGravity.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"
#include <fstream>

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {


    GravityModel::GravityModel()
    {
        m_GravModelType = E_NotDefinedGravModel;
    }

    GravityModel::GravityModel(GravModelType modelType)
    {
        m_GravModelType = modelType;
        ifstream fileStream;
        int line = 0;
        switch (m_GravModelType)
        {
        case E_NotDefinedGravModel:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "Undefined Gravity Model Type!");
            break;
        case E_EGM08Model:
            this->m_GM = GM_Earth;//WGS84
            this->m_Radius = EarthRadius;//WGS84

            fileStream.open("./astrodata/Earth/EGM2008.gm");
            if (fileStream.is_open() == false)
            {  
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "EGM2008.gm File is Invalid!");
            }

            while ( !fileStream.eof() )
            {
                char buffer[101];
                fileStream.getline(buffer, 101);
                ++line;
                string bufferstr = buffer;
                string orderStr = bufferstr.substr(0,5);
                m_Nmax = m_Mmax = stoi(orderStr);
                m_MatrixCS.resize(m_Nmax+1, m_Mmax+1);
                m_MatrixCS.fill(0);

                /********* From C_, S_ in Gravity Model File to C, S ********/
                //C, S Will  make up a matrix
                //The lower triangle matrix CS holds the non-sectorial C
                // coefficients C_n,m (n!=m). Sectorial C coefficients C_n,n are the
                // diagonal elements of CS and the upper triangular matrix stores
                // the S_n,m (m!=0) coefficients in columns,for the same degree n.
                // Such as:
                // C(0,0)      S(1,1)   S(2,1)   S(3,1)   ...   S(n_max,1)
                // C(1,0)      C(1,1)   S(2,2)   S(3,2)   ...   S(n_max,2)
                // C(2,0)      C(2,1)   C(2,2)   S(3,3)   ...   S(n_max,3)
                // ...         ...       ...     C(3,3)   ...   S(n_max,4)
                // ...         ...       ...      ...     ...      ...
                // ...         ...       ...      ...     ...   S(n_max,m_max)
                // C(n_max,0) C(n_max,1) ...      ...     ...   C(n_max,m_max)
                // Mapping of CS to C, S is achieved through C_n,m = CS(n,m), S_n,m = CS(m-1,n),m>=1.
                /***********************************************************************/
                while( !fileStream.eof() )
                {
                    fileStream.getline(buffer, 101);
                    ++line;
                    string bufferstr = buffer;
                    int     n = stoi( bufferstr.substr(0,5));
                    int     m = stoi( bufferstr.substr(5,5));
                    double  C_nm = stod( bufferstr.substr(10,25));
                    double  S_nm = stod( bufferstr.substr(35,25));

                    if (m == 0)
                    {
                         m_MatrixCS(n, m) = C_nm * sqrt( (2 - Delta(n,m)) * (2*n + 1) * Factorial(n - m)/Factorial(n + m) );
                    }
                    else
                    {
                        m_MatrixCS(n, m) = C_nm * sqrt( (2 - Delta(n,m)) * (2*n + 1) * Factorial(n - m)/Factorial(n + m) );
                        m_MatrixCS(m-1, n) = S_nm * sqrt( (2 - Delta(n,m)) * (2*n + 1) * Factorial(n - m)/Factorial(n + m) );
                    }

                }
            }
            fileStream.close();
            break;
        default:
            break;
        }
    }

    GravityModel::~GravityModel()
    {

    }

    void GravityModel::SetModelType(GravModelType modelType)
    {
        m_GravModelType = modelType;
    }

    GravModelType GravityModel::GetModelType()
    {
        return m_GravModelType;
    }

    Vector3d GravityModel::AccelHarmonicGravity(const Vector3d &pos, const Matrix3d &ECItoBFCMtx, int n_max, int m_max)
    {
        if (m_max > n_max )
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "GravityModel: m_max > n_max");
        if (n_max > m_Nmax)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "GravityModel: n_max > Nmax");

        // Local variables
        int     n,m;                                // Loop counters
        double  r_sqr, rho, Fac;                    // Auxiliary quantities
        double  x0,y0,z0;                           // Normalized coordinates
        double  ax,ay,az;                           // Acceleration vector
        double  C,S;                                // Gravitational coefficients
        Vector3d  r_bf(3);  r_bf.fill(0);           // Body-fixed position
        Vector3d  a_bf(3);  a_bf.fill(0);           // Body-fixed acceleration

        MatrixXd  V(n_max+2,n_max+2);   V.fill(0);  // Harmonic functions

        MatrixXd  W(n_max+2,n_max+2);   W.fill(0);  // work array (0..n_max+1,0..n_max+1)

        // Body-fixed position
        r_bf = ECItoBFCMtx * pos;

        // Auxiliary quantities
        r_sqr =  r_bf.dot(r_bf);               // Square of distance
        rho   =  m_Radius*m_Radius / r_sqr;

        x0 = m_Radius * r_bf(0) / r_sqr;          // Normalized
        y0 = m_Radius * r_bf(1) / r_sqr;          // coordinates
        z0 = m_Radius * r_bf(2) / r_sqr;

        //
        // Evaluate harmonic functions
        //   V_nm = (R_ref/r)^(n+1) * P_nm(sin(phi)) * cos(m*lambda)
        // and
        //   W_nm = (R_ref/r)^(n+1) * P_nm(sin(phi)) * sin(m*lambda)
        // up to degree and order n_max+1
        //

        // Calculate zonal terms V(n,0); set W(n,0)=0.0
        V(0,0) = m_Radius / sqrt(r_sqr);
        W(0,0) = 0.0;

        V(1,0) = z0 * V(0,0);
        W(1,0) = 0.0;

        for (n = 2; n <= n_max+1; n++)
        {
            V(n,0) = ( (2*n - 1) * z0 * V(n-1,0) - (n - 1) * rho * V(n-2,0) ) / n;
            W(n,0) = 0.0;
        }

        // Calculate tesseral and sectorial terms
        for (m = 1; m <= m_max+1; m++)
        {
            // Calculate V(m,m) .. V(n_max+1,m)

            V(m,m) = (2*m-1) * ( x0*V(m-1,m-1) - y0*W(m-1,m-1) );
            W(m,m) = (2*m-1) * ( x0*W(m-1,m-1) + y0*V(m-1,m-1) );

            if (m <= n_max)
            {
             V(m+1,m) = (2*m+1) * z0 * V(m,m);
             W(m+1,m) = (2*m+1) * z0 * W(m,m);
            }

            for (n=m+2; n<=n_max+1; n++)
            {
                V(n,m) = ( (2*n-1)*z0*V(n-1,m) - (n+m-1)*rho*V(n-2,m) ) / (n-m);
                W(n,m) = ( (2*n-1)*z0*W(n-1,m) - (n+m-1)*rho*W(n-2,m) ) / (n-m);
            }

        }

        //
        // Calculate accelerations ax,ay,az
        //
        ax = ay = az = 0.0;

        for (m=0; m<=m_max; m++)
        {
            for (n=m; n<=n_max ; n++)
            {
                if (m==0)
                {
                    C = m_MatrixCS(n,m);   // = C_n,0
                    ax -=       C * V(n+1,1);
                    ay -=       C * W(n+1,1);
                    az -= (n+1)*C * V(n+1,0);
                }
                else
                {
                    C = m_MatrixCS(n,m);   // = C_n,m
                    S = m_MatrixCS(m-1,n); // = S_n,m
                    Fac = 0.5 * (n-m+1) * (n-m+2);
                    ax +=   + 0.5 * ( - C * V(n+1,m+1) - S * W(n+1,m+1) )
                           + Fac * ( + C * V(n+1,m-1) + S * W(n+1,m-1) );
                    ay +=   + 0.5 * ( - C * W(n+1,m+1) + S * V(n+1,m+1) )
                           + Fac * ( - C * W(n+1,m-1) + S * V(n+1,m-1) );
                    az += (n-m+1) * ( - C * V(n+1,m)   - S * W(n+1,m)   );
                }
            }
        }
        // Body-fixed acceleration 
        a_bf = (m_GM/(m_Radius*m_Radius)) * Vector3d(ax,ay,az);

        // Inertial acceleration
        return  ECItoBFCMtx.transpose()*a_bf;
    }


}
