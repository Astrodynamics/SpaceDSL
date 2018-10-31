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
*   SpGravity.h
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

#ifndef SPGRAVITY_H
#define SPGRAVITY_H

#include "SpaceDSL_Global.h"
#include "SpMath.h"
#include "SpConst.h"
#include "SpUtils.h"
#include <fstream>
#include <Eigen/Core>

using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: The class of Gravity Model of the Central Body
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API GravityModel
    {
    public:
        enum GravModelType
        {
            E_NotDefinedGravModel   = 0,
            E_EGM08Model            = 1,
        };

        explicit GravityModel();
        explicit GravityModel(GravModelType modelType);
        virtual ~GravityModel();

    public:
        void            SetModelType(GravModelType modelType);

        GravModelType   GetModelType();

    public:
        //********************************************************************
        /// Computes the Acceleration Due to the Harmonic Gravity Field of the Central Body
        /// @author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	pos             Satellite position vector in the inertial system
        /// @Param	ECItoBFCMtx     ECI Transformation Matrix to Body-Fixed System
        /// @Param	n_max           Maximum degree
        /// @Param	m_max           Maximum order (m_max<=n_max; m_max=0 for zonals, only)
        /// @Return Acceleration
        //********************************************************************
        Vector3d        AccelHarmonicGravity (const Vector3d& pos, const Matrix3d& J2000toECFMtx, int n_max, int m_max );
    protected:

        void            DataPreprocess();

    protected:

        GravModelType   m_GravModelType;
        double          m_GM;
        double          m_Radius;
        int             m_Nmax;
        int             m_Mmax;
        MatrixXd        m_MatrixCS;

    };
    /*************************************************
     * struct type: Third Body Gravity Sign
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    struct SPACEDSL_API ThirdBodyGravitySign
    {
        bool                bIsUseMercuryGrav = false;
        bool                bIsUseVenusGrav   = false;
        bool                bIsUseEarthGrav   = false;
        bool                bIsUseMarsGrav    = false;
        bool                bIsUseJupiterGrav = false;
        bool                bIsUseSaturnGrav  = false;
        bool                bIsUseUranusGrav  = false;
        bool                bIsUseNeptuneGrav = false;
        bool                bIsUsePlutoGrav   = false;
        bool                bIsUseMoonGrav    = false;
        bool                bIsUseSunGrav     = false;
    };

}
#endif //SPGRAVITY_H
