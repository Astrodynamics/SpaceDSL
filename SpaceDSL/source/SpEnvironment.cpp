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
* Date:2018-07-30
* Description:
*   SpEnvironment.cpp
*
*   Purpose:
*
*       Space Environment Configuration Class
*
*
*   Last modified:
*
*   2018-07-30  Niu Zhiyong (1st edition)
*
*************************************************************************/


#include "SpaceDSL/SpEnvironment.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"

#include <type_traits>

template <class T>

int getArrayLen(T& array)

{//使用模板定义一 个函数getArrayLen,该函数将返回数组array的长度

return (sizeof(array) / sizeof(array[0]));

}

using namespace std;

namespace SpaceDSL {

    /*************************************************
     * Class type: Space Environment Configuration Class
     * Author: Niu ZhiYong
     * Date:2018-07-30
     * Description:
    **************************************************/
    Environment::Environment()
    {
        m_CenterStarType = E_Earth;
        m_GravModelType = GravityModel::GravModelType::E_NotDefinedGravModel;
        m_MaxDegree = 0;
        m_MaxOrder = 0;
        m_ThirdBodySign = ThirdBodyGravitySign();
        m_GeodeticCoordType = GeodeticCoordSystem::GeodeticCoordType::E_WGS84System;
        m_AtmModelType =  AtmosphereModel::AtmosphereModelType::E_NotDefinedAtmosphereModel;
        m_F107A = 150;
        m_F107 = 150;
        m_Ap.resize(7);
        m_Ap.fill(0.0);
        m_bIsUseDrag = false;
        m_bIsUseSRP = false;
    }

    Environment::Environment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType,
                             const int maxDegree, const int maxOrder, const ThirdBodyGravitySign thirdBodyGravSign,
                             const GeodeticCoordSystem::GeodeticCoordType geodeticType,
                             const AtmosphereModel::AtmosphereModelType atmModelType,
                             const double f107A, const double f107, VectorXd ap,
                             const bool isUseDrag, const bool isUseSRP)
    {
        m_CenterStarType = centerStarType;
        m_GravModelType = gravModelType;
        m_MaxDegree = maxDegree;
        m_MaxOrder = maxOrder;
        m_ThirdBodySign = thirdBodyGravSign;
        m_GeodeticCoordType = geodeticType;
        m_AtmModelType = atmModelType;
        m_F107A = f107A;
        m_F107 = f107;
        m_Ap.resize(7);
        this->SetGeomagneticIndex(ap);

        m_bIsUseDrag = isUseDrag;
        m_bIsUseSRP = isUseSRP;
    }

    Environment::~Environment()
    {

    }

    void Environment::SetGeomagneticIndex(VectorXd ap)
    {

        if (  ap.size() != 7 )
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                              "Environment: SetGeomagneticIndex ap size is not 7 ");

        for (int i = 0; i < 7; i++)
        {
            m_Ap(i) = ap(i);
        }
    }

	

}

