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
* Date:2018-12-26
* Description:
*   SpAttitude.cpp
*
*   Purpose:
*
*         Calculation of All Kinds of Attitude.
*
*
*   Last modified:
*
*   2018-12-26  Niu Zhiyong (1st edition)
*
*************************************************************************/
#include "SpaceDSL/SpAttitude.h"
#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


namespace SpaceDSL {

    /*************************************************
     * Class type: The class of SpaceDSL YPR Angles
     * Author: Niu ZhiYong
     * Date:2018-12-26
     * Description:
     *  Angle Unit is [rad]
     *  Rotation Order String Can be "YPR" "PYR" ...
     *  This Class is Thread Safe!
    **************************************************/
    YPREulerAngle::YPREulerAngle()
    {
        m_Yaw = 0;
        m_Pitch = 0;
        m_Roll = 0;
    }

    YPREulerAngle::YPREulerAngle(const double yaw, const double pitch, const double roll)
    {
        m_Yaw = yaw;
        m_Pitch = pitch;
        m_Roll = roll;
    }

    YPREulerAngle::YPREulerAngle(const Matrix3d &rotationMatrix, const string &rotationOrder)
    {
        if(rotationOrder[0] == 'Y' && rotationOrder[1] == 'P' && rotationOrder[2] == 'R')
        {
            Vector3d angleVec = rotationMatrix.eulerAngles(2,1,0);
            m_Yaw = angleVec(0);
            m_Pitch = angleVec(1);
            m_Roll = angleVec(2);
        }
        else if(rotationOrder[0] == 'R' && rotationOrder[1] == 'P' && rotationOrder[2] == 'Y')
        {
            Vector3d angleVec = rotationMatrix.eulerAngles(0,1,2);
            m_Roll = angleVec(0);
            m_Pitch = angleVec(1);
            m_Yaw = angleVec(2);
        }
        else
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "YPREulerAngle::GetRotationMatrix Unsupport Rotation Order! ");
        }
    }

    YPREulerAngle::~YPREulerAngle()
    {

    }

    const Matrix3d YPREulerAngle::GetRotationMatrix(const string &rotationOrder) const
    {
        if(rotationOrder[0] == 'Y' && rotationOrder[1] == 'P' && rotationOrder[2] == 'R')
        {
            Matrix3d rotateMat;
            rotateMat = AngleAxisd(m_Yaw, Vector3d::UnitZ())
                        * AngleAxisd(m_Pitch, Vector3d::UnitY())
                        * AngleAxisd(m_Roll, Vector3d::UnitX());
            return rotateMat;
        }
        else if(rotationOrder[0] == 'R' && rotationOrder[1] == 'P' && rotationOrder[2] == 'Y')
        {
            Matrix3d rotateMat;
            rotateMat = AngleAxisd(m_Roll, Vector3d::UnitZ())
                        * AngleAxisd(m_Pitch, Vector3d::UnitY())
                        * AngleAxisd(m_Yaw, Vector3d::UnitX());
            return rotateMat;
        }
        else
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                      "YPREulerAngle::GetRotationMatrix Unsupport Rotation Order! ");
        }
    }

    const Quaterniond YPREulerAngle::GetQuaternion() const
    {
        double cy = cos(m_Yaw * 0.5);
        double sy = sin(m_Yaw * 0.5);
        double cp = cos(m_Pitch * 0.5);
        double sp = sin(m_Pitch * 0.5);
        double cr = cos(m_Roll * 0.5);
        double sr = sin(m_Roll * 0.5);

        Quaterniond q;
        q.w() = cy * cp * cr + sy * sp * sr;
        q.x() = cy * cp * sr - sy * sp * cr;
        q.y() = sy * cp * sr + cy * sp * cr;
        q.z() = sy * cp * cr - cy * sp * sr;
        return q;
    }

}
