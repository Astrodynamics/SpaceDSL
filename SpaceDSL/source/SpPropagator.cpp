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
*   SpPropagator.cpp
*
*   Purpose:
*
*       Space Propagator Configuration Class
*
*
*   Last modified:
*
*   2018-07-30  Niu Zhiyong (1st edition)
*
*************************************************************************/


#include "SpaceDSL/SpPropagator.h"



namespace SpaceDSL {

    /*************************************************
     * Class type: The Base Class of All Kinds of Space Vehicle
     * Author: Niu ZhiYong
     * Date:2018-07-30
     * Description:
    **************************************************/
    Propagator::Propagator()
    {
        m_IntegMethodType = IntegMethodType::E_NotDefindIntegMethodType;
        m_InitialStep = 0;
        m_Accuracy = 0;
        m_MinStep = 0;
        m_MaxStep = 0;
        m_MaxStepAttempts = 0;
        m_bStopIfAccuracyIsViolated = true;
    }

    Propagator::Propagator(const IntegMethodType integMethodType, const double initialStep, const double accuracy,
                           const double minStep, const double maxStep, const double maxStepAttempts,
                           const bool bStopIfAccuracyIsViolated, const bool isUseNormalize)
    {
        m_IntegMethodType = integMethodType;
        m_InitialStep = initialStep;
        m_Accuracy = accuracy;
        m_MinStep = minStep;
        m_MaxStep = maxStep;
        m_MaxStepAttempts = maxStepAttempts;
        m_bStopIfAccuracyIsViolated = bStopIfAccuracyIsViolated;
        m_bIsUseNormalize = isUseNormalize;
    }

    Propagator::~Propagator()
    {

    }
}

