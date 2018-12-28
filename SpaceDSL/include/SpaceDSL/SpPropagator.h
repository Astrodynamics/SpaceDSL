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
*   SpPropagator.h
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

#ifndef SPPROPAGATOR_H
#define SPPROPAGATOR_H

#include "SpaceDSL_Global.h"
#include "SpIntegration.h"

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
	
    /*************************************************
     * Class type: Space Propagator Configuration Class
     * Author: Niu ZhiYong
     * Date:2018-07-30
     * Description:
    **************************************************/
    class SPACEDSL_API Propagator
    {
    public:
        explicit Propagator();
        Propagator(const IntegMethodType integMethodType, const double initialStep, const double accuracy = 0,
                   const double minStep = 0, const double maxStep = 0, const int maxStepAttempts = 0,
                   const bool bStopIfAccuracyIsViolated = true, const bool isUseNormalize = false);
        ~Propagator();

    public:
        inline void     SetIntegMethodType(const IntegMethodType  integMethodType)        { m_IntegMethodType = integMethodType; }

        inline void     SetInitialStep(const double initialStep)                          { m_InitialStep = initialStep; m_AdaptedStep = initialStep; }

        inline void     SetAdaptedStep(const double adaptedStep)                          { m_AdaptedStep = adaptedStep; }

        inline void     SetAccuracy(const double accuracy)                                { m_Accuracy = accuracy; }

        inline void     SetMinStep(const double  minStep)                                 { m_MinStep = minStep; }

        inline void     SetMaxStep(const double  maxStep)                                 { m_MaxStep = maxStep; }

        inline void     SetMaxStepAttempts(const int maxStepAttempts)                     { m_MaxStepAttempts = maxStepAttempts; }

        inline void     SetStopIfAccuracyIsViolated(const bool bStopIfAccuracyIsViolated) { m_bStopIfAccuracyIsViolated = bStopIfAccuracyIsViolated; }

        inline void     SetIsUseNormalize(const bool isUseNormalize)                      { m_bIsUseNormalize = isUseNormalize; }

        inline IntegMethodType GetIntegMethodType() const           { return m_IntegMethodType; }

        inline double GetInitialStep() const                        { return m_InitialStep; }

        inline double GetAdaptedStep() const                        { return m_AdaptedStep; }

        inline double GetAccuracy() const                           { return m_Accuracy; }

        inline double GetMinStep() const                            { return m_MinStep; }

        inline double GetMaxStep() const                            { return m_MaxStep; }

        inline int    GetMaxStepAttempts() const                    { return m_MaxStepAttempts; }

        inline bool   GetStopIfAccuracyIsViolated() const           { return m_bStopIfAccuracyIsViolated; }

        inline bool   GetIsUseNormalize() const                     { return m_bIsUseNormalize; }

    private:
        IntegMethodType         m_IntegMethodType;
        double                  m_InitialStep;
        double                  m_AdaptedStep;
        double                  m_Accuracy;
        double                  m_MinStep;
        double                  m_MaxStep;
        int                     m_MaxStepAttempts;
        bool                    m_bStopIfAccuracyIsViolated;
        bool                    m_bIsUseNormalize;
    };
}

#endif //SPPROPAGATOR_H

