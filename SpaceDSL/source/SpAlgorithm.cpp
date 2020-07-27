/************************************************************************
* Copyright (C) 2020 Niu ZhiYong
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
* Date:2020-04-14
* Description:
*   SpAlgorithm.cpp
*
*   Purpose:
*
*       Optimization Algorithm Class
*
*
*   Last modified:
*
*   2020-04-14  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include "SpaceDSL/SpAlgorithm.h"


namespace SpaceDSL {

    /*************************************************
     * Class type:  Algorithm Result Information Base Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    ResultInfoBase::ResultInfoBase()
    {

    }

    ResultInfoBase::~ResultInfoBase()
    {

    }

    /*************************************************
     * Class type:  Algorithm Config Base Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    AlgorithmConfigBase::AlgorithmConfigBase()
    {

    }

    AlgorithmConfigBase::~AlgorithmConfigBase()
    {

    }

    /*************************************************
     * Class type:  Algorithm Base Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    AlgorithmBase::AlgorithmBase()
    {

    }

    AlgorithmBase::~AlgorithmBase()
    {

    }

    /*************************************************
     * Class type:  Sequential-Quadratic-Programming Derivative-Free Optimizatio Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    SQPDF::SQPDF()
    {

    }

    SQPDF::~SQPDF()
    {

    }
    SQPDF::SQPDFResultInfo::SQPDFResultInfo()
    {

    }

    SQPDF::SQPDFResultInfo::~SQPDFResultInfo()
    {

    }

    SQPDF::SQPDFConfig::SQPDFConfig()
    {

    }

    SQPDF::SQPDFConfig::~SQPDFConfig()
    {

    }

    ResultInfoBase SQPDF::StartOptimize(AlgorithmConfigBase &config, double (*pObjectFunc)(VectorXd), VectorXd &(*pConstraintFunc)(VectorXd), VectorXd &x0, VectorXd &lowBound, VectorXd &upBound, int eConstraintNum, int ineConstraintNum)
    {
        m_SQPDFConfig = *static_cast<SQPDFConfig *>(&config);
        SQPDFResultInfo result;
        return result;
    }

    void SQPDF::AlgoPreliminary()
    {

    }

    void SQPDF::AlgoOptions()
    {
        m_SQPDFConfig;
    }








}


