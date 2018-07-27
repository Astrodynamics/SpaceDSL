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
* Date:2018-07-27
* Description:
*   SpModerator.h
*
*   Purpose:
*
*        Implements operations of the GMAT executive.  
*        It is a singleton class only one instance of this class can be created
*
*
*   Last modified:
*
*   2018-07-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPMODERATOR_H
#define SPMODERATOR_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"
#include "SpTimeSystem.h"
#include "SpJplEph.h"
#include "SpCoordSystem.h"
#include "SpGravity.h"
#include "SpAtmosphere.h"
#include "SpPerturbation.h"
#include "SpInterpolation.h"
#include "SpRightFunction.h"
#include "SpIntegration.h"
#include "SpOrbitPredict.h"
#include "SpThread.h"
#include "SpMath.h"
#include "SpConst.h"
#include "SpUtils.h"


/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {


    /*************************************************
     * Class type: The class of SpaceDSL Moderator
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    class SPACEDSL_API Moderator
    {
    public:
        static Moderator*   Instance();
        void                Initialize();

    private:
        explicit Moderator();
        ~Moderator();

    private:
        static Moderator    *instance;
        bool                bIsInitialized;
    };


}
#endif //SPMODERATOR_H
