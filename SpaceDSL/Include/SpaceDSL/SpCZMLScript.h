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
*   SpCZMLScript.h
*
*   Purpose:
*
*        Read and Write CZML File by Jsoncpp
*
*
*   Last modified:
*
*   2018-07-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPCZMLSCRIPT_H
#define SPCZMLSCRIPT_H

#include "SpaceDSL_Global.h"

#include <Jsoncpp/json.hpp>

#include <fstream>
#include <string>

using namespace std;
using namespace nlohmann;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    class Mission;
    /*************************************************
     * Class type: The class of SpaceDSL Moderator
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    class SPACEDSL_API CZMLScript
    {
    public:
        explicit CZMLScript();
        ~CZMLScript();

    public:
        /********************************************************************/
        /// Writing CZML Content For Use With Cesium.
        /// @Author	Niu Zhiyong
        /// @Date	2018-07-29
        /// @Input
        /// @Param	filePath
        /// @Return void
        /********************************************************************/
        void        WirteCZML(const string &filePath);

    private:
        json        m_Json;
        Mission   	*m_pMission;




    };


}


#endif //SPCZMLSCRIPT_H
