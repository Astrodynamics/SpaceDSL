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
*   SpUtils.h
*
*   Purpose:
*
*       The Exception and Log Operations are Defined
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*   2018-11-01  xiaogongwei
*
*************************************************************************/
#ifndef SPUTILS_H
#define SPUTILS_H

#include <iostream>
#include <exception>
#include <string>

#include "SpaceDSL_Global.h"

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
    Class type: Class of Exception Handling
    Author: Niu ZhiYong
    Date:2018-09-08
    Description:
    Defined self Exception Class
    **************************************************/
    class SPACEDSL_API SPException : public exception
    {

    public:
        SPException(const char *file, const char *func, int line_num, const char *reason) :
                m_pFileChar(file), m_pFunctionChar(func), m_nLine(line_num), m_pReasonChar(reason) {}
    /// Member variables
    protected:
        const char                  *m_pFileChar;           /*The File where throw the Exception */
        const char                  *m_pFunctionChar;       /*The Function where throw the Exception */
        const char                  *m_pReasonChar;         /*The Reason defined by yourself */
        int                         m_nLine;                /*The Line where throw the Exception */
    /// Member Methods
    public:
        /// @brief  Override the what() in <exception>
        /// @input  <void>
        /// @return <void>
        const char * what() const  noexcept;
    };
}

#endif // SPUTILS_H
