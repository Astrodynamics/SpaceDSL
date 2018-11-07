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
*   SpaceDSL_Global.h
*
*   Purpose:
*
*       Define SPACEDSL_LIBRARY to Compiling Dynamic Library
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/
#ifndef SPACEDSL_GLOBAL_H
#define SPACEDSL_GLOBAL_H

#include <exception>

/********************************************************************/
/// Define Linux and windows export library functions
/// @Author	xiaogongwei
/// @Date	2018-11-06
/********************************************************************/
#ifndef _WIN32
    #define SHARELIBSHARED_EXPORT     __attribute__((visibility("default")))
    #define SHARELIBSHARED_IMPORT     __attribute__((visibility("default")))
    #define SHARELIBSHARED_HIDDEN     __attribute__((visibility("hidden")))
#elif
    #define SHARELIBSHARED_EXPORT     __declspec(dllexport)
    #define SHARELIBSHARED_IMPORT     __declspec(dllimport)
#endif

/********************************************************************/
/// If you build code into executable programs, you need to define EXPORT_SPACEDSL_LIB.
/// @Author	xiaogongwei
/// @Date	2018-11-06
/********************************************************************/
//#define EXPORT_SPACEDSL_LIB

#ifndef EXPORT_SPACEDSL_LIB

#define SPACEDSL_API
#define EXPIMP_TEMPLATE

#else

#ifdef SPACEDSL_SHARED_LIBRARY
#define SPACEDSL_API SHARELIBSHARED_EXPORT
#define EXPIMP_TEMPLATE
#else
    #define SPACEDSL_API SHARELIBSHARED_IMPORT
    #define EXPIMP_TEMPLATE extern
#endif

#endif



#ifdef SPACEDSL_STATIC_LIBRARY
    #define SPACEDSL_API
    #define EXPIMP_TEMPLATE
#endif

namespace std {
    EXPIMP_TEMPLATE class SPACEDSL_API exception;
}

#pragma warning(disable: 4251)

#endif // SPACEDSL_GLOBAL_H
