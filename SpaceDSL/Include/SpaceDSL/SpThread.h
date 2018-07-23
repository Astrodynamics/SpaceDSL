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
* Date:2018-05-20
* Description:
*   SpThread.h
*
*   Purpose:
*
*       Cross Platform Thread Class
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPTHREAD_H
#define SPTHREAD_H


#include "SpaceDSL_Global.h"

#include <time.h>
#include <atomic>

#ifdef _WIN32
    #include <windows.h>
    #include <process.h>
#else
    #include <pthread.h>
#endif

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {

    /*************************************************
     * Class type: Thread Class of SpaceDSL
     * Author: Niu ZhiYong
     * Date:2018-05-20
     * Description:
    **************************************************/
    class SPACEDSL_API SpThread
    {
    public:
        SpThread();
        virtual ~SpThread();

        enum Priority
        {
            NormalPriority          =0,
            IdlePriority            =1,
            LowestPriority          =2,
            LowPriority             =3,
            HighPriority            =4,
            HighestPriority         =5,
            TimeCriticalPriority    =6,
        };
    public:
        /********************************************************************/
        /// Set Thread Priority
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Param      priority
        /// @Return
        /**********************************************************************/
        void SetPriority(Priority priority);

        /********************************************************************/
        /// Get Thread ID
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return     Priority
        /**********************************************************************/
        Priority Getpriority() const;

        /********************************************************************/
        /// Get Thread ID
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return     Thread ID
        /**********************************************************************/
        size_t      GetThreadID() const;

        /********************************************************************/
        /// Get Thread Create Time
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return     Thread ID
        /**********************************************************************/
        clock_t      GetCreateTime() const;

        /********************************************************************/
        /// Thread Start Function
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return
        /**********************************************************************/
        void         Start();

        /********************************************************************/
        /// Thread Suspend Function
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return
        /**********************************************************************/
        void        Suspend();

        /********************************************************************/
        /// Thread Resume Function
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return
        /**********************************************************************/
        void        Resume();

        /********************************************************************/
        /// Thread Wait Function
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return
        /**********************************************************************/
        void        Wait();

        /********************************************************************/
        /// Get Thread  Running State
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return     bool
        /**********************************************************************/
        bool        isRunning() const;

        /********************************************************************/
        /// Get Thread  Finished State
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return     bool
        /**********************************************************************/
        bool        isFinished() const;

    protected:

        /********************************************************************/
        /// Thread Class Virtual Interface
        /// @Author     Niu Zhiyong
        /// @Date       2018-05-20
        /// @Input/Output
        /// @Return
        /// Note:
        ///     Inherits This Class Must Implement This Interface.
        /**********************************************************************/
        virtual void Run() = 0;

     private:
        #ifdef WIN32
            static unsigned __stdcall ThreadFunc(void* arg);
        #else
            static void* ThreadFunc(void* arg);
        #endif
    ///
    ///Attribute.
    ///
    protected:

        size_t                  m_ThreadID;
        clock_t                 m_CreateTime;
        Priority                m_Priority;

    private:

        static atomic<size_t>   OriginalThreadID;
        #ifdef _WIN32
            HANDLE              m_Handle;
            int                 m_SuspendCount;
        #else
            pthread_attr_t      m_Thread_attr;
            pthread_t           m_Thread_t;
            int                 m_MaxPriority;
            int                 m_MinPriority
        #endif

    };

    /********************************************************************/
    /// Returns The Number of Concurrent Threads Supported
    /// @Author     Niu Zhiyong
    /// @Date       2018-05-20
    /// @Input/Output
    /// @Return
    /**********************************************************************/
    size_t  SPACEDSL_API GetHardwareConcurrency();

}

#endif  //SPTHREAD_H
