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
*   SpThread.cpp
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

#include "SpaceDSL/SpThread.h"
#include "SpaceDSL/SpUtils.h"

#include <functional>
#include <mutex>
#include <thread>

namespace SpaceDSL {
    /*************************************************
     * Class type: Thread Class of SpaceDSL
     * Author: Niu ZhiYong
     * Date:2018-05-20
     * Description:
     *  Reencapsulates C++11 STL:: thread Class
    **************************************************/
    atomic<size_t> SpThread::OriginalThreadID = 10000;

    SpThread::SpThread()
    {
        m_ThreadID = OriginalThreadID++;
        m_CreateTime = clock();

        #ifdef _WIN32
            m_Handle = NULL;
            m_SuspendCount = 0;
        #else
            m_Thread_t = 0;
            int policy = 0;
            pthread_attr_getschedpolicy(&m_Thread_attr, &policy);
            m_MaxPriority = sched_get_priority_max(policy);
            m_MinPriority = sched_get_priority_min(policy);
        #endif

    }

    SpThread::~SpThread()
    {
        #ifdef WIN32
            if (NULL != m_Handle)
            {
                CloseHandle(m_Handle);
            }
            m_Handle = NULL;
        #else
            m_Thread_t = 0;
            pthread_attr_destroy(m_Thread_attr);
        #endif
    }

    void SpThread::SetPriority(SpThread::Priority priority)
    {
        #ifdef _WIN32
            switch (priority)
            {
            case NormalPriority:
                SetThreadPriority(m_Handle, THREAD_PRIORITY_NORMAL);
                break;
            case HighPriority:
                SetThreadPriority(m_Handle, THREAD_PRIORITY_ABOVE_NORMAL);
                break;
            case HighestPriority:
                SetThreadPriority(m_Handle, THREAD_PRIORITY_HIGHEST);
                break;
            case LowPriority:
                SetThreadPriority(m_Handle, THREAD_PRIORITY_BELOW_NORMAL);
                break;
            case LowestPriority:
                SetThreadPriority(m_Handle, THREAD_PRIORITY_LOWEST);
                break;
            case TimeCriticalPriority:
                SetThreadPriority(m_Handle, THREAD_PRIORITY_TIME_CRITICAL);
                break;
            case IdlePriority:
                SetThreadPriority(m_Handle, THREAD_PRIORITY_IDLE);
                break;
            default:
                break;
            }
        #else
            struct sched_param param;
            if (m_MaxPriority == 0)
                param.__sched_priority = 0;
            switch (priority)
            {
            case NormalPriority:
                param.__sched_priority = 50;
                break;
            case HighPriority:
                param.__sched_priority = 70;
                break;
            case HighestPriority:
                param.__sched_priority = 99;
                break;
            case LowPriority:
                param.__sched_priority = 30;
                break;
            case LowestPriority:
                param.__sched_priority = 1;
                break;
            default:
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Priority won't Support it!");
                break;
            }
            pthread_attr_setschedparam(&m_Thread_attr, &param);

        #endif
    }

    SpThread::Priority SpThread::Getpriority() const
    {
        #ifdef _WIN32
            switch (GetThreadPriority(m_Handle))
            {
            case THREAD_PRIORITY_NORMAL:
                return NormalPriority;
                break;
            case THREAD_PRIORITY_ABOVE_NORMAL:
                return HighPriority;
                break;
            case THREAD_PRIORITY_HIGHEST:
                return HighestPriority;
                break;
            case THREAD_PRIORITY_BELOW_NORMAL:
                return LowPriority;
                break;
            case THREAD_PRIORITY_LOWEST:
                return LowestPriority;
                break;
            case THREAD_PRIORITY_TIME_CRITICAL:
                return TimeCriticalPriority;
                break;
            case THREAD_PRIORITY_IDLE:
                return IdlePriority;
                break;
            default:
                break;
            }
        #else
            struct sched_param param;
            pthread_attr_getschedparam(&m_Thread_attr, &param);
            if (param.__sched_priority == 0
                || (param.__sched_priority >= 40 && param.__sched_priority <= 59))
                return NormalPriority;
            else if (param.__sched_priority >= 1 && param.__sched_priority <= 19)
                return LowestPriority;
            else if (param.__sched_priority >= 20 && param.__sched_priority <= 39)
                return LowPriority;
            else if (param.__sched_priority >= 60 && param.__sched_priority <= 79)
                return HighPriority;
            else if (param.__sched_priority >= 80 && param.__sched_priority <= 99)
                return HighestPriority;
            else
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Thread Sched_param Error!");
        #endif

        return m_Priority;
    }

    size_t SpThread::GetThreadID() const
    {
        return m_ThreadID;
    }

    clock_t SpThread::GetCreateTime() const
    {
        return m_CreateTime;
    }

    void SpThread::Start()
    {
       #ifdef WIN32
           m_Handle = (HANDLE)_beginthreadex(NULL, 0, ThreadFunc, this, 0, NULL);
           if (m_Handle == NULL)
           {
               throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Start Thread Error!");
           }
       #else
           if (pthread_create(&m_Thread_t, NULL, ThreadFunc, this) != 0)
           {
               throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Start Thread Error!");
           }
       #endif

    }

#ifdef _WIN32
    unsigned __stdcall SpThread::ThreadFunc(void* arg)
#else
    void* SpThread::ThreadFunc(void* arg)
#endif
    {
        SpThread *pThis = (SpThread*)arg;
        pThis->Run();
        return NULL;
    }

    void SpThread::Suspend()
    {
        #ifdef _WIN32
            m_SuspendCount = int(SuspendThread(m_Handle));
            if(m_SuspendCount == -1)
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Suspend Thread Error!");
        #else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Suspend won't Support it!");
        #endif
    }

    void SpThread::Resume()
    {
        #ifdef _WIN32
            m_SuspendCount = int(ResumeThread(m_Handle));
            if(m_SuspendCount == -1)
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Suspend Thread Error!");
        #else
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Suspend won't Support it!");
        #endif
    }

    void SpThread::Wait()
    {
        #ifdef _WIN32
            WaitForSingleObject(m_Handle, INFINITE);
            if (NULL != m_Handle)
            {
                CloseHandle(m_Handle);
            }
            m_Handle = NULL;
        #else
            pthread_join(m_Thread_t, NULL);
            m_Thread_t = 0;
        #endif // WIN32
    }

    bool SpThread::isRunning() const
    {
        #ifdef _WIN32
            if (m_Handle == NULL)// Not Start is Not Running
                return false;
            DWORD exitCode;
            GetExitCodeThread(m_Handle, &exitCode);
            if (exitCode == STILL_ACTIVE)
                return true;
            else
                return false;
        #else

        #endif

    }

    bool SpThread::isFinished() const
    {
        #ifdef _WIN32
            if (m_Handle == NULL)// Not Start is Finished
                return true;
            DWORD exitCode;
            GetExitCodeThread(m_Handle, &exitCode);
            if (exitCode == STILL_ACTIVE)
                return false;
            else
                return true;
        #else

        #endif
    }

    size_t GetHardwareConcurrency()
    {
       return thread::hardware_concurrency();
    }

}
