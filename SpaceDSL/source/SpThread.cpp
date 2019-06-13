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
*   2018-11-01  xiaogongwei
*
*************************************************************************/

#include "SpaceDSL/SpThread.h"
#include "SpaceDSL/SpUtils.h"

#include <functional>
#include <mutex>
#include <chrono>
#include <iostream>
using namespace std;
using namespace std::chrono;

namespace SpaceDSL {

    /*************************************************
     * Class type: Thread Class of SpaceDSL
     * Author: Niu ZhiYong
     * Date:2018-05-20
     * Description:
     *  Reencapsulates C++11 STL:: thread Class
    **************************************************/
    atomic<size_t> SpThread::ThreadGlobalCounter(0);

    SpThread::SpThread()
    {
        m_ThreadID = (++ThreadGlobalCounter) + 10000; //start from 10001
        m_CreateTime = clock();

        #ifdef _WIN32
            m_Handle = nullptr;
            m_SuspendCount = 0;
        #else
            m_Thread_t = 0;
            int policy = 0;
            pthread_attr_init (&m_Thread_attr);
            pthread_attr_setdetachstate (&m_Thread_attr, PTHREAD_CREATE_DETACHED);
            pthread_attr_getschedpolicy(&m_Thread_attr, &policy);
            m_MaxPriority = sched_get_priority_max(policy);
            m_MinPriority = sched_get_priority_min(policy);
        #endif

    }

    SpThread::~SpThread()
    {
        #ifdef _WIN32
            if (nullptr != m_Handle)
            {
                CloseHandle(m_Handle);
            }
            m_Handle = nullptr;
        #else
            m_Thread_t = 0;
            pthread_attr_destroy(&m_Thread_attr);
        #endif
        --ThreadGlobalCounter;
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
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                                  "SpThread::SetPriority Unsupport Thread Priority");
            }
        #else
            struct sched_param param;
            if (m_MaxPriority == 0)
                param.sched_priority = 0;
            switch (priority)
            {
            case NormalPriority:
                param.sched_priority = 50;
                break;
            case HighPriority:
                param.sched_priority = 70;
                break;
            case HighestPriority:
                param.sched_priority = 99;
                break;
            case LowPriority:
                param.sched_priority = 30;
                break;
            case LowestPriority:
                param.sched_priority = 1;
                break;
            default:
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread::SetPriority Unsupport Thread Priority");
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
            case THREAD_PRIORITY_ABOVE_NORMAL:
                return HighPriority;
            case THREAD_PRIORITY_HIGHEST:
                return HighestPriority;
            case THREAD_PRIORITY_BELOW_NORMAL:
                return LowPriority;
            case THREAD_PRIORITY_LOWEST:
                return LowestPriority;
            case THREAD_PRIORITY_TIME_CRITICAL:
                return TimeCriticalPriority;
            case THREAD_PRIORITY_IDLE:
                return IdlePriority;
            default:
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                                  "SpThread::Getpriority Unsupport Thread Priority");
            }
        #else
            struct sched_param param;
            pthread_attr_getschedparam(&m_Thread_attr, &param);
            if (param.sched_priority == 0
                || (param.sched_priority >= 40 && param.sched_priority <= 59))
                return NormalPriority;
            else if (param.sched_priority >= 1 && param.sched_priority <= 19)
                return LowestPriority;
            else if (param.sched_priority >= 20 && param.sched_priority <= 39)
                return LowPriority;
            else if (param.sched_priority >= 60 && param.sched_priority <= 79)
                return HighPriority;
            else if (param.sched_priority >= 80 && param.sched_priority <= 99)
                return HighestPriority;
            else
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread::Getpriority Unsupport Thread Priority");
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
       #ifdef _WIN32
           m_Handle = (HANDLE)_beginthreadex(nullptr, 0, ThreadFunc, this, 0, nullptr);
           if (m_Handle == nullptr)
           {
               throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Start Thread Error!");
           }
       #else
           if (pthread_create(&m_Thread_t, &m_Thread_attr, ThreadFunc, this) != 0)
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
        SpThread *pThis = static_cast<SpThread*>(arg);
        pThis->Run();
        return NULL;
    }

#ifdef _WIN32
    void SpThread::Suspend()
    {

        m_SuspendCount = int(SuspendThread(m_Handle));
        if(m_SuspendCount == -1)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Suspend Thread Error!");


    }

    void SpThread::Resume()
    {

        m_SuspendCount = int(ResumeThread(m_Handle));
        if(m_SuspendCount == -1)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: Suspend Thread Error!");

    }
#endif

    void SpThread::Wait()
    {
        #ifdef _WIN32
            WaitForSingleObject(m_Handle, INFINITE);
            if (nullptr != m_Handle)
            {
                CloseHandle(m_Handle);
            }
            m_Handle = nullptr;
        #else
            pthread_join(m_Thread_t, nullptr);
            m_Thread_t = 0;
        #endif
    }

    bool SpThread::isRunning() const
    {
        #ifdef _WIN32
            if (m_Handle == nullptr)// Not Start is Not Running
                return false;
            DWORD exitCode;
            GetExitCodeThread(m_Handle, &exitCode);
            if (exitCode == STILL_ACTIVE)
                return true;
            else
                return false;
        #else
            int exitCode;
            exitCode = pthread_kill(m_Thread_t, 0);
            if(exitCode == ESRCH)
                return false;
            else if(exitCode == EINVAL)
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: isRunning Pthread Illegal Signal!");
            else
                return true;
        #endif

    }

    bool SpThread::isFinished() const
    {
        #ifdef _WIN32
            if (m_Handle == nullptr)// Not Start is Finished
                return true;
            DWORD exitCode;
            GetExitCodeThread(m_Handle, &exitCode);
            if (exitCode == STILL_ACTIVE)
                return false;
            else
                return true;
        #else
            int exitCode = -1;
            exitCode = pthread_kill(m_Thread_t, 0);
            if(exitCode == ESRCH)
                return true;
            else if(exitCode == EINVAL)
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThread: isFinished Pthread Illegal Signal!");
            else
                return false;
        #endif
    }

    size_t GetHardwareConcurrency()
    {
        return thread::hardware_concurrency();
    }

    /*************************************************
     * Class type: Thread Pool Class of SpaceDSL
     * Author: Niu ZhiYong
     * Date:2018-05-20
     * Description:
    **************************************************/
    SpThreadPool::SpThreadPool()
    {
        m_bIsStarted = false;
        m_MaxThreadCount = 0;
        m_ActiveThreadCount = 0;
        m_pMonitor = new MonitorThread();
        m_pMonitor->Initializer(&m_bIsStarted, &m_ThreadPool, &m_ThreadBuffer, &m_ActiveThreadCount, &m_CheckLock);
    }

    SpThreadPool::~SpThreadPool()
    {
        m_ThreadPool.clear();
        m_ThreadBuffer.clear();  
        m_pMonitor->Stop();
        m_pMonitor->Wait();
        delete m_pMonitor;
    }

    void SpThreadPool::Start(SpThread *thread)
    {
        if (m_MaxThreadCount <= 0)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThreadPool: m_MaxThreadCount <= 0");

        if (m_pMonitor->isFinished())
            m_pMonitor->Start();

        lock_guard<mutex>guard(m_CheckLock);
        if ( m_ActiveThreadCount < m_MaxThreadCount)
        {
            m_ThreadPool.push_back(thread);
            ++m_ActiveThreadCount;
            if (m_ActiveThreadCount != int(m_ThreadPool.size()))
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "SpThreadPool: m_ActiveThreadCount != m_ThreadPool.size()");
            thread->Start();
        }
        else
        {
            m_ThreadBuffer.push_back(thread);
        }

        m_bIsStarted = true;
    }

    void SpThreadPool::Clear()
    {
        lock_guard<mutex>guard(m_CheckLock);
        m_ThreadBuffer.clear();
    }

    bool SpThreadPool::WaitForDone(int msecs)
    {
        auto startT = steady_clock::now();
        int  dT = 0;
        do
        {
            if (msecs != -1)
            {
                auto endT = steady_clock::now();
                auto deltaT = duration_cast<milliseconds> (endT - startT);
                dT = int(deltaT.count());
                if (dT > msecs)
                {
                    return false;
                }
            }

            if (m_ActiveThreadCount == 0)
            {
                break;
            }
        }while(true);

        m_bIsStarted = false;
        return true;
    }

    void SpThreadPool::SetMaxThreadCount(int maxCount)
    {
        m_MaxThreadCount = maxCount;
    }

    int SpThreadPool::GetMaxThreadCount() const
    {
        return m_MaxThreadCount;
    }

    int SpThreadPool::GetActiveThreadCount() const
    {
        return m_ActiveThreadCount;
    }
    /*************************************************
     * Class type: Thread Pool Monitor Thread
     * Author: Niu ZhiYong
     * Date:2018-05-20
     * Description:
    **************************************************/
    MonitorThread::MonitorThread()
    {
        m_pIsStarted = nullptr;
        m_pActiveThreadCount = nullptr;
        m_pThreadPool = nullptr;
        m_pThreadBuffer = nullptr;
        m_pCheckLock = nullptr;
        m_bIsInitialized = false;
        m_MonitorStart = true;
    }

    MonitorThread::~MonitorThread()
    {

    }

    void MonitorThread::Initializer(bool *pIsStarted, vector<SpThread *> *pPool,
                                    deque<SpThread *> *pBuffer, atomic<int> *pActiveThreadCount,
                                    mutex *pMutex)
    {
        m_pIsStarted = pIsStarted;
        m_pActiveThreadCount = pActiveThreadCount;
        m_pThreadPool = pPool;
        m_pThreadBuffer = pBuffer;
        m_pCheckLock = pMutex;
        m_bIsInitialized = true;
    }

    void MonitorThread::Stop()
    {
        m_MonitorStart = false;
    }

    void MonitorThread::Run()
    {
        if (m_bIsInitialized == false)
        {
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "MonitorThread Uninitialized!");
        }

        while (m_MonitorStart == true)
        {
            while ( *m_pIsStarted == false )
            {
                #ifdef _WIN32
                    Sleep(1);
                #else
                    usleep(1000);
                #endif
                if(*m_pIsStarted == true || m_MonitorStart == false)
                    break;
            }

            vector<SpThread *>::iterator pool_iter;
            while ( *m_pIsStarted == true )
            {
                lock_guard<mutex>guard(*m_pCheckLock);

                auto poolSize = (*m_pThreadPool).size();
                auto buffSize = (*m_pThreadBuffer).size();
                if ( poolSize == 0 && buffSize == 0)
                    break;

                for(pool_iter = m_pThreadPool->begin(); pool_iter != m_pThreadPool->end();)
                {

                    if(m_pThreadPool->size() == 0)
                        break;

                    if ((*pool_iter) == nullptr || (*pool_iter)->isFinished())
                    {
                        #ifdef _WIN32
                            delete (*pool_iter);
                        #endif
                        pool_iter = (*m_pThreadPool).erase(pool_iter);

                        if(m_pThreadBuffer->size() > 0)
                        {
                            (*m_pThreadPool).push_back(m_pThreadBuffer->front());
                            (*m_pThreadBuffer).front()->Start();
                            (*m_pThreadBuffer).pop_front();
                        }
                        else
                        {
                            --(*m_pActiveThreadCount);
                        }

                    }
                    else
                        ++pool_iter;
                }

                #ifdef _WIN32
                    Sleep(50);
                #else
                    usleep(50000);
                #endif
            }

        }

    }


}
