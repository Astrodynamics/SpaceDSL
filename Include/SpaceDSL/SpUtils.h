/************************************************************************
* Copyright (C) 2017 Niu ZhiYong
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
* Date:2017-03-20
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
*   2017-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/
#ifndef SPUTILS_H
#define SPUTILS_H

#include <exception>
#include <string>

#include "SpaceDSL_Global.h"
#include "spdlog/spdlog.h"

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
	/*************************************************
	Class type: Class of Logging
	Author: Niu ZhiYong
	Date:2018-01-29
	Description:
	Repackaged the spdlog From (https://github.com/gabime/spdlog)
	**************************************************/
    class SPACEDSL_API Log
	{
	public:
		Log();
		~Log();
	public:
		//
		// Return an existing logger or nullptr if a logger with such name doesn't exist.
		// example: spdlog::get("my_logger")->info("hello {}", "world");
		//
		std::shared_ptr<spdlog::logger> get(const std::string& name);


		//
		// Set global formatting
		// example: spdlog::set_pattern("%Y-%m-%d %H:%M:%S.%e %l : %v");
		//
		void set_pattern(const std::string& format_string);
		void set_formatter(spdlog::formatter_ptr f);

		//
		// Set global logging level
		//
		void set_level(spdlog::level::level_enum log_level);

		//
		// Set global flush level
		//
		void flush_on(spdlog::level::level_enum log_level);

		//
		// Set global error handler
		//
		//void set_error_handler(spdlog::log_err_handler);

		//
		// Turn on async mode (off by default) and set the queue size for each async_logger.
		// effective only for loggers created after this call.
		// queue_size: size of queue (must be power of 2):
		//    Each logger will pre-allocate a dedicated queue with queue_size entries upon construction.
		//
		// async_overflow_policy (optional, block_retry by default):
		//    async_overflow_policy::block_retry - if queue is full, block until queue has room for the new log entry.
		//    async_overflow_policy::discard_log_msg - never block and discard any new messages when queue  overflows.
		//
		// worker_warmup_cb (optional):
		//     callback function that will be called in worker thread upon start (can be used to init stuff like thread affinity)
		//
		// worker_teardown_cb (optional):
		//     callback function that will be called in worker thread upon exit
		//
		void set_async_mode(size_t queue_size,
							const spdlog::async_overflow_policy overflow_policy =spdlog:: async_overflow_policy::block_retry,
							const std::function<void()>& worker_warmup_cb = nullptr,
							const std::chrono::milliseconds& flush_interval_ms = std::chrono::milliseconds::zero(),
							const std::function<void()>& worker_teardown_cb = nullptr);

		// Turn off async mode
		void set_sync_mode();


		//
		// Create and register multi/single threaded basic file logger.
		// Basic logger simply writes to given file without any limitations or rotations.
		//
		std::shared_ptr<spdlog::logger> basic_logger_mt(const std::string& logger_name,
														const spdlog::filename_t& filename, bool truncate = false);
		std::shared_ptr<spdlog::logger> basic_logger_st(const std::string& logger_name,
														const spdlog::filename_t& filename, bool truncate = false);

		//
		// Create and register multi/single threaded rotating file logger
		//
		std::shared_ptr<spdlog::logger> rotating_logger_mt(const std::string& logger_name,
														   const spdlog::filename_t& filename, size_t max_file_size, size_t max_files);
		std::shared_ptr<spdlog::logger> rotating_logger_st(const std::string& logger_name,
														   const spdlog::filename_t& filename, size_t max_file_size, size_t max_files);

		//
		// Create file logger which creates new file on the given time (default in  midnight):
		//
		std::shared_ptr<spdlog::logger> daily_logger_mt(const std::string& logger_name,
														const spdlog::filename_t& filename, int hour=0, int minute=0);
		std::shared_ptr<spdlog::logger> daily_logger_st(const std::string& logger_name,
														const spdlog::filename_t& filename, int hour=0, int minute=0);

		//
		// Create and register stdout/stderr loggers
		//
		std::shared_ptr<spdlog::logger> stdout_logger_mt(const std::string& logger_name);
		std::shared_ptr<spdlog::logger> stdout_logger_st(const std::string& logger_name);
		std::shared_ptr<spdlog::logger> stderr_logger_mt(const std::string& logger_name);
		std::shared_ptr<spdlog::logger> stderr_logger_st(const std::string& logger_name);
		//
		// Create and register colored stdout/stderr loggers
		//
		std::shared_ptr<spdlog::logger> stdout_color_mt(const std::string& logger_name);
		std::shared_ptr<spdlog::logger> stdout_color_st(const std::string& logger_name);
		std::shared_ptr<spdlog::logger> stderr_color_mt(const std::string& logger_name);
		std::shared_ptr<spdlog::logger> stderr_color_st(const std::string& logger_name);


		//
		// Create and register a syslog logger
		//
		#ifdef SPDLOG_ENABLE_SYSLOG
		std::shared_ptr<spdlog::logger> syslog_logger(const std::string& logger_name, const std::string& ident = "", int syslog_option = 0, int syslog_facilty = (1<<3));
		#endif

		#if defined(__ANDROID__)
		std::shared_ptr<spdlog::logger> android_logger(const std::string& logger_name, const std::string& tag = "spdlog");
		#endif

		// Create and register a logger with a single sink
		//std::shared_ptr<spdlog::logger> create(const std::string& logger_name, const spdlog::sink_ptr& sink);

		// Create and register a logger with multiple sinks
		//std::shared_ptr<spdlog::logger> create(const std::string& logger_name, spdlog::sinks_init_list sinks);
		//template<class It>
		//std::shared_ptr<spdlog::logger> create(const std::string& logger_name, const It& sinks_begin, const It& sinks_end);


		// Create and register a logger with templated sink type
		// Example:
		// spdlog::create<daily_file_sink_st>("mylog", "dailylog_filename");
		//template <typename Sink, typename... Args>
		//std::shared_ptr<spdlog::logger> create(const std::string& logger_name, Args...);

		// Create and register an async logger with a single sink
		/*
		std::shared_ptr<spdlog::logger> create_async(const std::string& logger_name,
													 const spdlog::sink_ptr& sink, size_t queue_size,
													 const spdlog::async_overflow_policy overflow_policy = spdlog::async_overflow_policy::block_retry,
													 const std::function<void()>& worker_warmup_cb = nullptr,
													 const std::chrono::milliseconds& flush_interval_ms = std::chrono::milliseconds::zero(),
													 const std::function<void()>& worker_teardown_cb = nullptr);
		*/
		// Create and register an async logger with multiple sinks
		/*
		std::shared_ptr<spdlog::logger> create_async(const std::string& logger_name, spdlog::sinks_init_list sinks, size_t queue_size,
													 const spdlog::async_overflow_policy overflow_policy = spdlog::async_overflow_policy::block_retry,
													 const std::function<void()>& worker_warmup_cb = nullptr,
													 const std::chrono::milliseconds& flush_interval_ms = std::chrono::milliseconds::zero(),
													 const std::function<void()>& worker_teardown_cb = nullptr);

		template<class It>
		std::shared_ptr<spdlog::logger> create_async(const std::string& logger_name, const It& sinks_begin, const It& sinks_end, size_t queue_size,
													 const spdlog::async_overflow_policy overflow_policy = spdlog::async_overflow_policy::block_retry,
													 const std::function<void()>& worker_warmup_cb = nullptr,
													 const std::chrono::milliseconds& flush_interval_ms = std::chrono::milliseconds::zero(),
													 const std::function<void()>& worker_teardown_cb = nullptr);
		*/
		// Register the given logger with the given name
		void register_logger(std::shared_ptr<spdlog::logger> logger);

		// Apply a user defined function on all registered loggers
		// Example:
		// spdlog::apply_all([&](std::shared_ptr<spdlog::logger> l) {l->flush();});
		void apply_all(std::function<void(std::shared_ptr<spdlog::logger>)> fun);

		// Drop the reference to the given logger
		void drop(const std::string &name);

		// Drop all references from the registry
		void drop_all();

	};
    /*************************************************
    Class type: Class of Exception Handling
    Author: Niu ZhiYong
    Date:2017-09-08
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
        const char                *m_pFileChar;           /*The File where throw the Exception */
        const char                *m_pFunctionChar;       /*The Function where throw the Exception */
        const char                *m_pReasonChar;         /*The Reason defined by yourself */
        int                 m_nLine;                /*The Line where throw the Exception */
    /// Member Methods
    public:
        /// @brief  Override the what() in <exception>
        /// @input  <void>
        /// @return <void>
        const char * what() const throw ();
    };
}

#endif // SPUTILS_H
