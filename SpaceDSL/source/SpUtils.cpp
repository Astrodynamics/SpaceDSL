#include "SpaceDSL/SpUtils.h"
#include <iostream>

using namespace std;
namespace SpaceDSL{
    /*************************************************
    Class type: Class of Logging
    Author: Niu ZhiYong
    Date:2018-01-29
    Description:
    Repackaged the spdlog From (https://github.com/gabime/spdlog)
    **************************************************/
    Log::Log()
    {

    }

    Log::~Log()
    {

    }

    std::shared_ptr<spdlog::logger> Log::get(const string &name)
    {
        return spdlog::get(name);
    }

    void Log::set_pattern(const string &format_string)
    {
        spdlog::set_pattern(format_string);
    }

    void Log::set_formatter(std::unique_ptr<spdlog::formatter> &formatter)
    {
        spdlog::set_formatter(std::move(formatter));
    }

    void Log::set_level(spdlog::level::level_enum log_level)
    {
        spdlog::set_level(log_level);
    }

    void Log::flush_on(spdlog::level::level_enum log_level)
    {
        spdlog::flush_on(log_level);
    }

    void Log::set_error_handler(spdlog::log_err_handler handler)
    {
        spdlog::set_error_handler(handler);
    }

    std::shared_ptr<spdlog::logger> Log::basic_logger_mt(const string &logger_name,
                                                         const spdlog::filename_t &filename, bool truncate)
    {
        return spdlog::basic_logger_mt(logger_name, filename, truncate);
    }

    std::shared_ptr<spdlog::logger> Log::basic_logger_st(const string &logger_name,
                                                         const spdlog::filename_t &filename, bool truncate)
    {
        return spdlog::basic_logger_st(logger_name, filename, truncate);
    }

    std::shared_ptr<spdlog::logger> Log::rotating_logger_mt(const string &logger_name, const spdlog::filename_t &filename,
                                                            size_t max_file_size, size_t max_files)
    {
        return spdlog::rotating_logger_mt(logger_name, filename, max_file_size, max_files);
    }

    std::shared_ptr<spdlog::logger> Log::rotating_logger_st(const string &logger_name, const spdlog::filename_t &filename,
                                                            size_t max_file_size, size_t max_files)
    {
        return spdlog::rotating_logger_st(logger_name, filename, max_file_size, max_files);
    }

    std::shared_ptr<spdlog::logger> Log::daily_logger_mt(const string &logger_name, const spdlog::filename_t &filename,
                                                         int hour, int minute)
    {
        return spdlog::daily_logger_mt(logger_name,filename, hour, minute);
    }

    std::shared_ptr<spdlog::logger> Log::daily_logger_st(const string &logger_name, const spdlog::filename_t &filename,
                                                         int hour, int minute)
    {
        return spdlog::daily_logger_st(logger_name,filename, hour, minute);
    }

    std::shared_ptr<spdlog::logger> Log::stdout_logger_mt(const string &logger_name)
    {
        return spdlog::stdout_logger_mt(logger_name);
    }

    std::shared_ptr<spdlog::logger> Log::stdout_logger_st(const std::string& logger_name)
    {
        return spdlog::stdout_logger_st(logger_name);
    }

    std::shared_ptr<spdlog::logger> Log::stderr_logger_mt(const std::string& logger_name)
    {
        return spdlog::stderr_logger_mt(logger_name);
    }

    std::shared_ptr<spdlog::logger> Log::stderr_logger_st(const std::string& logger_name)
    {
        return spdlog::stderr_logger_st(logger_name);
    }

    std::shared_ptr<spdlog::logger> Log::stdout_color_mt(const std::string& logger_name)
    {
        return spdlog::stdout_color_mt(logger_name);
    }

    std::shared_ptr<spdlog::logger> Log::stdout_color_st(const std::string& logger_name)
    {
        return spdlog::stdout_color_st(logger_name);
    }

    std::shared_ptr<spdlog::logger> Log::stderr_color_mt(const std::string& logger_name)
    {
        return spdlog::stderr_color_mt(logger_name);
    }

    std::shared_ptr<spdlog::logger> Log::stderr_color_st(const std::string& logger_name)
    {
        return spdlog::stderr_color_st(logger_name);
    }

    void Log::register_logger(std::shared_ptr<spdlog::logger> logger)
    {
        spdlog::register_logger(logger);
    }

    void Log::apply_all(std::function<void(std::shared_ptr<spdlog::logger>)> fun)
    {
        spdlog::apply_all(fun);
    }

    void Log::drop(const std::string &name)
    {
        spdlog::drop(name);
    }

    void Log::drop_all()
    {
        spdlog::drop_all();
    }

    /*************************************************
    Class type: Class of Exception Handling
    Author: Niu ZhiYong xiaogongwei
    Date:2018-09-08
    Description:
    Defined self Exception Class
    **************************************************/
    const char *SPException::what() const noexcept
    {
        Log *pLog = new Log();
        pLog->set_pattern("*** [%Y-%m-%d %H:%M:%S %z] [thread %t] %v ***");
        auto exception_logger = pLog->rotating_logger_mt("Exception_Logger", "Exception.log", 1048576 * 2, 2);
        char buf[10];
#if defined(_WIN32)
// Windows OS
        sprintf_s(buf, "%d", m_nLine);
#else
// Linux OS or MAC OS
        snprintf(buf, m_nLine, "%d");
#endif
        string sLine = buf;
        string sFile = m_pFileChar;
        string sFunc = m_pFunctionChar;
        string sReason = m_pReasonChar;
        string Exceptstr = "The File is:" + (sFile) + " \n"
                + "The Function Name is:" + (sFunc) + " \n"
                + "The Line is:" + sLine + " \n"
                + "The Reason is: " + (sReason) + " \n";
        //cout << Exceptstr << endl;
        static const char *Exceptchar ;
        Exceptchar = Exceptstr.c_str();
        cout << "An Exception has been violated here! \n" << endl;
        cout << Exceptchar << endl;
        exception_logger->info("{}",Exceptstr );
        pLog->drop_all();
        delete pLog;
        return "The Program has Exited!";
    }
}


