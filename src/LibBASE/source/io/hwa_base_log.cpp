#include <algorithm>
#include "hwa_base_fileconv.h"
#include "hwa_base_log.h"
#include "spdlog.h"
#include "sinks/rotating_file_sink.h"
#include "sinks/basic_file_sink.h"
#include "sinks/daily_file_sink.h"
#include "sinks/stdout_color_sinks.h"

#define DEF_FORMAT_LOG "[%Y-%m-%d %H:%M:%S] [%L] [%-4#] <%-40!> : %v"
using namespace spdlog;

namespace hwa_base
{
    base_rtlog::base_rtlog()
    {
    }
    base_rtlog::base_rtlog(const std::string &log_type, const level::level_enum &log_level, const std::string &log_name)
    {
        _type = log_type;
        transform(_type.begin(), _type.end(), _type.begin(), ::toupper);
        _name = log_name;
        _level = log_level;
        _pattern = std::string(DEF_FORMAT_LOG);

        // Creat and std::set the log file : clk.log
        spdlog::set_level(_level);
        spdlog::set_pattern(_pattern);
        spdlog::flush_on(_level);

        std::string str_log_name = _name;

        if (_type.find("CONSOLE") != std::string::npos)
        {
            _spdlog = spdlog::stdout_color_mt(_name);
        }
        else if (_type.find("ROTATING") != std::string::npos)
        {
            _spdlog = spdlog::rotating_logger_mt(_name, str_log_name, 1024 * 1024, 10);
        }
        else if (_type.find("BASIC") != std::string::npos)
        {
            if (ACCESS(str_log_name.c_str(), 0) != -1)
            {
                int re = (remove(str_log_name.c_str()));
                spdlog::info(re);
            }
            _spdlog = spdlog::basic_logger_mt(_name, str_log_name);
        }
        else if (_type.find("DAILY") != std::string::npos)
        {
            if (ACCESS(str_log_name.c_str(), 0) != -1)
            {
                int re = (remove(str_log_name.c_str()));
                spdlog::info(re);
            }
            _spdlog = spdlog::daily_logger_mt(_name, str_log_name, 0, 0);
        }
        else
        {
            _spdlog = spdlog::stdout_color_mt(_name);
        }

        _spdlog->set_level(_level);
        _spdlog->flush_on(_level);
        _spdlog->set_pattern(_pattern);
    }

    base_rtlog::~base_rtlog()
    {
        spdlog::drop(_name);
    }

    std::shared_ptr<logger> base_rtlog::spdlog()
    {
        return _spdlog;
    }

    void base_rtlog::reset_pattern(const std::string &pattern)
    {
        spdlog::set_pattern(_pattern);
        _spdlog->set_pattern(_pattern);
    }

    void base_rtlog::reset_level(const level::level_enum &level)
    {
        spdlog::flush_on(level);
        _spdlog->flush_on(level);
        spdlog::set_level(level);
        _spdlog->set_level(level);
    }
    void base_rtlog::set_log(const std::string &log_type, const level::level_enum &log_level, const std::string &log_name)
    {
        _type = log_type;
        transform(_type.begin(), _type.end(), _type.begin(), ::toupper);
        _name = log_name;
        _level = log_level;
        _pattern = std::string(DEF_FORMAT_LOG);

        // Creat and std::set the log file : clk.log
        spdlog::set_level(_level);
        spdlog::set_pattern(_pattern);
        spdlog::flush_on(_level);

        if (_type.find("CONSOLE") != std::string::npos)
        {
            _spdlog = spdlog::stdout_color_mt(_name);
        }
        else if (_type.find("ROTATING") != std::string::npos)
        {
            _spdlog = spdlog::rotating_logger_mt(_name, _name + ".spd_log", 1024 * 1024, 10);
        }
        else if (_type.find("BASIC") != std::string::npos)
        {
            _spdlog = spdlog::basic_logger_mt(_name, _name + ".spd_log");
        }
        else if (_type.find("DAILY") != std::string::npos)
        {
            _spdlog = spdlog::daily_logger_mt(_name, _name + ".spd_log", 0, 0);
        }
        else
        {
            _spdlog = spdlog::stdout_color_mt(_name);
        }

        _spdlog->set_level(_level);
        _spdlog->flush_on(_level);
        _spdlog->set_pattern(_pattern);
    }

    void base_rtlog::set_name(const std::string& name)
    {
        _name = name;
    }

    void throw_logical_error(base_log spdlog, const std::string &message)
    {
        if (nullptr != spdlog)
        {
            SPDLOG_LOGGER_CRITICAL(spdlog, message);
            throw std::logic_error("");
        }
        else
        {
            spdlog::critical(message);
            throw std::logic_error("");
        }
    }

    void throw_logical_error(const std::string &message)
    {
        spdlog::critical(message);
        throw std::logic_error("");
    }
}
