#ifndef hwa_base_log_h
#define hwa_base_log_h

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#endif

#include <string>
#include "spdlog.h"

using namespace spdlog;
namespace hwa_base
{
    typedef std::shared_ptr<spdlog::logger> base_log;

    /**
     * @brief 
     * 
     */
    class base_rtlog
    {
    public:
        /**
         * @brief Construct a new t grtlog object
         * 
         */
        base_rtlog();

        /**
         * @brief Construct a new t grtlog object
         * 
         * @param log_type 
         * @param log_level 
         * @param log_name 
         */
        base_rtlog(const std::string &log_type, const level::level_enum &log_level, const std::string &log_name);

        /**
         * @brief Destroy the t grtlog object
         * 
         */
        virtual ~base_rtlog();

        /**
         * @brief 
         * 
         * @return std::shared_ptr<logger> 
         */
        std::shared_ptr<logger> spdlog();

        /**
         * @brief 
         * 
         * @param pattern 
         */
        void reset_pattern(const std::string &pattern);

        /**
         * @brief 
         * 
         * @param level 
         */
        void reset_level(const level::level_enum &level);

        /**
         * @brief Set the log object
         * 
         * @param log_type 
         * @param log_level 
         * @param log_name 
         */
        void set_log(const std::string &log_type, const level::level_enum &log_level, const std::string &log_name);

        std::string name() { return _name; }
        void set_name(const std::string& name);

    private:
        std::string _name;
        std::string _type;
        std::string _pattern;
        level::level_enum _level;
        base_log _spdlog;
    };

    /**
     * @brief 
     * 
     * @param spdlog 
     * @param message 
     * @return 
     */
    void throw_logical_error(base_log spdlog, const std::string &message);

    /**
     * @brief 
     * 
     * @param message 
     * @return 
     */
    void throw_logical_error(const std::string &message);
}
#endif
