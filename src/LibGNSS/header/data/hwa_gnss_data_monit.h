
#ifndef hwa_gnss_data_monit_H
#define hwa_gnss_data_monit_H

/**
*
* @file        gmonit.h
* @brief    basic class for monitoring purpose
*.
*/

#include <string>
#include <sstream>

namespace hwa_gnss
{

    /**
    *@brief       basic class for monitoring purpose
    */
    class gnss_data_monit
    {
    public:
        /** @brief constructor. */
        explicit gnss_data_monit(const std::string &id);

        /** @brief default destructor. */
        virtual ~gnss_data_monit();

        /** @brief show the monitoring information. */
        virtual void show(std::ostringstream &os, int verb);
        //  virtual string show( int verb, int repeat, string& buff ) = 0;

    protected:
        std::string _moni_id; ///< monitoring id

    private:
    };

} // namespace

#endif
