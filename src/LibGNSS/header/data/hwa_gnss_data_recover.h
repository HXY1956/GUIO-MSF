/**
*
* @file          grecoverdata.h
* @brief      Storage all recover observ equation/ parameter information
*
*/

#ifndef hwa_gnss_data_recover_H
#define hwa_gnss_data_recover_H
#include "hwa_base_time.h"
#include "hwa_base_par.h"
#include "hwa_base_data.h"
#include "hwa_gnss_data_obs.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for grcover head storaging
    */
    class gnss_data_recover_head
    {
    public:
        /** @brief default constructor. */
        gnss_data_recover_head();

        /** @brief default destructor. */
        ~gnss_data_recover_head();

        /** @brief get the begin time. */
        base_time get_beg_time() const;

        /** @brief get the end time. */
        base_time get_end_time() const;

    public:
        double interval;        ///< the interval of two adjacent epochs
        double sigma0 = 0;      ///< the sigma value
        std::set<std::string> sat_list;   ///< the list of satellites
        std::set<std::string> site_list;  ///< the list of sites
        std::set<base_time> time_list; ///< the list of time
    };

    /**
    * @brief       Class for grecover data storaging
    */
    class gnss_data_recover : public base_data
    {

    public:
        /** @brief default constructor. */
        gnss_data_recover();

        gnss_data_recover(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_recover();

    public:
        /** 
        * @brief get the recover time
        * @return      recover time
        */
        virtual base_time get_recover_time() const = 0;

        /**
        * @brief convert line to string
        * @return      a string
        */
        virtual std::string convert2strline() const = 0;
    };

    /**
    * @brief       Class for grecover equation data storaging
    */
    class gnss_data_recover_equation : public gnss_data_recover
    {
    public:
        /**
        * @brief constructor 1.
        * @param[in]  time        the time of current epoch.
        * @param[in]  site        the name of site.
        * @param[in]  sat        the name of satellite.
        * @return      gnss_data_recover_equation
        */
        gnss_data_recover_equation(const base_time &time, const std::string &site, const std::string &sat);

        /**
        * @brief constructor 1.
        * @param[in]  time        the time of current epoch.
        * @param[in]  site        the name of site.
        * @param[in]  sat        the name of satellite.
        * @return      gnss_data_recover_equation
        */
        gnss_data_recover_equation(base_log spdlog, const base_time &time, const std::string &site, const std::string &sat);

        /**
        * @brief copy constructor 2.
        * @param[in]  other        a gnss_data_recover_equation object.
        * @return      gnss_data_recover_equation.
        */
        gnss_data_recover_equation(const gnss_data_recover_equation &other);

        /** @brief default destructor. */
        ~gnss_data_recover_equation();

        /** @brief override operator =. */
        void operator=(const gnss_data_recover_equation &other);

        /** 
        * @brief override function get_recover_time.
        * @return the time of recover.
        */
        base_time get_recover_time() const override;

        /**
        * @brief override function convert2strline.
        * @return a string.
        */
        std::string convert2strline() const override;

        /**
        * @brief set the recover equation.
        * @param[in]  obstype    the combination type of observation.
        * @param[in]  resinfo    the residuals of observation.
        * @param[in]  is_newamb    whether is a new ambiguity.
        * @return      void.
        */
        void set_recover_equation(const gnss_data_obscombtype &obstype, const std::pair<double, double> &resinfo, int is_newamb);

    public:
        base_time time;           ///< the time.
        std::string sat_name;        ///< the name of satellite.
        std::string site_name;       ///< the name of site.
        gnss_data_obscombtype obstype; ///< the combination type of observation.
        double weight;          ///< the weight of observation.
        double resuidal;        ///< the resuidal of observation.
        int is_newamb;          ///< whether is a new ambiguity.
    };

    /**
    * @brief       Class for grecover parameter data storaging
    */
    class gnss_data_recover_par : public gnss_data_recover
    {
    public:
        /** @brief copy constructor 1. */
        gnss_data_recover_par(const gnss_data_recover_par &ohter);

        /**
        * @brief constructor 2. 
        * @param[in]  par                the type of parameter.
        * @param[in]  correct_value        the correct value of parameter.
        * @return      gnss_data_recover_par    
        */
        gnss_data_recover_par(base_log spdlog, const base_par &par, double correct_value);

        /** @brief default destructor. */
        virtual ~gnss_data_recover_par();

        /**
        * @brief override function get_recover_time.
        * @return the time of recover.
        */
        base_time get_recover_time() const override;

        /**
        * @brief override function convert2strline.
        * @return a string.
        */
        std::string convert2strline() const override;

        /** @brief override operator <. */
        bool operator<(const gnss_data_recover_par &) const;

    public:
        base_par par;           ///< the type of parameter.
        double correct_value; ///< the correct value of parameter.
    };
}

#endif // !GRECOVERDATA_H