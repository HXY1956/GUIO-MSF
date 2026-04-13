/**
*
* @file              gcycleslip.h
* @brief          CRS coordinate transform to ACR coordinate
*
*/

#ifndef hwa_gnss_data_cycleslip_H
#define hwa_gnss_data_cycleslip_H

#include "hwa_set_base.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_base_log.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for cycle slip
    */
    class  gnss_data_cycleslip
    {
    public:
        /** @brief default constructor. */
        explicit gnss_data_cycleslip();

        /**
        * @brief constructor 1.
        * @param[in]  head        ambflag file head data.
        * @return      void
        */
        explicit gnss_data_cycleslip(set_base *set, base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_cycleslip();

        /**
        * @brief set ambiguity flag
        * @param[in]  rec        the name of reveiver.
        * @param[in]  sat        the name of satellite
        * @param[in]  falg        the flag of ambiguity
        * @return      void
        */
        virtual void set_amb_flag(const std::string &rec, const std::string &sat, const int &flag) = 0;

        /**
        * @brief get ambiguity flag
        * @param[in]  rec        the name of reveiver.
        * @param[in]  sat        the name of satellite
        * @return      the flag of ambiguity
        */
        virtual int get_amb_flag(const std::string &rec, const std::string &sat) = 0;

        /**
        * @brief get active ambiguity.
        * @param[in]  site        the name of site.
        * @return      the flag of ambiguity.
        */
        virtual int get_active_amb(const std::string &site) = 0;

        /**
        * @brief set active ambiguity. override
        * @param[in]  site          the name of site.
        * @param[in]  active_num  active ambiguity value.
        * @return      void.
        */
        virtual void set_active_amb(const std::string &site, const int &active_num) = 0;

        /** @brief get active ambiguity. override*/
        virtual std::map<std::string, int> get_active_amb() = 0;

        /**
        * @brief get the number of ambiguity arc.
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current time.
        * @return      the number of ambiguity arc.
        */
        virtual int num_of_amb_arc(const std::string &site, const std::string &prn, const base_time &time) = 0;

        /**
        * @brief whether using the arc observation.
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current epoch.
        * @return      whether using the arc observation.
        */
        virtual bool use_of_obs(const std::string &site, const std::string &prn, const base_time &time) = 0;

        /**
        * @brief whether cycle slipped?
        * @param[in]  obsdata      observation data.
        * @param[in]  time          the time of current epoch.
        * @return      whether cycle slipped?
        */
        virtual bool cycle_slip(const gnss_data_sats &obsdata, const base_time &time) = 0;

        /**
        * @brief whether cycle slipped(123 ferq)?
        * @param[in]  obsdata      observation data.
        * @param[in]  time          the time of current epoch.
        * @return      whether cycle slipped?
        */
        bool cycle_slip123(gnss_data_sats &obsdata, const base_time &time) { return false; }

        /**
        * @brief whether is the carrier observation.
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current epoch.
        * @return      whether is the carrier observation.
        */
        virtual bool is_carrier_range(const std::string &site, const std::string &prn, const base_time &time) = 0;

        /**
        * @brief whether is the carrier observation. override
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current epoch.
        * @param[in]  c1
        * @param[in]  c2
        * @return      whether is the carrier observation.
        */
        virtual bool is_carrier_range(const std::string &site, const std::string &prn, const base_time &time, double &c1, double &c2) = 0;

        /**
        * @brief add the ambiguity flag
        * @param[in]  site          the name of site.
        * @param[in]  sat          the name of satellite.
        * @param[in]  description the flag of ambiguity.
        * @param[in]  beg          the begin time.
        * @param[in]  end          the end time.
        * @return      void
        */
        virtual void add_ambflag(const std::string &site, const std::string &sat, const std::string &description, const base_time &beg, const base_time &end) = 0; // glfeng

    protected:
        base_log _spdlog;      ///< spdlog
        set_base *_set;      ///< settings
        SLIPMODEL _slip_model; ///< cylce slip model
    };
}
#endif // !G_CYCLESLIP_H
