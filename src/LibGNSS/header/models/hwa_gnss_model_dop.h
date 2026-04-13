#ifndef hwa_gnss_model_dop_H
#define hwa_gnss_model_dop_H

#include "hwa_base_eigendef.h"
#include "hwa_gnss_all_nav.h"
#include "hwa_gnss_all_obs.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_model_dop. */
    class gnss_model_dop
    {
    public:
        /** @brief default constructor. */
        explicit gnss_model_dop();

        explicit gnss_model_dop(base_log spdlog);
        /** @brief constructor 1. */
        explicit gnss_model_dop(base_log spdlog, gnss_all_nav *gnav, gnss_all_obs *gobs, std::string site);
        explicit gnss_model_dop(gnss_all_nav *gnav, gnss_all_obs *gobs, std::string site);

        /** @brief constructor 2. */
        explicit gnss_model_dop(gnss_all_nav *gnav, std::set<std::string> sats);

        explicit gnss_model_dop(base_log spdlog, gnss_all_nav *gnav, std::set<std::string> sats);
        /** @brief default destructor. */
        ~gnss_model_dop();

        /** @brief Calculate dop - _Qx. */
        int calculate(const base_time &epoch, Triple &rec, GSYS gnss = GNS);

        /** @brief Position dilution of precision. */
        double pdop();

        /** @brief Geom dilution of precision. */
        double gdop();

        /** @brief Time dilution of precision. */
        double tdop();

        /** @brief Horizontal dilution of precision. */
        double hdop();

        /** @brief Vertical dilution of precision. */
        double vdop();

        /** @brief std::set nav, obs, site. */
        void set_data(gnss_all_nav *gnav, gnss_all_obs *gobs, std::string site);

        /** @brief std::set log. */
        void set_log(base_log spdlog);

        /** @brief std::set satellite list for calculation. */
        void set_sats(std::set<std::string> &sats);

    private:
        std::string _site;        ///< site name
        gnss_all_nav *_gnav;    ///< ephemerides
        gnss_all_obs *_gobs;    ///< observations
        std::set<std::string> _sats;   ///< std::set of visible satellites
        Triple _rec;      ///< receiver position
        Symmetric _Qx; ///< variance-covariance matrix
        base_log _spdlog;
    };
}

#endif
