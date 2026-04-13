/**
* @file          gallambupd.h
* @brief      Storage the ambiguity upd files' data(more than one site) for narrow-lane upd compute
*/
#ifndef hwa_gnss_all_ambupd_H
#define hwa_gnss_all_ambupd_H

#include <vector>
#include "hwa_gnss_data_ambupd.h"
#include "hwa_base_time.h"
#include "hwa_base_log.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for all ambupd file data of all stations storaging
    */
    class gnss_all_ambupd : public base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_all_ambupd();

        /** @brief default constructor. */
        explicit gnss_all_ambupd(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_ambupd();

        /** @brief std::map for storaging upd of all stations. */
        typedef std::map<std::string, gnss_data_ambupd> hwa_map_allambupd;

        /**
        * @brief add all epochs ambupd data of one station.
        * @param[in]  sta        station's name.
        * @param[in]  ambupd    ambupd data of one station.
        * @return      void
        */
        void addAmbUpd(const std::string &site_name, const gnss_data_ambupd &ambupd);

        /**
        * @brief add one epoch ambupd data of one stationn.
        * @param[in]  sta        station's name.
        * @param[in]  epoch        epoch.
        * @param[in]  sat        satellite's name.
        * @param[in]  data        class for one station/one epoch/one satellite ambupd data.
        * @return      void
        */
        void addAmbUpd(const std::string &site_name, const base_time &epoch, const std::string &sat_name, const gnss_data_ambupd_batch &data); //one station/one epoch/one sat

        /**
        * @brief get all epoch ambupd data of one station.
        * @param[in]  sta        station's name.
        * @return      all epoch ambupd data of one station
        */
        gnss_data_ambupd &getOneSiteAmbUpd(const std::string &site_name); //one station

        /**
        * @brief get all satellite ambupd data of one station and one epoch.
        * @param[in]  sta        station's name.
        * @param[in]  t          epoch
        * @return      all satellite ambupd data of one station and one epoch
        */
        hwa_map_id_ambupd &getOneSiteOneEpoAmbUpd(const std::string &site_name, const base_time &epoch); //one sation one time

        /**
        * @brief get one satellite/one epoch ambupd data of one station.
        * @param[in]  epoch     epoch
        * @param[in]  sta        station's name.
        * @param[in]  sat        satellite's name.
        * @return      one satellite/one epoch ambupd data of one station
        */
        std::shared_ptr<gnss_data_ambupd_batch> &getOneSiteOneEpoOneSatAmbUpd(const base_time &epoch, const std::string &site_name, const std::string &sat_name); // one epoch / one station / one satellite

        /**
        * @brief get all station ambupd file data.
        * @return all station ambupd file data
        */
        hwa_map_allambupd &getAllAmbUpd(); // all stations /all epochs / all satelltes

        /**
        * @brief judge if the ambupd usable.
        * @param[in]  epoch     epoch
        * @param[in]  sta        station's name.
        * @param[in]  sat        satellite's name.
        * @return     true  usable/ false not usable
        */
        bool isAmbUpdValid(const base_time &epoch, const std::string &site_name, const std::string &sat_name);

        /**
        * @brief  get all sites' name of all ambupd file data.
        * @return  std::string container of site list
        */
        std::set<std::string> get_sites();

        /**
        * @brief  get all sats' name of all ambupd file data.
        * @return  std::string container of site list
        */
        std::set<std::string> get_sats();

        /** @brief return the last record of sites. */
        std::map<std::string, base_time> getSitesEndRecord();

    protected:
        hwa_map_allambupd _allambupd; ///< std::map of station name and ambupd data for all epochs and all satellites
    };

} // namespace

#endif
