/**
* @file        gallambflag.h
* @brief    Storage the XXXXddd0.yyo.log/log13 files' data(more than one site)
*                XXXX  ---- SITE name
*                 ddd  ---- Doy of the file
*                  yy  ---- year
*/

#ifndef hwa_gnss_all_ambflag_H
#define hwa_gnss_all_ambflag_H

#include "hwa_gnss_data_ambflag.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** map container using station name as a index for storaging hwa_vector_ambflag data of all station */
    typedef std::map<std::string, gnss_data_ambflag> hwa_map_all_ambflag;

    /**
    *@brief       Class for all XXXXddd0.yyo.log/log13 file data storaging
    */
    class gnss_all_ambflag : public base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_all_ambflag();

        /**
         * @brief Construct a new t gallambflag object
         * 
         * @param t 
         */
        explicit gnss_all_ambflag(const ID_TYPE &t);

        /**
         * @brief Construct a new t gallambflag object
         * 
         * @param spdlog 
         * @param t 
         */
        explicit gnss_all_ambflag(base_log spdlog, ID_TYPE t);

        /** @brief default destructor. */
        virtual ~gnss_all_ambflag();

        /**
        * @brief add ambflag head of one station.
        * @param[in]  site_name        station's name.
        * @param[in]  header_data    ambflag head of one station.
        * @return      void
        */
        void addAmbFlagHead(const std::string &sta, const gnss_data_ambflag_head &header_data);

        /**
        * @brief add ambflag data of one station.
        * @param[in]  site_name        station's name.
        * @param[in]  amb_flag        ambflag data of one station.
        * @return      void
        */
        void addAmbFlag(const std::string &sta, const gnss_data_ambflag &amb_flag);

        /**
        * @brief add ambflag data of one satellite in one station.
        * @param[in]  sat_name        satellite's name.
        * @param[in]  site_name        station's name.
        * @param[in]  data            ambflag data of one satellite in one station.
        * @return      void
        */
        void addAmbFlagData(const std::string &sta, const std::string &sat, const gnss_data_ambflag_data &data);

        /**
        * @brief return ambflag data of one station.
        * @param[in]  site_name        station's name.
        * @return ambflag data of one station
        */
        gnss_data_ambflag &getOneAmbFlag(const std::string &sta);

        /**
        * @brief return ambflag data of all station.
        * @return ambflag data of all station
        */
        hwa_map_all_ambflag &getAllAmbFlag();

        /**
        * @brief  return all system designated satellites' prn of all station.
        * @return all system designated satellites' prn of all station
        */
        std::set<std::string> getSatList();

        /**
        * @brief  return all station.
        * @return all station
        */
        std::set<std::string> getSiteList();

        /**
        * @brief  clear the data for log12/log13/log14/....
        * @return all station
        */
        void clearSiteAmbFlag(const std::string &site) { _map_allambflag[site].clearAllAmbFlagData(); };

    protected:
        hwa_map_all_ambflag _map_allambflag; ///< map of station name and ambflag data for all sites
    };

} // namespace

#endif
