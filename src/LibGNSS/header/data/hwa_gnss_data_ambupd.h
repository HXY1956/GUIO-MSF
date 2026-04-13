/**
* @file        gambupd.h
* @brief    Storage the ambiguity upd files' data(only one site) for narrow-lane upd compute.
*/

#ifndef hwa_gnss_data_ambupd_H
#define hwa_gnss_data_ambupd_H

#include "hwa_base_fileconv.h"
#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include <set>

namespace hwa_gnss
{
    /**
    *@brief       Class for storaging ambupd data of one stations/one satellite/one epoch 
    */
    class gnss_data_ambupd_batch
    {
    public:
        /** @brief default constructor. */
        gnss_data_ambupd_batch(){};

        /** @brief default destructor. */
        ~gnss_data_ambupd_batch(){};

        double amb_if = 0.0;       ///< ambiguity ionosphere-free value meter  /N1 in UCUD mode (m)
        double amb_wl = 0.0;       ///< ambiguity Wide-Lane value       cycle  /N2 in UCUD mode (m)
        double amb_l3 = 0.0;       /// < N3 in UCUD mode (m)
        double amb_l4 = 0.0;       /// < N4 in UCUD mode (m)
        double amb_l5 = 0.0;       /// < N5 in UCUD mode (m)
        int flag = 0;              ///< flag
        double amb_wl_sigma = 0.0; ///< std of ambiguity Wide-Lane value
    };

    /** std::map container using satellite name as a index for storaging gnss_data_ambupd_batch ptr , one station/one epoch/all satellite */
    typedef std::map<std::string, std::shared_ptr<gnss_data_ambupd_batch>> hwa_map_id_ambupd;

    typedef std::map<hwa_base::base_time, std::shared_ptr<gnss_data_ambupd_batch>> hwa_map_time_ambupd;

    /** std::map container using epoch as a index for storaging hwa_map_id_ambupd , one station/all epoch/all satellite */
    typedef std::map<hwa_base::base_time, hwa_map_id_ambupd> hwa_map_ti_ambupd;

    /**
    *@brief       Class for storaging ambupd data of one stations/all satellite/all epoch
    */
    class gnss_data_ambupd : public hwa_base::base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_ambupd();

        gnss_data_ambupd(hwa_base::base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_ambupd();

        /**
        * @brief add gnss_data_ambupd_batch of one station/one epoch/one satellite.
        * @param[in]  epoch        epoch.
        * @param[in]  sat        satellite prn.
        * @param[in]  data        gnss_data_ambupd_batch, data of one station/one epoch/one satellite ambupd file.
        * @return      void
        */
        void addAmbUpd(hwa_base::base_time epoch, std::string sat, const gnss_data_ambupd_batch &data);

        /**
        * @brief get gnss_data_ambupd_batch ptr of one station/one epoch/one satellite.
        * @param[in]  epoch        epoch.
        * @param[in]  sat        satellite prn.
        * @return      gnss_data_ambupd_batch ptr, data ptr of one station/one epoch/one satellite ambupd file.
        */
        std::shared_ptr<gnss_data_ambupd_batch> &getSatAmbUpd(hwa_base::base_time epoch, std::string sat);

        /**
        * @brief get hwa_map_id_ambupd data of one station/one epoch/all satellite.
        * @param[in]  epoch        epoch.
        * @return      hwa_map_id_ambupd data of one station/one epoch/all satellite.
        */
        hwa_map_id_ambupd &getEpoAmbUpd(hwa_base::base_time epoch);

        hwa_map_time_ambupd getSatepoAmbUpd(hwa_base::base_time &beg, hwa_base::base_time &end, std::string sat);
        /**
        * @brief get hwa_map_ti_ambupd data of one station/all epoch/all satellite.
        * @return     hwa_map_ti_ambupd data of one station/all epoch/all satellite.
        */
        hwa_map_ti_ambupd &getAllAmbUpd();

        /**
        * @brief   return the last record of site.
        * @return  end time of the site.
        */
        hwa_base::base_time getSatEndTime();
        /**
        * @brief   get satellite list
        * @return  
            @retval std::string satellite list
        */
        std::set<std::string> getSatList() { return _sat_list; }

    protected:
        std::set<std::string> _sat_list; ///< site name
        hwa_map_ti_ambupd _ambupd;  ///< internal variable of hwa_map_ti_ambupd storaging data
                               ///< of one station/all epoch/all satellite

    private:
    };
}
#endif