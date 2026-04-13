/**
* @file        gifcb.h
* @brief    Storage the ifcb files' data
*/

#ifndef hwa_gnss_data_ifcb_H
#define hwa_gnss_data_ifcb_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for storaging one satellite ifcb data
    */
    class gnss_data_ifcb_rec
    {
    public:
        /** @brief default constructor. */
        gnss_data_ifcb_rec();

        /** @brief default destructor. */
        ~gnss_data_ifcb_rec(){};

        std::string obj;   ///< ifcb objection may be site or satellite
        int npoint;   ///< site number
        double value; ///< ifcb value
        double sigma; ///< std
        bool isRef;   ///< true set as a reference
    };

    /** std::map container using satellite name as a index for storaging gnss_data_ifcb_rec ptr , one epoch/all satellite
    *   for wide-lane only one sign epoch "WL_IDENTIFY"   */
    typedef std::map<std::string, std::shared_ptr<gnss_data_ifcb_rec>> hwa_map_id_ifcb;

    /**
    *@brief     Class for storaging all epoch/all satellite ifcb data
    */
    class gnss_data_ifcb : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_ifcb();

        /** @brief default constructor. */
        gnss_data_ifcb(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_ifcb();

        /**
        * @brief add ifcb data of one epoch/one satellite.
        * @param[in]  epoch          epoch.
        * @param[in]  prn          satellite name.
        * @param[in]  one_sagnss_coder_ifcb ifcb data of one epoch/one satellite
        * @return      void
        */
        void add_sat_ifcb(base_time epoch, std::string prn, gnss_data_ifcb_rec one_sagnss_coder_ifcb);

        /**
        * @brief add ifcb data of one epoch/all satellite.
        * @param[in]  epoch          epoch.
        * @param[in]  one_sagnss_coder_ifcb ifcb data of one epoch/all satellite
        * @return      void
        */
        void add_epo_ifcb(base_time epoch, hwa_map_id_ifcb hwa_map_id_ifcb);

        /**
        * @brief get ifcb data, all epoch/all satellite.
        * @return     ifcb data
        */
        std::map<base_time, hwa_map_id_ifcb> &get_icfb() { return _ifcb; };

        /**
        * @brief get ifcb data's value in class gnss_data_ifcb_rec,one epoch/one satellite.
        * @param[in] t epoch time
        * @param[in] str satellite name
        * @return the ifcb value of satellite in epoch
        */
        double get_icfb_value(const base_time &t, const std::string &str) { return _ifcb[t][str]->value; };

        /**
        * @brief get ifcb data's sigma in class gnss_data_ifcb_rec,one epoch/one satellite.
        * @param[in] t epoch time
        * @param[in] str satellite name
        * @return the ifcb sigma of satellite in epoch
        */
        double get_icfb_sigma(const base_time &t, const std::string &str) { return _ifcb[t][str]->sigma; };

        /**
        * @brief get upd data's npoint in class gnss_data_updrec,one epoch/one satellite.
        * @param[in] t epoch time
        * @param[in] str satellite name
        * @return number of sites used in this sat's upd estimation
        */
        double get_icfb_npoint(const base_time &t, const std::string &str) { return _ifcb[t][str]->npoint; };

        /**
        * @brief get ifcb data of one epoch/all satellite.
        * @param[in] t epoch time
        * @param[in] str satellite name
        * @return the ifcb data in epoch
        */
        hwa_map_id_ifcb &get_epo_ifcb(const base_time &t);

        /**
        * @brief reset ifcb data's value in class gnss_data_ifcb_rec of one epoch/one satellite.
        * @param[in]  t          epoch.
        * @param[in]  str      satellite name.
        * @param[in]  value      value that ifcb data's value will reset.
        * @param[in]  sigma      std that ifcb data's sigma will reset.
        * @param[in]  npoint  site number that ifcb data's npoint will reset.
        * @return      void
        */
        void reset_icfb(const base_time &t, const std::string &str, const double &value,
                        const double &sigma, const int &npoint);

        /**
        * @brief reset ifcb data's value in class gnss_data_ifcb_rec of one epoch/one satellite.
        * @param[in]  t          epoch.
        * @param[in]  str      satellite name.
        * @param[in]  value      value that ifcb data's value will reset.
        * @return      void
        */
        void reset_icfb_value(const base_time &t, const std::string &str, const double &value);

        /**
        * @brief reinitialize ifcb data of one epoch/one satellite.
        * @param[in]  t          epoch.
        * @param[in]  str      satellite name.
        * @return      void
        */
        void re_init_icfb(const base_time &t, std::string str); //str maybe site/sats

        /**
        * @brief delete ifcb data of one epoch/one satellite.
        * @param[in]  t          epoch.
        * @param[in]  str      satellite name.
        * @return      void
        */
        void delete_ifcb(const base_time &t, const std::string &str) { _ifcb[t].erase(str); };

        /**
        * @brief judge ifcb data is usable or not, one epoch/one satellite.
        * @param[in]  t          epoch.
        * @param[in]  str      satellite name.
        * @return      void
        */
        bool ifcb_usable(const base_time &t, const std::string &str);

        /**
        * @brief reset ifcb data's value in class gnss_data_ifcb_rec of one epoch/one satellite.
        * @param[in]  pre_t          previous epoch.
        * @param[in]  str          satellite name.
        * @param[in]  current_t      current epoch.
        * @param[in]  is_first      if it's first epoch , re-init ifcb data.
        * @param[in]  is_site     if it's site ifcb, remove old ifcb,add new ifcb.
        * @return      void
        */
        void copy_ifcb(const base_time &pre_t, const std::string &str, const base_time &current_t,
                       const bool &is_first, const bool &is_site);

        /**
        * @brief set the valid begin epoch.
        * @param[in]  t        current epoch.
        * @return      void
        */
        void set_valid_beg(const base_time &t);

        /**
        * @brief get the valid begin epoch.
        * @return the valid begin epoch of ifcb
        */
        base_time get_valid_beg() { return _valid_beg; }

    protected:
        std::map<base_time, hwa_map_id_ifcb> _ifcb; ///< ifcb std::map container of all epoch/all satellite
        base_time _valid_beg;                 ///< valid begin epoch
        hwa_map_id_ifcb _null_epoch_ifcb;    ///< ione epoch ifcb value

    private:
    };

}
#endif // !GALLPLANETEPH_H