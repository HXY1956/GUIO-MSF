#ifndef GUPD_H
#define GUPD_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include "hwa_set_amb.h"

using namespace hwa_gnss;

#ifndef EWL_IDENTIFY
#define EWL_IDENTIFY 666666   ///< the sign epoch for extra-wide-lane upd
#endif

#ifndef EWL24_IDENTIFY
#define EWL24_IDENTIFY 777777 ///< the sign epoch for extra-wide-lane(24) upd
#endif

#ifndef EWL25_IDENTIFY
#define EWL25_IDENTIFY 888888 ///< the sign epoch for extra-wide-lane(25) upd
#endif

#ifndef WL_IDENTIFY
#define WL_IDENTIFY 999999    ///< the sign epoch for wide-lane upd
#endif

namespace hwa_gnss
{

    /**
    *@brief       Class for storaging one satellite upd data
    */
    class gnss_data_updrec
    {
    public:
        /** @brief default constructor. */
        gnss_data_updrec();

        /** @brief default destructor. */
        ~gnss_data_updrec(){};

        std::string obj;   ///< upd objection may be site or satellite
        int npoint;   ///< site number
        double ratio; ///< number of good observation divide number of all observation based on rec.
        double value; ///< upd value
        double sigma; ///< std
        bool isRef;   ///< true set as a reference
    };

    /** 
    * map container using satellite name as a index for storaging gnss_data_updrec ptr , one epoch/all satellite
    * for wide-lane only one sign epoch "WL_IDENTIFY"   
    */
    typedef std::map<std::string, std::shared_ptr<gnss_data_updrec>> one_epoch_upd;

    /**
    *@brief     Class for storaging all epoch/all satellite upd data
    */
    class gnss_data_upd : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_upd();

        gnss_data_upd(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_upd();

        /**
        * @brief add upd data of one epoch/one satellite.
        * @param[in]  upd_type    upd type
        * @param[in]  epoch          epoch.
        * @param[in]  prn          satellite name.
        * @param[in]  one_sat_upd upd data of one epoch/one satellite
        * @return      void
        */
        void add_sat_upd(UPDTYPE upd_type, base_time epoch, std::string prn, gnss_data_updrec one_sat_upd);

        /**
        * @brief add upd data of one epoch/all satellite.
        * @param[in]  upd_type    upd type
        * @param[in]  epoch          epoch.
        * @param[in]  one_sat_upd upd data of one epoch/all satellite
        * @return      void
        */
        void add_epo_upd(UPDTYPE upd_type, base_time epoch, one_epoch_upd one_epoch_upd);

        /**
        * @brief set upd estimation mode, supported EWL/WL/NL currently.
        * @param[in]  mode          upd mode , EWL/WL/NL.
        * @return      void
        */
        void set_est_updtype(UPDTYPE mode);

        /**
        * @brief get upd estimation mode, supported EWL/WL/NL currently.
        * @return     upd mode
        */
        UPDTYPE get_est_updtype();

        /** 
        * @brief get upd data, all epoch/all satellite. 
        * @param[in] upd_type    upd type
        * @return     upd data
        */
        std::map<base_time, one_epoch_upd> &get_upd(UPDTYPE upd_type) { return _upd[upd_type]; };

        /** 
        * @brief get upd data's value in class gnss_data_updrec,one epoch/one satellite.
        * @param[in] upd_type    upd type
        * @param[in] t           epoch time
        * @param[in] str satellite name
        * @return the upd value of satellite in epoch
        */
        double get_upd_value(const UPDTYPE &upd_type, const base_time &t, const std::string &str) { return _upd[upd_type][t][str]->value; };

        /**
        * @brief get upd data's sigma in class gnss_data_updrec,one epoch/one satellite. 
        * @param[in] upd_type    upd type
        * @param[in] t           epoch time
        * @param[in] str         satellite name
        * @return the upd sigma of satellite in epoch
        */
        double get_upd_sigma(const UPDTYPE &upd_type, const base_time &t, const std::string &str) { return _upd[upd_type][t][str]->sigma; };

        /**
        * @brief get upd data's npoint in class gnss_data_updrec,one epoch/one satellite.
        * @param[in] upd_type    upd type
        * @param[in] t           epoch time
        * @param[in] str         satellite name
        * @return number of sites used in this sat's upd estimation
        */
        double get_upd_npoint(const UPDTYPE &upd_type, const base_time &t, const std::string &str) { return _upd[upd_type][t][str]->npoint; };

        /**
        * @brief get upd data of one epoch/all satellite.
        * @param[in] upd_type    upd type
        * @param[in] t           epoch time
        * @param[in] str         satellite name
        * @return the upd data in epoch
        */
        one_epoch_upd &get_epo_upd(const UPDTYPE &upd_type, const base_time &t); // add leo

        /**
        * @brief reset upd data's value in class gnss_data_updrec of one epoch/one satellite.
        * @param[in] upd_type upd type
        * @param[in] t        epoch time
        * @param[in] str      satellite name
        * @param[in]  value      value that upd data's value will reset.
        * @param[in]  sigma      std that upd data's sigma will reset.
        * @param[in]  npoint  site number that upd data's npoint will reset.
        * @return      void
        */
        void reset_upd(const UPDTYPE& upd_type, const base_time& t, const std::string& str, const double& value,
                       const double &sigma, const int &npoint, const double &ratio = 0.0);

        /**
        * @brief reset upd data's value in class gnss_data_updrec of one epoch/one satellite.
        * @param[in] upd_type    upd type
        * @param[in] t           epoch time
        * @param[in] str         satellite name
        * @param[in]  value      value that upd data's value will reset.
        * @return      void
        */
        void reset_upd_value(const UPDTYPE &upd_type, const base_time &t, const std::string &str, const double &value);

        /**
        * @brief reinitialize upd data of one epoch/one satellite.
        * @param[in] upd_type    upd type
        * @param[in] t           epoch time
        * @param[in] str         satellite name
        * @return      void
        */
        void re_init_upd(const UPDTYPE &upd_type, const base_time &t, std::string str); //str maybe site/sats

        /**
        * @brief delete upd data of one epoch/one satellite.
        * @param[in] upd_type    upd type
        * @param[in] t           epoch time
        * @param[in] str         satellite name
        * @return      void
        */
        void delete_upd(const UPDTYPE &upd_type, const base_time &t, const std::string &str) { _upd[upd_type][t].erase(str); };

        /**
        * @brief judge upd data is usable or not, one epoch/one satellite.
        * @param[in] t           epoch time
        * @param[in] str         satellite name
        * @return      void
        */
        bool upd_usable(const base_time &t, const std::string &str);

        /**
        * @brief reset upd data's value in class gnss_data_updrec of one epoch/one satellite.
        * @param[in]  upd_type    upd type
        * @param[in]  pre_t          previous epoch.
        * @param[in]  str          satellite name.
        * @param[in]  current_t      current epoch.
        * @param[in]  is_first      if it's first epoch , re-init upd data.
        * @param[in]  is_site     if it's site upd, remove old upd,add new upd.
        * @return      void
        */
        void copy_upd(const UPDTYPE &upd_type, const base_time &pre_t, const std::string &str, const base_time &current_t,
                      const bool &is_first, const bool &is_site);

        /**
        * @brief set the valid begin epoch.
        * @param[in]  upd_type    upd type
        * @param[in]  t        current epoch.
        * @return      void
        */
        void set_valid_beg(const UPDTYPE &upd_type, const base_time &t);

        /**
        * @brief get the valid begin epoch.
        * @param[in]  upd_type    upd type
        * @return the valid begin epoch of upd
        */
        base_time get_valid_beg(const UPDTYPE &upd_type) { return _valid_beg[upd_type]; }

        /*Judge real time upd available*/
        bool upd_avail(const base_time &now);
        void set_end(const base_time &now);

        /**
        * @ set/get wl upd mode (Epoch by epoch/ Single day solution)
        */
        void wl_epo_mode(bool is) { _wl_epo_mode = is; };
        bool wl_epo_mode() { return _wl_epo_mode; };

    protected:
        std::map<UPDTYPE, std::map<base_time, one_epoch_upd>> _upd; ///< upd map container of all epoch/all satellite(different type)
        UPDTYPE _est_upd_type;                          ///< upd mode (for estimation)
        std::map<UPDTYPE, base_time> _valid_beg;               ///< valid begin epoch (for encoder)
        one_epoch_upd _null_epoch_upd;
        bool _wait_stream; ///< Real-time upd interrupt to maintain float solution results
        bool _wl_epo_mode; ///< WL epoch by epoch or not?
        base_time _tend;     ///< The end Time of real-time upd.
        double _upd_intv;  ///< The interval of real-time upd.

    private:
        base_time _ewl_flag;   ///< ewl flag
        base_time _ewl24_flag; ///< ewl24 flag
        base_time _ewl25_flag; ///< ewl25 flag
        base_time _wl_flag;    ///< wl flag
    };

}
#endif // !GALLPLANETEPH_H