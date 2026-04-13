
/**
* @file            qualityControl.h
* @author       Guolong Feng, Wuhan University
*/

#ifndef hwa_gnss_proc_qualitycontrol_H
#define hwa_gnss_proc_qualitycontrol_H

#include "hwa_gnss_all_Nav.h"
#include "hwa_gnss_all_Obs.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_set_gen.h"
#include "hwa_set_rec.h"
#include "hwa_set_gbase.h"
#include "hwa_set_gproc.h"
#include "hwa_set_turboedit.h"

namespace hwa_gnss
{
    enum SMOOTH_model
    {
        SMT_DOPPLER,
        SMT_PHASE,
        SMT_NONE
    };

    /**
     * @brief Class for smooth
     */
    class gnss_proc_smooth
    {
    public:
        /**
         * @brief Construct a new t gsmooth object
         * @param[in]  settings  std::setbase control
         */
        gnss_proc_smooth(set_base *settings);
        /**
         * @brief Destroy the t gsmooth object
         */
        virtual ~gnss_proc_smooth(){};
        /**
         * @brief smooth range of observation
         * @param[in]  obsdata   observation
         * @param[in]  now       now time
         */
        void smooth_range_obs(std::vector<gnss_data_sats> &obsdata, const base_time &now);

    private:
        /**
         * @brief doppler smooth range
         * @param[in]  obsdata   observation
         * @param[in]  now       now time
         */
        void _doppler_smt_range(std::vector<gnss_data_sats> &obsdata, const base_time &now);
        /**
         * @brief doppler smooth range
         * @param[in]  obsdata   observation
         * @param[in]  now       now time
         */
        void _doppler_smt_range_new(std::vector<gnss_data_sats> &obsdata, const base_time &now);
        /**
         * @brief phase smooth range
         * @param[in]  obsdata   observation
         * @param[in]  now       now time
         */
        void _phase_smt_range(std::vector<gnss_data_sats> &obsdata, const base_time &now);

        SMOOTH_model _smoothModel;  ///< smooth model
        int _smoothWindow;          ///< window of smooth model
        double _smoothFactor;       ///< TODO
        double _sampling;           ///< sampling
        RECEIVERTYPE _receiverType; ///< type of receiver

        std::map<std::string, std::map<std::string, std::map<GOBS, double>>> _pre_smt_range; ///< site/sat/range_obs/pre_smt_value

        std::map<std::string, std::map<std::string, base_time>> _smt_beg_time;                 ///< smooth begin time
        std::map<std::string, std::map<std::string, std::map<GOBSBAND, base_time>>> _smt_last_time; ///< smooth last time
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;                  ///< index band

        std::map<std::string, std::map<std::string, std::map<GOBSBAND, std::pair<GOBS, double>>>> _pre_orig_val;     ///< TODO
        std::map<std::string, std::map<std::string, std::map<GOBSBAND, std::pair<GOBS, double>>>> _pre_pre_orig_val; ///< TODO
    };

    /**
     * @brief class for BeiDou satellite-induced code pseudorange variations correct
     */
    class gnss_proc_bds_codebias_corr
    {
    public:
        /**
         * @brief Construct a new t gbds codebias cor object
         * @param[in]  settings  std::setbase control
         */
        gnss_proc_bds_codebias_corr(set_base *settings);
        /**
         * @brief Destroy the t gbds codebias cor object
         */
        virtual ~gnss_proc_bds_codebias_corr(){};
        /**
        * @brief apply IGSO_MEO satellite coordinate and velocity
        * @param [in]  rec               station name
        * @param [in]  rec_crd           station coordinate
        * @param [in]  gnav               all navigation
        * @param [in]  obsdata           all observation
        * @return
               *    @retval   false     unsuccessfully apply IGSO
               *    @retval   true      successfully apply MEO
        */
        void apply_IGSO_MEO(const std::string &rec, Triple &rec_crd, gnss_all_nav *gnav, std::vector<gnss_data_sats> &obsdata);

    private:
        set_base *_set;                               ///< std::setbase control
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< index of band
        bool _correct_bds_code_bias;                    ///< BEIDOU correct bias
        // Wanninger & Beer : BeiDou satellite-induced code pseudorange variations: diagnosis and therapy [unit:m]
        std::map<GOBSBAND, std::map<std::string, std::map<int, double>>> _IGSO_MEO_Corr; ///< TODO

        bool _recAprCoordinate(const std::string &rec, Triple &rec_crd, gnss_all_nav *gnav, std::vector<gnss_data_sats> &obsdata);
    };

    /**
     * @brief Class for outliers process
     */
    class gnss_proc_OUTLIER
    {
    public:
        /**
         * @brief Construct a new t goutliers process object
         * @param[in]  settings  std::setbase control
         * @param[in]  spdlog      logbase control
         */
        gnss_proc_OUTLIER(set_base *settings);

        /**
         * @brief Construct a new t goutliers process object
         * @param[in]  settings  std::setbase control
         * @param[in]  spdlog      logbase control
         */
        gnss_proc_OUTLIER(set_base *settings, base_log spdlog);
        /**
         * @brief Destroy the t goutliers process object
         */
        virtual ~gnss_proc_OUTLIER();
        /**
         * @brief Set the Log
         * @param[in]  spdlog      logbase control
         */
        void setLog(base_log spdlog)
        {
            // std::set spdlog
            if (nullptr == spdlog)
            {
                spdlog::critical("your spdlog is nullptr !");
                throw std::logic_error("");
            }
            if (nullptr != spdlog)
            {
                _spdlog = spdlog;
            }
        };
        base_log spdlog() { return _spdlog; };
        void spdlog(base_log spdlog)
        {
            // std::set spdlog
            if (nullptr == spdlog)
            {
                spdlog::critical("your spdlog is nullptr !");
                throw std::logic_error("");
            }
            if (nullptr != spdlog)
            {
                _spdlog = spdlog;
            }
        };

        /**
         * @brief move bad observation
         * @param[in]  obsdata   all observation data
         */
        void excludeBadObs(std::vector<gnss_data_sats> &obsdata); // Low SNR < 10dB

        void flagRangeOutliers(std::shared_ptr<gnss_data_obs_manager> ObsPre, std::shared_ptr<gnss_data_obs_manager> Obs, double sampling);

    private:
        base_log _spdlog;                  ///< logbase control
        set_base *_set;                  ///< std::setbase control
        base_iof *_debug_outliers = nullptr; ///< TODO
        int _frequency;                    ///< frequency
        OBSCOMBIN _observ;                 ///< type of obsevation
        //std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>>  _band_index;
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index; ///< frequncy of type
        std::set<OBSCOMBIN> _single_mix;                     ///< TODO
        RECEIVERTYPE _receiverType;                     ///< type of receiver
    };

    //TODO COMMENT
    /**
     * @brief Class for qualitycontrol
     */
    class gnss_proc_quality_control
    {
    public:
        /**
         * @brief Construct a new t gqualitycontrol object
         * @param[in]  settings  std::setbase control
         * @param[in]  gnav      navigation data
         */
        gnss_proc_quality_control(base_log spdlog, set_base *settings, gnss_all_nav *gnav);

        /**
         * @brief Construct a new t gqualitycontrol object
         * @param[in]  settings  std::setbase control
         * @param[in]  gnav      navigation data
         */
        gnss_proc_quality_control(set_base *settings, gnss_all_nav *gnav);
        /**
         * @brief Destroy the t gqualitycontrol object
         */
        virtual ~gnss_proc_quality_control();
        /**
         * @brief process one epoch
         * @param[in]  now       current time
         * @param[in]  rec       station name
         * @param[in]  rec_crd   station coodinate
         * @param[in]  obsdata   observation data
         */
        int processOneEpoch(const base_time &now, const std::string &rec, Triple &rec_crd, std::vector<gnss_data_sats> &obsdata);
        /**
         * @brief Set the Nav data
         * @param[in]  gnav      navigation data
         */
        void setNav(gnss_all_nav *gnav) { _gnav = gnav; };

        base_log spdlog() { return _spdlog; };
        void spdlog(base_log spdlog)
        {
            // std::set spdlog
            if (nullptr == spdlog)
            {
                spdlog::critical("your spdlog is nullptr !");
                throw std::logic_error("");
            }
            if (nullptr != spdlog)
            {
                _spdlog = spdlog;
            }
        };
        void set_log(base_log spdlog)
        {
            // std::set spdlog
            if (nullptr == spdlog)
            {
                spdlog::critical("your spdlog is nullptr !");
                throw std::logic_error("");
            }
            if (nullptr != spdlog)
            {
                _spdlog = spdlog;
            }
        };

    protected:
        base_log _spdlog = nullptr; ///< logbase control
        set_base *_set = nullptr; ///< std::setbase control
        gnss_all_nav *_gnav = nullptr; ///< navigation daya

        gnss_proc_smooth           _smooth_range;     ///< smooth range
        gnss_proc_bds_codebias_corr _bds_codebias_cor; ///< TODO
        gnss_proc_OUTLIER _outliers_proc;    ///< TODO
    };

} // namespace

#endif
