/**
* @file             gexeturboedit.h
* @brief            main control class for turboedit
*/

#ifndef hwa_exeturboedit_H
#define hwa_exeturboedit_H

#include "hwa_set_gen.h"
#include "hwa_set_base.h"
#include "hwa_set_inp.h"
#include "hwa_set_turboedit.h"
#include "hwa_base_log.h"
#include "hwa_base_file.h"
#include "hwa_base_fileconv.h"
#include "hwa_gnss_data_turboedit.h"
#include "hwa_set_gbase.h"
#include "hwa_set_rec.h"
#include "hwa_gnss_all_ambflag.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_nav.h"
#include "hwa_base_allproc.h"
#include "hwa_gnss_proc_sppflt.h"
#include "hwa_gnss_proc_qualitycontrol.h"
#include "hwa_gnss_coder_satparam.h" // For LEO GLO
#include "hwa_gnss_coder_ambflag.h"
#include <algorithm>
#include <stdint.h>
#include <string>
#if defined _WIN32 || defined _WIN64
#include <io.h>
#include <direct.h>
#define ACCESS(fileName, accessMode) _access(fileName, accessMode)
#define MKDIR(path) _mkdir(path)
#else
#include <unistd.h>
#include <sys/stat.h>
#define ACCESS(fileName, accessMode) access(fileName, accessMode)
#define MKDIR(path) mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /*
    * Data flag
    * -2147483648    NODATA
      -1073741824    NO4
        536870912    LOWELE
                8    MWjump
               32    gap
        268435488    short
    */
    // ---------------------
    enum DATAFLAG
    {

        GOOD = 0,
        GFJUMP = 2, ///< ambiguity inserted due to jump in ionosphere-lane
        MWJUMP = 3, ///< ambiguity inserted due to jump in wide-lane
        GAP = 5,    ///< ambiguity inserted after long gap
        LLI = 8,    ///< ambiguity inserted accoording to LLI in rinex-file

        PCBAD = 16, ///< not used because of range is bad

        NOBRD = 22, ///< not used because not broadcast ephemeris is available
        GFBAD = 24, ///< not used because ionosphere - lane is bad
        MWBAD = 25, ///< not used because wide - lane is bad

        ASHADOW = 27, ///< not used because II / IIA after shadow
        SHRT = 28,    ///< not used because data piece is too short
        LOWELE = 29,  ///< not used because of lower elevation
        NO4 = 30,     ///< not used because the data std::set is not complete
        NODATA = 31,  ///< No data in rinex-file
    };

    /** @brief Specified sat data for post-processing turboedit. */
    class gnss_proc_TURBO_POST
    {
    public:
        /** @brief default constructor. */
        gnss_proc_TURBO_POST(){};
        /** @brief default destructor. */
        virtual ~gnss_proc_TURBO_POST(){};

        std::map<int, int> iflag;            ///< epoch, flag_index
        std::map<int, double> elev;          ///< epoch, elevation
        std::map<int, std::map<int, double>> obs; ///< epoch, index, obs
        int last_epo;                   ///< last epoch
    };

    /** @brief Smooth information for real-time turboedit. */
    class t_gturbo_rt
    {
    public:
        /** @brief default constructor. */
        t_gturbo_rt(){};
        /** @brief default destructor. */
        virtual ~t_gturbo_rt(){};
        std::map<GOBS, bool> mwslip;                    ///< MW slip
        std::map<GOBS, bool> gfslip;                    ///< GF slip
        std::map<GOBS, base_time> last_time;              ///< system & time
        std::map<GOBS, int> epo_num;                    ///< Store the number of consecutive epochs
        std::map<GOBS, double> GF;                      ///< store GF value
        std::map<GOBS, std::pair<double, double>> smooth_MW; ///< store MW smooth value, std
        std::map<GOBS, std::vector<double>> origin_MWs;      ///< sort according _gap
    };

    /**
    * @brief main control class for caculate clk
    */
    class gnss_proc_exeturboedit
    {
    public:
        /**
         * @brief Construct a new t gexeturboedit object
         * @param[in]  std::set       std::setbase control
         * @param[in]  spdlog      logbase control
         * @param[in]  gobs      aobservation data
         * @param[in]  gnav      navigation data
         */
        gnss_proc_exeturboedit(set_base *set, base_log spdlog, gnss_all_obs *gobs, gnss_all_nav *gnav);
        /**
         * @brief Construct a new t gexeturboedit object
         * @param[in]  std::set       std::setbase control
         * @param[in]  spdlog      logbase control
         * @param[in]  gdata     all data
         */
        gnss_proc_exeturboedit(set_base *set, base_log spdlog,  base_all_proc *gdata);
        /**
         * @brief Construct a new t gexeturboedit object
         * @param[in]  std::set          std::setbase control
         * @param[in]  spdlog       logbase control
         * @param[in]  gdata        all data
         * @param[in]  site         current site
         */
        gnss_proc_exeturboedit(set_base* set, base_log spdlog,  base_all_proc* gdata, std::string site);
        /**
         * @brief Construct a new t gexeturboedit object
         * @param[in]  std::set          std::setbase control
         * @param[in]  spdlog       logbase control
         * @param[in]  gdata        all data
         * @param[in]  site         current site
         */
        gnss_proc_exeturboedit(set_base* set, base_log spdlog,  base_all_proc* gdata, std::string site, bool isBase);
        /**
         * @brief Destroy the t gexeturboedit object
         */
        virtual ~gnss_proc_exeturboedit();
        /**
         * @brief batch process for turboedit
         * @param[in]  site      station name
         * @param[in]  begT      begin time
         * @param[in]  endT      end time
         * @param[in]  sampling  sampling
         */
        bool ProcessBatch(std::string site, const base_time &begT, const base_time &endT, double sampling);
        /**
         * @brief std::set object
         * @param[in]  gobj      all object
         */
        void setOBJ(gnss_all_obj *gobj) { _allobj = gobj; };
        /**
         * @brief std::set observation
         * @param[in]  gobs      all observation
         */
        void setBS(gnss_all_obs *gobs) { _allobs = gobs; };
        /**
         * @brief std::set GLONASS frequency id
         * @param[in]  gsatpars  satellite infomation
         */
        void setGLOFreqID(gnss_all_satinfo *gsatpars); // For simulation data
        /**
         * @brief Set the site crd 
         * @param[in]  site      station name
         * @param[in]  crd       coordinate
         */
        void set_site_crd(std::string site, Triple crd); // std::set site crd added by xiongyun
        /**
         * @brief Set the site crd
         * @param[in]  site     station name
         * @return              the station coordinate
         */
        Triple get_site_crd(std::string site);
        /**
         * @brief Set the Single Epo Data
         * @param[in]  input_data one epoch data
         */
        void setSingleEpoData(std::vector<gnss_data_sats> *input_data);
        /**
         * @brief Get the Site Cycle Slip
         * @param[in]  site      station name
         * @param[in]  idx       index
         * @return std::shared_ptr<gnss_data_cycleslip> cycle slip
         */
        std::shared_ptr<gnss_data_cycleslip> getSiteCycleSlip(std::string site, int idx) { return _cycleslip[site][idx]; };

        base_log spdlog();
        void spdlog(base_log spdlog);

    protected:
        ///< currenct processing site
        int _crt_epo;
        bool _isBase = false;   ///< For Log file
        std::string _crt_rec = "";   ///< For Log file
        base_time _crt_time;
        set_base *_set; ///< std::set
        base_log _spdlog; ///< spdlog pointer

        ///< Global Settings
        int _epo_num;                     ///< number of epoch
        double _interval;                 ///< interval
        base_time _beg_time;                ///< begin time
        base_time _end_time;                ///< end time
        std::set<std::string> _sys_list;            ///< system list
        std::set<std::string> _sat_list;            ///< satellite list
        std::set<std::string> _rec_list;            ///< station list
        std::map<std::string, Triple> _rec_crds; ///< site coordinates

        ///< input data
        gnss_all_obs *_allobs;
        gnss_all_nav *_allnav;
        gnss_all_obj *_allobj; ///< For sppflt

        ///< Post-processing Settings
        gnss_all_ambflag* _allambflag;          ///< save_ambflag
        std::map<int, base_time> _post_time;
        std::map<std::string, int> _glonass_freq_id;   ///< glonass frequency id
        std::map<std::string, double> _scaling_factor; ///< For check large cycle, post-processing
        std::map<std::string, gnss_proc_sppflt*> _sppflt;    ///< sppflt, provide init crd
        std::map<GSYS, std::vector<GOBS>> _post_phase_obs; ///< std::set phase observation type
        std::map<std::string, gnss_proc_TURBO_POST> _post_data;
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index;
        std::map<GSYS, std::pair<GOBSBAND, GOBSBAND>> _post_crt_band;           ///< std::set current post-processing bands
        std::map<std::string, std::vector<std::shared_ptr<gnss_data_cycleslip>>> _cycleslip;
        std::map<int, std::map<GSYS, std::pair<GOBSBAND, GOBSBAND>>> _post_all_band; ///< std::set post-processing bands

        ///< Real-time processing mode
        int _frequency;
        int _smooth_window;
        OBSCOMBIN _observ;
        base_iof* _debug_turbo = nullptr;
        base_iof* _cycleslipfile = nullptr;           // hlgou add 2022-05-07
        gnss_proc_OUTLIER _outliers_proc;
        std::map<std::string, std::map<std::string, std::map<GOBS, t_gturbo_rt>>> _rt_data;
        std::map<std::string, std::map<std::string, std::shared_ptr<gnss_data_obs_manager>>> _epoDataPre;

        //< And/F9p
        RECEIVERTYPE _receiverType;
        std::map<std::string, std::map<std::string, std::map<GOBSBAND, base_time>>> _epo_diff_time;
        std::map<std::string, std::map<std::string, std::map<GOBSBAND, std::vector<double>>>> _epo_diff;

        // for npp
        std::vector<gnss_data_sats>* _satdata_npp = nullptr;
        std::vector<std::shared_ptr<gnss_data_obs_manager>> _inputEpoData;
        std::map<std::string, std::map<std::string, int>> _map_nppdata_idx; // site sat idx

        ///< turboedit settings
        bool _realtime;       ///< whether realtime
        bool _liteMode;       ///< lite mode or not, same with real time mode
        bool _amb_output;     ///< whether output
        bool _ephemeris;      ///< whether use ephemeris, default true
        bool _simulation;     ///< whether simulation data, default False;
        bool _site_kinematic; ///< kinematic station is True, default False
        bool _check_pc;       ///< check PC, default
        bool _check_mw;       ///< check MW conbination (TURBOEDIT), default True
        bool _check_gf;       ///< check GF conbination (TURBOEDIT), default True
        bool _check_sf;       ///< check single frequency data, default is not.
        bool _check_gap;      ///< check data gap, ambiguity is inserted if data missing longer
        bool _check_short;    ///< check short arc, data piece shorter considered as short piece
                              //bool   _check_with_doppler; ///< check single frequency phase with doppler observations

        //double _delta_PL_limit;     ///< epoch-difference range - epoch-difference phase
        //double _delta_LD_limit;     ///< epoch-difference phase - integration of doppler observation
        int _gap_limit;         ///< epoch number, ambiguity is inserted if data missing longer, default 20 epoch
        int _short_limit;       ///< epoch number, data piece shorter considered as short piece, default 10 epoch
        int _min_mean_nprn;     ///< Minimum mean sat numbers (sum of obs_number(>4 sats) / epoch_number(> 4 sats) )
        int _max_mean_namb;     ///< Ambiguities Number / sat_number
        bool _check_statistics; ///< For Network Processing, PPP should Close; Whether consider following threshold, default not=false
        double _pc_limit;       ///< default 250m
        double _mw_limit;       ///< widelane limit, default 4 cycles
        double _gf_limit;       ///< geometry_free_limit, default 1 cycles       ///////// in real-time/lite mode , unit is meters
        double _gf_rms_limit;   ///< default 2 cycles
        double _sf_limit;       ///< default ????
        double _min_elev;       ///< cutoff elevation, default is 0
        double _min_percent;    ///< The percentage of epochs(>4 sats) in all useful epochs
        

        ///< Functions
        /**
        * @brief  output control informations
        */
        void _outputControlInfo();

        /**
        * @brief  initial function of processing turboedit
        */
        void _initCommonProcess();

        /**
        * @brief  initial function of post-processing turboedit
        */
        void _initRealTimeProcess();

        /**
        * @brief  main function of real-time processing turboedit
        */
        bool _realtimeProcess(const base_time &begT, const base_time &endT);

        /**
        * @brief  Turboedit Check[MW+GF] of real-time processing turboedit
        */
        void _realTimeMWGF(std::shared_ptr<gnss_data_obs_manager> oneObs, GOBSBAND b1, GOBSBAND b2);

        void _realTimeMWGF_test(std::shared_ptr<gnss_data_obs_manager> oneObs, GOBSBAND b1, GOBSBAND b2); //xiongyun test

        void _realTimeMWGF_new(std::shared_ptr<gnss_data_obs_manager> oneObs, GOBSBAND b1, GOBSBAND b2);

        void _checkSlipWithDoppler(std::shared_ptr<gnss_data_obs_manager> ObsPre, std::shared_ptr<gnss_data_obs_manager> Obs);

        void _checkSlipWithHighDiff(std::shared_ptr<gnss_data_obs_manager> Obs);

        /**
        * @brief  initial function of post-processing turboedit
        */
        void _initPostProcess();

        /**
        * @brief  main function of post-processing turboedit
        */
        bool _postProcess();

        /**
        * @brief  initial _post_data/_post_time for each station
        */
        void _initPostTurboInfo();

        /**
        * @brief  init SPPflt for kinematic stations
        */
        bool _siteSPP(const std::string &site, const base_time &now);

        /**
        * @brief  basic check is performed. Receiver clock bias is computed
        *         and corrected to DB%ti() if BRD is used.
        */
        void _postBasicCheck();

        /**
        * @brief  check observations vaid or not, as well as combine observation
        * @param[in] obsdata   single observation
        * @return              valid true; not false
        */
        bool _postPrepareObs(gnss_data_obs_manager &obsdata);

        /**
        * @brief  compute distance between satellite and station, satellite
        *         elevation, satellite clock bias,  using the specified brd
        *         record, or the nearest record.
        * @param[in] sat        sat name
        * @param[in] distance   distance between satellite and station, meters 
        * @param[in] elev       elevation, degree
        * @param[in] sat_clk    satellite clock bias, meters
        * @return               Ok is true; not is false
        */
        bool _getSatInfo(std::string sat, double &distance, double &elev, double &sat_clk);

        /**
        * @brief  Combination of observations
        * @param[in] sat   sat name
        */
        void _postCheckLargeSlips(std::string sat);

        /**
        * @brief  Check range residuals at the epoch for outliers
        */
        void _postCheckRange();

        /**
        * @brief  remove data piece with data no longer than len_short and 
        *         insert ambiguities if data gap longer than len_gap
        * @param[in] length_short  length of short piece, seconds
        * @param[in] sat           sat need to be remove short
        * @return                  false: removeShort work have been done; true: need further removement        
        */
        bool _postCheckGapShort(double length_short, std::string sat);

        /**
        * @brief  single-station GPS data editing based on the turboedit
        *         algorithm by J. Blewitt. 
        * @param[in] sat       sat name
        */
        void _postCheckMWGF(std::string sat);

        /**
        * @brief   edit M-W combined observations (Nw )
        * @param[in] sat       sat name
        */
        void _postEditWidelane(std::string sat);

        /**
        * @brief  check jumps in ionospheric observaitons
        * @param[in] sat       sat name
        */
        void _postCheckIonosphere(std::string sat);

        /**
        * @brief  check for jump using polynomial-fitting
        * @param[in]     strflag        name of time series
        * @param[in]     ti             time series (seconds)
        * @param[in]     values         values corresponding to time series
        * @param[in&out] flag           data flag
        * @param[out]    res            std::vector with residuals (diff values)
        * @param[out]    rms            unit middle error of residuals
        */
        bool _checkForJump(std::string strflag, const std::vector<double> &ti, const std::vector<double> &values,
                           std::vector<int> &flag, std::vector<double> &res, double &rms);

        /**
        * @brief  approximation of a std::set of discrete functional values
        *         by a polynomial and determining the integral of the
        *         polynomial using differences of consecutive values
        * @param[in]     degree        degree of polynomial
        * @param[in]     X             std::vector with independent arguments
        * @param[in]     Y             std::vector with functional values
        * @param[in]     L             l(i)=0: value # i is used for approx.
        *                                  =1: value # i is not used for approx.
        *                          =2,3,4,...: value # i is not used at all                        
        * @param[in&out] C             std::vector with coefficients of polynomial
        * @param[out]    V             std::vector with residuals (diff approx-y)
        * @param[out]    ipt           used data flagged as its position
        * @param[out]    rms           unit middle error of residuals
        */
        bool _polynomialFit(int degree, const std::vector<double> &X, const std::vector<double> &Y, const std::vector<int> &L,
                            std::vector<double> &C, std::vector<double> &V, std::vector<int> &ipt, double &rms);

        /**
        * @brief   write the report in spdlog files
        * @param[out] decision   decide whether remove short again
        */
        void _postPrintReport(std::string &decision);

        /**
        * @brief   add ambflag information of a site to _all_ambflag (ambflag decoder)
        * @return     success or not
        */
        bool _postSetAmbflag();

        /**
        * @brief  std::set ambflag data to _all_ambflag
        * @param[in]  sat_epo_beg_end       ambflag infomation[sat/valid beg epo/valid end epo]
        * @return     success or not
        */
        bool _postSetAmbflagData(std::map<std::string, std::map<int, int>> &sat_epo_beg_end);

        /**
        * @brief  get flag description(written in ambflag files)
        * @param[in]  sat         sat name
        * @param[in]  str3        AMB/DEL/BAD
        * @param[in]  epo         current epoch
        * @return                 descriptions
        */
        std::string _strFlagDescription(std::string sat, std::string str3, int epo);

        /**
        * @brief  Encoder ambflag files
        * @param[in]  logsuffix   suffix of ambflag(e.g., spdlog) file
        * @return                 success or not
        */
        bool _postEncoderProduct(std::string logsuffix);

        ///< Tool Functions
        /**
        * @brief  std::set data flag to the kind what the std::string described.
        * @param[in] iflag  original iflag
        * @param[in] type   std::string description
        * @return           new iflag
        */
        int _setFlag(int iflag, std::string type);

        /**
        * @brief  find the next epoch with the flag means `flag_type`
        * @param[in] istart     start epoch
        * @param[in] istop      end epoch. Search is done from istart to istop
        * @param[in] flg        flags
        * @param[in] flag_type  std::string flag
        * @param[in] direction  "+" means forward, while "-" means backward
        * @return               return found epoch, if can't find, return -999
        */
        int _findFlag(int istart, int istop, const std::vector<int> &flg, std::string flag_type, std::string direction);

        /**
        * @brief  check the data flag and std::string-decribtion are consistant. If yes, return true.
        * @param[in] iflag  input iflag
        * @param[in] type   std::string description
        * @return           true are consistent, otherwise is not
        */
        bool _isTrue(int iflag, std::string type);

        /**
        * @brief  search no more than 5 points are needs forwards and backwards to check LG Jump
        * @param[in] flg        searched flags
        * @param[in] crt_epo    current epoch
        * @param[in] limit_epo  std::set Search range (if "-",use_epo > limit_epo ; if "+", use_epo < limit_epo)
        * @param[in] mode       "+"  means forward, while "-" means backward
        * @param[in] use_epo    result
        * @return               number of eligible epochs
        */
        int _searchEpoch(const std::vector<int> &flg, int crt_epo, int limit_epo, std::string mode, int &use_epo);

        /**
        * @brief     transform
        * @param[in] data_flag       std::string format of data_flag
        * @return                    DATAFLAG type of data_flag
        */
        int _str2DataFlag(std::string data_flag);
    };

}

#endif /*GTURBOEDIT_H*/