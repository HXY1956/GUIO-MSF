/**
* @file             gexemultipath.h
* @brief            main control class for multi-path calculate
*/

#ifndef hwa_gnss_proc_EXEMULTIPATH_H
#define hwa_gnss_proc_EXEMULTIPATH_H

#include "hwa_set_base.h"
#include "hwa_set_inp.h"
#include "hwa_set_turboedit.h"
#include "hwa_set_gen.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_file.h"
#include "hwa_gnss_proc_exeturboedit.h"
#include "hwa_gnss_proc_sppflt.h"
#include "hwa_gnss_proc_qualitycontrol.h"
#include "hwa_gnss_data_turboedit.h"
#include "hwa_set_gbase.h"
#include "hwa_set_rec.h"
#include "hwa_gnss_coder_satparam.h" // For LEO GLO
#include "hwa_gnss_coder_ambflag.h"
#include "hwa_gnss_all_ambflag.h"
#include "hwa_gnss_all_Obs.h"
#include "hwa_gnss_all_Nav.h"
#include "hwa_base_allproc.h"
#include <algorithm>
#include <stdint.h>
#include <string>
#include <io.h>
#include <direct.h> 
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_gnss
{
    /**
    * @brief main control class for calculate multi-path
    */
    class gnss_proc_EXEMULTIPATH
    {
    public:
        /**
         * @brief Construct a new gnss_proc_EXEMULTIPATH object
         * @param[in]    site        site name
         * @param[in]    std::set         std::setbase control
         * @param[in]    spdlog      logbase control
         * @param[in]    gdata       aobservation data
         */
        gnss_proc_EXEMULTIPATH(std::string site, set_base* set, base_log spdlog,  base_all_proc* gdata);
        /**
        *@brief Destroy the gnss_proc_EXEMULTIPATH object
        */
        virtual ~gnss_proc_EXEMULTIPATH();

        /**
         * @brief batch process for Extact_Noise
         * @param[in]  site         station name
         * @param[in]  begT         begin time
         * @param[in]  endT         end time
         */
        bool ProcessBatch(std::string site, const base_time& begT, const base_time& endT);
        /**
         * @brief batch process for multipath
         * @param[in]  site         station name
         * @param[in]  begT         begin time
         * @param[in]  endT         end time
         */
        bool ExtractMP(std::string site, const base_time& begT, const base_time& endT);
        /**
         * @brief batch process for Phase and Code Noise
         * @param[in]  site         station name
         * @param[in]  begT         begin time
         * @param[in]  endT         end time
         */
        bool ExtractNoise(std::string site, const base_time& begT, const base_time& endT);


    protected:        
        set_base*                         _set;               ///< std::set
        base_log                            _spdlog;            ///< spdlog pointer
        int                                 _epo_num;
        int                                 _interval;
        int                                 _minimum_elev;      ///< Cut-off Elevation, default=10
        base_time                             _beg_time;
        base_time                             _end_time;
        std::set<std::string>                         _sys_list;
        std::set<std::string>                         _sat_list;
        std::set<std::string>                         _rec_list;
        std::vector<std::string>                      _rec_list_base;
        std::map<std::string, Triple>              _rec_crds;          ///< site coordinates
        std::map<base_time,std::map<std::string,double>>     _epo_avaisatlist;   ///< the available satellites of the current epoch.
        base_time                             _crt_time;
        std::string                              _crt_rec; 
        std::string                              _site_base;
        bool                                _site_kinematic;    ///< kinematic station is True, default False
        std::map<base_time, std::map<std::string, std::map<int, double>>> _mp_orig;       ///< [epo_num1,band1_mp,epo_num2,band2_mp,epo_num3,band3_mp]
        std::map<base_time, std::map<std::string, std::map<int, double>>> _mp_avearc;     ///< average in an arc
        OBSCOMBIN                           _obscomb;
        std::vector<gnss_data_sats>                  _data;
        std::vector<gnss_data_sats>                  _data_base;
        std::shared_ptr<hwa_gnss::gnss_proc_exeturboedit>  _gturboedit;
        std::map<std::string, gnss_proc_sppflt*>             _sppflt;            ///< sppflt, provide init crd
        ///< input data
        gnss_all_obs*                          _allobs;
        gnss_all_nav*                          _allnav;
        gnss_all_obj*                          _allobj;            ///< For sppflt
        ///< output file
        base_iof*                             _multipathfile = nullptr;
        base_iof*                             _cycleslipflagfile = nullptr;
        base_iof*                             _noisefile = nullptr;
        std::map< GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;
        std::unordered_map <std::string, std::map<GOBSBAND, std::vector<double>>>  _res_l, _res_d; // <sat, <epoch,phase> >  -  Three epochs difference res
        std::map<base_time, std::map<std::string, std::map<GOBSBAND, std::map<int, double>>>>  _noise_code, _noise_phase;
        std::map<base_time, std::map<std::string, std::map<GOBSBAND, std::map<int, double>>>>  _noise_phase_base;

        /**
        * @brief  combine multipath
        * @param[in] obsdata    single sat observation
        * @return               int, success or fail
        */
        int _combineMP(gnss_data_sats& satdata);    //added by fyc
        /**
        * @brief  prepare before multi-path combination
        * @param[in] ssite      the site name
        * @param[in] sdata      single sat observation
        * @return               int, success or fail
        */
        int _prepareDataMP(const std::string& ssite, std::vector<gnss_data_sats>& sdata);
        /**
        * @brief  get the satellite position
        * @param[in] epo        the current epoch time
        * @param[in] gsatdata   single sat observation
        * @return               int, success or fail
        */
        int _satPos(base_time& epo, gnss_data_sats& gsatdata);
        /**
        * @brief  get the satellite position
        * @param[in] site_name  the site name
        * @param[in] xyz_s      the coordinate of the satellite
        * @param[in] xyz_r      the coordinate of the receiver
        * @param[in] obs_sat    single sat observation
        * @return               void
        */
        void _add_rho_azel(const std::string& site_name, Triple& xyz_s, const Triple& xyz_r, gnss_data_sats& obs_sat);

        /**
        * @brief  get the satellite information
        * @param[in] sat        the satellite prn
        * @param[in] distance   the distance between the satellite and the site
        * @param[in] elev       the elevation of the satellite
        * @param[in] sat_clk    the probable clock offset of the satellite
        * @return               bool, success or fail
        */
        bool _getSatInfo(std::string sat, double& distance, double& elev, double& sat_clk);
        /**
        * @brief  initial function of processing multipath
        */
        void _initCommonProcess();
        /**
        * @brief  get the site position by SPP
        * @param[in] site       the site name
        * @param[in] now        the current epoch time
        * @return               bool, success or fail
        */
        bool _siteSPP(const std::string& site, const base_time& now);
        /**
        * @brief  minus the average value of MP in the an arc, get _mp_avearc for post-processing.
        */
        bool _average_mp_post();
        /**
        * @brief  minus the average value of MP in the an arc, sub-function of _average_mp_post.
        * @param[in] crt_time   the current time
        * @param[in] prn        the prn of the satellite
        * @param[in] i          the index, 1/3/5 represent BAND_1/BAND_2/BAND_3
        * @return               bool, success or fail
        */
        bool _average_mp_post_freq(const base_time& crt_time, const std::string& prn, const int& i);
        /**
        * @brief  phase 3 difference for reover station
        * @param[in] now        the current time
        * @return               int, success or fail
        */
        int _phaseHighDiff(const base_time& now);
        /**
        * @brief  phase 3 difference for base station
        * @param[in] now        the current time
        * @return               int, success or fail
        */
        int _phaseHighDiffBase(const base_time& now);
        /**
        * @brief  code single difference between base and rover station.
        * @param[in] crt_time   the current time
        * @return               int, success or fail
        */
        int _codeSigleDiff_site(const base_time& crt_time);
        /**
        * @brief  phase 3 difference for single frequency of single satellite 
        * @param[in] it         satdata iterator
        * @param[in] band       the band
        * @param[in] d_res_l    the phase value after 3 difference 
        * @param[in] d_res_d    the doppler value after 3 difference 
        * @return               int, success or fail
        */
        int _phaseHighDiffFreq(std::vector<gnss_data_sats>::iterator& it, const GOBSBAND& band, double& d_res_l, double& d_res_d);
        /**
        * @brief  output Noise >> file
        * @return               int, success or fail
        */
        int _outputNoise();
        /**
        * @brief  output Noise Header >> file
        * @return               int, success or fail
        */
        int _outputNoise_header();
        /**
        * @brief  output the information >> file
        */
        int _outputMP();
        /**
        * @brief  output the header of MP file
        */
        int _outputMP_header();
        /**
        * @brief  output the mp information(not average in arcs) >> file(just for test)
        */
        int _outputMP_test();
        /**
        * @brief  output the slip flag >> file(just for test)
        */
        int _output_slip_flag();
    };
    
}

#endif