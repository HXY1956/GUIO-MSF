/**
* @file            glsqproc.h
* @brief        virtual class for process
*/

#ifndef hwa_gnss_proc_lsq_H
#define hwa_gnss_proc_lsq_H

#include "hwa_base_data.h"
#include "hwa_set_amb.h"
#include "hwa_base_allproc.h"
#include "hwa_gnss_all_prod.h"
#include "hwa_gnss_all_obj.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_SATPARAM.h"
#include "hwa_gnss_all_pcvneq.h"
#include "hwa_gnss_all_recover.h"
#include "hwa_gnss_all_ambupd.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_data_cycleslip.h"
#include "hwa_gnss_data_upd.h"
#include "hwa_gnss_data_ion.h"
#include "hwa_gnss_data_ambcon.h"
#include "hwa_gnss_data_navde.h"
#include "hwa_gnss_data_leapsecond.h"
#include "hwa_gnss_data_ifcb.h"
#include "hwa_gnss_model.h"
#include "hwa_gnss_model_comb.h"
#include "hwa_gnss_amb_ambiguity.h"
#include "hwa_gnss_proc_lsqbase.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    typedef std::vector<std::tuple<std::string, std::string, std::string>> hwa_vector_gnss_lsq_EQU_INFO;

    /**
    * @brief virtual class for process
    */
    class gnss_proc_lsq
    {
    public:
        /**
         * @brief Construct a new t glsqproc object
         */
        gnss_proc_lsq();
        /**
         * @brief Construct a new t glsqproc object
         * @param[in]  std::set       std::setbase control
         * @param[in]  data      data
         * @param[in]  spdlog       logbase control
         */
        gnss_proc_lsq(set_base *set,  base_all_proc *data, base_log spdlog);
        /**
         * @brief Destroy the t glsqproc object
         */
        virtual ~gnss_proc_lsq();
        /**
         * @brief batch process for lsq
         * @param[in]  data      data
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         */
        virtual bool ProcessBatch( base_all_proc *data, const base_time &beg, const base_time &end);
        /**
         * @brief generate function
         */
        virtual bool GenerateProduct();
        /**
         * @brief get observation interval
         * @return double interval
         */
        double obs_intv() { return _obs_intv; };
        /**
         * @brief get satellite list
         * @return std::set<std::string>& satellite list
         */
        std::set<std::string> &sat_list() { return _sat_list; };
        /**
         * @brief get station list
         * @return std::set<std::string>& station list
         */
        std::set<std::string> &rec_list() { return _rec_list; };
        /**
         * @brief get system list
         * @return std::set<std::string>& system list
         */
        std::set<std::string> &sys_list() { return _sys_list; };
        /**
         * @brief Get the spdlog
         * @return t_glog* logbase control
         */
        base_log spdlog() { return _spdlog; };
        /**
         * @brief Get the beg time
         * @return base_time begin time
         */
        base_time get_beg_time() { return _beg_time; };
        /**
         * @brief Get the end time
         * @return base_time end time
         */
        base_time get_end_time() { return _end_time; };
        /**
         * @brief Get the crt time
         * @return base_time current time
         */
        base_time get_crt_time() { return _crt_time; };
        /**
         * @brief Set the crt time
         * @param[in]  time      current time
         */
        void set_crt_time(const base_time &time) { _crt_time = time; };
        /**
         * @brief Set the end time
         * @param[in]  time      end time
         */
        void set_end_time(const base_time &time) { _end_time = time; };
        /**
         * @brief Set the beg time
         * @param[in]  time      begin time
         */
        void set_beg_time(const base_time &time) { _beg_time = time; };

    protected:
        std::vector<std::string> _rover_list;  ///< rover sites for PPPRTK
        base_log _spdlog;            ///< spdlog file for process
        set_base *_gset = nullptr; ///< setting from xml file

        /// The following can get from xml file
        std::set<std::string> _rec_list;            ///< all sites of proc
        std::set<std::string> _sat_list;            ///< all sats of proc
        std::set<std::string> _sys_list;            ///< sat systems of proc
        std::set<std::string> _leo_list;            ///< short name of leos
        std::map<std::string, Triple> _rec_crds; ///< site coordinates
        std::map<std::string, Triple> _rec_stds; ///< site coordinates std

        bool _realtime;
        int _frequency = 0;
        double _obs_intv = 0.0;     ///< sampling of proc
        double _pwc_intv = 0.0;     ///< sampling of pwc
        double _grd_pwc_intv = 0.0; ///< sampling of pwc for gradients, glfeng
        base_time _beg_time;          ///< beg time of proc
        base_time _end_time;          ///< end time of proc
        base_time _crt_time;          ///< current epoch of proc

        std::set<std::pair<FREQ_SEQ, FREQ_SEQ>> _freq_pair;
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index;
        std::map<GSYS, int> _freq_number;

        LSQ_MODE _lsq_mode = LSQ_MODE::LSQ_MODE_UNDEF;
        OBSCOMBIN _obs_mode = OBSCOMBIN::DEF_OBSCOMBIN;
        CONSTRPAR _crd_est = CONSTRPAR::CONSTRPAR_UNDEF;
        FIX_MODE _fix_mode = FIX_MODE::NO;
        UPD_MODE _upd_mode = UPD_MODE::OSB;
        SLIPMODEL _slip_model = SLIPMODEL::SLIPMODEL_UNDEF;
        ZTDMODEL _ztd_model = ZTDMODEL::DEF_ZTDMODEL;
        SYSBIASMODEL _sysbias_model = SYSBIASMODEL::AUTO_WHIT;
        IFCB_MODEL _ifcb_model = IFCB_MODEL::IFCB_MODEL_UNDEF;
        IFB_model _ifb_model = IFB_model::NONE;
        GRDMODEL _grd_model = GRDMODEL::DEF_GRDMODEL; //glfeng
        bool _tropo_grad = false;                     //glfeng

        /// The following is public for all lsq
        std::string _crt_rec; ///< current site of proc
        std::string _crt_sat; ///< current sat  of proc

        gnss_all_obs *_gall_obs = nullptr;     ///< all observ info
        gnss_all_obj *_gall_obj = nullptr;     ///< all obj info include pcv and intial crd
        gnss_all_recover *_gall_rcv = nullptr; ///< all recover info
        gnss_all_bias *_gall_bias = nullptr;
        gnss_data_upd *_gall_upd = nullptr;
         base_all_proc *_gall_proc = nullptr; ///< all     the data used in proc
        gnss_all_nav *_gall_nav = nullptr;   ///< used in kin ppp
        gnss_all_prec *_gall_prec = nullptr;
        gnss_data_poleut *_gdata_erp = nullptr; ///< all poleut data for init ERP parameters
        gnss_data_ion *_gdata_ion = nullptr;
        gnss_data_ambcon *_gdata_ambcon = nullptr;
        gnss_all_satinfo *_gdata_satinfo = nullptr; ///< sat information for glonass fid
        gnss_proc_lsqbase *_glsq = nullptr;                 ///< least square estimator
        gnss_proc_lsqbase _glsqtmp;                         ///< least square estimator

        std::shared_ptr<gnss_data_cycleslip> _slip12 = nullptr; ///< turboedit class for IF
        std::shared_ptr<gnss_data_cycleslip> _slip13 = nullptr; ///< turboedit class for 13 if
        std::shared_ptr<gnss_data_cycleslip> _slip14 = nullptr;
        std::shared_ptr<gnss_data_cycleslip> _slip15 = nullptr;
        std::shared_ptr<gnss_model_base> _base_model = nullptr;
        std::shared_ptr<gnss_model_bias> _bias_model = nullptr;

        // record now process data
        gnss_proc_lsq_equationmatrix _crt_equ;

        double _recclk_threshold = 0.0;

        int _num_threads = 1;
        bool _matrix_remove = false;
        bool _is_prefix_res = false;
        bool _cmb_equ_multi_thread = false;
        bool _write_equ = true;

        int _remove_par_msec;
        int _cmb_equ_msec;

    protected:
        /// The lsq process module for override
        /**
         * @brief init all kinds of parameters
         * @param [in]  lsq        lsq infomation
         * @return
                *    @retval   false     correctly
                *    @retval   true        failed
         */
        bool _init_crs_erp_pars(gnss_proc_lsqbase *lsq);
        bool _init_crs_geoc_pars(gnss_proc_lsqbase *lsq); //added by HwangShih
        bool _init_rec_trop_pars(gnss_proc_lsqbase *lsq);
        bool _init_gns_vion_pars(gnss_proc_lsqbase *lsq);
        bool _init_gns_sion_pars(gnss_proc_lsqbase *lsq);
        bool _init_rec_clk_pars(gnss_proc_lsqbase *lsq);
        bool _init_sat_clk_pars(gnss_proc_lsqbase *lsq);
        bool _init_sagnss_coder_ifcb_pars(gnss_proc_lsqbase *lsq); //add by xiongyun
        bool _init_sys_bia_pars(gnss_proc_lsqbase *lsq);
        bool _init_sys_rec_ifb_pars(gnss_proc_lsqbase *lsq);
        bool _init_sys_sat_ifb_pars(gnss_proc_lsqbase *lsq);
        bool _init_sys_ifb_pars(gnss_proc_lsqbase *lsq);
        bool _init_sys_bia_pars_pce(gnss_proc_lsqbase *lsq);
        bool _init_rec_clk_1X_pars(const int &freq_num, gnss_proc_lsqbase *lsq);
        bool _init_IF_amb_1X_pars(const int &freq_num, gnss_proc_lsqbase *lsq);

        void _set_isb_par_setting(gnss_proc_lsqbase *lsq, base_par &par, SYSBIASMODEL model, double value, double apriori);

        // -- for whole
        /**
        * @brief init the product data class.
        * @param[in] data   the container for product data class.
        */
        virtual bool _initLsqProdData(gnss_all_prod *data);
        /**
        * @brief init the pars for processing.
        * @param[in] lsq   the class for lsq.
        */
        virtual bool _initLsqProcPars(gnss_proc_lsqbase *lsq);

        /**
        * @brief init the process data class.
        * @param[in] data   the container for process data class.
        */
        virtual bool _initLsqProcData( base_all_proc *data);
        /**
         * @brief select observation
         * @param[in]  epoch     current time
         * @param[in]  all_obs   all observsation
         */
        bool _select_obs(const base_time &epoch, std::vector<gnss_data_sats> &all_obs);
        /**
         * @brief init satellite clk for IONO free
         * @param[in]  crt_obs   current observation
         * @param[in]  l         the class for lsq
         * @param[in]  lsq
         */
        bool _init_sat_clk_IF(std::vector<gnss_data_sats> &crt_obs, Vector &l, gnss_proc_lsqbase *lsq);
        /**
         * @brief init satellite clk
         */
        bool _init_sat_clk();

        /**
        * @brief extract clkfile from recover data
        * @return success or fail
        */
        bool _extract_clkfile(gnss_all_recover &recover_data, gnss_all_prec::clk_type type);
        bool _extract_clkfile(gnss_all_recover &recover_data, gnss_all_prec::clk_type type, GSYS gsys, par_type clk_type);
        bool _extract_clkfile(gnss_all_recover &recover_data, gnss_all_prec::clk_type type, bool isTriple);
        bool _extract_resfile(gnss_all_recover &recover_data, std::string resfile = "");
        bool _extract_ambupd(gnss_all_ambupd *allambupd);
        bool _extract_ion_file(gnss_all_recover &recover_data);
    };

    /**
    * @brief check code range res
    * @param[in] omc all compute res of equations
    * @param[out] recclk  add average of res to recclk
    * @param[out] sigma  sigma of res
    * @return number of use res
    */
    int Check_Range(std::vector<double> &omc, double &recclk, double &sigma);

}
#endif