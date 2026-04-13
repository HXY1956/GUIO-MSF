#ifndef hwa_gnss_proc_pvtflt_h
#define hwa_gnss_proc_pvtflt_h

#include "hwa_base_eigendef.h"
#include "hwa_gnss_proc_pppflt.h"
#include "hwa_gnss_proc_exeturboedit.h"
#include "hwa_gnss_proc_qualitycontrol.h"
#include "hwa_gnss_model_ppp.h"
#include "hwa_gnss_amb_ambiguity.h"
#include "hwa_base_posdata.h"
#include "hwa_gnss_model_comb.h"
#include "hwa_gnss_proc_preproc.h"
#include "hwa_gnss_model_iono.h"
#include "hwa_gnss_proc_updateparif.h"
#include "hwa_gnss_proc_updateparall.h"
#include "hwa_gnss_data_aug.h"
#include "hwa_gnss_data_interpol.h"
#include "hwa_gnss_data_obj.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    * @brief class for gpvtflt, derive from gpppflt
    */
    class gnss_proc_pvtflt : public gnss_proc_pppflt
    {
    public:
        /** @brief default constructor. */
        gnss_proc_pvtflt(std::string mark, std::string mark_base, set_base *set,  base_all_proc *allproc = nullptr);

        /**
        * @brief Constructor
        * @note std::set parameter value by setS
        * 
        * @param[in] mark            mark
        * @param[in] std::set            std::setbase
        * @param[in] allproc        all process
        */
        explicit gnss_proc_pvtflt(std::string mark, std::string mark_base, set_base *set, base_log spdlog, base_all_proc *allproc = nullptr);

        /** @brief default destructor. */
        virtual ~gnss_proc_pvtflt();

        /** @brief processBatch. */
        virtual int processBatch(const base_time &beg, const base_time &end, bool prtOut);

        /** @brief Initializing some settings. */
        virtual bool InitProc(const base_time &begT, const base_time &endT, double *subint = NULL);

        /**
        * @brief One epoch processing
        * @note virtual function,lvhb Created for NPP
        * @param[in] Now    current time
        * @param[in] data_rover
        * @param[in] data_base
        * @return
               -1,failed; 
                0,float;
                1,fixed.
        */
        virtual int ProcessOneEpoch(const base_time &now, std::vector<gnss_data_sats> *data_rover = NULL, std::vector<gnss_data_sats> *data_base = NULL);

        /**
        * @brief Get Uncombined Parameters
        * @note lvhb added for NPP
        * @param[out] data_base
        * @return Uncombined parameter
        */
        base_allpar &getUCparam(std::set<std::string> *satref = NULL); // For NPP,lvhb

        /**
        * @brief Forward and Backward process
        * @note virtual function
        * @param[in] beg    begin time
        * @param[in] end    end time
        * @return
            retval > 0.   success
            retval < 0.   fail
        */
        virtual int processBatchFB(const base_time &beg, const base_time &end);

        virtual void Add_UPD(gnss_data_upd *gupd);

        /** @brief calculate azel and rho
        * @Note,lvhb modified for PVT/NPP
        * @param[in] current site name
        * @param[in] current satellite position
        * @param[in] current site  position
        * @param[in/out] current site  data
        * @return void */
        void Add_rho_azel(/*const base_time &runEpoch, */ const std::string &site_name, Triple &xyz_s, const Triple &xyz_r, gnss_data_sats &obs_sat);

        /** @brief Setting given reference satellites.
        * @Note, lvhb added for NPP in 202010 
        * @param[in] current reference satellites
        */
        void setefsat(std::map<GSYS, std::string> satref) { _ipSatRep = satref; }

        void segnss_coder_aug(hwa_map_id_augtype_value *aug)
        {
            if (aug)
            {
                _sagnss_coder_aug = *aug;
                _exist = true;
            }
            else
            {
                _exist = false;
            }
        };

        /** @brief some interface for cps
        * @Note, tyx added for CPS in 202112
        */
        bool grec_exist() { return _grec != nullptr; };
        void grec_clear();

        bool slip_detect(const base_time& now) { return _slip_detect(now); };

        bool interpolAug(const base_time& now) { return _gInterpol->interpolAug(now, _data, _data_base); };

        Triple get_interpolcrd() { return _gInterpol->get_crd_rover(); };

        void set_success(bool success) { _success = success; };

        void get_result(base_time &epo, base_posdata::data_pos &pos) { _get_result(epo, pos); };

        void get_result(base_time &epo, base_posdata::rtk_pos &pos) { _get_result(epo, pos); };

        void get_result(base_time &epo, Vector & pos) { _get_result(epo, pos); }

        int set_Crd(const base_time& t) {  return _setCrd(t); };

        bool isBase() { return _isBase; };

        bool getexeturboedit(std::shared_ptr<gnss_proc_exeturboedit>& tb);

        bool get_amb_state() { return _amb_state; }

        double get_ratio() { return _ambfix->get_ratio(); }

    protected:
        /*************** RTK process function,created by lvhb *****************/

        /**  @brief initiate RTK parameters */
        int _rtkinit();

        /**  @brief Setting base or rover coordinates in SPP/RINEXH/FIX */
        int _setCrd();

        int _setCrd(const base_time& t);

        /**  @brief update RTK SD ambiguity parameters */
        void _udsdAmb();

        /** @brief lvhb added to adapting baseline changing in 20200821. */
        virtual void _delPar(const par_type par);

        /**
        * @brief select common sats between base and rover sites,created by lvhb
        * @param[in/out] base-site satellite data
        * @param[in/out] rover-site satellite data
        * @return number of common sats
        */
        int _selcomsat(std::vector<gnss_data_sats> &data_base, std::vector<gnss_data_sats> &data_rover);

        /**  @brief To jude whether exist UD Phase/Range residuals in current satellite 
        * return true: exist
                 false: no exist
        */
        bool _valid_residual(bool phase_process, std::string sat_name, enum FREQ_SEQ &f, std::map<std::string, std::map<FREQ_SEQ, std::pair<int, int>>> &index_l);

        /**
        * @brief add pseudorange and carrier-phase measurement to model. Rewritten for RTK application.
        * @note,Lvhb created  for RTK/SD-PPP
        * @param[in/out] A            Coff Matrix
        * @param[in/out] l            measurement std::vector
        * @param[in/out] P            measurement variance
        * @return
            retval > 0.   success
            retval < 0.   fail
        */
        int _combineDD(Matrix &A, Symmetric &P, Vector &l);

        /*************** rtk process /end *****************/

        /** @brief kalman filter time update. */

        void _syncAmb();

        /**  @brief update PPP UD ambiguity parameters(zzwu) */
        void _udAmb();

        void _post_turbo_syncAmb();

        int _get_spplsq_result(std::string site, base_time current, Triple &crd, double &clk);

        /*obtain satcrd,satclk,satele,satazi,rho, it must be two bands*/
        virtual int _preprocess(const std::string &ssite, std::vector<gnss_data_sats> &sdata); //rename by zzwu

        /** @brief prepare data.    */
        virtual int _prepareData();

        virtual int _processEpoch(const base_time &runEpoch);

        virtual int _amb_resolution();

        bool _getSatRef();

        virtual int _outlierDetect(const Vector &v, const Symmetric &Qsav, std::string &sat);
        virtual int _outlierDetect_lhb(const Vector &v_post, const Vector &v_norm, const Symmetric &Qsav, std::string &sat);

        virtual std::string _getsat_from_obs(int idx);

        /**
        * @brief add doppler measurement to model
        *
        * @param[in] satdata    satellite data
        * @param[in] iobs        index
        * @param[in] XYZ        ECEF coordinate
        * @param[out] A            Coff Matrix
        * @param[out] l            measurement std::vector
        * @param[out] P            measurement variance
        * @return
            retval > 0.   success
            retval < 0.   fail
        */
        int _addObsD(gnss_data_sats &satdata, unsigned int &iobs, base_allpar &param, Triple &XYZ, Matrix &A, Vector &l, Diag &P);

        /** @brief lvhb added post residual for rtk/ins */
        int _postRes(const Matrix &A, const Symmetric &P, const Vector &l, Matrix &pA, Symmetric &pP, Vector &pl);

        /** @brief lvhb modified for PPP/RTK processing */
        virtual int _combineMW(gnss_data_sats &satdata);

        virtual bool _external_pos(const Triple &xyz_r, const Triple &rms);
        virtual bool _pos_virtual_obs(const Triple &xyz_r, const Triple &rms, Matrix &A, Vector &l, Symmetric &P);

        /** @brief process one epoch Velocity. */
        virtual int _processEpochVel();

        void _print_ele();

        virtual int _prt_info(const base_time &runEpoch);
        virtual std::string _gen_kml_description(const base_time &epoch, const base_posdata::data_pos &pos);
        virtual std::string _quality_grade(const base_posdata::data_pos &pos);

        /** @brief kalman filter time update. */
        virtual void _predict(const base_time& runEpoch); //zzwu change because of ion_constraint

        /** @brief print  the result. */
        virtual void _prtOut(base_time &epo, base_allpar &X, const Symmetric &Q, std::vector<gnss_data_sats> &data, std::ostringstream &os, xml_node &node, bool saveProd = true);

        virtual void _prt_port(base_time &epo, base_allpar &X, const Symmetric &Q, std::vector<gnss_data_sats> &data);

        void _get_result(base_time &epo, base_posdata::data_pos &pos);

        void _get_result(base_time &epo, base_posdata::rtk_pos &pos);

        void _get_result(base_time &epo, Vector &pos);                  //get spp results (tyx added for cps)

        /** @brief print  the result. */
        void _prtOutHeader();

        void _combineEWL(gnss_data_sats &satdata);

        void _equationToMatrix(gnss_proc_lsq_equationmatrix &equ, Matrix &A, Symmetric &P, Vector &l,
                               unsigned int nPar);
        void _generateObsIndex(gnss_proc_lsq_equationmatrix &equ);

        void _extern_ion_constraint(const base_time &runEpoch);

        bool _calculate_timediff_baseline(const std::string &site, const base_time &epoch0, const base_time &epoch1, Triple &baseline);

        bool _post_turbo_select_obs(const base_time &epoch, std::vector<gnss_data_sats> &data);

        bool _addconstraint(Matrix &A, Symmetric &P, Vector &l);

        bool _addconstraint(gnss_proc_flt *filter);

        //all below add by zzwu
        bool _available_data_realtime(const base_time &now);

        bool _slip_detect(const base_time& now);

        int _getData(const base_time& now, std::vector <gnss_data_sats>* data, bool isBase);

        bool _crd_xml_valid();  //avoid same name with _valid_crd_xml

        void _remove_sat(const std::string &satid);

        bool _check_sat(const std::string& ssite, gnss_data_sats * const iter, Matrix &BB, int &iobs);

        bool _cmp_rec_crd(const std::string& ssite, Matrix& BB);

        bool _cmp_sat_info(const std::string& ssite, gnss_data_sats* const iter);

        void _predictCrd();

        void _predictClk();

        void _predictBias();

        void _predictIono(const double &bl, const base_time& runEpoch);

        void _predictTropo();

        void _predictAmb();

        unsigned int _cmp_equ(gnss_proc_lsq_equationmatrix &equ);

        void _posterioriTest(const Matrix& A, const Matrix& R, const Vector& l,
            const Vector& dx, const Symmetric& Q, Vector& v_norm, double& vtpv);
        void _posterioriTest(const Matrix& A, const Symmetric& P, const Vector& l,
            const Vector& dx, const Symmetric& Q, Vector& v_norm, double& vtpv);

    protected:
        gnss_model_white_noise *_dclkStoModel; /// clock drift model
        gnss_model_white_noise *_velStoModel;     /// velocity model

        base_time _beg_time; ///< beg time of proc
        base_time _end_time; ///< end time of proc
        bool _isFirstFix;
        gnss_data_upd *_gupd = nullptr;
        gnss_ambiguity *_ambfix = nullptr;
        FIX_MODE _fix_mode;
        UPD_MODE _upd_mode = UPD_MODE::UPD;
        hwa_map_mw _MW, _EWL, _LW, _LE, _LWL, _rMW, _rEWL, _IMW, _IEWL, _PW, _AFIF;
        bool _amb_state;
        int _fixed_pct;
        base_allpar _param_fixed;
        Triple _vel;
        Symmetric _Qx_vel;
        std::map<std::string, double> _crt_ele;
        std::map<std::string, std::map<FREQ_SEQ, double>> _crt_SNR; //
        std::map<std::string, std::map<FREQ_SEQ, int>> _crt_ObsLevel;

        double _max_res_norm; // maximum normalize res
        bool _isBase;          /**<if using base,true; other: fasle */
        bool _sd_sat;          // single differented between sat and sat_ref
        bool _pos_constrain;
        bool _motion_model;
        Triple _extn_pos, _extn_rms; // external position and its rms
        Matrix _post_A;
        Symmetric _post_P;
        Vector _post_l; // post A P L for RTK

        std::string _site_base;               ///< base site name;
        std::vector<gnss_data_sats> _data_base; ///< base data in every epoch;
        std::vector<std::pair<std::string, std::pair<FREQ_SEQ, GOBSTYPE>>> _obs_index;
        gnss_model_ppp *_gModel_base;
        std::set<std::string> _sat_ref;
        //std::map<GSYS, std::map<FREQ_SEQ, std::string>> _sat_refs;//lvhb expended to setting in every freq
        std::map<GSYS, std::string> _ipSatRep; ///< input reference satellite,lvhb
        // for RAW_ALL PPP   : G01, AMB_L1, L1C   X   X   X
        // for IONO_FREE PPP : G01, AMB_IF, L1C L2W   X   X
        // for RAW_ALL RTK   : G01, AMB_L1, L1C   X L1C   X
        // for IONO_FREE RTK : G01, AMB_IF, L1C L2W L1C L2W
        std::map<std::pair<std::string, par_type>, std::tuple<GOBS, GOBS, GOBS, GOBS>> _amb_obs;

        Vector _vBanc_base;
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index;
        int _frequency;
        // PRECISEMODEL STUFF
         base_all_proc *_allproc = nullptr;
        std::shared_ptr<gnss_model_base> _base_model = nullptr;
        std::shared_ptr<gnss_proc_update_par> _update_AMB = nullptr;
        std::shared_ptr<gnss_data_cycleslip> _slip12 = nullptr; ///< turboedit class for IF
        std::shared_ptr<gnss_data_cycleslip> _slip13 = nullptr; ///< turboedit class for 13 if
        std::shared_ptr<gnss_data_cycleslip> _slip14 = nullptr;
        std::shared_ptr<gnss_data_cycleslip> _slip15 = nullptr;

        //preprocess
        SLIPMODEL _slip_model;
        bool _turbo_liteMode;
        std::shared_ptr<gnss_proc_exeturboedit> _gturboedit;
        std::shared_ptr<gnss_proc_preproc> _gpre;

        // glfeng
        gnss_proc_quality_control _gquality_control;
        gnss_data_ionex *_gionex_GIM;
        bool _iono_constraint;
        int _epoch_cout;

        //for npp,lvhb 20200917
        bool _isClient = false;
        hwa_SPT_interpol _gInterpol;

        // add for realtime
        base_time _wl_Upd_time;
        base_time _ewl_Upd_time;
        NPP_MODEL _nppmodel;

        bool _realtime;
        bool _reset_amb_ppprtk = false;
        hwa_map_id_augtype_value _sagnss_coder_aug;
        bool _exist;
        bool _isCompAug;
        bool _isCorObs;
        int _realnobs;
        std::map<base_time, std::map<std::string, int>> _sagnss_coder_aug_map; // 0 false; 1 true
        RECEIVERTYPE _receiverType;
        std::map<std::string, std::string> _sat_freqs;
        int _obs_level = 3;
    };

    /** @brief Test by lvhb 201906 */
    template <class T1, class T2>
    void t_out(T1 const &name, T2 const &matrix)
    {
        std::cout << name << std::endl;
        std::cout << std::fixed << std::setprecision(6) << std::setw(15) << matrix << std::endl;
        return;
    }
}

#endif