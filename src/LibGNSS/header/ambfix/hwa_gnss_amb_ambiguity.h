#ifndef hwa_gnss_amb_ambiguity_h
#define hwa_gnss_amb_ambiguity_h

#include "hwa_set_amb.h"
#include "hwa_set_out.h"
#include "hwa_set_npp.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_amb_lambda.h"
#include "hwa_gnss_amb_bdeci.h"
#include "hwa_gnss_amb_common.h"
#include "hwa_gnss_data_upd.h"
#include "hwa_gnss_data_cycleslip.h"
#include "hwa_gnss_proc_lsq.h"
#include "hwa_gnss_proc_lsqbase.h"
#include "hwa_gnss_proc_flt.h"
#include "hwa_gnss_sys.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    * @brief class for storing ambiguity resolution common value.
    */
    class gnss_amb_cmn
    {
    public:
        /**
         * @brief Construct a new t gamb cmn object
         */
        explicit gnss_amb_cmn();
        /**
         * @brief Construct a new t gamb cmn object
         * @param[in]  t         time
         * @param[in]  lsq       lsq class
         */
        explicit gnss_amb_cmn(const base_time &t, gnss_proc_lsqbase *lsq);
        /**
         * @brief Construct a new t gamb cmn object
         * @param[in]  t         time
         * @param[in]  flt       filter
         */
        explicit gnss_amb_cmn(const base_time &t, gnss_proc_flt *flt);
        /**
         * @brief Destroy the t gamb cmn object
         */
        ~gnss_amb_cmn();
        /**
         * @brief set active ambiguity
         * @param[in]  active_amb ambiguity
         */
        void active_amb(std::map<std::string, int> active_amb);
        /**
         * @brief get certain station amb
         * @param[in]  site      station name
         * @return int
         */
        int active_amb(std::string site);
        /**
         * @brief set whether
         * @param[in]  b         whether std::fixed
         */
        void amb_fixed(bool b);
        /**
         * @brief get whether ambstd::fixed
         */
        bool amb_fixed();
        /**
         * @brief Set the ratio value
         * @param[in]  r         value of ratio
         */
        void set_ratio(double r);
        /**
         * @brief Get the ratio value
         * @return double value of ratio
         */
        double get_ratio();
        /**
         * @brief Set the boot value
         * @param[in]  b         value of boot
         */
        void set_boot(double b);
        /**
         * @brief Get the boot value
         * @return double value of boot
         */
        double get_boot();
        /**
         * @brief Set the mode
         * @param[in]  mode      mode
         */
        void set_mode(std::string mode);
        /**
         * @brief Get the mode
         * @return std::string mode
         */
        std::string get_mode();
        /**
         * @brief get now
         * @return base_time now
         */
        base_time now() const;
        /**
         * @brief get sigma0
         * @return double sigma0
         */
        double sigma0() const;
        /**
         * @brief get parameter
         * @return base_allpar parameter
         */
        base_allpar param() const;
        /**
         * @brief get correction of parameter
         * @return Vector correction of parameter
         */
        Vector dx() const;
        /**
         * @brief get Standard deviation
         * @return Vector Standard deviation
         */
        Vector stdx() const;
        /**
         * @brief get qx
         * @return Symmetric qx
         */
        Symmetric Qx() const;

    private:
        base_time _now;                  ///< now time
        int _nobs_total, _npar_number; ///< number of observation and parameter
        double _sigma0;                ///< sigma0
        double _vtpv;                  ///< vtpv
        base_allpar _param;              ///< parameter
        Vector _dx;              ///< correction of parameter
        Vector _stdx;            ///< Standard deviation
        Symmetric _Qx;           ///< qx
        std::map<std::string, int> _active_amb;  ///< acctive ambiguity
        bool _amb_fixed;               ///< whether std::fixed
        double _ratio;                 ///< ratio
        double _boot;                  ///< boot
        std::string _mode;                  ///< mode
    };
    /**
    * @brief class for fix ambiguities epoch-wisely.
    */
    class gnss_ambiguity
    {
    public:
        /** @brief default constructor. */
        explicit gnss_ambiguity();

        /*@brief constructor, init some internal variables.
        * @param[in]  site    site name.
        * @param[in]  gset    ambiguity fixing setting of xml.
        */
        explicit gnss_ambiguity(std::string site, set_base *gset);

        /** @brief default destructor. */
        virtual ~gnss_ambiguity();

        /**
        * @brief batch processing loop over epoch.
        * @param[in] cycle_slip Initialize total number of the checked ambiguity set.
        * @param[in] glsq       least squares estimations include parameters and Qx dx etc.
        * @param[in] t          current epoch.
        * @return     0 - compute successfully
        *            other - failure
        */
        virtual int processBatch(const base_time &t, gnss_proc_lsqbase *glsq, gnss_data_cycleslip *cycle_slip, std::string mode);
        virtual int processBatch(const base_time &t, gnss_proc_flt *gflt, gnss_data_cycleslip *cycle_slip, std::string mode);

        bool amb_fixed();
        /**
        *@brief set upd file.
        * @param[in] gupd upd data
        */
        void setUPD(gnss_data_upd *gupd) { _gupd = gupd; }
        //void setUPD_epoch(gnss_data_upd_epoch* gupd_epoch) { _gupd_epoch = gupd_epoch; }

        /**
        *@brief set MW.
        * @param[in] hwa_map_mw& MW
        */
        void setMW(std::map<std::string, std::map<int, double>> &MW) { _MW = MW; }

        /**
        *@brief set elevation in raw_all mode.
        * @param[in] hwa_map_mw& MW
        */
        void setELE(const std::map<std::string, double> &crt_ele) { _ELE = crt_ele; }
        void setSNR(const std::map<std::string, std::map<FREQ_SEQ, double>> &crt_snr) { _SNR = crt_snr; }
        void setObsLevel(const std::map<std::string, std::map<FREQ_SEQ, int>> &crt_obsLevel) { _ObsLevel = crt_obsLevel; }
        //void setValidity(std::map<std::string, int> b) { _sats_vilidity = b; }

        /**
        *@brief set general spdlog file.
        * @param[in] spdlog spdlog file
        */
        void setLOG(base_log spdlog)
        {
            // set spdlog
            if (nullptr == spdlog)
            {
                spdlog::critical("your spdlog is nullptr !");
                throw std::logic_error("");
            }
            if (nullptr != spdlog)
            {
                _spdlog = spdlog;
            }
        }

        /**
        *@brief set IF/ALL.
        * @param[in] OBSCOMBIN type
        */
        void setObsType(OBSCOMBIN type) { _obstype = type; }

        /**
        *@brief set glonass freq ID.
        * @param[in] gnss_all_nav* allnav
        */
        void setWaveLength(std::map<std::string, int> &glofrq_num);

        /**
        * @brief Set ambiguity resolution reference satellite,
        *        used in AR with ref sat in PPP-RTK NRTK etc.
        * @param[in] satRef  Reference satellites of each system or all system.
        * @return     True  - set reference satellite successfully
        *            False - failure
        * lvhb,wb added or modified for npp 202006
        */
        void setSatRef(std::set<std::string> &satRef);
        void updateFixParam(base_allpar &param, Vector &dx, Vector *stdx = NULL);
        base_allpar &getFinalParams();
        base_allpar &updateUCAmb(std::set<std::string> *satref = NULL);
        hwa_vector_amb_dd &getDD() { return _DD; } //lvhb added for rtk/ins, 20200826
        hwa_vector_amb_dd& getDD_sav() { return _DD_save; };
        double get_ratio() { return _outRatio; };

        /**
        * @brief set ratio threshold and lambda reduction and search
        */
        double lambdaSolve(double &ratio, const Matrix &anor, const Vector &fltpar, Vector &ibias, bool parlamb = false);
        //double lambdaSolve(double& ratio,const Matrix& anor, const std::vector<double>& fltpar, std::vector<int>& ibias,bool parlamb = false);

        Vector out_firstambD() { return _mDia; }

        void setActiveAmb(int max) { _max_active_amb_one_epo = max; }
        void simulation(bool simu) { _simulation = simu; } //xjhan

    protected:
        CONSTRPAR _crd_est;
        OBSCOMBIN _obstype; ///< UDUC/IF ambiguity fixing
        base_time _beg;       ///< begin epoch
        base_time _end;       ///< end epoch
        base_time _crt_time;  ///< current epoch
        double _interval;   ///< sampling interval /s
        std::set<std::string> _sys;   ///< system

        std::string _site;          ///< current site
        std::set<std::string> _sat_rm;   ///< satellites being removed
        std::set<std::string> _sat_refs; ///< reference satellites

        int _frequency;
        hwa_vector_amb_dd _DD;                      ///< DD over all baslines
        hwa_vector_amb_dd _DD_save;                 ///< DD over all baslines
        std::map<std::string, std::map<int, double>> _MW; ///< MW for IF ambiguity fixing of current time
        std::map<std::string, double> _ELE;
        std::map<std::string, std::map<FREQ_SEQ, double>> _SNR;
        std::map<std::string, std::map<FREQ_SEQ, int>> _ObsLevel;
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;
        std::map<std::string, std::map<std::string, double>> _sys_wavelen; ///< wave length
        std::map<std::string, std::map<std::string, std::map<std::string,bool>>> _WL_flag;       //  <DD WL flag: true or false
        std::map<std::string, std::map<std::string, std::map<std::string,bool>>> _EWL_flag;      //  <DD EWL flag: true or false
        std::map<std::string, std::map<std::string, bool>> _EWL24_flag;    //  <DD EWL24 flag: true or false
        std::map<std::string, std::map<std::string, bool>> _EWL25_flag;    //  <DD EWL25 flag: true or false
        std::map<std::string, std::map<std::string, std::map<std::string,int>>> _IWL;            //  < Fixed DD WL ambiguity
        std::map<std::string, std::map<std::string, std::map<std::string,int>>> _IEWL;           //  < Fixed DD EWL ambiguity
        std::map<std::string, std::map<std::string, int>> _IEWL24;         //  < Fixed DD EWL24 ambiguity
        std::map<std::string, std::map<std::string, int>> _IEWL25;         //  < Fixed DD EWL25 ambiguity
        gnss_data_upd *_gupd;                                 ///< upd data
        //gnss_data_upd_epoch* _gupd_epoch;  ///< upd data
        set_base *_gset; ///< set from xml
        base_log _spdlog;  ///< spdlog file

        FIX_MODE _fix_mode;                    ///< set ambiguity fixing mode
        UPD_MODE _upd_mode;                    ///< set wl and nl upd mode
        bool _part_fix;                        ///< set whether take partial ambiguity std::fixed mode, default is false
        int _part_fix_num;                     ///< if value's size less than num, stop part fix
        double _ratio;                         ///< threshold in LAMBDA method
        double _boot;                          ///< threshold of bootstrapping rate in amb fix
        double _min_common_time;               ///< the Minimum common time of two observation arc
        bool _simulation;                      //xjhan
        std::map<std::string, double> _map_EWL_decision; ///< deriation, sigma in WL/NL-cycle
        std::map<std::string, double> _map_WL_decision;  ///< deriation, sigma in WL/NL-cycle
        std::map<std::string, double> _map_NL_decision;

        bool _is_first = false; ///< whether it is first epoch to be std::fixed
        bool _is_first_nl = false;
        bool _is_first_wl = false;
        bool _is_first_ewl = false;
        bool _is_first_ewl24 = false;
        bool _is_first_ewl25 = false;

        Vector _mDia;

        double _outRatio;             ///< outRatio
        base_iof *_ratiofile = nullptr; ///< ratio file
        base_iof *_bootfile = nullptr;  ///< BootStrapping file

        double *_pdE = nullptr;
        double *_pdC = nullptr;

        std::ostringstream _os_ratio;
        std::ostringstream _os_boot;

        int _max_active_amb_one_epo;

        bool _amb_fixed;
        int _total_amb_num, _fixed_amb_num;

        base_allpar _param;
        std::map<std::string, std::map<std::string,std::vector<FREQ_SEQ>>> _amb_freqs;

        // for real-time PPP_AR / PPP_RTK
        bool _realtime;
        base_time _ewl_Upd_time, _ewl24_Upd_time, _ewl25_Upd_time;
        base_time _wl_Upd_time;
        std::map<std::string, std::map<std::string, base_time>> _last_fix_time; // mode sat time
        std::map<std::string, std::map<std::string, int>> _fix_epo_num;
        std::map<std::string, int> _lock_epo_num; // satellite counts until cycle slip
        std::map<std::string, hwa_vector_amb_dd> _DD_previous;
        std::map<std::string, std::map<std::string, int>> _sats_index;
        double _FloatFixSep;
        double _FixFixSep;
        int _ctrl_last_fixepo_gap = 999999;
        int _ctrl_min_fixed_num = 0;
        int _full_fix_num;

    protected:
        /**
        * @brief get UPD correction for sat
        * @param[in] mode - UPD mode[optional choose: NL WL]
        * @return     true - compute successfully , false - failure
        */
        bool _getSingleUpd(std::string mode, base_time t, std::string sat, double &value, double &sigma);

        /**
        * @brief apply UPD correction for sat
        * @param[in] t    - target epoch
        * @return     true - compute successfully , false - failure
        */
        bool _applyUpd(base_time t);
        bool _applyWLUpd(base_time t, std::string mode);

        /**
        * @brief check if a new ambiguity is dependant of the already selected set expressed by a set of orth. unit std::vector Ei.
        * @param[in] isFirst            judge whether is the first.
        * @param[in] iNamb              number of related one-way ambiguties.
        * @param[in] iNdef              number of already selected independent ones if independent +1 as output.
        * @param[in] iN_oneway          number of oneway ambiguities used in the to be checked input .
        * @param[in] arriIpt2ow         position of the correponding ow-ambiguities.
        * @param[in] iMaxamb_ow         number of total number of ow-ambiguties for the differenced ambiguties to be checked, not zero, for allocation, zero for check.
        * @param[in] iMaxamb_for_check  number of the total independent ones of the checked set, only used for the allocaiton .
        * @param[out] iNdef
        * @param[out] arriIpt2ow
        * @return     true - compute successfully , false - failure
        */
        bool _checkAmbDepend(bool isFirst, int iNamb, int *iNdef, int iN_oneway, int *arriIpt2ow, int iMaxamb_ow, int iMaxamb_for_check);

        /**
        /**
        * @brief define double-difference ambiguities over one baseline.
        * @param[in] amb_cmn    estimations include parameters and Qx dx etc.
        * @return     true - compute successfully , false - failure
        */
        int _defineDDAmb(gnss_amb_cmn *amb_cmn);
        /*
        * @brief define double-difference widelane ambiguities over one baseline.
        * @return     true - compute successfully , false - failure
        */
        bool _calDDAmbWL();
        bool _calDDAmbWLWL(); //Wangbo: add for PPP-RTK Client WL
        bool _calDDAmbWLALL(gnss_amb_cmn *amb_cmn, std::string mode);

        void _DDsitecheck(std::map<std::string,int>& DDsite);
        void _DDsiteerase(std::map<std::string, int>& DDsite);
        /**
        * @brief fix widelane and narrowlane ambiguities.
        * @return     true - compute successfully , false - failure
        */
        bool _fixAmbIF();
        bool _fixAmbUDUC();
        bool _fixAmbWL();
        bool _fixAmbWL(std::string mode);

        /**
        * @brief select from (or reorder) a set of DD-ambiguities with their widelane and narrowlane ambiguities.
        * First, with std::fixed widelane, and their narrowlane near to an integer
        * Second, with std::fixed widelane
        * Last, form an full range and indepandent set
        * @param[in]  korder  =1, both std::fixed, =2 also not fixed.
        * @param[in]  namb    number of ambiguity
        * @param[out] ndef    number of std::fixed ambiguity
        * @return     true - compute successfully , false - failure
        */
        int _selectAmb(int korder, int namb);

        /**
        * @brief get covariance-matrix for or DD-ambiguities based on the defined DD-ambiguities and the Qx with oneway.
        * @param[in] filter      kalman filter estimations include parameters and Qx dx etc.
        * @param[in] covariance  Covariance matrix of DD.
        * @param[int] value      rnl column matrix of DD.
        * @param[out] covariance
        * @param[out] value
        * @return     true - compute successfully , false - failure
        */
        bool _prepareCovariance(gnss_amb_cmn *amb_cmn, Symmetric &covariance, std::vector<double> &value);
        bool _prepareCovarianceWL(gnss_amb_cmn *amb_cmn, Symmetric &covariance, std::vector<double> &value, std::string mode);

        /**
        * @brief resolve integer ambiguities using LAMBDA-method.
        * @param[in] anor    inverted N-matrix, full-matrix stored in one dim.
        * @param[in] fltpar  parameter of free solution, ambiguities at the end.
        * @param[int] ibias  std::fixed solution.
        * @param[out] ibias
        * @return      ratio value
        */
        double _lambdaSearch(const Matrix&anor, const std::vector<double> &fltpar, std::vector<int> &ibias, double *boot);

        /**
        /**
        * @brief resolve integer ambiguities using LAMBDA-method [combination of _prepareCovariance+_lambdaSearch].
        * @param[in]  amb_cmn    estimations include parameters and Qx dx etc.
        * @param[int] fixed_amb      std::fixed solution.
        * @return     true - compute successfully , false - failure
        */
        bool _ambSolve(gnss_amb_cmn *amb_cmn, std::vector<int> &fixed_amb, std::string mode);

        /**
        * @brief applied fixing constraints into Qx to get the std::fixed solutiond.
        * @param[in] glsq  least squares estimations include parameters and Qx dx etc.
        * @return     true - compute successfully , false - failure
        */
        bool _addFixConstraint(gnss_proc_lsqbase *glsq);
        bool _addFixConstraint(gnss_proc_flt *gflt);
        bool _addFixConstraint_before(gnss_proc_flt *gflt);
        bool _addFixConstraintWL(gnss_proc_lsqbase *glsq, std::string mode);
        bool _addFixConstraintWL(gnss_proc_flt *gflt, std::string mode);

        /**
        * @brief init ratio-file.
        * @return    void
        */
        void _initRatiofile();
        void _initBootfile();
        /**
        * @brief write ratio-file.
        * @return    void
        */
        void _writeRatio(double ratio);
        void _writeBoot(double BootStrapping);

        /**
        * @brief Calculate GLONASS receiver upd <Single Difference can't eliminate receiver upd in FDMA>.
        * @return    void
        */
        double _glonassRecUpd(std::string site);

        /**
        * @brief Find Single-Difference include reference satellite
        * @return  True  - Successful
        *          False - Failure
        */
        bool _findRefSD();

        /**
        * @brief Recover No-difference ambiguity both frequency.
        * @param[in] satref Reference Satellites.
        * @param[in] glsq   Least squares estimations include parameters and Qx dx etc.
        * @param[in] MW     MW obs.
        * @return     True  - No-Difference  ambiguity recovery successfully
        *            False - failure
        */
        bool _NDRecovery(std::string satref, base_allpar &params, std::map<std::string, std::map<FREQ_SEQ, double>> &singleND);

        int _fix_fix_check(gnss_proc_flt *gfltnow, gnss_proc_flt &gfltpre, base_time now, std::string mode);
        int _float_fix_check(gnss_proc_flt *gfltnow, gnss_proc_flt &gfltpre, base_time now);
    };

    bool _ddCompare(const gnss_amb_dd_base &dd1, const gnss_amb_dd_base &dd2);
}

#endif
