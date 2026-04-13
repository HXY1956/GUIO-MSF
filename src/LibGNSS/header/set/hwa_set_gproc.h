#ifndef hwa_set_gproc_h
#define hwa_set_gproc_h
#define XMLKEY_PROC "process" 
#include "hwa_base_typeconv.h"
#include "hwa_set_base.h"
#include "hwa_set_gproc_type.h"
#include "hwa_set_gtype.h"
using namespace hwa_gnss;

namespace hwa_set
{
    /// The class for setting of process modelue in XML file
    class set_gproc : public virtual hwa_set::set_base
    {
    public:
        /// constructor
        set_gproc();
        /// destructor
        ~set_gproc() override;

        /// settings check
        void check() override;
        /// settings help
        void help() override;

        /**
         * @brief         get the decimals for sampling interval (for high-rate) in process
         * @return bool : decimals for sampling interval (for high-rate) in process
         */
        bool isleo();
        bool tropo();
        void tropo(bool b);
        bool iono();
        void iono(bool b);
        bool tropo_slant();
        bool tropo_grad();
        bool phase();
        bool doppler();
        bool pos_kin();
        bool use_eclipsed();
        bool auto_band();
        bool sd_sat();
        bool IRC();
        bool carrier_range();       //xy
        bool apply_carrier_range(); // jqwu
        bool pre_fix_residuals();

        bool ambfix();

        bool realtime(); // added by zhShen
        bool ultrasp3(); // added by xiongyun
        //int dynamics();
        int frequency();
        int num_threads();
        bool matrix_remove();
        bool cmb_equ_multi_thread();
        double sig_init_ztd();
        double sig_init_kbr();
        double sig_init_lri();
        double sig_init_vion();
        double sig_init_grd();
        double sig_init_crd();
        double sig_init_vel();
        double sig_init_amb();
        double sig_init_glo();
        double sig_init_gal();
        double sig_init_bds();
        double sig_init_qzs();

        double sig_ref_clk();
        double minimum_elev();
        double minimum_elev_leo(); // add by zhangwei for leo processing
        double max_res_norm();
        double max_att_res_norm();
        int minsat();

        double sig_init_sat_pcv();
        double sig_init_rec_pcv();
        double sig_init_sat_pcox();
        double sig_init_sat_pcoy();
        double sig_init_sat_pcoz();
        double sig_init_rec_pcox();
        double sig_init_rec_pcoy();
        double sig_init_rec_pcoz();
        bool sat_pcv_est();
        bool rec_pcv_est();
        bool sat_zero_con();
        bool rec_zero_con();
        std::map<std::string, bool> sat_pco_xyz();
        std::map<std::string, bool> rec_pco_xyz();
        PCV_MODE sat_pcv_mode();
        PCV_MODE rec_pcv_mode();
        double sat_azi_beg();
        double sat_azi_end();
        double sat_dazi();
        double sat_zen_beg();
        double sat_zen_end();
        double sat_dzen();
        double rec_azi_beg();
        double rec_azi_end();
        double rec_dazi();
        double rec_zen_beg();
        double rec_zen_end();
        double rec_dzen();
        std::map<GOBS_LC, std::set<std::string>> exc_sat_list();
        std::map<GOBS_LC, std::set<std::string>> exc_rec_list();
        bool sat_ext_pco();
        bool rec_ext_pco();
        bool sat_pcv_neq();
        bool rec_pcv_neq();

        bool sat_orb_est();
        bool sat_clk_est();
        bool geoc_est();

        std::string ref_clk();

        LSQ_MODE lsq_mode();
        SLIPMODEL slip_model();
        KBRLRIMODE kbr_mode();
        KBRLRIMODE lri_mode();
        IONMODEL ion_model();
        int lsq_buffer_size();

        TROPMODEL tropo_model();
        ZTDMODEL ztd_model(double *dt = nullptr);
        ZTDMPFUNC tropo_mf();
        IONMPFUNC iono_mf();
        GRDMPFUNC grad_mf();
        CONSTRPAR crd_est();
        HMTCONSTR hmt_cst(); //HwangShih added
        OBSWEIGHT weighting();
        RESIDTYPE residuals();
        OBSCOMBIN obs_combin();
        void obs_combin(std::string str);
        ATTITUDES attitudes();
        std::string trop_corr();
        CBIASCHAR cbiaschar();

        SYSBIASMODEL sysbias_model();             //glfeng
        SYSBIASMODEL ifbbias_model();             //LX
        GRDMODEL grd_model(double *dt = nullptr); //glfeng

        GRDMPFUNC str2grdmpfunc(const std::string &mf);
        ZTDMPFUNC str2ztdmpfunc(const std::string &mf);
        IONMPFUNC str2ionmpfunc(const std::string &mf);
        OBSWEIGHT str2obsweight(const std::string &wg);
        TROPMODEL str2tropmodel(const std::string &tm);
        static ZTDMODEL str2ztdmodel(const std::string &ztd);
        RESIDTYPE str2residtype(const std::string &rs);
        OBSCOMBIN str2obscombin(const std::string &oc);
        ATTITUDES str2attitudes(const std::string &at);
        CBIASCHAR str2cbiaschar(const std::string &cb);
        SYSBIASMODEL str2sysbiasmodel(const std::string &sys); //glfeng
        static GRDMODEL str2grdmodel(const std::string &grd);  //glfeng

        static std::string grdmpfunc2str(GRDMPFUNC MF);
        static std::string ztdmpfunc2str(ZTDMPFUNC MF);
        static std::string ionmpfunc2str(IONMPFUNC MF);
        static std::string obsweight2str(OBSWEIGHT WG);
        static std::string tropmodel2str(TROPMODEL TM);
        static std::string ztdmodel2str(ZTDMODEL ZTD);
        static std::string residtype2str(RESIDTYPE RS);
        static std::string obscombin2str(OBSCOMBIN OC);
        static std::string attitude2str(ATTITUDES AT);
        static std::string cbiaschar2str(CBIASCHAR CB);
        static std::string basepos2str(base_pos BP);

        bool simulation();

        IFCB_MODEL ifcb_model(); // jdhuang : fcb
        ISFIRSTSOL is_firstsol();
        ISCONSPW is_conspw();
        IFB_model ifb_model();
        bool trimcor(); //TRUE/False
        bool write_equ();

        //IONO_ORDER                          iono_order(); // jdhuang : used for ion order
        //rtk
        base_pos basepos(); ///< obtain base coordinate
        void basepos(std::string str);
        base_pos str2basepos(const std::string &str);
        OFSTREAM_MODE read_ofile_mode(); // Hwang Shih : used for reading O files
        // smooth range
        std::string range_smooth_mode(int *smt_windows = 0);
        // correct bds code bias
        bool bds_code_bias_correction();
        RECEIVERTYPE get_receiverType();
        NPP_MODEL npp_model();
        modeofmeanpole mean_pole_model();
        modeofmeanpole str2meanpolemodel(const std::string &tm);
        bool motion_model();

    protected:
        bool _phase;            ///< use phase data [true=1, false=0]
        bool _tropo;            ///< estimate troposphere [true=1, false=0]
        bool _iono;             ///< estimate ionosphere [true=1, false=0]
        bool _tropo_grad;       ///< estimate troposphere gradinet [true=1, false=0]
        bool _tropo_slant;      ///< estimate tropo slant delays
        TROPMODEL _tropo_model; ///< tropospheric model [SAASTAMOINEN, DAVIS, HOPFIELD, ...]
        ZTDMODEL _ztd_model;    ///< ztdmodle [PWC STO]
        ZTDMPFUNC _tropo_mf;    ///< tropo mapPing function [COSZ, NIELL, GMF, VMF1, ... ]
        IONMPFUNC _iono_mf;     ///< iono mapPing function [COSZ, QFAC, NONE, ... ]
        GRDMPFUNC _grad_mf;     ///< grad mapPing function [tilt, CHH]
        OBSWEIGHT _obs_weight;  ///< observations weighting
        RESIDTYPE _res_type;    ///< types of residuals
        OBSCOMBIN _obs_combin;  ///< observation combination
        ATTITUDES _attitudes;   ///< satellite attitude model
        CBIASCHAR _cbiaschar;   ///< forcing code bias signals

        double _sig_init_ztd;  ///< accuracy of initial zenith path delays [m]
        double _sig_init_vion; ///< accuracy of initial vertical iono path delays [m]
        double _sig_init_grd;  ///< accuracy of initial tropo gradients [m]
        double _sig_init_crd;  ///< accuracy of initial coordinates [m]
        double _sig_init_vel;  ///< accuracy of initial velocities [m/s]
        double _sig_init_amb;  ///< accuracy of initial ambiguity [m]
        double _sig_init_glo;  ///< accuracy of initial GLONASS system time difference
        double _sig_init_gal;  ///< accuracy of initial Galileo system time difference
        double _sig_init_bds;  ///< accuracy of initial BeiDou system time difference
        double _sig_init_qzs;  ///< accuracy of initial QZSS system time difference
        double _minimum_elev;  ///< elevation angle cut-off [degree]
        double _max_res_norm;  ///< normalized residuals threshold
        std::string _crd_est;       ///< FIX or estimate CRD
        bool _pos_kin;         ///< static/kinematic receiver (true == kinematic)
        bool _use_eclipsed;    ///< used eclipsed satellites
        bool _auto_band;       ///< automatic band selection (mainly for Anubis purpose)
        int _dynamics;
        int _frequency;
        bool _realtime;
        bool _simulation; ///< judge whether simulation data

        // Satellite/Receiver PCO/PCV estimation parts, added by Wei Zhang
        double _sig_init_sat_pcv;       ///< satelite PCV estimaiton
        double _sig_init_rec_pcv;       ///< receiver PCV estimaiton
        double _sig_init_sat_pcox;      ///< satellite X PCO estimaion
        double _sig_init_sat_pcoy;      ///< satellite Y PCO estimaion
        double _sig_init_sat_pcoz;      ///< satellite Z PCO estimaion
        double _sig_init_rec_pcox;      ///< receiver X PCO estimaion
        double _sig_init_rec_pcoy;      ///< receiver Y PCO estimaion
        double _sig_init_rec_pcoz;      ///< receiver Z PCO estimaion
        std::map<std::string, bool> _sat_pco_xyz; ///< which component of satellite PCO to be estimated(X/Y/Z)
        std::map<std::string, bool> _rec_pco_xyz; ///< which component of receiver PCO to be estimated(X/Y/Z)
        double _sat_azi_beg;            ///< begin azimuth angle of satellite PCV
        double _sat_azi_end;            ///< end azimuth angle of satellite PCV
        double _sat_dazi;               ///< azimuth interval of satellite PCV
        double _sat_zen_beg;            ///< begin zenith angle of satellite PCV
        double _sat_zen_end;            ///< end zenith angle of satellite PCV
        double _sat_dzen;               ///< zenith angle of satellite PCV
        double _rec_azi_beg;            ///< begin azimuth angle of receiver PCV
        double _rec_azi_end;            ///< end azimuth angle of receiver PCV
        double _rec_dazi;               ///< azimuth interval of receiver PCV
        double _rec_zen_beg;            ///< begin zenith angle of receiver PCV
        double _rec_zen_end;            ///< end zenith angle of receiver PCV
        double _rec_dzen;               ///< zenith angle of receiver PCV
        int _minsat;                    ///<  minimum satellite number
        base_pos _basepos;
        bool _sd_sat; ///< single differented between sat and sat_ref

        NPP_MODEL _nppmodel;
        modeofmeanpole _meanpolemodel; // yqyuan added for different mean pole modeling

    private:

    };
} // namespace

#endif
