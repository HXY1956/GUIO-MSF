#ifndef hwa_base_par_h
#define hwa_base_par_h

#include "hwa_base_time.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_data_satdata.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_model_gmf.h"

namespace hwa_base
{
    enum class par_type
    {
        CRD_X,
        CRD_Y,
        CRD_Z, ///< coordinates
        TRP,
        GRD_N,
        GRD_E,
        SION,
        VION, ///< atmospheric parameters
        CLK,
        CLK_SAT, ///< clocks
        CLK13_G,
        CLK13_E,
        CLK13_C,
        CLK13_J, ///< receiver clocks for different systems in IF_mode
        CLK14_G,
        CLK14_E,
        CLK14_C,
        CLK14_J, ///< receiver clocks for different systems in IF_mode
        CLK15_G,
        CLK15_E,
        CLK15_C,
        CLK15_J, ///< receiver clocks for different systems in IF_mode
        CLK15_G_SAT,
        E15CLK_SAT,
        C15CLK_SAT,
        J15CLK_SAT, ///< satellite clocks for 13 combination (add by xiongyun)
        CLK_G,
        CLK_E,
        CLK_C,
        CLK_R,
        CLK_J, ///< receiver clocks for different systems (add by xiongyun)
        CLK_VPW,
        CLK_VR,
        CLK_VQ,
        IFB_C3,
        IFB_C4,
        IFB_C5,
        IFCB_F3,
        IFCB_F4,
        IFCB_F5, ///< inter-freq. code biases for FREQ_3, FREQ_4, FREQ_5, inter-freq. clock bias for GPS FREQ_3
        SAT_IFB_C3,
        SAT_IFB_C4,
        SAT_IFB_C5,
        GPS_REC_IFB_C3,
        GPS_REC_IFB_C4,
        GPS_REC_IFB_C5,
        GAL_REC_IFB_C3,
        GAL_REC_IFB_C4,
        GAL_REC_IFB_C5,
        BDS_REC_IFB_C3,
        BDS_REC_IFB_C4,
        BDS_REC_IFB_C5,
        QZS_REC_IFB_C3,
        QZS_REC_IFB_C4,
        QZS_REC_IFB_C5,
        GPS_IFB_C3,
        GPS_IFB_C4,
        GPS_IFB_C5,
        GAL_IFB_C3,
        GAL_IFB_C4,
        GAL_IFB_C5,
        BDS_IFB_C3,
        BDS_IFB_C4,
        BDS_IFB_C5,
        QZS_IFB_C3,
        QZS_IFB_C4,
        QZS_IFB_C5, ///< inter frequency 3 bias for gnss
        CLK_ICB,
        CLUSTERB, ///< initial clock bias, cluster-dependent bias
        AMB_IF,
        AMB13_IF,
        AMB14_IF,
        AMB15_IF,
        AMB_WL,
        AMB_L1,
        AMB_L2,
        AMB_L3,
        AMB_L4,
        AMB_L5, ///< ambiguities for indiv. freq. (number indicates freq not band)
        FCBS_IF,
        FCBS_L1,
        FCBS_L2,
        FCBS_L3,
        FCBS_L4,
        FCBS_L5, ///< satellite base_type_conv::fractional cycle biases for indiv. freq.
        FCBR_IF,
        FCBR_L1,
        FCBR_L2,
        FCBR_L3,
        FCBR_L4,
        FCBR_L5, ///< receiver  base_type_conv::fractional cycle biases for indiv. freq.
        GLO_ISB,
        GLO_ifcb,
        GLO_IFPB,
        GAL_ISB,
        BDS_ISB,
        QZS_ISB, ///< multi-GNSS
        LEO_ISB,
        IFB_GPS,
        IFB_BDS,
        IFB_QZS,
        IFB_GAL,
        IFB_GAL_2,
        IFB_GAL_3, ///< inter frequency bias for multi-frquency
        IFB_BDS_2,
        IFB_BDS_3,
        RCB_GPS_1,
        RCB_GPS_2,
        RCB_BDS_1,
        RCB_BDS_2,
        RCB_GAL_1,
        RCB_GAL_2,
        RCB_QZS_1,
        RCB_QZS_2, ///< pseudorange bias for PPP-RTK client
        ATT_X,
        ATT_Y,
        ATT_Z,
        eb_X,
        eb_Y,
        eb_Z,
        db_X,
        db_Y,
        db_Z,
        gyro_scale_X, ///< gyro scale factor
        gyro_scale_Y,
        gyro_scale_Z,
        acce_scale_X, ///< acce scale factor
        acce_scale_Y,
        acce_scale_Z,
        IMU_INST_ATT_X, ///< imu installation angle
        IMU_INST_ATT_Y,
        IMU_INST_ATT_Z,
        ODO_k,
        CAM_ATT_X,
        CAM_ATT_Y,
        CAM_ATT_Z,
        CAM_CRD_X,
        CAM_CRD_Y,
        CAM_CRD_Z,
        EX_CAM_ATT_X,
        EX_CAM_ATT_Y,
        EX_CAM_ATT_Z,
        EX_CAM_CRD_X,
        EX_CAM_CRD_Y,
        EX_CAM_CRD_Z,
        EXTRINSIC_ATT_X,
        EXTRINSIC_ATT_Y,
        EXTRINSIC_ATT_Z,
        EXTRINSIC_CRD_X,
        EXTRINSIC_CRD_Y,
        EXTRINSIC_CRD_Z,
        EXTRINSIC_T,
        LIDAR_ATT_X,
        LIDAR_ATT_Y,
        LIDAR_ATT_Z,
        LIDAR_CRD_X,
        LIDAR_CRD_Y,
        LIDAR_CRD_Z,
        EX_LIDAR_ATT_X,
        EX_LIDAR_ATT_Y,
        EX_LIDAR_ATT_Z,
        EX_LIDAR_CRD_X,
        EX_LIDAR_CRD_Y,
        EX_LIDAR_CRD_Z,
        GLO_IFB, ///< inter frequency bias for glonass
        VTEC,
        DCB_REC,
        DCB_SAT, ///< for DCB estimator
        P1P2G_REC,
        P1P2E_REC,
        P1P2R_REC,
        P1P2C_REC, ///< GNSS-specific receiver code DCB P1-P2
        VEL_X,
        VEL_Y,
        VEL_Z, ///< velocity
        ACC_X,
        ACC_Y,
        ACC_Z,   ///< acceleration
        CLK_RAT, ///< satellite clock speed
        SCALE,
        GEOCX,
        GEOCY,
        GEOCZ, ///< ERP parameter add by lijie
        XPOLE,
        YPOLE,
        DXPOLE,
        DYPOLE,
        UT1,
        DUT1,
        UT1VLBI,
        NUTDX,
        NUTDY, ///< EOP parameter
        PXSAT,
        PYSAT,
        PZSAT,
        VXSAT,
        VYSAT,
        VZSAT, ///< satellite parameter
        SR_scale,
        DRAG_c,
        ACC_Bx,
        ACC_By,
        ACC_Bz,
        ACC_Sx,
        ACC_Sy,
        ACC_Sz,
        EMP_Sc1,
        EMP_Cc1,
        EMP_Sa1,
        EMP_Ca1,
        EMP_Sr1,
        EMP_Cr1,
        EMP_Sc2,
        EMP_Cc2,
        EMP_Sa2,
        EMP_Ca2,
        EMP_Sr2,
        EMP_Cr2,
        EMP_Ac,
        EMP_Bc,
        EMP_Aa,
        EMP_Ba,
        EMP_Ar,
        EMP_Br,
        D0,
        Dc,
        Ds,
        Y0,
        Yc,
        Ys,
        B0,
        Bc,
        Bs, ///< ECOM Force parameter
        ECOMT_T30,
        ECOMT_T3c2,
        ECOMT_T3s2,
        ECOMT_T3c4,
        ECOMT_T3s4, ///< ECOM-T
        ECOMT_T20,
        ECOMT_T2c2,
        ECOMT_T2s2,
        ECOMT_T1s2,
        ECOM2_D0,
        ECOM2_D2c,
        ECOM2_D2s,
        ECOM2_D4c,
        ECOM2_D4s, ///< ECOM2 Force parameters
        ECOM2_Y0,
        ECOM2_X0,
        ECOM2_X1c,
        ECOM2_X1s, ///< ECOM2
        ABW_SP,
        ABW_SB,
        ABW_Y0,
        ABW_PXAD,
        ABW_PZAD,
        ABW_NZAD, ///< ABW Force parameters
        ABW_PXR,
        ABW_PZR,
        ABW_NZR, ///< ABW
        IFBR_RAT1,
        IFBR_RAT2,
        RB,
        RB_GLO,  /// for GLONASS
        RB_GAL,  /// for Galileo
        RB_BDS,  /// for BDS
        RB_GPS,  /// for GPS
        RB_PSM4, /// for LEO 4-prism LRA
        RB_PSM7, /// for LEO 7-prism LRA
        RB_PSM9, /// for LEO 9-prism LRA
        RB_all,  /// for all satellite each station
        SION_3,
        RA,
        DE,
        VRA,
        VDE,
        C_kbr,
        C_lri, ///< kbr,lri biased add by Junjie Han
        PCV_SAT,
        PCO_SAT_X,
        PCO_SAT_Y,
        PCO_SAT_Z, ///< SAT PCV/PCO pars
        PCV_REC,
        PCO_REC_X,
        PCO_REC_Y,
        PCO_REC_Z, ///< REC PCV/PCO pars
        NO_DEF
    };

    class base_time_arc
    {
    public:
        /** @brief constructor 1. */
        base_time_arc(const base_time& beg, const base_time& end);

        /** @brief default destructor. */
        ~base_time_arc();

        /** @brief override operator. */
        bool operator!=(const base_time_arc& Other) const;
        bool operator==(const base_time_arc& Other) const;
        bool operator<(const base_time_arc& Other) const;
        bool operator<=(const base_time_arc& Other) const;
        bool operator>(const base_time_arc& Other) const;
        bool operator>=(const base_time_arc& Other) const;

        /** @brief onside. */
        bool inside(const base_time_arc& Other) const;

        base_time begin; ///< begin time
        base_time end;   ///< end time
    };

    class base_par_head
    {
    public:
        /** @brief constructor 1. */
        base_par_head(par_type type, std::string site, std::string sat);

        /** @brief constructor 2. */
        base_par_head(const base_par_head& Other);

        /** @brief default destructor. */
        ~base_par_head();

        /** @brief override operator. */
        bool operator==(const base_par_head& Other) const;
        bool operator<(const base_par_head& Other) const;
        bool operator<=(const base_par_head& Other) const;
        bool operator>(const base_par_head& Other) const;
        bool operator>=(const base_par_head& Other) const;

        par_type type; ///< par type
        std::string site;   ///< site name
        std::string sat;    ///< satellite name
    };

    class base_par
    {
    public:
        const static std::string par_str_sep;
        const static std::vector<std::string> par_str;

        /** @brief is amb?. */
        static bool is_amb(par_type tp)
        {
            return (tp == par_type::AMB_IF || tp == par_type::AMB13_IF || tp == par_type::AMB14_IF || tp == par_type::AMB15_IF ||
                tp == par_type::AMB_L1 || tp == par_type::AMB_L2 || tp == par_type::AMB_L3 || tp == par_type::AMB_L4 || tp == par_type::AMB_L5);
        }

        /** @brief is crd?. */
        static bool is_crd(par_type tp)
        {
            return (tp == par_type::CRD_X || tp == par_type::CRD_Y || tp == par_type::CRD_Z);
        }

        /** @brief constructor 1. */
        base_par(const std::string& site, par_type t, unsigned i, const std::string& p, bool remove = true);

        /** @brief default constructor. */
        base_par();

        /** @brief default destructor. */
        virtual ~base_par();

        /** @brief override operator. */
        bool operator==(const base_par&) const;
        bool operator<(const base_par&) const;
        bool operator>(const base_par&) const;
        base_par operator-(const base_par&) const;
        base_par operator+(const base_par&) const;

        /** @brief set time. */
        void setTime(const base_time&, const base_time&);

        /** @brief Partial derivatives. */
        double partial(hwa_gnss::gnss_data_sats&, base_time&, Triple, hwa_gnss::gnss_data_obs& gobs);

        /** @brief Partial doppler. */
        double partial_doppler(hwa_gnss::gnss_data_sats& satData,
            Triple& groundXYZ,
            Triple& groundVEL); // set conf for doppler (by zhshen)

        /** @brief Partial eop/orb/orb_leo. */
        double partial_eop(hwa_gnss::gnss_data_sats&);
        double partial_orb(hwa_gnss::gnss_data_sats&);
        double partial_orb_leo(hwa_gnss::gnss_data_sats&);

        /** @brief set/get val. */
        void value(double val) { _value = val; }
        double value() const { return _value; }

        /** @brief set/get apriori. */
        void apriori(double apr) { _apriori = apr; }
        double apriori() const { return _apriori; }

        par_type parType; ///< par type
        int index;        ///< index
        std::string prn;       ///< satellite name
        std::string site;      ///< site name
        base_time beg;      ///< begin time
        base_time end;      ///< end time
        base_time stime;    ///< s time
        double aprval;    ///< apr value
        double pred;      ///< pred
        double zhd;       ///< for ztd par
        double zwd;       ///< for zwd par
        //for amb
        bool amb_ini = false; ///< add for amb to be initialized
        // for LEO
        bool lremove;       ///< add for LEO satellite(default: false)
        bool lPWC;          ///< add for LEO PWC parameters(default: false)
        int nPWC;           ///< number of parameters for this kind of PWC parameter
        int idx_orb;        ///< index of LEO orb par in all LEO orb pars
        int piecewise_time; ///< time
        // for PCV/PCO estimation
        int idx_pcv; ///< index of PCV (zenith dependent: 1, 2; zenith and azimuth dependent: 1, 2, 3, and 4)
        int nazi;    ///< point num of PCV azimuth grids
        int nzen;    ///< point num of PCV zenith grids
        double dzen;
        double dazi;
        double zen1;
        double zen2;
        double azi1;
        double azi2;
        std::vector<hwa_gnss::GFRQ> fq; ///< single frequency: G01/G02/C02/R01..., LC combination: G01+G02/C02+C07...
        bool sat_zero_con;
        bool rec_zero_con;
        int nobs;
        //
        std::string str_type() const;
        base_par_head get_head() const;
        base_time_arc get_timearc() const;

        void setMF(hwa_gnss::ZTDMPFUNC MF);
        void setMF(hwa_gnss::GRDMPFUNC MF);

    protected:
        double _value;     ///< value
        hwa_gnss::ZTDMPFUNC _mf_ztd; ///< mapping function for ZTD
        hwa_gnss::GRDMPFUNC _mf_grd; ///< mapping function for GRD
        double _apriori;   ///< apriori
        void _getmf(hwa_gnss::gnss_data_sats& satData,
            const Triple& crd,
            const base_time& epoch,
            double& mfw,
            double& mfh,
            double& dmfw,
            double& dmfh);
    };

    /** @brief convert par to std::string. */
    std::string gpar2str(const base_par& par);

    /** @brief convert partype to std::string. */
    std::string ptype2str(const par_type& par);

    /** @brief convert std::string to par. */
    base_par str2gpar(const std::string& str_par);

}

#endif