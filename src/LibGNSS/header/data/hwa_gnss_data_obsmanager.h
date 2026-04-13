#ifndef hwa_gnss_data_obs_manager_H
#define hwa_gnss_data_obs_manager_H

#include "hwa_set_gtype.h"
#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_gnss_sys.h"

#define DEF_CHANNEL 255

namespace hwa_gnss
{
    const static double NULL_GOBS = 0.0;

    ///< priority tables for choice of available signals (code [m])
    const static GOBS code_choise[9][19] = {
        {X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X},                               //
        {C1A, C1B, C1C, X, C1I, C1L, C1M, X, C1P, C1Q, C1S, C1W, C1X, C1Y, C1Z, P1, C1, CA, CB}, //  C1
        {X, X, C2C, C2D, C2I, C2L, C2M, X, C2P, C2Q, C2S, C2W, C2X, C2Y, X, P2, C2, CC, CD},     //  C2
        {X, X, X, X, C3I, X, X, X, X, C3Q, X, X, C3X, X, X, X, X, X, X},                         //
        {X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X},                               //
        {X, X, X, X, C5I, X, X, X, X, C5Q, X, X, C5X, X, X, P5, C5, X, X},                       //  C5
        {C6A, C6B, C6C, X, C6I, X, X, X, X, C6Q, X, X, C6X, X, C6Z, X, C6, X, X},                //  C6
        {X, X, X, X, C7I, X, X, X, X, C7Q, X, X, C7X, X, X, X, C7, X, X},                        //  C7
        {X, X, X, X, C8I, X, X, X, X, C8Q, X, X, C8X, X, X, X, C8, X, X}                         //  C8
    };

    ///< priority tables for choice of available signals (phase [full-cycles])
    const static GOBS phase_choise[9][19] = {
        {X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X},                                //
        {L1A, L1B, L1C, X, L1I, L1L, L1M, L1N, L1P, L1Q, L1S, L1W, L1X, L1Y, L1Z, X, L1, LA, LB}, //  L1
        {X, X, L2C, L2D, L2I, L2L, L2M, L2N, L2P, L2Q, L2S, L2W, L2X, L2Y, X, X, L2, LC, LD},     //  L2
        {X, X, X, X, L3I, X, X, X, X, L3Q, X, X, L3X, X, X, X, X, X, X},                          //
        {X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X},                                //
        {X, X, X, X, L5I, X, X, X, X, L5Q, X, X, L5X, X, X, X, L5, X, X},                         //  L5
        {L6A, L6B, L6C, X, L6I, X, X, X, X, L6Q, X, X, L6X, X, L6Z, X, L6, X, X},                 //  L6
        {X, X, X, X, L7I, X, X, X, X, L7Q, X, X, L7X, X, X, X, L7, X, X},                         //  L7
        {X, X, X, X, L8I, X, X, X, X, L8Q, X, X, L8X, X, X, X, L8, X, X}                          //  L8
    };

    ///< jdhuang
    const static std::map<GSYS, std::map<GOBSBAND, std::string>> range_order_attr_raw = {
        {GPS, {{BAND_1, "CPW"}, {BAND_2, "CLXPW"}, {BAND_5, "QX"}}},
        {GAL, {{BAND_1, "CX"}, {BAND_5, "IQX"}, {BAND_7, "IQX"}, {BAND_8, "IQX"}, {BAND_6, "ABCXZ"}}},
        {BDS, {{BAND_2, "IQX"}, {BAND_7, "IQX"}, {BAND_6, "IQX"}, {BAND_5, "DPX"}, {BAND_9, "DPZ"}, {BAND_8, "DPX"}, {BAND_1, "DPX"}}},
        {GLO, {{BAND_1, "CP"}, {BAND_2, "CP"}}},
        {QZS, {{BAND_1, "CSLX"}, {BAND_2, "LX"}, {BAND_5, "IQX"}}}};

    const static std::map<GSYS, std::map<GOBSBAND, std::string>> range_order_attr_cmb = {
        {GPS, {{BAND_1, "CPW"}, {BAND_2, "CLXPW"}, {BAND_5, "QX"}}},
        {GAL, {{BAND_1, "CX"}, {BAND_5, "IQX"}, {BAND_7, "IQX"}, {BAND_8, "IQX"}, {BAND_6, "ABCXZ"}}},
        {BDS, {{BAND_2, "IQX"}, {BAND_7, "IQX"}, {BAND_6, "IQX"}, {BAND_5, "DPX"}, {BAND_9, "DPZ"}, {BAND_8, "DPX"}, {BAND_1, "DPX"}}},
        {GLO, {{BAND_1, "CP"}, {BAND_2, "CP"}}},
        {QZS, {{BAND_1, "CSLX"}, {BAND_2, "LX"}, {BAND_5, "IQX"}}}};

    const static std::map<GSYS, std::map<GOBSBAND, std::string>> phase_order_attr_raw = {
        {GPS, {{BAND_1, "CSLXPWYM"}, {BAND_2, "CDLXPWYM"}, {BAND_5, "IQX"}}},
        {GAL, {{BAND_1, "ABCXZ"}, {BAND_5, "IQX"}, {BAND_7, "IQX"}, {BAND_8, "IQX"}, {BAND_6, "ABCXZ"}}},
        //LX changed for BDS3
        {BDS, {{BAND_2, "XIQ"}, {BAND_7, "IQX"}, {BAND_6, "IQX"}, {BAND_5, "DPX"}, {BAND_9, "DPZ"}, {BAND_8, "DPX"}, {BAND_1, "DPX"}}},
        {GLO, {{BAND_1, "PC"}, {BAND_2, "CP"}}},                   // fix bugs change Band_1 CP->PC  glfeng
        {QZS, {{BAND_1, "CSLX"}, {BAND_2, "LX"}, {BAND_5, "IQX"}}} //glfeng
    };

    const static std::map<GSYS, std::map<GOBSBAND, std::string>> phase_order_attr_cmb =
        {
            {GPS, {{BAND_1, "CSLXPWYM"}, {BAND_2, "CDLXPWYM"}, {BAND_5, "IQX"}}},
            {GAL, {{BAND_1, "ABCXZ"}, {BAND_5, "IQX"}, {BAND_7, "IQX"}, {BAND_8, "IQX"}, {BAND_6, "ABCXZ"}}},
            {BDS, {{BAND_2, "QXI"}, {BAND_7, "IQX"}, {BAND_6, "IQX"}, {BAND_5, "DPX"}, {BAND_9, "DPZ"}, {BAND_8, "DPX"}, {BAND_1, "DPX"}}},
            {GLO, {{BAND_1, "PC"}, {BAND_2, "CP"}}},                   // fix bugs change Band_1 CP->PC  glfeng
            {QZS, {{BAND_1, "CSLX"}, {BAND_2, "LX"}, {BAND_5, "IQX"}}} //glfeng

    };

    class gnss_data_obscmb;
    class gnss_all_bias;

    /** @brief class for gnss_data_obs_manager derive from base_data. */
    class gnss_data_obs_manager : public hwa_base::base_data
    {

    public:
        /** @brief default constructor. */
        gnss_data_obs_manager();
        /**
         * @brief Construct a new t gobsgnss object
         * 
         * @param spdlog 
         */
        gnss_data_obs_manager(hwa_base::base_log spdlog);
        /** @brief constructor 1. */
        gnss_data_obs_manager(hwa_base::base_log spdlog, const std::string &sat);

        /** @brief constructor 2. */
        gnss_data_obs_manager(hwa_base::base_log spdlog, const std::string &site, const std::string &sat, const hwa_base::base_time &t);

        /** @brief constructor 3. */
        gnss_data_obs_manager(hwa_base::base_log spdlog, const std::string &site, const std::string &sat, const hwa_base::base_time &t, const bool &is_leo); //add leo

        /** @brief default destructor. */
        virtual ~gnss_data_obs_manager();

        /** @brief add a single observation. */
        void addobs(const GOBS &obs, const double &d);

        /** @brief add a lost-of-lock indicator. */
        void addlli(const GOBS &obs, const int &i);

        /** @brief add an estimated cycle slip. */
        void addslip(const GOBS &obs, const int &i);

        /** @brief approximate elevation. */
        void addele(const double &d);

        /** @brief add range outliers. */
        void addoutliers(const GOBS &obs, const int &i); //glfeng

        /**
         * @brief 
         * 
         * @param name 
         * @param b 
         */
        void setrangestate(const std::string &name, const bool &b);

        /**
         * @brief 
         * 
         * @param name 
         * @return true 
         * @return false 
         */
        bool getrangestate(const std::string &name);

        /**
         * @brief 
         * 
         * @param gobs 
         * @param flag 
         */
        void setobsLevelFlag(const GOBS &gobs, const int &flag);

        /**
         * @brief 
         * 
         * @param gobs 
         * @return int 
         */
        int getobsLevelFlag(const GOBS &gobs);

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool obsWithCorr();

        /** @brief get number of observations. */
        size_t size() const;

        /** @brief get system-specific frequency  for band i. */
        double frequency(const GOBSBAND &b) const;

        /** @brief get system-specific frequency  for band i. */
        double frequency(const int &b) const;

        /** @brief get system-specific wavelength for band i. */
        double wavelength(const GOBSBAND &b) const;

        /** @brief get wavelength for iono-free LC. */
        double wavelength_L3(const GOBSBAND &b1 = BAND_1, const GOBSBAND &b2 = BAND_2) const;

        /** @brief get wavelength for wild lane. */
        double wavelength_WL(const GOBSBAND &b1 = BAND_1, const GOBSBAND &b2 = BAND_2) const;

        /** @brief get wavelength for narrow lane. */
        double wavelength_NL(const GOBSBAND &b1 = BAND_1, const GOBSBAND &b2 = BAND_2) const;

        /** @brief get GNSS system from satellite IDOC. */
        GSYS gsys() const;

        /** @brief gppflt NEW! */
        GOBS id_range(const GOBSBAND &b) const { return _id_range(b); };

        /**
         * @brief 
         * 
         * @param b 
         * @return GOBS 
         */
        GOBS id_phase(const GOBSBAND &b) const { return _id_phase(b); };

        /** @brief get code obs of selected band (Used the same as Panda). */
        GOBS select_range(const GOBSBAND &b, const bool &isRawAll = true) const;

        /** @brief get pahse obs of selected band (Used the same as Panda). */
        GOBS select_phase(const GOBSBAND &b, const bool &isRawAll = true) const;

        /** @brief get std::vector of available observations. */
        std::vector<GOBS> obs() const;

        /** @brief get std::set of available phase observations for a band. */
        std::set<GOBS> obs_phase(const int &band) const;

        /** @brief get a single observation (DIFFERENT UNITS!). */
        double getobs(const GOBS &obs) const;

        /** @brief get a single observation (DIFFERENT UNITS!). */
        double getobs(const std::string &obs) const;

        /** @brief get approximate elevation. */
        const double &getele() const;

        /** @brief get a lost-of-lock indicator. */
        int getlli(const GOBS &obs) const;

        /** @brief get an estimated cycle slip. */
        int getslip(const GOBS &obs) const;

        /** @brief get a lost-of-lock indicator. */
        int getlli(const std::string &obs) const;

        /** @brief get number of code/phase available bands. */
        void nbands(std::pair<int, int> &nb);

        /**
         * @brief 
         * 
         * @param obs 
         * @return int 
         */
        int getoutliers(const GOBS &obs) const; //glfeng

        /**
         * @brief 
         * 
         * @param obs 
         * @param v 
         */
        void resetobs(const GOBS &obs, const double &v); // add by glfeng

        /**
         * @brief 
         * 
         * @param obs 
         */
        void eraseobs(const GOBS &obs); // add by glfeng

        /**
         * @brief 
         * 
         * @param b 
         */
        void eraseband(const GOBSBAND &b); // add by glfeng

        /**
         * @brief 
         * 
         * @param allbias 
         * @return true 
         * @return false 
         */
        bool apply_bias(gnss_all_bias *allbias);

        /**
         * @brief 
         * 
         * @param allbias 
         * @return true 
         * @return false 
         */
        bool apply_code_phase_bias(gnss_all_bias *allbias);

        /**
         * @brief 
         * 
         * @param allbias 
         * @return true 
         * @return false 
         */
        bool apply_bias_tmp(gnss_all_bias &allbias);

        /**
         * @brief 
         * 
         * @param allbias 
         * @return true 
         * @return false 
         */
        bool apply_bias_tmp2(gnss_all_bias &allbias);

        /**
         * @brief 
         * 
         * @param allbias 
         * @return true 
         * @return false 
         */
        bool apply_dcb(gnss_all_bias *allbias);

        /**
         * @brief 
         * 
         * @param allbias 
         * @return true 
         * @return false 
         */
        bool apply_osb(gnss_all_bias *allbias);

        /**
         * @brief 
         * 
         * @param allbias 
         * @param band1 
         * @param band2 
         * @return double 
         */
        double interFreqDcb(gnss_all_bias &allbias, const GOBSBAND &band1, const GOBSBAND &band2) const;

        /**
         * @brief 
         * 
         * @param mark 
         */
        void reset_dcbmark(const bool &mark) { _dcb_correct_mark = mark; }; //xjhan

        /** @brief std::set channel number for Glonass satellites. */
        void channel(const int &canal);

        /** @brief get channel number for Glonass satellites. */
        const int &channel() const;

        /** @brief std::set RTCM Multiple Message bit. */
        void rtcm_end(const unsigned int &end) { _rtcm_end = end; }

        /** @brief get RTCM Multiple Message bit. */
        const unsigned int &rtcm_end() const { return _rtcm_end; }

        ///** @brief get  code observation [m] only requested type NEW + AUTO! */
        //double obs_range(const gnss_data_band &gb) const;

        ///** @brief get phase observation [m] only requested type NEW + AUTO! */
        //double obs_phase(const gnss_data_band &gb) const;

        /** @brief get code observation [m] only requested type! */
        double obs_C(const gnss_data_obs &gobs) const; // NEW
        /**
         * @brief 
         * 
         * @param gobs 
         * @return double 
         */
        double obs_C(const GOBS &gobs) const;

        /** @brief get phase observation [m] only requested type! */
        double obs_L(const gnss_data_obs &gobs) const; // NEW
        /**
         * @brief 
         * 
         * @param gobs 
         * @return double 
         */
        double obs_L(const GOBS &gobs) const;

        /** @brief get doppler observation [m/s] only requested type! */
        double obs_D(const gnss_data_obs &gobs) const; // NEW
        /**
         * @brief 
         * 
         * @param gobs 
         * @return double 
         */
        double obs_D(const GOBS &gobs) const; // NEW

        /** @brief get snr  observation [dbHz] only requested type! by wh */
        double obs_S(const gnss_data_obs &gobs) const; // NEW

        /** @brief get code observation [m] only requested type */
        double obs_C(const gnss_data_band &gb) const; // NEW

        /** @brief get phase observation [m] only requested type */
        double obs_L(const gnss_data_band &gb) const; // NEW

        /** @brief get doppler observation [m/s] only requested type */
        double obs_D(const gnss_data_band &gb) const; // NEW

        /** @brief get snr observation [dbHz] only requested type by wh */
        double obs_S(const gnss_data_band &gb) const; // NEW

        /** @brief modify phase observation by adding dL: i = 0 [cycles], i = 1 [m] */
        int mod_L(const double &dL, const GOBS &gobs = X, const int i = 1);

        /** @brief modify phase observation by adding dL: i = 0 [cycles], i = 1 [m], i = 2 [std::set val to 0] */
        int mod_L(const double &dL, const GOBSBAND &band, const int i = 1);

        /**
         * @brief 
         * 
         * @param band1 
         * @param coef1 
         * @param band2 
         * @param coef2 
         * @param band3 
         * @param coef3 
         * @return double 
         */
        double frequency_lc(const int &band1,    // return value [Hz] of phase linear combination frequency (c1*O1 + ... )
                            const double &coef1, // for 2 or 3 bands with given coefficients
                            const int &band2,
                            const double &coef2,
                            const int &band3 = 0,
                            const double &coef3 = 0) const; // -->> OLD INTERFACE !!

        /**
         * @brief 
         * 
         * @param band1 
         * @param coef1 
         * @param band2 
         * @param coef2 
         * @return double 
         */
        double isf_lc(const int &band1,    // return value of phase linear combination ionospheric scale factor (c1*O1 + ... )
                      const double &coef1, // for 2 or 3 bands with given coefficients
                      const int &band2,
                      const double &coef2) const;
        //     const    int& band3 = 0,
        //     const double& coef3 = 0); // -->> OLD INTERFACE !!

        /**
         * @brief 
         * 
         * @param band1 
         * @param coef1 
         * @param band2 
         * @param coef2 
         * @param band3 
         * @param coef3 
         * @return double 
         */
        double pnf_lc(const int &band1,    // return value of linear combination phase noise factor
                      const double &coef1, // for 2 or 3 bands with given coefficients
                      const int &band2,
                      const double &coef2,
                      const int &band3 = 0,
                      const double &coef3 = 0) const;

        // POTENCIALNE NOVY (uvidime jak bude uzitecne, ale kdyz uz mame nastaveno gnss_data_obs, melo by byt)
        // --------------------------------------------------------------------------------------------
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double P3(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get ionosphere-free combination for code  [m]
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double P3(const GOBS &g1, const GOBS &g2) const;
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double P4(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get geometry-free combination for code    [m]

        // POTENCIALNE NOVY (uvidime jak bude uzitecne, ale kdyz uz mame nastaveno gnss_data_obs, melo by byt)
        // --------------------------------------------------------------------------------------------
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double L3(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get ionosphere-free combination for phase [m]
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double L3(const GOBS &g1, const GOBS &g2) const;
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double L3_cycle(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get ionosphere-free combination for phase [CYCLE] glfeng
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double L4(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get geometry-free combination for phase   [m]
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double L4_cycle(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get geometry-free combination for phase   [cycle] glfeng
        /**
         * @brief 
         * 
         * @param code 
         * @param L1 
         * @param L2 
         * @return double 
         */
        double MP(const gnss_data_obs &code,
                  const gnss_data_obs &L1, const gnss_data_obs &L2) const; // get multipath for C-obs + Li/Lj
        /**
         * @brief 
         * 
         * @param gL1 
         * @param gL2 
         * @param gL3 
         * @return double 
         */
        double GFIF_meter(const gnss_data_obs &gL1, const gnss_data_obs &gL2, const gnss_data_obs &gL3) const;
        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double PWL(const gnss_data_obs &g1, const gnss_data_obs &g2) const;

        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double LWL(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get wide-lane combination for phase [m]!

        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @return double 
         */
        double LNL(const gnss_data_obs &g1, const gnss_data_obs &g2) const; // get narrow-lane combination for phase [m]!

        /**
         * @brief 
         * 
         * @param g1 
         * @param g2 
         * @param phacode_consistent 
         * @return double 
         */
        double MW(const gnss_data_obs &g1, const gnss_data_obs &g2,     // get Melbourne-Wuebenna combination for phase & code [m]!
                  bool phacode_consistent = false) const; // --> select phase observation + (optionally) use for code as well

        /**
         * @brief 
         * 
         * @param gL1 
         * @param gL2 
         * @param gC1 
         * @param gC2 
         * @return double 
         */
        double MW_cycle(const gnss_data_obs &gL1, const gnss_data_obs &gL2,
                        const gnss_data_obs &gC1, const gnss_data_obs &gC2) const; // get Melbourne-Wuebenna combination for phase & code [cycle]!);

        /**
         * @brief 
         * 
         * @param gL1 
         * @param gL2 
         * @param gL3 
         * @param gC1 
         * @param gC2 
         * @return double 
         */
        double EWL_cycle(const gnss_data_obs &gL1, const gnss_data_obs &gL2, const gnss_data_obs &gL3,
                         const gnss_data_obs &gC1, const gnss_data_obs &gC2) const; // get extra-wide-lane Melbourne-Wuebenna combination for phase & code [cycle]!);

        /**
         * @brief 
         * 
         * @param gL1 
         * @param gL2 
         * @return double 
         */
        double LW_meter(const gnss_data_obs &gL1, const gnss_data_obs &gL2) const; // get LW combination for phase [meter]!);

        /**
         * @brief 
         * 
         * @param gL2 
         * @param gL3 
         * @return double 
         */
        double LE_meter(const gnss_data_obs &gL2, const gnss_data_obs &gL3) const; // get LE combination for phase [meter]!);

        /**
         * @brief 
         * 
         * @param gL1 
         * @param gL3 
         * @return double 
         */
        double LWL_factor13(const gnss_data_obs &gL1, const gnss_data_obs &gL3) const; // get LWL combination factor);
        // NOVE FUNKCE!
        // --------------------------------------------------------------------------------------------
        /**
         * @brief 
         * 
         * @return base_time 
         */
        const hwa_base::base_time &epoch() const { return _epoch; } ///< get reference epoch
        /**
         * @brief 
         * 
         * @return std::string 
         */
        const std::string &site() const { return _staid; } ///< get station id
        /**
         * @brief 
         * 
         * @return std::string 
         */
        const std::string &sat() const { return _satid; } ///< get satellite id
        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string sys() const { return _satid.substr(0, 1); } ///< get satellite system id
        /**
         * @brief 
         * 
         * @return std::map<GOBS, int> 
         */
        const std::map<GOBS, int> &lli() const { return _glli; } ///< get satellite lost-of-lock identifications (add by xiongyun)
        /**
         * @brief 
         * 
         * @return std::map<GOBS, int> 
         */
        const std::map<GOBS, int> &slip() const { return _gslip; } ///< get mapS of estimated cycle slips (add by xiongyun)

        /**
         * @brief 
         * 
         * @param id 
         */
        void sat(const std::string &id)
        {
            _satid = id;
            _gsys = gnss_sys::char2gsys(id[0]);
        } // std::set satellite id

        /**
         * @brief 
         * 
         * @param id 
         */
        void site(const std::string &id) { _staid = id; } ///< std::set site id

        /**
         * @brief 
         * 
         * @param t 
         */
        void epo(const hwa_base::base_time &t) { _epoch = t; } ///< std::set epoch

        ///**
        // * @brief
        // *
        // * @param obs
        // * @return gnss_data_obs_manager
        // */
        //gnss_data_obs_manager operator-(gnss_data_obs_manager &obs);

        /**
         * @brief 
         * 
         */
        void clear();

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool valid() const;

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool obs_empty() const;

        /**
         * @brief 
         * 
         * @return std::set<GFRQ> 
         */
        std::set<GFRQ> freq_avail() const; // get available freq  ---> NOVA IMPLEMENTACE !!!!!

        /**
         * @brief 
         * 
         * @param _phase 
         * @return std::set<GOBSBAND> 
         */
        std::set<GOBSBAND> band_avail(bool _phase = true) const; // get available band  ---> NOVA IMPLEMENTACE !!!!!

        /**
         * @brief 
         * 
         * @param freq 
         * @return true 
         * @return false 
         */
        bool contain_freq(const FREQ_SEQ &freq) const;

        // OLD
        // ---------------------
        // double MW(GOBSBAND b1=BAND_1, GOBSBAND b2=BAND_2); // get Melbourne-Wuebenna combination for phase & code [m]!
        /**
         * @brief 
         * 
         * @param b1 
         * @param b2 
         * @return double 
         */
        double P3(const GOBSBAND &b1 = BAND_1, const GOBSBAND &b2 = BAND_2) const; ///< get ionosphere-free combination for code  [m]

        /**
         * @brief 
         * 
         * @param b1 
         * @param b2 
         * @return double 
         */
        double P4(const GOBSBAND &b1 = BAND_1, const GOBSBAND &b2 = BAND_2) const; ///< get geometry-free combination for code    [m]

        /**
         * @brief 
         * 
         * @param b1 
         * @param b2 
         * @return double 
         */
        double L3(const GOBSBAND &b1 = BAND_1, const GOBSBAND &b2 = BAND_2) const; ///< get ionosphere-free combination for phase [m]

        /**
         * @brief 
         * 
         * @param b1 
         * @param b2 
         * @return double 
         */
        double L4(const GOBSBAND &b1 = BAND_1, const GOBSBAND &b2 = BAND_2) const; ///< get geometry-free combination for phase   [m]

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int coef_ionofree(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the ionosphere-free linear combination
                          const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int coef_ionofree_phi(const GOBSBAND &b1, double &c1,
                              const GOBSBAND &b2, double &c2) const; ///< return coefficients for carrier phase observtion

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int coef_geomfree(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the geometry-free linear combination
                          const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int coef_narrlane(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the narrow-lane linear combination
                          const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int coef_widelane(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the wide-lane linear combination
                          const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &health() const { return _health; } ///< get sat health

        /**
         * @brief 
         * 
         * @param health 
         */
        void health(bool health) { _health = health; } ///< std::set sat health

    protected:
        /**
         * @brief 
         * 
         * @param go 
         * @return double 
         */
        double _obs_range(const gnss_data_obs &go) const; ///< get  code observation [m] only requested type NEW + AUTO!

        /**
         * @brief 
         * 
         * @param gb 
         * @return double 
         */
        double _obs_range(const gnss_data_band &gb) const; ///< get  code observation [m] only requested type NEW + AUTO!

        /**
         * @brief 
         * 
         * @param gb 
         * @return double 
         */
        double _obs_phase(const gnss_data_band &gb) const; ///< get phase observation [m] only requested type NEW + AUTO !

        /**
         * @brief 
         * 
         * @param gb 
         * @return double 
         */
        double _obs_doppler(const gnss_data_band &gb) const; ///< get doppler observation [m] only requested type NEW + AUTO !

        /**
         * @brief 
         * 
         * @param gb 
         * @return double 
         */
        double _obs_snr(const gnss_data_band &gb) const; ///< get snr observation [dbhz] only requested type NEW + AUTO !

        /**
         * @brief 
         * 
         * @param g1 
         * @param cf1 
         * @param g2 
         * @param cf2 
         * @param g3 
         * @param cf3 
         * @return double 
         */
        double _lcf_range(const gnss_data_obs *g1, const int &cf1,                ///< return value [m] of general pseudo-range
                          const gnss_data_obs *g2, const int &cf2,                ///< LC = c1*f1*O1 + ... / c1*f1 +
                          const gnss_data_obs *g3 = 0, const int &cf3 = 0) const; ///< for 2 or 3 bands with given coefficients  NEW !

        /**
         * @brief 
         * 
         * @param g1 
         * @param cf1 
         * @param g2 
         * @param cf2 
         * @param g3 
         * @param cf3 
         * @return double 
         */
        double _lcf_phase(const gnss_data_obs *g1, const int &cf1,                ///< return value [m] of general carrier-phase
                          const gnss_data_obs *g2, const int &cf2,                ///< LC = c1*f1*O1 + ... / c1*f1 +
                          const gnss_data_obs *g3 = 0, const int &cf3 = 0) const; ///< for 2 or 3 bands with given coefficients   NEW !

        /**
         * @brief 
         * 
         * @param g1 
         * @param cf1 
         * @param g2 
         * @param cf2 
         * @param g3 
         * @param cf3 
         * @return double 
         */
        double _lc_range(const gnss_data_obs *g1, const double &cf1,                ///< return value [m] of general pseudo-range
                         const gnss_data_obs *g2, const double &cf2,                ///< LC = c1*O1 + ...
                         const gnss_data_obs *g3 = 0, const double &cf3 = 0) const; ///< for 2 or 3 bands with given coefficients  NEW !

        /**
         * @brief 
         * 
         * @param g1 
         * @param cf1 
         * @param g2 
         * @param cf2 
         * @param g3 
         * @param cf3 
         * @return double 
         */
        double _lc_phase(const gnss_data_obs *g1, const double &cf1,                ///< return value [m] of general carrier-phase
                         const gnss_data_obs *g2, const double &cf2,                ///< LC = c1*O1 + ...
                         const gnss_data_obs *g3 = 0, const double &cf3 = 0) const; ///< for 2 or 3 bands with given coefficients   NEW !

        /**
         * @brief 
         * 
         * @param bC 
         * @param cC 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int _coef_multpath(const GOBSBAND &bC, double &cC, ///< get c1,c2,c3 coefficients of code multipath LC
                           const GOBSBAND &b1, double &c1, ///< (2 bands, 1x code & 2x phase)
                           const GOBSBAND &b2, double &c2) const;

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int _coef_ionofree(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the ionosphere-free linear combination
                           const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int _coef_geomfree(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the geometry-free linear combination
                           const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int _coef_narrlane(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the narrow-lane linear combination
                           const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @param b1 
         * @param c1 
         * @param b2 
         * @param c2 
         * @return int 
         */
        int _coef_widelane(const GOBSBAND &b1, double &c1,        ///< return coefficients (c1,c2) of the wide-lane linear combination
                           const GOBSBAND &b2, double &c2) const; // (2 bands)

        /**
         * @brief 
         * 
         * @param b 
         * @return GOBS 
         */
        GOBS _id_range(const GOBSBAND &b) const; ///< get  code ID of selected band (according to table)

        /**
         * @brief 
         * 
         * @param b 
         * @return GOBS 
         */
        GOBS _id_phase(const GOBSBAND &b) const; ///< get phase ID of selected band (according to table)

        /**
         * @brief 
         * 
         * @param b 
         * @return GOBS 
         */
        GOBS _id_doppler(const GOBSBAND &b) const; ///< get doppler ID of selected band (according to table)

        /**
         * @brief 
         * 
         * @param b 
         * @return GOBS 
         */
        GOBS _id_snr(const GOBSBAND &b) const; // get snr ID of selected band (according to table)

        /**
         * @brief 
         * 
         * @param band 
         * @return GOBS 
         */
        GOBS _cod_id(const int &band) const; // get  code ID of selected band (according to table)

        /**
         * @brief 
         * 
         * @param band 
         * @return GOBS 
         */
        GOBS _pha_id(const int &band) const; // get phase ID of selected band (according to table)

        /**
         * @brief 
         * 
         */
        virtual void _clear();

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        virtual bool _valid() const;

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        virtual bool _valid_obs() const;

        /**
         * @brief 
         * 
         * @return std::set<GFRQ> 
         */
        std::set<GFRQ> _freq_avail() const; // get available freq  for phase ---> NOVA IMPLEMENTACE !!!!!

        /**
         * @brief 
         * 
         * @return std::set<GOBSBAND> 
         */
        std::set<GOBSBAND> _band_avail() const; // get available bands for phase ---> NOVA IMPLEMENTACE !!!!!

        /**
         * @brief 
         * 
         * @return std::set<GOBSBAND> 
         */
        std::set<GOBSBAND> _band_avail_code() const; // get available bands for code  ---> NOVA IMPLEMENTACE !!!!!

    protected:
        std::map<GOBS, double> _gobs;  // mapS of observations
        std::map<GOBS, int> _glli;     // mapS of lost-of-lock identifications
        std::map<GOBS, int> _gslip;    // mapS of estimated cycle slips
        std::map<GOBS, int> _goutlier; // mapS of gross error of range/doppler observations
        std::map<GOBS, int> _gLevel;   //lvhb

        std::string _staid;          // station id
        std::string _satid;          // satellite id ["G??", "R??", "E??" ...]
        GSYS _gsys;             // system
        hwa_base::base_time _epoch;         // epoch of the observation
        double _apr_ele;        // approximate elevation
        int _channel;           // satellite channel number
        unsigned int _rtcm_end; // RTCM Multiple Message bit (0 = end, 1 = cont.)

        bool _health;

    private:
        bool _isDoubleIf = false;
        bool _dcb_correct_mark; // mark for correct dcb; true: dcb corrected; false: not corrected;
        bool _phase_correct_mark;
        bool _range_smooth_mark;  // true, smooth; false, not smooth
        bool _bds_code_bias_mark; // true, correct; false, not correct
    };

    class gnss_data_obscmb
    {
    public:
        gnss_data_obscmb()
        {
            num = 0.0;
            lam = 0.0;
        };
        double num;
        double lam;
        gnss_data_obs first;
        gnss_data_obs second;

        /**
         * @brief 
         * 
         * @param t 
         * @return true 
         * @return false 
         */
        bool operator<(const gnss_data_obscmb &t) const;
    };

} // namespace

#endif
