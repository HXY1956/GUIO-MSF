#ifndef hwa_set_inp_h
#define hwa_set_inp_h
#define XMLKEY_INP "inputs"
#include <map>
#include <string>
#include <iostream>
#include <vector>
#include "hwa_set_base.h"
#include "hwa_base_log.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_time.h"

using namespace pugi;

namespace hwa_gnss {
    /// input file order is important here!
    enum class IFMT : int
    {
        ACCELEROMETER_INP,
        ALBEDO_INP,
        AMBIGUITYLEO_INP, /// ambcon file
        AMBIGUITY_INP,    /// ambcon file
        AMBFLAG12_INP,    ///< ambflag file
        AMBFLAG13_INP,    ///< ambflag13 file
        AMBFLAG14_INP,    ///< ambflag14 file
        AMBFLAG15_INP,    ///< ambflag15 file
        AMBINP_INP,       /// ambinp file
        AMBUPD_INP,       ///< ambupd file
        APL_INP,          /// atmosphere non-tidal loading, added by Qian Zhang(VLBI)
        LEOATT_INP,       ///< attitude file
        ATX_INP,          ///< atx file, need to be before all OBSERVATIONS to create objects
        AUG_INP,
        AUGGRID_INP,
        BIAS_INP,
        BIASINEX_INP,
        BLQ_INP,
        BNCBRDC_INP, ///< added by zhShen on 20191025
        BNCCORR_INP, ///< added by zhShen on 20191025
        BNCOBS_INP,  ///< need to be after RINEXN (for GLONASS chanels)!
        BNCRTCM_INP, // need to be after RINEXN to fill the corrections !
        BNCRTCM_PRD,
        BNCRTCM_REF,
        BNCSEND_INP, ///< added by xiongyun
        BRIEF_PATTERN_INP,
        BRIEF_VOCABULARY_INP,
        COM_INP,
        DESAISCOPOLECOEF_INP,
        DE_INP,      ///< DE405 file
        GRAVITY_INP, ///< EGM file
        FCB_INP,
        FEATURE_INP,
        GRAD_INP, /// troposphere gradient, added by Qian Zhang(VLBI)
        GRIB_CHMI_INP,
        GRIB_ERA_INP,

        GRIB_HARM_INP,
        IOPLEO_INP, ///< ics file for leo
        IOPNAV_INP, ///< ics file for gns
        IFCB_INP,
        IMAGE_INP,
        IMU_INP,     ///< imu file for inertial navigation system (by zhshen)
        IMU_POS_INP, ///< GNSS positon and velocity file for loose-coupled integration (by zhshen)
        IONEX_INP,
        ION_INP, ///< ion file for gns
        KBR_INP,
        KIN_INP,
        LEAPSECOND_INP, ///< leapsecond file
        LEOPANNEL_INP,  ///< pannel file for leo
        LIDAR_INP,
        LRA_INP,
        LRI_INP,      ///GRACE-FO LRI
        MF_INP,       /// troposphere mapping function, added by Qian Zhang(VLBI)
        MANEUVER_INP, ///< orbit maneuver information file
        NCD3_ICS_INP,
        NGS_INP, /// VLBI observation file, added by Qian Zhang(VLBI)
        NPT2_INP,
        NPT_INP,
        OPL_INP,       // ocean pole loading
        OCEANTIDE_INP, ///< oceantide file
        ODO_INP,       ///< odo file (by zhshen)
        OPT_INP,       /// VLBI options file, added by Qian Zhang(VLBI)
        ORBITLEO_INP,  ///< orb file for leo
        ORBIT_INP,     ///< orb file for gns
        OUTLIER_INP,   /// outliers of VLBI observations, added by Qian Zhang(VLBI)
        PANDA_RECOVER_INP,
        PANNEL_INP, ///< pannel file for leo
        PCVNEQ_INP,
        EOP_INP, ///< poleut file
        PREOBS_INP,
        PSD_INP,
        RAO_BADC_INP,
        RAO_IGRA_INP,
        RAO_TEXT_INP,
        RAYTR_INP, /// ray-traced file, added by Qian Zhang(VLBI)
        RB_INP,
        RECOVER_INP, /// RESFILE
        RINEXCLEO_INP,
        RINEXCSIMU_INP, //xjhan
        RINEXC_INP,
        RINEXC_PRD,
        RINEXC_REF,
        RINEXM_INP,
        RINEXN_INP,
        RINEXN_PRD,
        RINEXN_REF,
        RINEXOLEO_INP,
        RINEXO_INP,  ///< rinexo file, need to be after RINEXN (for GLONASS chanels)!
        RSSIMAP_INP, /// WIFI/BLE MAP file
        RSSIMSG_INP, /// WIFI/BLE RSSI RSSI file
        RTCM_INP,
        SATINFO_INP, ///< sat parameters file
        SINEX_INP,
        SLRF_INP,
        SOLAR_INP, ///< solar parameters file
        SP3LEO_INP,
        SP3SIMU_INP,
        SP3_INP,
        SP3_PRD,
        SP3_REF,
        SSRCLK_INP, ///< added by xiongyun
        STA_INP,
        TAG_INP, /// img tag file
        TROGRID_INP,
        TROSINEX0_INP,
        TROSINEX_INP,
        TSMET_INP,
        UPD_EPOCH_INP,
        UPD_INP, ///< upd file
        UWBMSG_INP,
        VGOSDB_INP, /// added by Qian Zhang(VLBI)
        VLBIANTL_INP,
        VLBIANT_INP,
        VLBIATL_INP,
        VLBIBLQ_INP,
        VLBICLK_INP,
        VLBIECC_INP,
        VLBIEOP_INP,
        VLBIOPL_INP,
        VLBIPSD_INP,
        VLBISNX_INP,
        VLBISOU_INP,
        VLBIVMF_INP,
        VSO_INP, /// added by Qian Zhang(VLBI)
        XTR_INP,
        PPPXML_INP,
        RTKXML_INP,
        IGNXML_INP,
        IPP_INP,
        SLRPSD_INP,
        GNSSPSD_INP,
        ATMLOADING_INP,
        ATT_INP,
        MORBCLK_INP, // orbit and clock monit file, jdhuang
        UNDEF = -1
    };
};

using namespace hwa_gnss;

namespace hwa_set
{
    /// The class for input settings
    class set_inp : public virtual set_base
    {
    public:
        /// constructor
        set_inp();
        /// destructor
        ~set_inp();

        /// settings check
        void check();
        /// settings help
        void help();

        /**
         * @brief change from string to IFMT
         * @param[in] s file format
         * @return IFMT : file format
         */
        static IFMT str2ifmt(const std::string &s);

        /**
         * @brief check if rt ifmt data
         * @param[in] s file format
         * @return IFMT : file format
         */
        static bool isRtIfmt(const IFMT &f);

        /**
         * @brief change from IFMT to string
         * @param[in] f file format
         * @return string : file format
         */
        static std::string ifmt2str(const IFMT &f);

        /**
         * @brief get format input size
         * @param[in] fmt file format
         * @return int : format input size
         */
        int input_size(const std::string &fmt);

        // jdhuang : add for check input
        bool check_input(const std::string &fmt);

        void check_input(const std::string &fmt, const std::string &message);
        /**
         * @brief get format inputs (all in multimap)
         * @return multimap<IFMT, string> : format inputs (all in multimap)
         */
        std::multimap<IFMT, std::string> inputs_all();

        // jdhuang
        std::multimap<IFMT, std::string> inputs_all(base_time ref);
        std::multimap<IFMT, std::string> replace_all(base_time ref, const std::set<std::string> &sats, const std::set<std::string> &recs, const std::multimap<IFMT, std::string> &inputs);

        /**
         * @brief get format inputs (ordered)
         * @param[in] fmt file format
         * @return vector<string> : format inputs (ordered)
         */
        std::vector<std::string> inputs(const std::string &fmt);
        std::vector<std::string> inputs(const IFMT &ifmt);

        /**
         * @brief get formats       (ordered)
         * @return set<string> :    formats (ordered)
         */
        std::set<std::string> iformats();

        /**
         * @brief get rtcm correction stream
         * @return string : rtcm correction stream
         */
        std::string corrStream();

        std::string rinexo();

        std::string bncobs();

        std::string bncbrdc();

        std::string bnccorr();

        std::string pppxml();

        std::string rtkxml();

		int ignxml(std::set<std::string>& xmls);

        /// checking navigation Rinex
        bool chkNavig();
        /// checking satellite healthy status
        bool chkHealth();

        float inp_sample(const std::string &fmt);

        std::map<std::string, std::string> inp_caster(const std::string &fmt); // Download Caster

    protected:
        /**
         * @brief get all the file name of ftm
         * @param[in] ftm file format
         * @return vector<string> : all the file name of ftm
         */
        std::vector<std::string> _inputs(const std::string &fmt);

        // jdhuang :
        // add for get fmt file
        std::vector<std::string> _inputs(const IFMT &fmt);

        /**
         * @brief get all the ftm in input node
         * @return set<string> : all the ftm in input node
         */
        std::set<std::string> _iformats();

    protected:
        std::set<IFMT> _IFMT_supported; ///< vector of supported IFMTs (app-specific)

        bool _chkNavig;  ///< check navigation data or not
        bool _chkHealth; ///< check data health or not

        std::string _corrStream; ///< error message string

    private:
    };

} // namespace

#endif
