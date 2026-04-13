#ifndef hwa_set_out_h
#define hwa_set_out_h
#define XMLKEY_OUT "outputs" ///< The defination of outputs module in xml file
#define DEFAULT_FILE_VER ""    ///< default version for file format
#define DEFAULT_FILE_UPD 0     ///< default auto update [min] for file saving
#define DEFAULT_FILE_LEN 0     ///< default auto length [min] for file saving
#define DEFAULT_FILE_SMP 0     ///< default auto sample [sec] for file saving
#define DEFAULT_FILE_OFF 0     ///< default file offset [min] for file saving
#define DEFAULT_FILE_SYS "UTC" ///< default file system       for file saving
#include <string>
#include <iostream>
#include "hwa_base_typeconv.h"
#include "hwa_base_log.h"
#include "hwa_set_base.h"

using namespace pugi;

namespace hwa_set
{
    ///< the order is important here!
    enum OFMT
    {
        XXX_OUT,
        TSA_OUT,
        MPT_OUT,
        LOG_OUT,
        SUM_OUT,
        XTR_OUT,
        NAV_OUT,
        XML_OUT,
        XQC_OUT,
        PPP_OUT,
        FLT_OUT, //xjhan
        RATIO_OUT,
        KIN_OUT,
        SMT_OUT,
        lsq_OUT,
        RES_OUT,
        PREOBS_OUT,
        GRD_OUT,
        SRF_OUT,
        PRE_OUT,
        RB_OUT,
        STA_OUT,
        KBROMC_OUT,
        LRIOMC_OUT,
        FIT_OUT,
        CFG_OUT,
        RINEXN_OUT,
        RINEXN2_OUT,
        RINEXO_OUT,
        RINEXC_OUT,
        RTCM_OUT,
        BNCOBS_OUT,
        BNCRTCM_OUT,
        SINEX_OUT,
        TROSINEX_OUT,
        TROSINEX0_OUT,
        KML_OUT,
        INS_OUT,
        INSKF_OUT,
        ENU_OUT,
        INSKFPK_OUT,
        PRDQC_OUT,
        CLK_OUT,
        ERP_OUT,
        POLEUT1_OUT,
        ORB_OUT,      ///< orb out
        SP3_OUT,      ///< sp3 out
        RECCLK_OUT,   ///< rec clk out
        RECCLK13_OUT, ///< rec clk out for freq 13(add by xiongyun)
        SATCLK_OUT,   ///< sat clk out
        SATCLK13_OUT, ///< sat clk out for freq 13(add by xiongyun)
        UPD_OUT,      ///< upd out
        AMBUPD_OUT,
        AMBCONLEO_OUT, ///< ambupd out
        RESULT_OUT,
        ICS_OUT,    ///< ics out
        ICSLEO_OUT, ///< icsleo out
        ION_OUT,
        AMBCON_OUT, ///< ambcon out
        ORBDIF_OUT, ///< orbit differ out
        ORBFIT_OUT, ///< orbit fit out
        RECOVER_OUT,
        ATX_OUT, ///< PCV/PCO estimation
        AUG_OUT,
        TROP_OUT,
        AMBFLAG_DIR_OUT,
        PCVNEQ_OUT,
        IMU_OUT, //LVHB ADDED IN 20210312
        GPGGA_OUT,
        IPP_OUT,
        MORBCLK_OUT, // orbit and clock monit

        RSSIMAP_OUT,
        RSSIMATCH_OUT,
        CAMPOS_OUT,
        OBSQUALITY_OUT,
        OBSQUALITY_DIR_OUT
    };

    /// The class for settings of output
    class set_out : public virtual set_base
    {
    public:
        /// constructor
        set_out();
        /// destructor
        ~set_out();

        /**
         * @brief change from std::string to OFMT
         * @param[in] s file format
         * @return OFMT : file format
         */
        static OFMT str2ofmt(const std::string& s);

        /**
         * @brief change from OFMT to std::string
         * @param[in] f file format
         * @return std::string : file format
         */
        static std::string ofmt2str(const OFMT& f);

        /// settings check
        void check();
        /// settings help
        void help();

        // attributes
        /**
         * @brief  get verbosity attribute
         * @return int : verbosity attribute
         */
        int verb();

        /**
         * @brief  get append request
         * @return bool : append request
         */
        bool append();
        int sp3_obslimit();

        // elements
        /**
         * @brief  get format output size
         * @param[in] fmt file format
         * @return int : format output size
         */
        int output_size(const std::string& fmt);

        /**
         * @brief  get std::string outputs
         * @param[in] fmt file format
         * @return std::string : std::string outputs
         */
        std::string outputs(const std::string& fmt);

        /*
         * @brief  get std::string log type
        */
        std::string log_type();

        /*
         * @brief  get std::string log name
        */
        std::string log_name();

        level::level_enum log_level();

        /*
        * @brief  get std::string log pattern
        */
        std::string log_pattern();

        /**
         * output info via tcp port (added by zhshen)
         * return the tcp port
         */
        std::string output_port(const std::string& fmt);

        /**
         * @brief  get formats
         * @return std::set<std::string> : all the outputs
         */
        std::set<std::string> oformats();

        /**
         * @brief  get std::string output version
         * @param[in] fmt file format
         * @return std::string : std::string output version
         */
        std::string version(const std::string& fmt);

        /**
         * @brief  get time offset [min] for output filename
         * @param[in] fmt file format
         * @return int : time offset [min] for output filename
         */
        int file_toff(const std::string& fmt);


        int out_update(const std::string& fmt);

        /**
         * @brief  get length [min] for output file content
         * @param[in] fmt file format
         * @return int : length [min] for output file content
         */
        int out_length(const std::string& fmt);

        /**
         * @brief  get sample [sec] for output file data
         * @param[in] fmt file format
         * @return float : sample [sec] for output file data
         */
        float out_sample(const std::string& fmt);

        // add by glfeng
        std::map<std::string, std::string> out_caster(const std::string& fmt); // Upload Caster

    protected:
        /**
         * @brief  get std::string output file
         * @param[in] fmt file format
         * @return std::string : std::string output file
         */
        std::string _outputs(const std::string& fmt);

        /**
         * @brief  get all std::string output file
         * @return std::set<std::string> : all std::string output file
         */
        std::set<std::string> _oformats();

        std::set<OFMT> _OFMT_supported; ///< std::vector of supported OFMTs (app-specific)
        bool _append;              ///< append mode
        int _verb;                 ///< output verbosity
        int _upd;                  ///< update [min] for output file update
        int _len;                  ///< length [min] for output file content
        float _smp;                ///< sample [sec] for output file data

    private:
    };

} // namespace
#endif
