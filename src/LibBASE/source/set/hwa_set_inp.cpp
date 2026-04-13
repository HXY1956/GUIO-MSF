#include "hwa_set_inp.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_mutex.h"

using namespace pugi;

namespace hwa_set
{
    // check for RT formats
    // ----------
    bool set_inp::isRtIfmt(const IFMT &f)
    {
        bool isRt = false;
        switch (f)
        {
            case IFMT::BNCBRDC_INP: ///< added by zhShen on 20191025
            case IFMT::BNCCORR_INP: ///< added by zhShen on 20191025
            case IFMT::BNCOBS_INP :  ///< need to be after RINEXN (for GLONASS chanels)!
            case IFMT::BNCRTCM_INP: // need to be after RINEXN to fill the corrections !
            case IFMT::BNCRTCM_PRD:
            case IFMT::BNCRTCM_REF:
            case IFMT::BNCSEND_INP: ///< added by xiongyun
                isRt = true;
                break;
            default:
                isRt = false;
        }
        return isRt;

    }

    // Convertor for INP formats
    // ----------
    IFMT set_inp::str2ifmt(const std::string &s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "RINEXC" || tmp == "RNC")
            return IFMT::RINEXC_INP;

        if (tmp == "RINEXC_PRD")
            return IFMT::RINEXC_PRD;
        if (tmp == "RINEXC_REF")
            return IFMT::RINEXC_REF;
        if (tmp == "RINEXO" || tmp == "RNO")
            return IFMT::RINEXO_INP;

        if (tmp == "RINEXN" || tmp == "RNN")
            return IFMT::RINEXN_INP;

        if (tmp == "RINEXN_PRD")
            return IFMT::RINEXN_PRD;
        if (tmp == "RINEXN_REF")
            return IFMT::RINEXN_REF;

        if (tmp == "RINEXM")
            return IFMT::RINEXM_INP;
        if (tmp == "BNCOBS")
            return IFMT::BNCOBS_INP;
        if (tmp == "BNCBRDC")
            return IFMT::BNCBRDC_INP;
        if (tmp == "BNCCORR")
            return IFMT::BNCCORR_INP;
        if (tmp == "BNCSEND")
            return IFMT::BNCSEND_INP;
        if (tmp == "BNCRTCM")
            return IFMT::BNCRTCM_INP;
        if (tmp == "BNCRTCM_PRD")
            return IFMT::BNCRTCM_PRD;
        if (tmp == "BNCRTCM_REF")
            return IFMT::BNCRTCM_REF;
        if (tmp == "TROSINEX")
            return IFMT::TROSINEX_INP;
        if (tmp == "TROSINEX0")
            return IFMT::TROSINEX0_INP;
        if (tmp == "SINEX")
            return IFMT::SINEX_INP;
        if (tmp == "TROGRID")
            return IFMT::TROGRID_INP;
        if (tmp == "RTCM")
            return IFMT::RTCM_INP;
        if (tmp == "TSMET")
            return IFMT::TSMET_INP;
        if (tmp == "SP3")
            return IFMT::SP3_INP;
        if (tmp == "SP3LEO")
            return IFMT::SP3LEO_INP;
        if (tmp == "SP3_PRD")
            return IFMT::SP3_PRD;
        if (tmp == "SP3_REF")
            return IFMT::SP3_REF;
        if (tmp == "ATX")
            return IFMT::ATX_INP;
        if (tmp == "SSRCLK")
            return IFMT::SSRCLK_INP; // add by xiongyun
        if (tmp == "BLQ")
            return IFMT::BLQ_INP;
        if (tmp == "VLBIBLQ")
            return IFMT::VLBIBLQ_INP;
        if (tmp == "VLBIATL")
            return IFMT::VLBIATL_INP;
        if (tmp == "VLBIANTL")
            return IFMT::VLBIANTL_INP;
        if (tmp == "VLBIOPL" || tmp == "OPL" || tmp == "opl")
            return IFMT::OPL_INP;
        if (tmp == "MANEUVER")
            return IFMT::MANEUVER_INP;
        if (tmp == "VLBIEOP")
            return IFMT::VLBIEOP_INP;
        if (tmp == "SLRPSD")
            return IFMT::SLRPSD_INP;
        if (tmp == "GNSSPSD")
            return IFMT::GNSSPSD_INP;
        if (tmp == "S1S2")
            return IFMT::ATMLOADING_INP;
        if (tmp == "FCB")
            return IFMT::FCB_INP;
        if (tmp == "RB")
            return IFMT::RB_INP;
        if (tmp == "STA")
            return IFMT::STA_INP;
        if (tmp == "KBR")
            return IFMT::KBR_INP;
        if (tmp == "LRI")
            return IFMT::LRI_INP;
        if (tmp == "ACCELEROMETER")
            return IFMT::ACCELEROMETER_INP;
        if (tmp == "GRIB_CHMI")
            return IFMT::GRIB_CHMI_INP;
        if (tmp == "GRIB_HARM")
            return IFMT::GRIB_HARM_INP;
        if (tmp == "GRIB_ERA")
            return IFMT::GRIB_ERA_INP;
        if (tmp == "NCD3_ICS")
            return IFMT::NCD3_ICS_INP;

        if (tmp == "RAO_TEXT")
            return IFMT::RAO_TEXT_INP;
        if (tmp == "RAO_IGRA")
            return IFMT::RAO_IGRA_INP;
        if (tmp == "RAO_BADC")
            return IFMT::RAO_BADC_INP;

        if (tmp == "BIASINEX")
            return IFMT::BIASINEX_INP;
        if (tmp == "BIAS" || tmp == "BIABERN")
            return IFMT::BIAS_INP;

        if (tmp == "IONEX")
            return IFMT::IONEX_INP;

        if (tmp == "AUG")
            return IFMT::AUG_INP;
        if (tmp == "AUG_GRID")
            return IFMT::AUGGRID_INP;
        if (tmp == "DE")
            return IFMT::DE_INP;

        if (tmp == "OCEANTIDE")
            return IFMT::OCEANTIDE_INP;
        if (tmp == "POLETIDE")
            return IFMT::DESAISCOPOLECOEF_INP;
        if (tmp == "OCEANTIDE")
            return IFMT::OCEANTIDE_INP;

        if (tmp == "GRAVITY")
            return IFMT::GRAVITY_INP;
        if (tmp == "PANNEL")
            return IFMT::PANNEL_INP;
        if (tmp == "SOLARFLUX" || tmp == "GEOMAGINDEX")
            return IFMT::SOLAR_INP;
        if (tmp == "IOPNAV")
            return IFMT::IOPNAV_INP;
        if (tmp == "ION")
            return IFMT::ION_INP;

        // special for POD
        if (tmp == "ORBIT")
            return IFMT::ORBIT_INP;
        if (tmp == "ORBIT_LEO")
            return IFMT::ORBITLEO_INP;
        if (tmp == "IOPLEO")
            return IFMT::IOPLEO_INP;
        if (tmp == "EOP" || tmp == "POLEUT1") 
            return IFMT::EOP_INP; // optinal for xml node

        if (tmp == "LEAPSECOND")
            return IFMT::LEAPSECOND_INP;

        if (tmp == "SATINFO")
            return IFMT::SATINFO_INP;

        if (tmp == "LEOATT")
            return IFMT::LEOATT_INP;
        if (tmp == "AMBIGUITY")
            return IFMT::AMBIGUITY_INP;
        if (tmp == "AMBIGUITY_LEO")
            return IFMT::AMBIGUITYLEO_INP;

        if (tmp == "ALBEDO")
            return IFMT::ALBEDO_INP;
        if (tmp == "LEO_PANNEL")
            return IFMT::LEOPANNEL_INP;
        if (tmp == "AMBUPD")
            return IFMT::AMBUPD_INP;

        if (tmp == "AMBFLAG12")
            return IFMT::AMBFLAG12_INP;
        if (tmp == "AMBFLAG13")
            return IFMT::AMBFLAG13_INP;
        if (tmp == "AMBFLAG14")
            return IFMT::AMBFLAG14_INP;
        if (tmp == "AMBFLAG15")
            return IFMT::AMBFLAG15_INP;

        if (tmp == "UPD")
            return IFMT::UPD_INP;
        if (tmp == "XTR")
            return IFMT::XTR_INP;

        // special for recover pars
        if (tmp == "RECOVER")
            return IFMT::RECOVER_INP;
        if (tmp == "PANDA_RECOVER")
            return IFMT::PANDA_RECOVER_INP;
        if (tmp == "PREOBS")
            return IFMT::PREOBS_INP;

        if (tmp == "IMU")
            return IFMT::IMU_INP; // by zhshen
        if (tmp == "ODO")
            return IFMT::ODO_INP; // by zhshen
        if (tmp == "IMAGE")
            return IFMT::IMAGE_INP;
        if (tmp == "LIDAR")
            return IFMT::LIDAR_INP;
        if (tmp == "FEATURE")
            return IFMT::FEATURE_INP;

        if (tmp == "IMU_POS")
            return IFMT::IMU_POS_INP;
        if (tmp == "IFCB")
            return IFMT::IFCB_INP;
        if (tmp == "LRA")
            return IFMT::LRA_INP;
        if (tmp == "COM")
            return IFMT::COM_INP;
        if (tmp == "NPT")
            return IFMT::NPT_INP;
        if (tmp == "NPT2")
            return IFMT::NPT2_INP;
        if (tmp == "SLRF")
            return IFMT::SLRF_INP;
        if (tmp == "NGS")
            return IFMT::NGS_INP;
        if (tmp == "VLBISNX")
            return IFMT::VLBISNX_INP;
        if (tmp == "VLBIANT")
            return IFMT::VLBIANT_INP;
        if (tmp == "VLBIPSD")
            return IFMT::VLBIPSD_INP;
        if (tmp == "VLBIECC")
            return IFMT::VLBIECC_INP;
        if (tmp == "VLBIVMF")
            return IFMT::VLBIVMF_INP;
        if (tmp == "VLBISOU")
            return IFMT::VLBISOU_INP;
        if (tmp == "VLBICLK")
            return IFMT::VLBICLK_INP;
        if (tmp == "PCVNEQ")
            return IFMT::PCVNEQ_INP;
        if (tmp == "SP3_SIMU")
            return IFMT::SP3SIMU_INP; //xjhan
        if (tmp == "RINEXC_SIMU")
            return IFMT::RINEXCSIMU_INP;
        if (tmp == "PSD")
            return IFMT::PSD_INP;
        if (tmp == "UWB")
            return IFMT::UWBMSG_INP;
        if (tmp == "RSSIMSG")
            return IFMT::RSSIMSG_INP;
        if (tmp == "RSSIMAP")
            return IFMT::RSSIMAP_INP;
        if (tmp == "TAG")
            return IFMT::TAG_INP;
        if (tmp == "IPP")
            return IFMT::IPP_INP;
        if (tmp == "PPPXML")
            return IFMT::PPPXML_INP;
        if (tmp == "RTKXML")
            return IFMT::RTKXML_INP;
        if (tmp == "ATT")
            return IFMT::ATT_INP;		if (tmp == "IGNXML")
			return IFMT::IGNXML_INP;
		if (tmp == "RINEXCSIMU")
			return IFMT::RINEXCSIMU_INP;
		if (tmp == "SP3SIMU")
			return IFMT::SP3SIMU_INP;

        std::string message = "The Type : " + tmp + " is not support, check your xml";
        spdlog::warn(message);
        throw std::logic_error(message);
    }


    // Convertor for INP formats
    // ----------
    std::string set_inp::ifmt2str(const IFMT &f)
    {
        switch (f)
        {
        case IFMT::RINEXO_INP:
            return "RINEXO";
        case IFMT::RINEXC_INP:
            return "RINEXC";
        case IFMT::RINEXC_PRD:
            return "IFMT::RINEXC_PRD";
        case IFMT::RINEXC_REF:
            return "RINEXC_REF";
        case IFMT::RINEXN_INP:
            return "RINEXN";
        case IFMT::RINEXN_PRD:
            return "RINEXN_PRD";
        case IFMT::RINEXN_REF:
            return "RINEXN_REF";
        case IFMT::RINEXM_INP:
            return "RINEXM";
        case IFMT::BNCOBS_INP:
            return "BNCOBS";
        case IFMT::BNCBRDC_INP:
            return "BNCBRDC";
        case IFMT::BNCCORR_INP:
            return "BNCCORR";
        case IFMT::BNCSEND_INP:
            return "BNCSEND"; // add by xiongyun
        case IFMT::BNCRTCM_INP:
            return "BNCRTCM";
        case IFMT::BNCRTCM_PRD:
            return "BNCRTCM_PRD";
        case IFMT::BNCRTCM_REF:
            return "BNCRTCM_REF";
        case IFMT::SSRCLK_INP:
            return "SSRCLK"; // add by xiongyun
        case IFMT::TROSINEX_INP:
            return "TROSINEX";
        case IFMT::TROSINEX0_INP:
            return "TROSINEX0";
        case IFMT::SINEX_INP:
            return "SINEX";
        case IFMT::TROGRID_INP:
            return "TROGRID";
        case IFMT::RTCM_INP:
            return "RTCM";
        case IFMT::OPL_INP:
            return "OPL";
        case IFMT::MANEUVER_INP:
            return "MANEUVER";
        case IFMT::SLRPSD_INP:
            return "SLRPSD";
        case IFMT::GNSSPSD_INP:
            return "GNSSPSD";
        case IFMT::ATMLOADING_INP:
            return "s1s2";
        case IFMT::TSMET_INP:
            return "TSMET";
        case IFMT::SP3_INP:
            return "SP3";
        case IFMT::SP3LEO_INP:
            return "SP3LEO";
        case IFMT::PREOBS_INP:
            return "PREOBS";
        case IFMT::LRA_INP:
            return "LRA";
        case IFMT::COM_INP:
            return "COM";
        case IFMT::RB_INP:
            return "RB";
        case IFMT::KBR_INP:
            return "KBR";
        case IFMT::STA_INP:
            return "STA";
        case IFMT::NPT_INP:
            return "NPT";
        case IFMT::NPT2_INP:
            return "NPT2";
        case IFMT::SLRF_INP:
            return "SLRF";
        case IFMT::NGS_INP:
            return "NGS";
        case IFMT::VLBISNX_INP:
            return "VLBISNX";
        case IFMT::VLBIANT_INP:
            return "VLBIANT";
        case IFMT::VLBIPSD_INP:
            return "VLBIPSD";
        case IFMT::VLBIECC_INP:
            return "VLBIECC";
        case IFMT::VLBIVMF_INP:
            return "VLBIVMF";
        case IFMT::VLBISOU_INP:
            return "VLBISOU";
        case IFMT::VLBICLK_INP:
            return "VLBICLK";
        case IFMT::ION_INP:
            return "ION";
        case IFMT::SP3_PRD:
            return "SP3_PRD";
        case IFMT::SP3_REF:
            return "SP3_REF";
        case IFMT::ATX_INP:
            return "ATX";
        case IFMT::BLQ_INP:
            return "BLQ";
        case IFMT::VLBIBLQ_INP:
            return "VLBIBLQ";
        case IFMT::VLBIATL_INP:
            return "VLBIATL";
        case IFMT::VLBIANTL_INP:
            return "VLBIANTL";
        case IFMT::VLBIOPL_INP:
            return "VLBIOPL";
        case IFMT::VLBIEOP_INP:
            return "VLBIEOP";
        case IFMT::FCB_INP:
            return "FCB";
        case IFMT::GRIB_CHMI_INP:
            return "GRIB_CHMI";
        case IFMT::GRIB_HARM_INP:
            return "GRIB_HARM";
        case IFMT::GRIB_ERA_INP:
            return "GRIB_ERA";
        case IFMT::NCD3_ICS_INP:
            return "NCD3_ICS";
        case IFMT::RAO_TEXT_INP:
            return "RAO_TEXT";
        case IFMT::RAO_IGRA_INP:
            return "RAO_IGRA";
        case IFMT::RAO_BADC_INP:
            return "RAO_BADC";
        case IFMT::BIASINEX_INP:
            return "BIASINEX";
        case IFMT::BIAS_INP:
            return "BIAS";
        case IFMT::IONEX_INP:
            return "IONEX";
        case IFMT::IMU_INP:
            return "IMU";
        case IFMT::ODO_INP:
            return "ODO";
        case IFMT::IMAGE_INP:
            return "IMAGE";
        case IFMT::FEATURE_INP:
            return "FEATURE";
        case IFMT::LIDAR_INP:
            return "LIDAR";
        case IFMT::IMU_POS_INP:
            return "POS";
        case IFMT::AMBUPD_INP:
            return "AMBUPD";
        case IFMT::AMBFLAG12_INP:
            return "AMBFLAG";
        case IFMT::AMBFLAG13_INP:
            return "AMBFLAG13";
        case IFMT::AMBFLAG14_INP:
            return "AMBFLAG14";
        case IFMT::AMBFLAG15_INP:
            return "AMBFLAG15";
        case IFMT::AMBIGUITY_INP:
            return "AMBIGUITY";
        case IFMT::AMBIGUITYLEO_INP:
            return "AMBIGUITY_LEO";
        case IFMT::UPD_INP:
            return "UPD";
        case IFMT::RECOVER_INP:
            return "RECOVER";
        case IFMT::PANDA_RECOVER_INP:
            return "PANDA_RECOVER";
        case IFMT::IFCB_INP:
            return "IFCB";
        case IFMT::LEAPSECOND_INP:
            return "LEAPSECOND";
        case IFMT::SOLAR_INP:
            return "SOLARFLUX_GEOMAGINDEX";
        case IFMT::PANNEL_INP:
            return "PANNEL";
        case IFMT::LEOATT_INP:
            return "LEOATT";
        case IFMT::DESAISCOPOLECOEF_INP:
            return "POLETIDE";
        case IFMT::ALBEDO_INP:
            return "EARTH_ALBEDO";
        case IFMT::DE_INP:
            return "DE";
        case IFMT::ACCELEROMETER_INP:
            return "accelerometer";
        case IFMT::PCVNEQ_INP:
            return "PCVNEQ";
        case IFMT::PSD_INP:
            return "psd";
        case IFMT::UWBMSG_INP:
            return "UWB";
        case IFMT::RSSIMSG_INP:
            return "RSSI";
        case IFMT::RSSIMAP_INP:
            return "RSSIMAP";
        case IFMT::TAG_INP:
            return "TAG";
        case IFMT::PPPXML_INP:
            return "PPPXML";
        case IFMT::RTKXML_INP:
            return "RTKXML";
        case IFMT::ATT_INP:
            return "ATT";		case IFMT::IGNXML_INP:
			return "IGNXML";
		case IFMT::EOP_INP:
			return "EOP";
		case IFMT::RINEXCSIMU_INP:
			return "RINEXCSIMU";
		case IFMT::SP3SIMU_INP:
			return "SP3SIMU";
        default:
            spdlog::critical("No fmt for {}, check your inp.", f);
            throw std::logic_error("check your inp");
        }
    }

    // Constructor
    // ----------
    set_inp::set_inp()
        : set_base()
    {
        _set.insert(XMLKEY_INP);
        _chkNavig = true;
        _chkHealth = true;
        _corrStream = "";
    }

    // Destructor
    // ----------
    set_inp::~set_inp()
    {
    }

    // Get formats input size
    // ----------
    int set_inp::input_size(const std::string &fmt)
    {
        int tmp = _inputs(fmt).size();
        return tmp;
    }

    // jdhuang : add for check input
    bool set_inp::check_input(const std::string &fmt)
    {
        int tmp = _inputs(fmt).size();
        return (tmp > 0);
    }

    void set_inp::check_input(const std::string &fmt, const std::string &message)
    {
        int tmp = _inputs(fmt).size();
        if (tmp < 0)
        {
            spdlog::critical(message);
            throw std::logic_error(message);
        }
    }

    // Get formats inputs (all in multimap)
    // ----------
    std::multimap<IFMT, std::string> set_inp::inputs_all()
    {
        std::multimap<IFMT, std::string> map;

        std::set<std::string> ifmt = _iformats();
        std::set<std::string>::const_iterator itFMT = ifmt.begin();

        while (itFMT != ifmt.end())
        {
            std::string fmt = *itFMT;
            if (fmt.empty())
            {
                itFMT++;
                continue; // jdhuang, empty std::string will be skip
            }

            IFMT ifmt = str2ifmt(fmt);
            std::vector<std::string> inputs = _inputs(fmt); //get file name in input node
            std::vector<std::string>::const_iterator itINP = inputs.begin();
            while (itINP != inputs.end())
            {
                map.insert(map.end(), std::pair<IFMT, std::string>(ifmt, *itINP));
                itINP++;
            }
            itFMT++;
        }
        return map;
    }

    std::multimap<IFMT, std::string> set_inp::replace_all(base_time ref, const std::set<std::string> &sats, const std::set<std::string> &recs, const std::multimap<IFMT, std::string> &inputs)
    {
        std::multimap<IFMT, std::string> tmp;

        // jdhuang :
        // [YYYY] : year           ex. 0001-2019
        // [YY]   : year           ex.   01-  99
        // [DOY]  : doy of year    ex.  001- 365
        // [MM]   : month          ex.   01-  12
        // [DD]   : day of month   ex.   01-  30
        // [SAT]  : satellite      ex.  G01- J01
        // [STA]  : station        ex. ALGO-XXXX

        std::string str_YYYY("[YYYY]");
        std::string str_YY("[YY]");
        std::string str_DOY("[DOY]");
        std::string str_MM("[MM]");
        std::string str_DD("[DD]");
        std::string str_SAT("[SAT]");
        std::string str_STA("[STA]");
        for (const auto &input : inputs)
        {
            // IFMT ifmt = input.first;
            std::string path = input.second;
            if (path.find(str_YYYY) != std::string::npos)
            {
                //base_type_conv::substitute(path, str_YYYY,);
            }
        }

        return std::multimap<IFMT, std::string>();
    }

    // Get formats inputs
    // ----------
    std::vector<std::string> set_inp::inputs(const std::string &fmt)
    {

        // jdhuang : fix
        // change :
        // return _inputs(fmt);
        // to :
        IFMT ifmt = str2ifmt(fmt);
        return _inputs(ifmt);
    }

    std::vector<std::string> set_inp::inputs(const IFMT &ifmt)
    {
        return _inputs(ifmt);
    }

    // Get input formats
    // ----------
    std::set<std::string> set_inp::iformats()
    {
        return _iformats();
    }

    // Get correction stream
    // ----------
    std::string set_inp::corrStream()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("bncrtcm").attribute("stream").value();
        return tmp;
    }

    std::string set_inp::rinexo()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("rinexo").child_value();
        return tmp;
    }

    std::string set_inp::bncobs()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("bncobs").child_value();
        return tmp;
    }

    std::string set_inp::bncbrdc()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("bncbrdc").child_value();
        return tmp;
    }

    std::string set_inp::bnccorr()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("bnccorr").child_value();
        return tmp;
    }

    std::string set_inp::pppxml()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("pppxml").child_value();
        return tmp;
    }

    std::string set_inp::rtkxml()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("rtkxml").child_value();
        return tmp;
    }

	int set_inp::ignxml(std::set<std::string>& xmls)
	{
		std::string str;
		std::string tmp = base_type_conv::trim(_doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("ignxml").child_value());
		//tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("ignxml").child_value();
		std::istringstream is(tmp);
		while (is >> str && !is.fail())
		{
			//if (str.find("://") == std::string::npos)
			//	str = HWA_FILE_PREFIX + str;
			if (xmls.find(str) == xmls.end())
			{
				xmls.insert(str);
			}
			else
			{
				std::cerr << "READ : " + str + " multiple request ignored ! " << std::endl;
			}
		}
		return xmls.size();
	}

    // Get formats inputs
    // ----------
    std::vector<std::string> set_inp::_inputs(const std::string &fmt)
    {
        std::vector<std::string> tmp;
        std::set<std::string> list;
        std::string str;

        for (xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).first_child(); node; node = node.next_sibling())
        {
            if (node.name() == fmt)
            {
                std::istringstream is(node.child_value());
                while (is >> str && !is.fail())
                {
                    if (str.find("://") == std::string::npos)
                        str = HWA_FILE_PREFIX + str;
                    if (list.find(str) == list.end())
                    {
                        tmp.push_back(str);
                        list.insert(str);
                    }
                    else
                    {
                        std::cout << "READ : " + str + " multiple request ignored" << std::endl;
                    }
                }
            }
        }
        return tmp;
    }

    std::vector<std::string> set_inp::_inputs(const IFMT &fmt)
    {
        std::vector<std::string> tmp;
        std::set<std::string> list;
        std::string str;

        for (xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).first_child(); node; node = node.next_sibling())
        {
            // jdhuang
            IFMT ifmt = str2ifmt(node.name());
            if (ifmt == IFMT::UNDEF)
                continue;
            if (ifmt == fmt)
            {
                std::istringstream is(node.child_value());
                while (is >> str && !is.fail())
                {
                    if (str.find("://") == std::string::npos)
                        str = HWA_FILE_PREFIX + str;
                    if (list.find(str) == list.end())
                    {
                        tmp.push_back(str);
                        list.insert(str);
                    }
                    else
                    {
                        std::cout << "READ : " + str + " multiple request ignored" << std::endl;
                    }
                }
            }
        }
        return tmp;
    }

    // Get input formats
    // ----------
    std::set<std::string> set_inp::_iformats()
    {
        std::set<std::string> tmp;
        for (xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).first_child(); node; node = node.next_sibling())
        {
            tmp.insert(node.name());
        }
        return tmp;
    }

    // Checking navigation Rinex
    // ----------
    bool set_inp::chkNavig()
    {
        bool tmp;

        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).attribute("chk_nav").as_bool();

        return tmp;
    }

    // Checking satellite healthy status
    // ----------
    bool set_inp::chkHealth()
    {
        bool tmp;

        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).attribute("chk_health").as_bool();

        return tmp;
    }

    float set_inp::inp_sample(const std::string &fmt)
    {
        float smp = 0.0;
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child(fmt.c_str());
        if (!fmt.empty() &&
            !node.attribute("smp").empty())
        {
            smp = node.attribute("smp").as_float();
        }
        return smp;
    }

    std::map<std::string, std::string> set_inp::inp_caster(const std::string &fmt)
    {
        std::map<std::string, std::string> caster;

        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child(fmt.c_str());
        if (!fmt.empty())
        {
            std::string tmp = node.value();

            for (unsigned int i = 0; i < tmp.size(); i++)
            {
                if (tmp[i] == ':' || tmp[i] == '/' || tmp[i] == '@')
                    tmp[i] = ' ';
            }
            std::stringstream ss(tmp);
            std::string type;
            ss >> type >> caster["User"] >> caster["Pswd"] >> caster["Host"] >> caster["Port"] >> caster["MountPoint"];
        }
        return caster;
    }

    // settings check
    // ----------
    void set_inp::check()
    {
        // check existence of nodes/attributes
        xml_node parent = _doc.child(XMLKEY_ROOT);
        xml_node node = _default_node(parent, XMLKEY_INP);

        // check supported input formats (see IFMT enum !)
        std::set<std::string> ifmt = _iformats();
        std::set<std::string>::const_iterator itFMT = ifmt.begin();
        while (itFMT != ifmt.end())
        {
            std::string fmt = *itFMT;
            try
            {
                str2ifmt(fmt);
            }
            catch (const std::exception &e)
            {
                _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).remove_child(node.child(fmt.c_str()));
                itFMT++;
                continue;
            }
            // check application-specific output format
            if (_IFMT_supported.find(str2ifmt(fmt)) == _IFMT_supported.end())
            {
                _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).remove_child(node.child(fmt.c_str()));
                spdlog::warn(fmt + " inp format not supported by this application!");
            }
            check_input(fmt, "your fmt : " + fmt + " is empty");
            itFMT++;
        }

        _default_attr(node, "chk_nav", _chkNavig);
        _default_attr(node, "chk_health", _chkHealth);

        xml_node nodeBNCRTCM = _doc.child(XMLKEY_ROOT).child(XMLKEY_INP).child("bncrtcm");
        _default_attr(nodeBNCRTCM, "_corrStream", _corrStream);
        return;
    }

    // settings help
    // ----------
    void set_inp::help()
    {
        std::cerr << " <inputs>\n"
             << "   <rinexo> file://dir/name </rinexo> \t\t <!-- obs RINEX decoder -->\n"
             << "   <rinexn> file://dir/name </rinexn> \t\t <!-- nav RINEX decoder -->\n"
             << " </inputs>\n";

        std::cerr << "\t<!-- inputs description:\n"
             << "\t <decoder> path1 path2 path3  </decoder>\n"
             << "\t ... \n"
             << "\t where path(i) contains [file,tcp,ntrip]:// depending on the application\n"
             << "\t -->\n\n";
        return;
    }

} // namespace
