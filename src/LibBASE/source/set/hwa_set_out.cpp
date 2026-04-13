#include <iomanip>
#include <sstream>
#include <algorithm>
#include "hwa_set_out.h"
#include "hwa_base_fileconv.h"

using namespace pugi;

namespace hwa_set
{
    // Convertor for OUT formats
    // ----------
    OFMT set_out::str2ofmt(const std::string& s)
    {
        std::string tmp = s;
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "OUT")
            return XXX_OUT;
        if (tmp == "TSA")
            return TSA_OUT;
        if (tmp == "SUM")
            return SUM_OUT;
        if (tmp == "LOG")
            return LOG_OUT;
        if (tmp == "XTR")
            return XTR_OUT;
        if (tmp == "NAV")
            return NAV_OUT;
        if (tmp == "XML")
            return XML_OUT;
        if (tmp == "XQC")
            return XQC_OUT;
        if (tmp == "PPP")
            return PPP_OUT;
        if (tmp == "FLT")
            return FLT_OUT;
        if (tmp == "RATIO")
            return RATIO_OUT; //xjhan
        if (tmp == "OBSQUALITY")
            return OBSQUALITY_OUT;  //hlgou
        if (tmp == "SMT")
            return SMT_OUT;
        if (tmp == "lsq")
            return lsq_OUT;
        if (tmp == "RES")
            return RES_OUT;
        if (tmp == "GRD")
            return GRD_OUT;
        if (tmp == "SRF")
            return SRF_OUT;
        if (tmp == "CFG")
            return CFG_OUT;
        if (tmp == "FIT")
            return FIT_OUT;
        if (tmp == "RINEXN")
            return RINEXN_OUT;
        if (tmp == "RINEXN2")
            return RINEXN2_OUT;
        if (tmp == "RINEXO")
            return RINEXO_OUT;
        if (tmp == "RINEXC")
            return RINEXC_OUT;
        if (tmp == "SINEX")
            return SINEX_OUT;
        if (tmp == "TROSINEX")
            return TROSINEX_OUT;
        if (tmp == "TROSINEX0")
            return TROSINEX0_OUT;
        if (tmp == "KML")
            return KML_OUT;
        if (tmp == "PRDQC")
            return PRDQC_OUT;
        if (tmp == "ENU")
            return ENU_OUT;
        if (tmp == "RESULT")
            return RESULT_OUT;
        if (tmp == "PRE")
            return PRE_OUT;
        if (tmp == "RB")
            return RB_OUT;
        if (tmp == "STA")
            return STA_OUT;
        if (tmp == "KBROMC")
            return KBROMC_OUT;
        if (tmp == "LRIOMC")
            return LRIOMC_OUT;
        if (tmp == "INS")
            return INS_OUT;
        if (tmp == "PREOBS")
            return PREOBS_OUT;
        if (tmp == "INSKF")
            return INSKF_OUT;
        if (tmp == "INSKFPK")
            return INSKFPK_OUT;
        if (tmp == "ORBIT")
            return ORB_OUT;
        if (tmp == "UPD")
            return UPD_OUT;
        if (tmp == "SP3")
            return SP3_OUT;
        if (tmp == "IOPNAV")
            return ICS_OUT;
        if (tmp == "IOPLEO")
            return ICSLEO_OUT;
        if (tmp == "ION")
            return ION_OUT;
        if (tmp == "CAR")
            return RECCLK_OUT;
        if (tmp == "CAS")
            return SATCLK_OUT;
        if (tmp == "RECCLK")
            return RECCLK_OUT;
        if (tmp == "RECCLK13")
            return RECCLK13_OUT; // add by xiongyun
        if (tmp == "SATCLK")
            return SATCLK_OUT;
        if (tmp == "SATCLK13")
            return SATCLK13_OUT; // add by xiongyun
        if (tmp == "AMBUPD")
            return AMBUPD_OUT;
        if (tmp == "AMBFLAG_DIR")
            return AMBFLAG_DIR_OUT; // add by jqwu
        if (tmp == "ORBDIF")
            return ORBDIF_OUT;
        if (tmp == "DIF")
            return ORBDIF_OUT;
        if (tmp == "ORBFIT")
            return ORBFIT_OUT; // added by yqyuan
        if (tmp == "AMBIGUITY_LEO")
            return AMBCONLEO_OUT;
        if (tmp == "AMBIGUITY")
            return AMBCON_OUT;
        //if (tmp == "CON")         return AMBCON_OUT;
        if (tmp == "RECOVER")
            return RECOVER_OUT;
        if (tmp == "RCV")
            return RECOVER_OUT;
        if (tmp == "ATX")
            return ATX_OUT;
        if (tmp == "AUG")
            return AUG_OUT;
        if (tmp == "TROP")
            return TROP_OUT;
        if (tmp == "IMU")
            return IMU_OUT;
        if (tmp == "PCVNEQ")
            return PCVNEQ_OUT;
        if (tmp == "GPGGA")
            return GPGGA_OUT;
        if (tmp == "RSSIMAP")
            return RSSIMAP_OUT;
        if (tmp == "RSSIMATCH")
            return RSSIMATCH_OUT;
        if (tmp == "CAMPOS")
            return CAMPOS_OUT;
        if (tmp == "IPP")
            return IPP_OUT;
        if (tmp == "ERP")
            return ERP_OUT;
        if (tmp == "POLEUT1")
            return POLEUT1_OUT;
        if (tmp == "MORBCLK")
            return MORBCLK_OUT;
        return OFMT(-1);
    }

    // Convertor for OUT formats
    // ----------
    std::string set_out::ofmt2str(const OFMT& f)
    {
        switch (f)
        {
        case XXX_OUT:
            return "OUT";
        case TSA_OUT:
            return "TSA";
        case LOG_OUT:
            return "LOG";
        case SUM_OUT:
            return "SUM";
        case XTR_OUT:
            return "XTR";
        case NAV_OUT:
            return "NAV";
        case XML_OUT:
            return "XML";
        case XQC_OUT:
            return "XQC";
        case PPP_OUT:
            return "PPP";
        case lsq_OUT:
            return "lsq";
        case FLT_OUT:
            return "FLT";
        case RATIO_OUT:
            return "RATIO"; //xjhan
        case OBSQUALITY_OUT:
            return "OBSQUALITY";
        case SMT_OUT:
            return "SMT";
        case RES_OUT:
            return "RES";
        case GRD_OUT:
            return "GRD";
        case SRF_OUT:
            return "SRF";
        case PRE_OUT:
            return "PRE";
        case PREOBS_OUT:
            return "PREOBS";
        case RB_OUT:
            return "RB";
        case KBROMC_OUT:
            return "KBROMC";
        case STA_OUT:
            return "STA";
        case FIT_OUT:
            return "FIT";
        case CFG_OUT:
            return "CFG";
        case RINEXN_OUT:
            return "RINEXN";
        case RINEXN2_OUT:
            return "RINEXN2";
        case RINEXO_OUT:
            return "RINEXO";
        case RINEXC_OUT:
            return "RINEXC";
        case SINEX_OUT:
            return "SINEX";
        case TROSINEX_OUT:
            return "TROSINEX";
        case TROSINEX0_OUT:
            return "TROSINEX0";
        case KML_OUT:
            return "KML";
        case INS_OUT:
            return "INS";
        case INSKF_OUT:
            return "INSKF";
        case INSKFPK_OUT:
            return "INSKFPK";
        case UPD_OUT:
            return "UPD_OUT";
        case SP3_OUT:
            return "SP3";
        case RECCLK_OUT:
            return "RECCLK";
        case RECCLK13_OUT:
            return "RECCLK13"; // add by xiongyun
        case SATCLK_OUT:
            return "SATCLK";
        case SATCLK13_OUT:
            return "SATCLK13"; // add by xiongyun
        case AMBUPD_OUT:
            return "AMBUPD";
        case RESULT_OUT:
            return "RESULT";
        case RECOVER_OUT:
            return "RECOVER";
        case ICS_OUT:
            return "IOPNAV";
        case ICSLEO_OUT:
            return "IOPLEO";
        case ION_OUT:
            return "ION";
        case ATX_OUT:
            return "ATX";
        case AUG_OUT:
            return "AUG";
        case TROP_OUT:
            return "TROP";
        case IMU_OUT:
            return "IMU";
        case PCVNEQ_OUT:
            return "PCVNEQ";
        case CLK_OUT:
            return "CLK";
        case GPGGA_OUT:
            return "GPGGA";
        case RSSIMATCH_OUT:
            return "WIFIMATCH";
        case RSSIMAP_OUT:
            return "RSSIMAP";
        case CAMPOS_OUT:
            return "CAMPOS";
        case ERP_OUT:
            return "ERP";
        case POLEUT1_OUT:
            return "POLEUT1";
        case MORBCLK_OUT:
            return "MORBCLK";
        default:
            return "UNDEF";
        }
        return "UNDEF";
    }

    // Constructor
    // ----------
    set_out::set_out()
        : hwa_set::set_base(),
        _append(false),
        _verb(0),
        _upd(0),
        _len(0),
        _smp(0)
    {
        _set.insert(XMLKEY_OUT);
    }

    // Destructor
    // ----------
    set_out::~set_out()
    {
    }

    // Get formats output size
    // ----------
    int set_out::output_size(const std::string& fmt)
    {
        int tmp = _outputs(fmt).size();
        return tmp;
    }

    // Return value
    // ----------
    int set_out::verb()
    {
        int tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).attribute("verb").as_int();
        return tmp;
    }

    // Retrun value
    // -----------
    bool set_out::append()
    {
        bool tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).attribute("append").as_bool();
        return tmp;
    }

    // Get std::string outputs
    // ----------
    std::string set_out::outputs(const std::string& fmt)
    {
        std::string tmp = _outputs(fmt);
        return tmp;
    }

    std::string set_out::log_type()
    {
        //auto test = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).find_child("spdlog");
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child("log").attribute("type").as_string();
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty())
        {
            return "CONSOLE";
        }
        else
        {
            return tmp;
        }
    }

    std::string set_out::log_name()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child("log").attribute("name").as_string();
        str_erase(tmp);
        if (tmp.empty())
        {
            return "my_logger";
        }
        else
        {
            return tmp;
        }
    }

    level::level_enum set_out::log_level()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child("log").attribute("level").as_string();
        str_erase(tmp);
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty())
        {
            return level::level_enum::info;
        }
        else
        {
            if (tmp.find("ERROR") != std::string::npos)
            {
                return level::level_enum::err;
            }
            else if (tmp.find("DEBUG") != std::string::npos)
            {
                return level::level_enum::debug;
            }
            else if (tmp.find("WARN") != std::string::npos)
            {
                return level::level_enum::warn;
            }
            else if (tmp.find("CRITICAL") != std::string::npos)
            {
                return level::level_enum::critical;
            }
            else if (tmp.find("TRACE") != std::string::npos)
            {
                return level::level_enum::trace;
            }
            else if (tmp.find("INFO") != std::string::npos)
            {
                return level::level_enum::info;
            }
            else
            {
                return level::level_enum::off;
            }
        }
    }

    std::string set_out::log_pattern()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child("log").attribute("pattern").as_string();
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty())
        {
            return std::string("[%Y-%m-%d %H:%M:%S] <thread %t> [%l] [%@] %v");
        }
        else
        {
            return tmp;
        }
    }

    int set_out::sp3_obslimit()
    {
        int obs;
        obs = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).attribute("obslimit").as_int();
        return obs;
    }

    // get std::string output version
    // ---------
    std::string set_out::version(const std::string& fmt)
    {
        std::string ver = DEFAULT_FILE_VER;
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child(fmt.c_str());
        if (!fmt.empty() &&
            !node.attribute("ver").empty())
        {
            ver = node.attribute("ver").as_string();
        }
        return ver;
    }

    // get time offset [min] for the output file name
    // ---------
    int set_out::file_toff(const std::string& fmt)
    {
        int upd = DEFAULT_FILE_OFF;
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child(fmt.c_str());
        if (!fmt.empty() &&
            !node.attribute("toff").empty())
        {
            upd = node.attribute("toff").as_int();
        }
        return upd;
    }

    int set_out::out_update(const std::string& fmt)
    {
        int upd = DEFAULT_FILE_UPD;
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child(fmt.c_str());
        if (!fmt.empty() &&
            !node.attribute("upd").empty())
        {
            upd = node.attribute("upd").as_int();
        }
        return upd;
    }

    // get period [min] for output file content
    // ----------
    int set_out::out_length(const std::string& fmt)
    {
        int len = DEFAULT_FILE_LEN;
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child(fmt.c_str());

        if (!fmt.empty() &&
            !node.attribute("len").empty())
        {
            len = node.attribute("len").as_int();
        }
        return len;
    }

    // get sample [sec] for output file data sampling
    // ---------
    float set_out::out_sample(const std::string& fmt)
    {
        float smp = DEFAULT_FILE_SMP;
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child(fmt.c_str());
        if (!fmt.empty() &&
            !node.attribute("smp").empty())
        {
            smp = node.attribute("smp").as_float();
        }
        return smp;
    }

    std::string set_out::output_port(const std::string& fmt)
    {
        std::string port("");
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child(fmt.c_str());
        if (!fmt.empty() &&
            !node.attribute("port").empty())
        {
            port = node.attribute("port").as_string();
        }
        return port;
    }

    std::map<std::string, std::string> set_out::out_caster(const std::string& fmt)
    {
        std::map<std::string, std::string> caster;

        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).child(fmt.c_str());
        if (!fmt.empty() &&
            !node.attribute("caster").empty())
        {
            std::string tmp = node.attribute("caster").as_string();

            for (unsigned int i = 0; i < tmp.size(); i++)
            {
                if (tmp[i] == ':' || tmp[i] == '/' || tmp[i] == '@')
                    tmp[i] = ' ';
            }
            std::stringstream ss(tmp);
            ss >> caster["User"] >> caster["Pswd"] >> caster["Host"] >> caster["Port"] >> caster["MountPoint"];
        }
        return caster;
    }

    // Get output formats
    // ----------
    std::set<std::string> set_out::oformats()
    {
        return _oformats();
    }

    // Get output formats
    // ----------
    std::set<std::string> set_out::_oformats()
    {
        std::set<std::string> tmp;
        for (xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).first_child(); node; node = node.next_sibling())
        {
            tmp.insert(node.name());
        }
        return tmp;
    }

    // Get formats outputs
    // ----------
    std::string set_out::_outputs(const std::string& fmt)
    {
        std::string str;
        xml_node parent = _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT);
        if (!parent) {
            std::cerr << "[ERROR] Cannot find XML node <" << XMLKEY_OUT << ">" << std::endl;
            return "";
        }
        for (xml_node node = parent.first_child(); node; node = node.next_sibling())
        {
            if (node.name() == fmt)
            {
                std::istringstream is(node.child_value());
                while (is >> str && !is.fail())
                {
                    if (str.find("://") == std::string::npos)
                        str = HWA_FILE_PREFIX + str;
                    return str;
                }
            }
        }
        return "";
    }

    // settings check
    // ----------
    void set_out::check()
    {
        // check existence of nodes/attributes
        xml_node parent = _doc.child(XMLKEY_ROOT);
        xml_node node = _default_node(parent, XMLKEY_OUT);

        // check existence of attributes
        _default_attr(node, "append", _append);

        // check supported input formats (see OFMT enum !)
        std::set<std::string> ofmt = _oformats();
        std::set<std::string>::const_iterator itFMT = ofmt.begin();
        while (itFMT != ofmt.end())
        {
            std::string fmt = *itFMT;
            OFMT ofmt = str2ofmt(fmt);
            if (ofmt < 0)
            {
                _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).remove_child(node.child(fmt.c_str()));
                std::cout << "Warning: " + fmt + " out format not implemented [gsetout::check()]!" << std::endl;
                itFMT++;
                continue;
            }

            // check application-specific output format
            if (_OFMT_supported.find(ofmt) == _OFMT_supported.end())
            {
                _doc.child(XMLKEY_ROOT).child(XMLKEY_OUT).remove_child(node.child(fmt.c_str()));

                std::cout << "Warning: " + fmt + " out format not supported by this application!" << std::endl;
            }
            itFMT++;
        }
        return;
    }

    // settings help
    // ----------
    void set_out::help()
    {
        std::cerr << " <outputs append=\"" << _append << "\" verb=\"" << _verb << "\" >\n"
            << "   <flt> file://dir/name </flt>    \t\t <!-- filter output encoder -->\n"
            << " </outputs>\n";

        std::cerr << "\t<!-- outputs description:\n"
            << "\t <encoder> path </encoder>\n"
            << "\t ... \n"
            << "\t where path contains [file,tcp,ntrip]:// depending on the application\n"
            << "\t -->\n\n";
        return;
    }

} // namespace
