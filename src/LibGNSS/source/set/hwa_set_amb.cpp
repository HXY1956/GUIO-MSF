#include "hwa_set_amb.h"
#include "hwa_set_gbase.h"
#include <sstream>

using namespace pugi;

namespace hwa_set
{
    /** @brief default constructor. */
    set_amb::set_amb()
        : set_base()
    {
        _set.insert(XMLKEY_AMBIGUITY);

        //_upd_type = UPD_TYPE::NONE;
        //_fix_mode = FIX_MODE::NO;
        //_ratio = 3.0;
        //_addleo = false;
        //_independ_baseline = false;
        //_min_common_time = 900.0;
        //_wl_interval = 30.0;
        //_max_baseline_length = 5000.0;
        //_map_amb_decision[UPD_TYPE::WL]["maxdev"] = 0.15;
        //_map_amb_decision[UPD_TYPE::WL]["maxsig"] = 0.15;
        //_map_amb_decision[UPD_TYPE::WL]["alpha"] = 0.001;
        //_map_amb_decision[UPD_TYPE::NL]["maxdev"] = 0.15;
        //_map_amb_decision[UPD_TYPE::NL]["maxsig"] = 0.15;
        //_map_amb_decision[UPD_TYPE::NL]["alpha"] = 0.001;
    }

    /** @brief default destructor. */
    set_amb::~set_amb()
    {
    }

    /**
     * @brief settings check.
     * @return      void
     */
    void set_amb::check()
    {
        return;
    }

    /**
     * @brief settings help.
     * @return      void
     */
    void set_amb::help()
    {
        std::cerr << "<ambiguity>\n"
            << "<upd> EWL/WL/NL/NONE </upd>\n"
            << "<fix_mode> ROUND/SEARCH/NO </fix_mode>\n"
            << "<ratio> 3.0 </ratio>\n"
            << "<add_leo> NO </add_leo>\n"
            << "<all_baselines> NO </all_baselines>\n"
            << "<min_common_time> 30 </min_common_time>\n"
            << "<baseline_length_limit> 3500 </baseline_length_limit>\n"
            << "<widelane_interval> 30 </widelane_interval>\n"
            << "<widelane_decision     maxdev = \"0.15\" maxsig = \"0.10\" alpha = \"1000\"/>\n"
            << "<narrowlane_decision   maxdev = \"0.15\" maxsig = \"0.10\" alpha = \"1000\"/>\n";
        return;
    }

    UPDTYPE set_amb::get_updtype()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("upd");
        return str2updmode(hwa_base::base_type_conv::trim(tmp));
    }

    AMB_TYPE set_amb::amb_type()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("amb_type");
        return str2ambtype(tmp);
    }

    FIX_MODE set_amb::fix_mode()
    {
        if (!_doc)
            str2fixmode(std::string());
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("fix_mode");
        return str2fixmode(hwa_base::base_type_conv::trim(tmp));
    }

    void set_amb::fix_mode(FIX_MODE mode)
    {
        xml_node amb = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY);
        amb.remove_child("fix_mode");
        xml_node fixmode = amb.append_child("fix_mode");
        fixmode.append_child(xml_node_type::node_pcdata).set_value(fixmode2str(mode).c_str());
    }

    UPD_MODE set_amb::upd_mode()
    {
        if (!_doc)
            str2upd_mode(std::string());
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("upd_mode");
        return str2upd_mode(hwa_base::base_type_conv::trim(tmp));
    }

    DD_MODEL set_amb::dd_mode()
    {
        if (!_doc)
            str2fixmode(std::string());
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("dd_mode");

        DD_MODEL mode;
        if (tmp.find("IF_CB_WN") != std::string::npos)
            mode = DD_MODEL::IF_CB_WN;
        else if (tmp.find("RAW_CB_WN") != std::string::npos)
            mode = DD_MODEL::RAW_CB_WN;
        else if (tmp.find("RAW_CB_2") != std::string::npos)
            mode = DD_MODEL::RAW_CB_2;
        else if (tmp.find("RAW_CB") != std::string::npos)
            mode = DD_MODEL::RAW_CB;
        else if (tmp.find("NONE") != std::string::npos)
            mode = DD_MODEL::NONE;
        else
        {
            throw std::logic_error("wrong type of dd mode in node amb.");
        }

        return mode;
    }

    double set_amb::lambda_ratio()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("ratio");
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    double set_amb::bootstrapping()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("boot");

        if (tmp.empty())
        {
            return -0.001;
        }
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    bool set_amb::addleo()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("add_leo");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_amb::independ_baseline()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("all_baselines");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    double set_amb::min_common_time()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("min_common_time");
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    double set_amb::max_baseline_length()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("baseline_length_limit");
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    double set_amb::wl_interval()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("widelane_interval");
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    std::map<std::string, double> set_amb::get_amb_decision(std::string type)
    {
        std::map<std::string, std::string> type2child = {
            {"EWL", "extra_widelane_decision"}, {"WL", "widelane_decision"}, {"NL", "narrowlane_decision"} };
        std::map<std::string, double> amb_decision;
        if (_default_decision.find(type) == _default_decision.end())
            return amb_decision;
        amb_decision = _default_decision[type];
        for (auto iter = amb_decision.begin(); iter != amb_decision.end(); ++iter)
        {
            double tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child(type2child[type].c_str()).attribute(iter->first.c_str()).as_double();
            if (!hwa_base::double_eq(tmp, 0))
                iter->second = tmp;
        }
        return amb_decision;
    }

    bool set_amb::part_ambfix()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("part_fix");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_amb::carrier_range_out()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("carrier_range_out");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_amb::apply_carrier_range()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("apply_carrier_range");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    int set_amb::part_ambfix_num()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("part_fix_num");

        if (tmp.empty())
        {
            return 2;
        }
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    double set_amb::FixFixSep()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("FixFixSep");
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    double set_amb::FloatFixSep()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("FloatFixSep");
        return hwa_base::base_type_conv::str2dbl(tmp);
    }

    bool set_amb::clear_flag()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("clear_flag");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    //bool set_amb::reset_amb_ppprtk()
    //{
    // _gmutex.lock();

    // std::istringstream is(_doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("reset_amb_ppprtk"));
    // std::string tmp;
    // is >> tmp;
    // bool reset_amb_ppprtk = (tmp == "YES" || tmp == "yes");

    // _gmutex.unlock(); return reset_amb_ppprtk;
    //}

    void set_amb::refixsettings(int& last_fixepo_gap, int& min_fixed_num)
    {
        xml_node tmp_set;

        tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child("refixsettings");
        std::string tmp = tmp_set.attribute("last_fix_gap").value();
        last_fixepo_gap = tmp.empty() ? 999999 : hwa_base::base_type_conv::str2int(tmp);
        tmp = tmp_set.attribute("min_fixed_num").value();
        min_fixed_num = tmp.empty() ? 0 : hwa_base::base_type_conv::str2int(tmp);
        return;
    }

    int set_amb::full_fix_num()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("full_fix_num");

        if (tmp.empty())
        {
            return 3;
        }
        return hwa_base::base_type_conv::str2int(tmp);
    }

    FIX_MODE set_amb::str2fixmode(std::string str)
    {
        if (str == "NO")
        {
            return FIX_MODE::NO;
        }
        else if (str == "ROUND")
        {
            return FIX_MODE::ROUND;
        }
        else if (str == "SEARCH")
        {
            return FIX_MODE::SEARCH;
        }
        else if (str == "HOLD")
        {
            return FIX_MODE::HOLD;
        }
        else
        {
            std::cout << "*** warning: not defined ambiguity fixing mode [" << str << "]\n";
            std::cout.flush();
        }
        return FIX_MODE::NO;
    }

    std::string set_amb::fixmode2str(FIX_MODE mode)
    {
        switch (mode)
        {
        case FIX_MODE::NO:
            return "NO";
        case FIX_MODE::ROUND:
            return "ROUND";
        case FIX_MODE::SEARCH:
            return "SEARCH";
        case FIX_MODE::HOLD:
            return "HOLD";
        }
    }

    /**
     * @brief change std::string to UPD_MODE.
     * @param[in]  str   upd mode std::string
     * @return      UPD_MODE
     */
    UPD_MODE set_amb::str2upd_mode(std::string str)
    {
        if (str == "UPD" || str == "upd")
        {
            return UPD_MODE::UPD;
        }
        else if (str == "IRC" || str == "irc")
        {
            return UPD_MODE::IRC;
        }
        else if (str == "OSB" || str == "osb")
        {
            return UPD_MODE::OSB;
        }
        else
        {
            return UPD_MODE::UPD;
        }
    }

    AMB_TYPE set_amb::str2ambtype(std::string str)
    {
        str = hwa_base::base_type_conv::trim(str);
        if (str == "DD")
            return AMB_TYPE::DD;
        else if (str == "SD")
            return AMB_TYPE::SD;
        else if (str == "UD")
            return AMB_TYPE::UD;
        else
            return AMB_TYPE::UNDEF;
    }

    UPDTYPE set_amb::str2updmode(std::string str)
    {
        str = hwa_base::base_type_conv::trim(str);
        if (str == "wl" || str == "WL")
        {
            return UPDTYPE::WL;
        }
        else if (str == "ewl" || str == "EWL")
        {
            return UPDTYPE::EWL;
        }
        else if (str == "ewl24" || str == "EWL24")
        {
            return UPDTYPE::EWL24;
        }
        else if (str == "ewl25" || str == "EWL25")
        {
            return UPDTYPE::EWL25;
        }
        else if (str == "ewl_epoch" || str == "EWL_EPOCH")
        {
            return UPDTYPE::EWL_EPOCH;
        }
        else if (str == "nl" || str == "NL")
        {
            return UPDTYPE::NL;
        }
        else if (str == "ifcb" || str == "IFCB")
        {
            return UPDTYPE::IFCB;
        }
        else
        {
            std::cout << "*** warning: not defined upd mode [" << str << "]\n";
            std::cout.flush();
        }
        return UPDTYPE::NONE;
    }

    std::map<std::string, std::string> set_amb::ambiguity_engaged()
    {
        std::map<std::string, std::string> map_amb_egg;
        xml_node amb_egg = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child("ambiguity_engaged");
        if (!amb_egg.empty())
        {
            std::string adop = amb_egg.attribute("adop").value();
            if (adop.empty())adop = 0.1;
            map_amb_egg.insert(std::make_pair("adop", adop));

            std::string pdop = amb_egg.attribute("pdop").value();
            if (pdop.empty())pdop = 1.1;
            map_amb_egg.insert(std::make_pair("pdop", pdop));

            std::string nsat = amb_egg.attribute("nsat").value();
            if (nsat.empty())nsat = 15;
            map_amb_egg.insert(std::make_pair("nsat", nsat));

            std::string ratio = amb_egg.attribute("ratio").value();
            if (ratio.empty())ratio = 20;
            map_amb_egg.insert(std::make_pair("ratio", ratio));
        }
        else
        {
            map_amb_egg.clear();
        }
        return map_amb_egg;
    }

    //bool set_amb::apply_irc()
    //{
    // _gmutex.lock();

    // std::istringstream is(_doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("apply_irc"));
    // std::string tmp;
    // is >> tmp;
    // bool is_apply_irc = (tmp == "YES" || tmp == "yes");

    // _gmutex.unlock(); return is_apply_irc;
    //}

    bool set_amb::isSetRefSat()
    {
        std::istringstream is(_doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("set_refsat"));
        std::string tmp;
        is >> tmp;
        bool isSetRefsat = (tmp == "YES" || tmp == "yes");
        return isSetRefsat;
    }

}

namespace hwa_set{
    std::string updmode2str(UPDTYPE mode)
    {
        std::string str;
        switch (mode)
        {
        case UPDTYPE::EWL:
            str = "EWL";
            break;
        case UPDTYPE::EWL24:
            str = "EWL24";
            break;
        case UPDTYPE::EWL25:
            str = "EWL25";
            break;
        case UPDTYPE::EWL_EPOCH:
            str = "EWL_EPOCH";
            break;
        case UPDTYPE::NL:
            str = "NL";
            break;
        case UPDTYPE::WL:
            str = "WL";
            break;
        case UPDTYPE::IFCB:
            str = "IFCB";
            break;
        case UPDTYPE::RTPPP_SAVE:
            str = "RTPPP_SAVE";
            break;
        case UPDTYPE::NONE:
            str = "NONE";
            break;
        default:
            str = "NONE";
            break;
        }
        return str;
    }

    UPDTYPE str2updmode(std::string mode)
    {
        if (mode == "EWL_EPOCH")
            return UPDTYPE::EWL_EPOCH;
        else if (mode == "EWL")
            return UPDTYPE::EWL;
        else if (mode == "EWL24")
            return UPDTYPE::EWL24;
        else if (mode == "EWL25")
            return UPDTYPE::EWL25;
        else if (mode == "WL")
            return UPDTYPE::WL;
        else if (mode == "NL")
            return UPDTYPE::NL;
        else if (mode == "IFCB")
            return UPDTYPE::IFCB;
        else if (mode == "RTPPP_SAVE")
            return UPDTYPE::RTPPP_SAVE;
        else
            return UPDTYPE::NONE;
    }
}