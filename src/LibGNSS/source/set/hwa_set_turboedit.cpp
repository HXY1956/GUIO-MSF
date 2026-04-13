#include "hwa_set_turboedit.h"
#include "hwa_set_gbase.h"

using namespace std;
using namespace pugi;

namespace hwa_set
{
    set_turboedit::set_turboedit() : set_base()
    {
        _set.insert(XMLKEY_TURBOEDIT);
        _defaultPCLimit = 250.0;       // unit: meter
        _defaultMWLimit = 4.0;         // unit: cycle
        _defaultGFLimit = 1.0;         // unit: cycle
        _defaultGFRmsLimit = 2.0;      // unit: cycle
        _defaultSingleFreqLimit = 1.0; // unit: meter
        _defaultGapArcLimit = 20;      // unit: epoch
        _defaultShortArcLimit = 10;    // unit: epoch
        _defaultMinPercent = 60.0;     // unit: %
        _defaultMinMeanNprn = 4;       // unit:
        _defaultMaxMeanNamb = 3;       // unit:
    }

    bool set_turboedit::liteMode()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child_value("lite_mode");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_turboedit::isAmbOutput()
    {
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("amb_output");
        return tmp_set.attribute("valid").as_bool();
    }

    bool set_turboedit::isEphemeris()
    {
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("ephemeris");
        return tmp_set.attribute("valid").as_bool();
    }

    bool set_turboedit::checkPC(double &pc_limit)
    {
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("check_pc");

        string tmp = tmp_set.attribute("pc_limit").value();
        if (tmp.empty())
            pc_limit = _defaultPCLimit;
        else
            pc_limit = base_type_conv::str2dbl(tmp);

        return tmp_set.attribute("valid").as_bool();
    }

    bool set_turboedit::checkMW(double &mw_limit)
    {
        bool is_litemodel = this->liteMode();
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("check_mw");
        string tmp = tmp_set.attribute("mw_limit").value();

        if (is_litemodel)
        {
            if (tmp.empty())
                mw_limit = 0.0;
            else
                mw_limit = base_type_conv::str2dbl(tmp);
        }
        else
        {
            if (tmp.empty())
                mw_limit = _defaultMWLimit;
            else
                mw_limit = base_type_conv::str2dbl(tmp);
        }

        return tmp_set.attribute("valid").as_bool();
    }

    bool set_turboedit::checkGF(double &gf_limit, double &gf_rms_limit)
    {
        bool is_litemodel = this->liteMode();
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("check_gf");
        string tmp_gf = tmp_set.attribute("gf_limit").value();
        string tmp_rms = tmp_set.attribute("gf_rms_limit").value();
        if (is_litemodel)
        {
            if (tmp_gf.empty())
                gf_limit = 0.0;
            else
                gf_limit = base_type_conv::str2dbl(tmp_gf);

            if (tmp_rms.empty())
                gf_rms_limit = 0.0;
            else
                gf_rms_limit = base_type_conv::str2dbl(tmp_rms);
        }
        else
        {
            if (tmp_gf.empty())
                gf_limit = _defaultGFLimit;
            else
                gf_limit = base_type_conv::str2dbl(tmp_gf);

            if (tmp_rms.empty())
                gf_rms_limit = _defaultGFRmsLimit;
            else
                gf_rms_limit = base_type_conv::str2dbl(tmp_rms);
        }

        return tmp_set.attribute("valid").as_bool();
    }

    bool set_turboedit::checkSingleFreq(double &sf_limit)
    {
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("check_sf");

        string tmp = tmp_set.attribute("sf_limit").value();
        if (tmp.empty())
            sf_limit = _defaultSingleFreqLimit;
        else
            sf_limit = base_type_conv::str2dbl(tmp);

        return tmp_set.attribute("valid").as_bool();
    }

    bool set_turboedit::checkGap(int &gap_limit)
    {
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("check_gap");

        string tmp = tmp_set.attribute("gap_limit").value();

        if (tmp.empty())
            gap_limit = _defaultGapArcLimit;
        else
            gap_limit = base_type_conv::str2int(tmp);

        return tmp_set.attribute("valid").as_bool();
    }

    int set_turboedit::smoothWindows()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child_value("smooth_win");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = 25; //default value
        return tmp_int;
    }

    bool set_turboedit::checkShort(int &short_limit)
    {
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("check_short");

        string tmp = tmp_set.attribute("short_limit").value();

        if (tmp.empty())
            short_limit = _defaultShortArcLimit;
        else
            short_limit = base_type_conv::str2int(tmp);

        return tmp_set.attribute("valid").as_bool();
    }

    bool set_turboedit::checkStatistics(double &minPercent, int &minMeanNprn, int &maxMeanNamb)
    {
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).child("check_statistics");

        string tmp = tmp_set.attribute("min_percent").value();
        if (tmp.empty())
            minPercent = _defaultMinPercent;
        else
            minPercent = base_type_conv::str2dbl(tmp);

        tmp = tmp_set.attribute("min_mean_nprn").value();
        if (tmp.empty())
            minMeanNprn = _defaultMinMeanNprn;
        else
            minMeanNprn = base_type_conv::str2int(tmp);

        tmp = tmp_set.attribute("max_mean_namb").value();
        if (tmp.empty())
            maxMeanNamb = _defaultMaxMeanNamb;
        else
            maxMeanNamb = base_type_conv::str2int(tmp);

        return tmp_set.attribute("valid").as_bool();
    }

    string set_turboedit::outputs(const string& fmt)
    {
        string tmp = _outputs(fmt);
        return tmp;
    }

    void set_turboedit::help()
    {
        cerr << "<turboedit  lite_mode=\"false\" >" << endl
             << "<amb_output  valid=\"true\"  />  " << endl
             << "<simulation  valid=\"false\" />  " << endl
             << "<ephemeris   valid=\"true\"  />  " << endl
             << "<check_pc   pc_limit=\"250\" valid=\"true\"  />" << endl
             << "<check_mw   mw_limit=\"4\"   valid=\"true\"  />" << endl
             << "<check_gf   gf_limit=\"1\"   gf_rms_limit=\"2\"   valid=\"true\" />" << endl
             << "<check_sf   sf_limit=\"1\"   valid=\"false\" />         " << endl
             << "<check_gap    gap_limit=\"20\"    valid=\"true\" />     " << endl
             << "<check_short  short_limit=\"10\"  valid=\"true\" />     " << endl
             << "<check_statistics  min_percent=\"60\"  min_mean_nprn=\"4\"  max_mean_namb=\"3\"  valid=\"true\" /> " << endl
             << "</turboedit>" << endl
             << endl;
        return;
    }

    string set_turboedit::_outputs(const string& fmt)
    {
        string str;
        for (xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_TURBOEDIT).first_child(); node; node = node.next_sibling())
        {
            if (node.name() == fmt)
            {
                std::istringstream is(node.child_value());
                while (is >> str && !is.fail())
                {
                    if (str.find("://") == string::npos)
                        str = HWA_FILE_PREFIX + str;
                    return str;
                }
            }
        }
        return "";
    }

}