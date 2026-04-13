#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include "hwa_set_ionest.h"
#include "hwa_set_gbase.h"
#ifdef USE_OPENMP
#include <omp.h>
#endif

using namespace std;
using namespace pugi;

namespace hwa_set
{
    set_ionest::set_ionest()
        : hwa_set::set_base()
    {
        _n_ses = 13;
        _order = 15;
        _IONdir = "./ION";
        _minlat = -90.0;
        _minlon = -180.0;
        _maxlat = 90.0;
        _maxlon = 180.0;
        _step = 2.5;
        _sessLength = 7200.0;
        _maglat = 79.93;
        _maglon = -71.96;
        _Pretype = PRETYPE::IONO_PL;

        // for iono grid
        _mask = "HWA";
        _ID = "00";
        _RefLon = 0.0;
        _RefLat = 0.0;
        _SpaceLon = 0.0;
        _SpaceLat = 0.0;
        _CountLat = 0;
        _CountLon = 0;
    }
    set_ionest::~set_ionest() = default;
    void set_ionest::check()
    {
    }
    void set_ionest::help()
    {
        cerr << "no help" << endl;
    }
    int set_ionest::n_ses()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("n_ses");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stod(tmp);
        else
            tmp_int = _n_ses;
        return tmp_int;
    }

    int set_ionest::order()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("order");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stod(tmp);
        else
            tmp_int = _order;
        return tmp_int;
    }

    double set_ionest::minlat()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("minlat");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _minlat;
        return tmp_double;
    }

    double set_ionest::maxlat()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("maxlat");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _maxlat;
        return tmp_double;
    }

    double set_ionest::minlon()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("minlon");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _minlon;
        return tmp_double;
    }

    double set_ionest::maxlon()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("maxlon");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _maxlon;
        return tmp_double;
    }

    double set_ionest::step()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("step");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _step;
        return tmp_double;
    }

    double set_ionest::sessLength()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("sessLength");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sessLength;
        return tmp_double;
    }

    double set_ionest::maglat()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("maglat");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _maglat;
        return tmp_double;
    }

    double set_ionest::maglon()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("maglon");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _maglon;
        return tmp_double;
    }

    PRETYPE set_ionest::PreType()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("type");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(),::toupper);
        PRETYPE type;
        if (tmp != "")
        {
            if (!tmp.compare("IONO_P"))
                type = PRETYPE::IONO_P;
            else if (!tmp.compare("IONO_PL"))
                type = PRETYPE::IONO_PL;
            else if (!tmp.compare("IONO_PPP"))
                type = PRETYPE::IONO_PPP;
            else if (!tmp.compare("IONO_SION"))
                type = PRETYPE::IONO_SION;
            else if (!tmp.compare("IONO_PPP_AR"))
                type = PRETYPE::IONO_PPPAR;
            else
                type = _Pretype;
        }        
        else
            type = _Pretype;
        return type;
    }

    string set_ionest::IONdir()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONEST).child_value("ion_dir");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        string tmp_string;
        if (tmp != "")
            tmp_string = tmp;
        else
            tmp_string = _IONdir;
        return tmp_string;
    }
    string set_ionest::mask_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("mask");
        str_erase(tmp);
        string tmp_string;
        if (tmp != "")
            tmp_string = tmp;
        else
            tmp_string = _mask;
        return tmp_string;
    }
    string set_ionest::ID_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).attribute("id").as_string();
        str_erase(tmp);
        string tmp_string;
        if (tmp != "")
            tmp_string = tmp;
        else
            tmp_string = _ID;
        return tmp_string;
    }
    string set_ionest::ref_Site()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("rec_ref");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        string tmp_string;
        if (tmp != "")
            tmp_string = tmp;
        else
            tmp_string = "NONE";
        return tmp_string;
    }
    double set_ionest::reflon_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("ref_lon");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _RefLon;
        return tmp_double;
    }
    double set_ionest::reflat_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("ref_lat");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _RefLat;
        return tmp_double;
    }
    double set_ionest::spacelon_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("space_lon");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _SpaceLon;
        return tmp_double;
    }
    double set_ionest::spacelat_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("space_lat");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _SpaceLat;
        return tmp_double;
    }
    int set_ionest::countlon_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("count_lon");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stod(tmp);
        else
            tmp_int = _CountLon;
        return tmp_int;
    }
    int set_ionest::countlat_grid()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("count_lat");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stod(tmp);
        else
            tmp_int = _CountLat;
        return tmp_int;
    }
    int set_ionest::min_site()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("min_site");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stod(tmp);
        else
            tmp_int = 5;
        return tmp_int;
    }
    double set_ionest::max_Sigma()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("bias_sigma");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = 0.06;
        return tmp_double;
    }
    double set_ionest::max_Baseline()
    {
        string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_IONOGRID).child_value("bias_baseline");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = 0.5;
        return tmp_double;
    }
    set<string> set_ionest::recs_rm()
    {
        set<string> tmp = hwa_set::set_base::_setvals(XMLKEY_GNSS, XMLKEY_IONOGRID , "rec_rm");
        return tmp;
    }
} // namespace
