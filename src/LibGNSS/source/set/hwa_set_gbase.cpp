#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include "hwa_set_gbase.h"
#include "hwa_set_gtype.h"
#include "hwa_gnss_sys.h"

using namespace std;
using namespace pugi;

namespace hwa_set
{
    set_gnss::set_gnss()
        : set_base()
    {
        _set.insert(XMLKEY_GNSS);

        //  t_map_gnss::const_iterator itGNSS;
        //  t_map_type::const_iterator itTYPE;
        //  hwa_map_Attr::const_iterator itBAND;
        //  hwa_vec_attr::const_iterator itATTR;
        stringstream os;
        _sigma_def[GPS] = base_pair(2.0, 0.02);
        _sigma_def[GLO] = base_pair(4.0, 0.04);
        _sigma_def[GAL] = base_pair(3.0, 0.03); //modified by glfeng
        _sigma_def[BDS] = base_pair(5.0, 0.03);
        _sigma_def[QZS] = base_pair(5.0, 0.03);
        _sigma_def[IRN] = base_pair(5.0, 0.03);
        _sigma_def_doppler[GPS] = 0.04;
        _sigma_def_doppler[GLO] = 0.08;
        _sigma_def_doppler[GAL] = 0.06;
        _sigma_def_doppler[BDS] = 0.06;
        _sigma_def_doppler[QZS] = 0.06;
        _sigma_def_doppler[IRN] = 0.06;

        _sigma_def_LEO[GPS] = base_pair(15.0, 0.01);
        _sigma_def_LEO[GLO] = base_pair(15.0, 0.01);
        _sigma_def_LEO[GAL] = base_pair(15.0, 0.01); //for LEO by zhangwei
        _sigma_def_LEO[BDS] = base_pair(15.0, 0.01);
        _sigma_def_LEO[QZS] = base_pair(15.0, 0.01);
        _sigma_def_LEO[IRN] = base_pair(15.0, 0.01);

        _maxres_def[GPS] = base_pair(10.0, 0.08);
        _maxres_def[GLO] = base_pair(15.0, 0.08);
        _maxres_def[GAL] = base_pair(15.0, 0.08);
        _maxres_def[BDS] = base_pair(15.0, 0.08);
        _maxres_def[QZS] = base_pair(15.0, 0.08);
        _maxres_def[IRN] = base_pair(15.0, 0.08);
        _maxres_def_doppler[GPS] = 0.1;
        _maxres_def_doppler[GLO] = 0.1;
        _maxres_def_doppler[GAL] = 0.1;
        _maxres_def_doppler[BDS] = 0.1;
        _maxres_def_doppler[QZS] = 0.1;
        _maxres_def_doppler[IRN] = 0.1;

        hwa_map_gnss gnss_data = gnss_data_priority();
        for (auto itGNSS = gnss_data.begin(); itGNSS != gnss_data.end(); ++itGNSS) // using C++11 initializer
        {
            GSYS gsys = itGNSS->first;
            string gs = gnss_sys::gsys2str(gsys);

            for (auto itBAND = gnss_data[gsys].begin();
                 itBAND != gnss_data[gsys].end();
                 ++itBAND)
            {
                GOBSBAND gobsband = itBAND->first;
                string band = gobsband2str(gobsband);
                _band_str[gsys].push_back(band);
                //    _band_obs[gsys].push_back( gobsband );

                for (auto itTYPE = gnss_data[gsys][gobsband].begin();
                     itTYPE != gnss_data[gsys][gobsband].end();
                     ++itTYPE)
                {
                    GOBSTYPE gobstype = itTYPE->first;
                    string type = gobstype2str(gobstype);
                    os << gs << "  band: " << band << "  type: " << type << "  attr:";

                    // ONLY THOSE NOT YET INCLUDED
                    set<string> type_search(_type_str[gsys].begin(), _type_str[gsys].end());
                    if (type_search.find(type) == type_search.end())
                    {

                        _type_str[gsys].push_back(type);
                        //        _type_obs[gsys].push_back( gobstype );
                    }

                    for (auto itATTR = gnss_data[gsys][gobsband][gobstype].begin();
                         itATTR != gnss_data[gsys][gobsband][gobstype].end();
                         ++itATTR)
                    {
                        GOBSATTR gobsattr = *itATTR;
                        string attr = gobsattr2str(gobsattr);
                        os << " " << attr;

                        // ONLY THOSE NOT YET INCLUDED
                        set<string> attr_search(_attr_str[gsys].begin(), _attr_str[gsys].end());
                        if (attr_search.find(attr) == attr_search.end())
                        {
                            _attr_str[gsys].push_back(attr);
                            //          _attr_obs[gsys].push_back( gobsattr );
                        }
                    }
                    os << endl;
                }
            }
            os << endl;
        }

#ifdef DEBUG
        std::cout << endl
             << "GNSS DEFAULT SETTINGS:\n"
             << os.str();
#endif
    }

    // Destructor
    // ----------
    set_gnss::~set_gnss()
    {
    }

    // Return set (to replace gsetgen general functions, used in processing modules)
    // ----------
    set<string> set_gnss::sat()
    {
        set<string> tmp;
        set<string>::const_iterator it;

        hwa_map_sats _gnss_sats = gnss_sats();
        hwa_map_sats::const_iterator itGNS;
        for (itGNS = _gnss_sats.begin(); itGNS != _gnss_sats.end(); ++itGNS)
        {
            GSYS gsys = itGNS->first;
            string gs = gnss_sys::gsys2str(gsys);

            transform(gs.begin(), gs.end(), gs.begin(), ::towlower);
            set<string> sats = set_base::_setval(gs, "sat");

            // AUTO SET
            // if( sats.size() == 0 ) sats = gnss_sats[gsys];

            for (it = sats.begin(); it != sats.end(); ++it)
                tmp.insert(*it);
        }
        return tmp;
    }

    // Return set
    // ----------
    set<string> set_gnss::sat(GSYS gsys, bool def) // default=true enforce to fill
    {
        set<string> tmp = set_base::_setvals(XMLKEY_GNSS, _gsys(gsys), "sat");

        // AUTO SET
        if (def && tmp.size() == 0)
            tmp = gnss_sats()[gsys];
        return tmp;
    }

    // Return set
    // ----------
    set<string> set_gnss::nav(GSYS gsys, bool def) // default=true enforce to fill
    {
        set<string> tmp = set_base::_setvals(XMLKEY_GNSS, _gsys(gsys), "nav");

        // AUTO SET
        if (def && tmp.size() == 0)
            tmp = gnss_gnav()[gsys];
        return tmp;
    }

    // Return set
    // ----------
    set<string> set_gnss::obs(GSYS gsys, bool def)
    {
        string str;
        set<string> tmp;

        vector<string> type = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "type");
        vector<string> band = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "band");
        vector<string> attr = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "attr");

        vector<string>::const_iterator itT;
        vector<string>::const_iterator itB;
        vector<string>::const_iterator itA;

        string gs = gnss_sys::gsys2str(gsys);
        std::ostringstream os;
        os << gs << "  band: " << band.size() << "  type: " << type.size() << "  attr: " << attr.size() << endl;

        int bset = band.size();
        int tset = type.size();
        int aset = attr.size();

        // AUTO SET
        if (bset == 0)
            band.assign(_band_str[gsys].begin(), _band_str[gsys].end());
        if (tset == 0)
            type.assign(_type_str[gsys].begin(), _type_str[gsys].end());
        if (aset == 0)
            attr.assign(_attr_str[gsys].begin(), _attr_str[gsys].end());

        for (itB = band.begin(); itB != band.end(); ++itB)
        {
            GOBSBAND gband = char2gobsband((*itB)[0]);
            string b = gobsband2str(gband);
            if (gband == 999)
                continue;

            for (itT = type.begin(); itT != type.end(); ++itT)
            {
                GOBSTYPE gtype = char2gobstype((*itT)[0]);
                string t = gobstype2str(gtype);
                if (gtype == 999)
                    continue;

                os << gs << "  band: " << b << "  type: " << t << "  attr:";

                for (itA = attr.begin(); itA != attr.end(); ++itA)
                {
                    GOBSATTR gattr = char2gobsattr((*itA)[0]);
                    string a = gobsattr2str(gattr);
                    if (gattr == 999)
                        continue;

                    str = gobstype2str(gtype);
                    str += gobsband2str(gband);
                    str += gobsattr2str(gattr);

                    if (def || (bset || tset || aset))
                        tmp.insert(str);
                    // if( def || (bset && tset && aset) ) tmp.insert( str2gobs( str ) );
                    os << " " << a;
                }
                os << endl;
            }
        }
#ifdef DEBUG
        std::cout << endl
             << "GNSS USER SETTINGS:\n"
             << os.str();
#endif
        return tmp;
    }

    // Return set
    // ----------
    set<string> set_gnss::gobs(GSYS gsys)
    {
        string str;
        set<string> tmp;

        vector<string> vgobs = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "gobs");

        for (auto it = vgobs.begin(); it != vgobs.end(); it++)
        {
            tmp.insert(*it);
        }

#ifdef DEBUG
        std::cout << "Set GOBS: ";
        for (auto it = tmp.begin(); it != tmp.end(); it++)
            std::cout << *it << " ";
        std::cout << endl;
#endif
        return tmp;
    }

    // Return set
    // ----------
    std::vector<GOBSTYPE> set_gnss::type(GSYS gsys)
    {
        vector<GOBSTYPE> v_tmp = _type(gsys);
        return v_tmp;
    }

    // Return set
    // ----------
    std::vector<GOBSBAND> set_gnss::band(GSYS gsys)
    {
        std::vector<GOBSBAND> v_tmp = _band(gsys);
        return v_tmp;
    }

    std::vector<FREQ_SEQ> set_gnss::freqs(GSYS gsys)
    {
        std::vector<FREQ_SEQ> v_tmp = _freqs(gsys);

        if (v_tmp.empty())
        {
            v_tmp.push_back(FREQ_1);
            v_tmp.push_back(FREQ_2);
        }
        return v_tmp;
    }

    std::map<FREQ_SEQ, GOBSBAND> set_gnss::band_index(GSYS gsys)
    {
        auto XmlFreq = freqs(gsys);
        auto XmlBand = band(gsys);

        if (XmlFreq.size() != XmlBand.size())
        {
            throw std::logic_error("Band size should be the same with freq size in xml.");
        }

        std::map<FREQ_SEQ, GOBSBAND> tmp;
        for (unsigned int i = 0; i < XmlBand.size(); i++)
        {
            tmp[XmlFreq[i]] = XmlBand[i];
        }

        return tmp;
    }

    //vector<FREQ_SEQ> set_gnss::freqs(GSYS gsys)
    //{
    //    

    //    vector<FREQ_SEQ> v_tmp = _freqs(gsys);

    //    if (v_tmp.empty())
    //    {
    //        v_tmp.push_back(FREQ_1);
    //        v_tmp.push_back(FREQ_2);
    //    }

    //     return v_tmp;
    //}

    map<GOBSBAND, FREQ_SEQ> set_gnss::freq_index(GSYS gsys)
    {
        auto XmlFreq = freqs(gsys);
        auto XmlBand = band(gsys);

        if (XmlFreq.size() != XmlBand.size())
        {
            throw std::logic_error("Band size should be the same with freq size in xml.");
        }

        map<GOBSBAND, FREQ_SEQ> tmp;
        for (unsigned int i = 0; i < XmlBand.size(); i++)
        {
            tmp[XmlBand[i]] = XmlFreq[i];
        }

        return tmp;
    }

    // Return set
    // ----------
    vector<GOBSATTR> set_gnss::attr(GSYS gsys)
    {
        vector<GOBSATTR> v_tmp = _attr(gsys);
        return v_tmp;
    }

    // supported GNSS system
    set<GSYS> set_gnss::SYS_SUPPORTED()
    {
        set<GSYS> systems;

        systems.insert(GPS);
        systems.insert(GLO);
        systems.insert(GAL);
        systems.insert(BDS);
        systems.insert(QZS);
        systems.insert(IRN);

        return systems;
    }

    // Return set
    // ----------
    double set_gnss::sigma_C(GSYS gsys)
    {
        double tmp = _sigma_C(gsys);
        return tmp;
    }

    //for LEO
    double set_gnss::sigma_C_LEO(GSYS gsys)
    {
        double tmp = _sigma_C_LEO(gsys);
        return tmp;
    }

    double set_gnss::sigma_D(GSYS gsys)
    {
        double tmp = _sigma_D(gsys);
        return tmp;
    }

    // Return set
    // ----------
    double set_gnss::sigma_L(GSYS gsys)
    {
        double tmp = _sigma_L(gsys);
        return tmp;
    }

    //for LEO
    double set_gnss::sigma_L_LEO(GSYS gsys)
    {
        double tmp = _sigma_L_LEO(gsys);
        return tmp;
    }

    // Return set
    // ----------
    double set_gnss::maxres_C(GSYS gsys)
    {
        double tmp = _maxres_C(gsys);
        return tmp;
    }

    double set_gnss::maxres_D(GSYS gsys)
    {
        double tmp = _maxres_D(gsys);
        return tmp;
    }

    // Return set
    // ----------
    double set_gnss::maxres_L(GSYS gsys)
    {
        double tmp = _maxres_L(gsys);
        return tmp;
    }

    // settings check
    // ----------
    void set_gnss::check()
    {
        /*
          // check existence of nodes/attributes
          xml_node parent = _doc.child(XMLKEY_ROOT);

          for( itGNSS = _gnss.begin(); itGNSS != _gnss.end(); ++itGNSS ){

            xml_node node  = _default_node( parent, itGNSS->c_str() );

            _default_node( node, "sat"  );
            _default_node( node, "type" );
            _default_node( node, "band" );
            _default_node( node, "attr" );
          }
        */
        return;
    }

    // convert GNSS
    // ----------
    string set_gnss::_gsys(GSYS gsys)
    {
        string tmp = gnss_sys::gsys2str(gsys);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
        return tmp;
    }

    // Return set
    // ----------
    vector<GOBSTYPE> set_gnss::_type(GSYS gsys)
    {
        vector<string> v_str = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "type");
        vector<GOBSTYPE> v_tmp;
        for (vector<string>::const_iterator it = v_str.begin(); it != v_str.end(); ++it)
        {
            string str = *it;
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            GOBSTYPE gobstype = str2gobstype(str);
            if (gobstype != TYPE)
                v_tmp.push_back(gobstype);
        };
        return v_tmp;
    }

    // Return set
    // ----------
    vector<GOBSBAND> set_gnss::_band(GSYS gsys)
    {
        vector<string> v_str = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "band");
        vector<GOBSBAND> v_tmp;
        for (vector<string>::const_iterator it = v_str.begin(); it != v_str.end(); ++it)
        {
            string str = *it;
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            GOBSBAND gobsband = str2gobsband(str);
            if (gobsband != BAND)
            {
                v_tmp.push_back(gobsband);
            }
        };

        // no use
        //// jdhuang
        //// for case : The v_str is empty  --> set default bands glfeng
        if (v_tmp.empty())
        {
            // jdhuang : force to set band in xml
            throw std::logic_error("You need set your band for " + gnss_sys::gsys2str(gsys) + " in xml file");

            switch (gsys)
            {
            case GPS:
                v_tmp.push_back(BAND_1);
                v_tmp.push_back(BAND_2);
                break;
            case GLO:
                v_tmp.push_back(BAND_1);
                v_tmp.push_back(BAND_2);
                break;
            case GAL:
                v_tmp.push_back(BAND_1);
                v_tmp.push_back(BAND_5);
                break;
            case BDS:
                v_tmp.push_back(BAND_2);
                v_tmp.push_back(BAND_7);
                break;
            case QZS:
                v_tmp.push_back(BAND_1);
                v_tmp.push_back(BAND_2);
            default:
                break;
            }
        }

        return v_tmp;
    }

    // Return set
    // ----------
    vector<FREQ_SEQ> set_gnss::_sysfreq(GSYS gsys)
    {
        vector<string> v_str = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "freq");
        vector<FREQ_SEQ> v_tmp;
        for (vector<string>::const_iterator it = v_str.begin(); it != v_str.end(); ++it)
        {
            string str = *it;
            //transform(str.begin(), str.end(), str.begin(), ::toupper);
            FREQ_SEQ gsysfreq = str2sysfreq(str);
            if (gsysfreq != FREQ_X)
            {
                v_tmp.push_back(gsysfreq);
            }
        };

        // default frequency 12
        if (v_tmp.empty())
        {
            switch (gsys)
            {
            case GPS:
                v_tmp.push_back(FREQ_1);
                v_tmp.push_back(FREQ_2);
                break;
            case GLO:
                v_tmp.push_back(FREQ_1);
                v_tmp.push_back(FREQ_2);
                break;
            case GAL:
                v_tmp.push_back(FREQ_1);
                v_tmp.push_back(FREQ_2);
                break;
            case BDS:
                v_tmp.push_back(FREQ_1);
                v_tmp.push_back(FREQ_2);
                break;
            case QZS:
                v_tmp.push_back(FREQ_1);
                v_tmp.push_back(FREQ_2);
            default:
                break;
            }

            std::cout << "Warning : do not set the freq, use default settings" << endl;
        }

        return v_tmp;
    }

    vector<FREQ_SEQ> set_gnss::_freqs(GSYS gsys)
    {
        vector<string> v_str = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "freq");
        vector<FREQ_SEQ> v_tmp;
        for (vector<string>::const_iterator it = v_str.begin(); it != v_str.end(); ++it)
        {
            string str = *it;
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            FREQ_SEQ gobsband = str2gnssfreq(str);
            if (gobsband != FREQ_SEQ::FREQ_X)
            {
                v_tmp.push_back(gobsband);
            }
        };

        if (v_tmp.empty())
        {
            v_tmp.push_back(FREQ_1);
            v_tmp.push_back(FREQ_2);
        }

        return v_tmp;
    }

    // Return set
    // ----------
    vector<GOBSATTR> set_gnss::_attr(GSYS gsys)
    {
        vector<string> v_str = set_base::_vecval(XMLKEY_GNSS, _gsys(gsys), "attr");
        vector<GOBSATTR> v_tmp;
        for (vector<string>::const_iterator it = v_str.begin(); it != v_str.end(); ++it)
        {
            string str = *it;
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            GOBSATTR gobsattr = str2gobsattr(str);
            if (gobsattr != ATTR)
                v_tmp.push_back(gobsattr);
        };
        return v_tmp;
    }

    // Return set
    // ----------
    double set_gnss::_sigma_C(GSYS gsys)
    {
        double sig = 0.0;
        string sys = _gsys(gsys);
        //transform(sys.begin(), sys.end(), sys.begin(), ::toupper);
        sig = set_base::_dblatt(XMLKEY_GNSS, sys, "sigma_C");

        // default value
        if (double_eq(sig, 0.0))
            sig = _sigma_def[gsys][0]; // base_pair(sig_code, sig_phase)

        return sig;
    }

    //for LEO
    double set_gnss::_sigma_C_LEO(GSYS gsys)
    {
        double sig = 0.0;
        string sys = _gsys(gsys);
        //transform(sys.begin(), sys.end(), sys.begin(), ::toupper);
        sig = set_base::_dblatt(XMLKEY_GNSS, sys, "sigma_C_LEO");

        // default value
        if (double_eq(sig, 0.0))
            sig = _sigma_def_LEO[gsys][0]; // base_pair(sig_code, sig_phase)

        return sig;
    }

    double set_gnss::_sigma_D(GSYS gsys)
    {
        double sig = 0.0;

        sig = set_base::_dblatt(XMLKEY_GNSS, _gsys(gsys), "sigma_D");

        // default value
        if (double_eq(sig, 0.0))
            sig = _sigma_def_doppler[gsys]; // map<sys,double>

        return sig;
    }

    // Return set
    // ----------
    double set_gnss::_sigma_L(GSYS gsys)
    {
        double sig = 0.0;
        string sys = _gsys(gsys);
        //transform(sys.begin(), sys.end(), sys.begin(), ::toupper);
        sig = set_base::_dblatt(XMLKEY_GNSS, sys, "sigma_L");

        // default value
        if (double_eq(sig, 0.0))
            sig = _sigma_def[gsys][1]; // base_pair(sig_code, sig_phase)

        return sig;
    }

    // add for LEO by zhangwei
    // ----------
    double set_gnss::_sigma_L_LEO(GSYS gsys)
    {
        double sig = 0.0;
        string sys = _gsys(gsys);
        //transform(sys.begin(), sys.end(), sys.begin(), ::toupper);
        sig = set_base::_dblatt(XMLKEY_GNSS, sys, "sigma_L_LEO");

        // default value
        if (double_eq(sig, 0.0))
            sig = _sigma_def_LEO[gsys][1]; // base_pair(sig_code, sig_phase)

        return sig;
    }

    // Return set
    // ----------
    double set_gnss::_maxres_C(GSYS gsys)
    {
        double res = 0.0;

        res = set_base::_dblatt(XMLKEY_GNSS, _gsys(gsys), "maxres_C");

        // default value
        if (double_eq(res, 0.0))
            res = _maxres_def[gsys][0]; // base_pair(res_code, res_phase)

        return res;
    }

    double set_gnss::_maxres_D(GSYS gsys)
    {
        double sig = 0.0;
        sig = set_base::_dblatt(XMLKEY_GNSS, _gsys(gsys), "sigma_D");

        // default value
        if (double_eq(sig, 0.0))
            sig = _maxres_def_doppler[gsys]; // map<sys,double>

        return sig;
    }

    // Return set
    // ----------
    double set_gnss::_maxres_L(GSYS gsys)
    {
        double res = 0.0;

        res = set_base::_dblatt(XMLKEY_GNSS, _gsys(gsys), "maxres_L");

        // default value
        if (double_eq(res, 0.0))
            res = _maxres_def[gsys][1]; // base_pair(res_code, res_phase)

        return res;
    }

    // help body
    // ----------
    void set_gnss::help()
    {
        cerr << " <gps>            \t\t  <!-- any GNSS constellation: GPS GLO GAL BDS SBS QZS -->\n"
             << "   <sat>  </sat>  \t\t  <!-- list of GPS satellites: G01 G02 .. or empty(ALL) -->\n"
             << "   <type> </type> \t\t  <!-- list of GPS  obs types: C L D S P or empty(ALL)  -->\n"
             << "   <band> </band> \t\t  <!-- list of GPS  obs bands: 1 2 5 or empty(ALL) -->\n"
             << "   <attr> </attr> \t\t  <!-- list of PGS attributes: A B C D I L M N P Q S W X Y Z or empty(ALL) -->\n"
             << " </gps>\n\n";

        cerr << " <glo>            \t\t  <!-- any GNSS constellation: GPS GLO GAL BDS SBS QZS -->\n"
             << "   <sat>  </sat>  \t\t  <!-- list of GPS satellites: R01 R02 .. or empty(ALL) -->\n"
             << "   <type> </type> \t\t  <!-- list of GPS  obs types: C L D S P or empty(ALL)  -->\n"
             << "   <band> </band> \t\t  <!-- list of GPS  obs bands: 1 2 3 or empty(ALL) -->\n"
             << "   <attr> </attr> \t\t  <!-- list of PGS attributes: A B C D I L M N P Q S W X Y Z or empty(ALL) -->\n"
             << " </glo>\n\n";

        cerr << "\t<!-- gnss observation definition:\n"
             << "\t      .. ID name\n"
             << "\t name   .. full name\n"
             << "\t desc   .. description\n"
             << "\t -->\n\n";
        return;
    }

} // namespace
