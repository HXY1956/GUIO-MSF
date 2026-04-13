#include "hwa_set_gen.h"
#include "hwa_set_gbase.h"
#include "hwa_gnss_sys.h"

#define DEF_RECEIVER "   " ///< Default receiver : all !
#define DEF_SAMPLING 30    ///< Default sampling : 30s !

using namespace std;
using namespace pugi;
using namespace hwa_base;
using namespace hwa_gnss;

namespace hwa_set
{
    set_gen::set_gen(bool gnss)
        : set_base(),
          _gnss(gnss),
          _dec(0)
    {
        _set.insert(XMLKEY_GEN);

        // initiate GNSS std::string
        if (_gnss)
        {
            hwa_map_sats _gnss_sats = gnss_sats();
            for (auto itGNS = _gnss_sats.begin(); itGNS != _gnss_sats.end(); ++itGNS)
            {
                _sys += " " + gnss_sys::gsys2str(itGNS->first);
            }
        }
    }

    // Destructor
    // ----------
    set_gen::~set_gen()
    {
    }

    // Return gtime
    // ----------
    base_time set_gen::beg(bool conv)
    {
        std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("beg");
        base_type_conv::substitute(str, "\n", "");
        base_type_conv::substitute(str, "\"", "");

        base_time gt(base_time::GPS);

        if (str.empty())
        {
            base_time tmp(FIRST_TIME);
            return tmp;
        }

        gt.from_str("%Y-%m-%d %H:%M:%S", base_type_conv::trim(str), conv);
        return gt;
    }

    // Return gtime
    // ----------
    base_time set_gen::end(bool conv)
    {
        std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("end");
        base_type_conv::substitute(str, "\n", "");
        base_type_conv::substitute(str, "\"", "");

        base_time gt(base_time::GPS);

        if (str.empty())
        {
            base_time tmp(LAST_TIME);
            return tmp;
        }

        gt.from_str("%Y-%m-%d %H:%M:%S", base_type_conv::trim(str), conv);
        return gt;
    }

    // Return sampling
    // ----------
    double set_gen::sampling()
    {
        std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("int");

        // delete spaces
        str.erase(remove(str.begin(), str.end(), ' '), str.end());

        double tmp = base_type_conv::str2dbl(str);

        if (str.find(".") != std::string::npos)
        {
            _dec = str.substr(str.find(".") + 1).length(); // decimal digits resolution
        }
        return tmp;
    }

    double set_gen::sampling_default() const
    {
        return DEF_SAMPLING;
    }

    // Return std::set
    // ----------
    std::set<std::string> set_gen::sys()
    {
        std::set<std::string> xcl, tmp = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "sys");

        // exclude starting with '-'
        std::set<std::string>::iterator itSYS, itTMP;
        for (itTMP = tmp.begin(); itTMP != tmp.end();)
        {
            if ((*itTMP)[0] == '-')
            {
                xcl.insert(*itTMP);
                itSYS = itTMP;
                ++itTMP;
                tmp.erase(itSYS);
            } //"-"mean without this system?
            else
                ++itTMP;
        }

        // if empty, complete, i.e. if only exclusions listed (and gnss requested!)
        if (tmp.size() == 0 && _gnss)
        {
            hwa_map_sats _gnss_sats = gnss_sats();
            hwa_map_sats::const_iterator itGNS;
            // loop over all systems
            for (itGNS = _gnss_sats.begin(); itGNS != _gnss_sats.end(); ++itGNS)
            {
                std::string gs = gnss_sys::gsys2str(itGNS->first);
                if (xcl.find("-" + gs) == xcl.end())
                    tmp.insert(gs);
            }
        }
        return tmp;
    }

    void set_gen::sys(std::string str)
    {
        xml_node proc = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN);
        proc.remove_child("sys");
        xml_node obs_combination = proc.append_child("sys");
        obs_combination.append_child(xml_node_type::node_pcdata).set_value(str.c_str());
    }

    std::set<std::string> set_gen::recs()
    {
        std::set<std::string> tmp = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "rec");
        return tmp;
    }

    std::set<std::string> set_gen::slr()
    {
        std::set<std::string> tmp = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "slr");
        std::set<std::string> result;
        for (auto it = tmp.begin(); it != tmp.end(); it++)
        {
            if (*it == "GNS" && *it == "LEO")
            {
                result.insert(*it);
            }
        }
        return result;
    }

    bool set_gen::kbr()
    {
        bool tmp;
        std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("kbr");
        if (str == "")
        {
            tmp = false;
        }
        else
        {
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            if (str == "YES")
            {
                tmp = true;
            }
            else
            {
                tmp = false;
            }
        }
        return tmp;
    }

    bool set_gen::lri()
    {
        bool tmp;
        std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("lri");
        if (str == "")
        {
            tmp = false;
        }
        else
        {
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            if (str == "YES")
            {
                tmp = true;
            }
            else
            {
                tmp = false;
            }
        }
        return tmp;
    }

    bool set_gen::vlbi()
    {
        bool tmp;
        std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("vlbi");
        if (str == "")
        {
            tmp = false;
        }
        else
        {
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            if (str == "YES")
            {
                tmp = true;
            }
            else
            {
                tmp = false;
            }
        }
        return tmp;
    }

    bool set_gen::gnss()
    {
        bool tmp;
        std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("gnss");
        if (str == "")
        {
            tmp = true;
        }
        else
        {
            transform(str.begin(), str.end(), str.begin(), ::toupper);
            if (str == "YES")
            {
                tmp = true;
            }
            else
            {
                tmp = false;
            }
        }
        return tmp;
    }

    std::set<std::string> set_gen::mode()
    {
        //std::set<std::string> tmp;
        //std::set<std::string> sats = rec_all();
        //std::set<std::string> leos = rec_leo();
        //std::set<std::string> recs;
        //set_difference(sats.begin(), sats.end(), leos.begin(), leos.end(), inserter(recs, recs.begin()));
        //if (recs.size() > 0)
        //{
        //    tmp.insert("S");  //GNSS POD
        //    if (leos.size() > 0)
        //    {
        //        std::string leomode = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child("rec").attribute("mode").value();
        //        if (leomode == "D")
        //        {
        //            tmp.insert("D");  //Integrated POD(estimate LEO orbit)
        //        }
        //        else // if (leomode == "F")
        //        {
        //            tmp.insert("F");  //Integrated POD(fix LEO orbit)
        //        }
        //    }
        //}
        //else
        //{
        //    std::string leomode = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child("rec").attribute("mode").value();
        //    if (leomode == "K")
        //    {
        //        tmp.insert("K");  //LEO Kinematic POD
        //    }
        //    else if (leomode == "F")
        //    {
        //        tmp.insert("F");
        //    }
        //    else
        //    {
        //        tmp.insert("D");  //LEO Dynamic POD
        //    }
        //}
        //return tmp;
        std::set<std::string> tmp;
        auto xml_node = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).children("rec");
        for (auto rec_node = xml_node.begin(); rec_node != xml_node.end(); rec_node++)
        {
            std::string mode = rec_node->attribute("mode").value();
            if (mode == "")
            {
                mode = "S";
            }
            tmp.insert(mode);
        }
        return tmp;
    }

    std::map<std::string, std::set<std::string>> set_gen::mode_map()
    {
        std::string temp;
        std::string key;
        std::string word;
        std::map<std::string, std::set<std::string>> mapval;
        auto xml_node = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).children("rec");
        for (auto rec_node = xml_node.begin(); rec_node != xml_node.end(); rec_node++)
        {
            std::set<std::string> vals;
            temp = rec_node->child_value();
            key = rec_node->attribute("mode").value();
            if (key == "")
            {
                key = "S"; //Default ground station
            }
            std::istringstream is(temp);
            while (is >> word)
            {
                transform(word.begin(), word.end(), word.begin(), ::toupper);
                vals.insert(word);
            }
            mapval[key] = vals;
        }
        return mapval;
    }

    // Return reference clk std::set
    // ------------------------
    std::string set_gen::refsat()
    {
        std::set<std::string> src = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "refsat");
        if (src.empty())
        {
            std::string tmp("");
            return tmp;
        }
        else
        {
            return *src.begin();
        }
    }

    std::set<std::string> set_gen::rec_all()
    {
        std::set<std::string> tmp = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "rec");
        return tmp;
    }

    std::set<std::string> set_gen::rec_leo()
    {
        std::set<std::string> tmp = set_base::_setval_attribute(XMLKEY_GNSS, XMLKEY_GEN, "rec", "type", "leo");
        return tmp;
    }

    std::vector<std::string> set_gen::list_base()
    {
        std::vector<std::string> vals;
        std::string word;
        std::istringstream is(_doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("base"));
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vals.push_back(word);
        }
        return vals;
    }

    std::vector<std::string> set_gen::list_rover()
    {
        std::vector<std::string> vals;
        std::string word;
        std::istringstream is(_doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child_value("rover"));

        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vals.push_back(word);
        }
        return vals;
    }

    // Return reference clk std::set
    // ------------------------
    std::string set_gen::refsite()
    {
        std::set<std::string> src = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "refsite");
        if (src.empty())
        {
            std::string tmp("");
            return tmp;
        }
        else
        {
            return *src.begin();
        }
    }

    // Return reference clk sigma
    double set_gen::sig_refclk()
    {

        std::string child = "refsite";
        std::string refsite = this->refsite();
        if (refsite.empty())
        {
            child = "refsat";
        }

        double tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).child(child.c_str()).attribute("sigclk").as_double(0.0);
        //return tmp;
        //jdhuang : make sure the function unlock before return
        return tmp;
    }

    // Return estimator model
    // ------------------------
    std::string set_gen::estimator()
    {
        std::set<std::string> src = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "est");
        if (src.empty() || ((*src.begin() != "lsq" && *src.begin() != "FLT")))
        {
            std::string tmp("lsq");
            return tmp;
        }
        else
        {

            return *src.begin();
        }
    }

    // Return satellites which no used in calculation
    // ------------------------
    std::set<std::string> set_gen::sat_rm()
    {
        std::set<std::string> tmp = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "sat_rm");
        return tmp;
    }

    void set_gen::sat_rm(std::string str)
    {
        xml_node proc = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN);
        proc.remove_child("sat_rm");
        xml_node obs_combination = proc.append_child("sat_rm");
        obs_combination.append_child(xml_node_type::node_pcdata).set_value(str.c_str());
    }

    bool set_gen::isClient()
    {
        std::set<std::string> src = set_base::_setvals(XMLKEY_GNSS, XMLKEY_GEN, "client");
        if (src.size() == 0)
        {
            return false;
        }
        if ((*src.begin() == "YES" || *src.begin() == "yes"))
        {
            return true;
        }
        else
        {
            return false;
        }
        return false;
    }

    // settings check
    // ----------
    void set_gen::check()
    {
        // check existence of nodes/attributes
        xml_node parent = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS);
        xml_node node = _default_node(parent, XMLKEY_GEN);

        _default_node(node, "BEG", "");                            // all!
        _default_node(node, "END", "");                            // all!
        _default_node(node, "SYS", "");                            // all!
        _default_node(node, "REC", "");                            // none
        _default_node(node, "INT", base_type_conv::int2str(DEF_SAMPLING).c_str()); // default

        // TO CHECK USER-SETUP CONSISTENCY
        if (floor(sampling()) < 1 || _dec > 0)
        {

            if (sampling() < 0.0)
            {
                _default_node(node, "INT", "0.0", true); // reset!

                std::cout << "Warning: sampling rate settings negative: reset to 0" << std::endl;
            }
            else
            {

                std::cout << "gsetgen: sampling rate settings above 1Hz recognized" << std::endl;
            }
        }

        return;
    }

    // help body
    // ----------
    void set_gen::help()
    {
        base_time beg(base_time::GPS);
        beg = beg - beg.sod();
        base_time end(base_time::GPS);
        end = beg + 86399;

        std::cerr << "\n <gen>\n"
             << "   <beg> \"" << beg.str_ymdhms() << "\" </beg>\n"  // FIRST_TIME.str("\"%Y-%m-%d %H:%M:%S\"")
             << "   <end> \"" << end.str_ymdhms() << "\" </end>\n"; // LAST_TIME.str("\"%Y-%m-%d %H:%M:%S\"")

        if (_gnss)
            std::cerr << "   <sys> " << _sys << " </sys>\n"; // GNSS systems

        std::cerr << "   <rec> GOPE WTZR POTS                </rec>\n" // list of site identificators
             << "   <int>" + base_type_conv::int2str(DEF_SAMPLING) + "</int>\n"
             << " </gen>\n";

        std::cerr << "\t<!-- general description:\n"
             << "\t beg    .. beg time          (default: all)\n"
             << "\t end    .. end time          (default: all)\n"
             << "\t int    .. data sampling     (default: 30s)\n";

        if (_gnss)
            std::cerr << "\t sys    .. GNSS system(s)    (default: all)\n";

        std::cerr << "\t rec    .. GNSS receiver(s)  (rec active list, e.g.: GOPE ONSA WTZR ... )\n"
             << "\t -->\n\n";
        return;
    }

    gnss_data_obj* set_gen::grec_obj(const std::string& name, base_log spdlog)
    {
        gnss_data_obj* obj = nullptr;
        std::string sites;
        std::set<std::string> tmp = rec_all();

        if (tmp.find(name) == tmp.end())
        {
            return obj;
        }
        std::string sites_leo = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_GEN).find_child_by_attribute("type", "leo").child_value();
        transform(sites_leo.begin(), sites_leo.end(), sites_leo.begin(), ::toupper);
        obj = new gnss_data_rec(spdlog);
        obj->id(name);
        return obj;
    }

} // namespace
