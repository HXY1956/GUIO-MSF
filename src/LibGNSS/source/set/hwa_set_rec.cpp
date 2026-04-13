#include <iomanip>
#include <sstream>
#include <algorithm>
#include "hwa_set_rec.h"
#include "hwa_base_globaltrans.h"

using namespace std;
using namespace pugi;
using namespace hwa_gnss;

#ifndef XMLKEY_GEN
#define XMLKEY_GEN "gen"
#endif // !XMLKEY_GEN

#ifndef XMLKEY_REC_REC
#define XMLKEY_REC_REC "rec"
#endif // !1

namespace hwa_set
{
    set_rec::set_rec()
        : set_base()
    {
        _set.insert(XMLKEY_REC);
        _id = "";
        _name_rec = "";
        _desc = "";
        _domes = "";
        _X = 0.0;
        _Y = 0.0;
        _Z = 0.0;
        _dX = 0.0;
        _dY = 0.0;
        _dZ = 0.0;
        _dE = 0.0;
        _dN = 0.0;
        _dU = 0.0;
        _ZTD = 0.0;
        _ZTD_sig = 0.0;
        _crd_sig = "";
        _rec = "";
        _ant = "";
        _beg = FIRST_TIME;
        _end = LAST_TIME;
        _overwrite = false;
    }

    // Destructor
    // ----------
    set_rec::~set_rec()
    {
    }

    // Return value
    // ----------
    std::string set_rec::rec(std::string s)
    {
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute(XMLKEY_REC_REC, "id", s.c_str());
        std::string tmp = site.attribute("rec").value();
        return tmp;
    }

    // Return value
    // ----------
    std::string set_rec::ant(std::string s)
    {
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute(XMLKEY_REC_REC, "id", s.c_str());
        std::string tmp = site.attribute("ant").value();
        return tmp;
    }

    gnss_data_obj *set_rec::obj(std::string s)
    {
        gnss_data_obj *obj = nullptr;
        return obj;
    }

    // Get formats inputs
    // ----------
    set<std::string> set_rec::objects()
    {
        return _objects();
    }

    // Get formats inputs
    // ----------
    set<std::string> set_rec::_objects()
    {
        set<std::string> tmp;
        std::string str;

        for (xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).first_child(); node; node = node.next_sibling())
        {
            std::string name = node.name();
            if (name.compare(XMLKEY_REC_REC) == 0)
            {
                std::string str = node.attribute("id").as_string();
                if (!str.empty())
                {
                    tmp.insert(str);
                }
            }
        }
        return tmp;
    }

    // Return value
    // ----------
    double set_rec::_aprDX()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("DX").as_double();
    }

    // Return value
    // ----------
    double set_rec::_aprDY()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("DY").as_double();
    }

    // Return value
    // ----------
    double set_rec::_aprDZ()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("DZ").as_double();
    }

    // Return value
    // ----------
    double set_rec::_aprDE()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("DE").as_double();
    }

    // Return value
    // ----------
    double set_rec::_aprDN()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("DN").as_double();
    }

    // Return value
    // ----------
    double set_rec::_aprDU()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("DU").as_double();
    }

    // Return value
    // ----------
    double set_rec::_aprZTD()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("ZTD").as_double();
    }

    // Return value
    // ----------
    double set_rec::_sigZTD()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("ZTD_sig").as_double();
    }

    // Return value
    // ----------
    std::string set_rec::_sigCRD()
    {
        return _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).attribute("CRD_sig").as_string();
    }

    // Return value --> IN FUTURE OBSOLETE, BUT STILL USED IN GPPPFILTER (CHANGE TO _get_crd() below )
    // ----------
    int set_rec::get_crd_xyz(Triple &xyz, std::string s)
    {
        xyz = _get_crd_xyz(s);

        if (double_eq(xyz[0], 0.0) &&
            double_eq(xyz[1], 0.0) &&
            double_eq(xyz[2], 0.0))
        {
            return 0;
        }
        return 1;
    }

    Triple set_rec::get_crd_xyz(std::string s)
    {
        return _get_crd_xyz(s);
    }
    Triple set_rec::get_std_xyz(std::string s)
    {
        return _get_std_xyz(s);
    }

    // Return set
    // ----------
    set<std::string> set_rec::recs()
    {
        return _objects();
    }

    set<std::string> set_rec::all_rec()
    {
        set<std::string> tmp;
        std::string temp;
        auto xml_node = _doc.child(XMLKEY_ROOT).child(XMLKEY_GEN).children("rec");
        for (auto rec_node = xml_node.begin(); rec_node != xml_node.end(); rec_node++)
        {
            temp += rec_node->child_value();
        }
        std::istringstream is(temp);
        std::string word;
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            tmp.insert(word);
        }

        return tmp;
    }

    // Return value
    // ----------
    Triple set_rec::_get_crd_xyz(std::string s)
    {
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute(XMLKEY_REC_REC, "id", s.c_str());

        Triple xyz(site.attribute("X").as_double(),
                      site.attribute("Y").as_double(),
                      site.attribute("Z").as_double());
        return xyz;
    }

    Triple set_rec::_get_std_xyz(std::string s)
    {
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute(XMLKEY_REC_REC, "id", s.c_str());

        Triple xyz(site.attribute("dX").as_double(),
                      site.attribute("dY").as_double(),
                      site.attribute("dZ").as_double());

        return xyz;
    }

    // Return value
    // ----------
    Triple set_rec::_get_ecc_neu(std::string s)
    {
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute(XMLKEY_REC_REC, "id", s.c_str());

        Triple neu(site.attribute("DN").as_double(),
                      site.attribute("DE").as_double(),
                      site.attribute("DU").as_double());
        return neu;
    }

    // Return value
    // ----------
    Triple set_rec::_get_ecc_xyz(std::string s)
    {
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute(XMLKEY_REC_REC, "id", s.c_str());

        Triple xyz(site.attribute("DX").as_double(),
                      site.attribute("DY").as_double(),
                      site.attribute("DZ").as_double());
        return xyz;
    }

    // Return value
    // ----------

    // settings check
    // ----------
    void set_rec::check()
    {
        // check existence of nodes/attributes
        xml_node parent = _doc.child(XMLKEY_ROOT);
        xml_node node = _default_node(parent, XMLKEY_REC);

        // check existence of attributes
        _default_attr(node, "X", _X);
        _default_attr(node, "Y", _Y);
        _default_attr(node, "Z", _Z);
        _default_attr(node, "DX", _dX);
        _default_attr(node, "DY", _dY);
        _default_attr(node, "DZ", _dZ);
        _default_attr(node, "DN", _dN);
        _default_attr(node, "DE", _dE);
        _default_attr(node, "DU", _dU);
        _default_attr(node, "overwrite", _overwrite);

        _default_attr(node, "ZTD", _ZTD);
        _default_attr(node, "ZTD_sig", _ZTD_sig);
        _default_attr(node, "CRD_sig", _crd_sig);

        // CHECK USER-SETUP CONSISTENCY

        // check duplicities
        map<std::string, xml_node> mchild;
        for (xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC); node;)
        {
            xml_node rec = node;
            node = node.next_sibling(XMLKEY_REC); // get next immediately

            // check ID & NAME & DOMES
            std::string id = rec.attribute("id").value();
            std::string name = rec.attribute("name").value();
            std::string domes = rec.attribute("domes").value();

            if (name.empty())
            { // complete NAME
                rec.insert_attribute_after("name", rec.attribute("id")) = id.c_str();
            }
            if (domes.empty())
            { // complete NAME
                if (name.length() < 14)
                {
                    domes = "XXXXXXXXX";
                }
                if (name.length() >= 14)
                {
                    domes = tail(name, 9);
                    name = name.substr(0, name.size() - 10);
                }
                rec.insert_attribute_after("domes", rec.attribute("name")) = domes.c_str();
                rec.insert_attribute_after("name", rec.attribute("id")) = name.c_str();
            }

            if (mchild.find(id) == mchild.end())
            {
                mchild[id] = rec;
            }
            else
            {
                _doc.child(XMLKEY_ROOT).remove_child(rec);
                std::cout << "warning - removed duplicated record [id] for rec:" + id << endl;
            }

            // check 'set' duplicity
            map<std::string, xml_node> mattr;
            for (xml_node set = rec.child("set"); set;)
            {
                xml_node tmp = set;
                set = set.next_sibling("set"); // get next immediately

                std::string beg = tmp.attribute("beg").value();
                if (mattr.find(beg) == mattr.end())
                {
                    mattr[beg] = set;
                }
                else
                {
                    rec.remove_child(tmp);
                    std::cout << "warning - removed duplicated record [set] for rec:" + id + ", beg:" + beg << endl;
                }
            }
        }
        return;
    }

    // help body
    // ----------
    void set_rec::help()
    {
        cerr << " <rec"
             << " id=\"GOPE\""
             << " name=\"GOPE\""
             << " domes=\"11502M002\""
             << " desc=\"Geodetic Observatory Pecny, Czech Republic\" >\n"
             << "  <set"
             << " beg=\"1995 05 13 00 00 00\""
             << " end=\"1997 06 11 00 00 00\""
             << " rec=\"TRIMBLE 4000SSE\""
             << " ant=\"TRM14532.00     NONE\""
             << "\n\t"
             << " X=\"3979316.0\""
             << " Y=\"1050312.0\""
             << " Z=\"4857067.0\""
             << " dX=\"0.0\""
             << " dY=\"0.0\""
             << " dZ=\"0.0\""
             << " dN=\"0.0\""
             << " dE=\"0.0\""
             << " dU=\"0.0\""
             << "  />\n"
             << "  <set"
             << " beg=\"1997 06 11 00 00 00\""
             << " end=\"1997 06 20 00 00 00\""
             << " rec=\"SPP GEOTRACER100\""
             << " ant=\"TRM14532.00     NONE\""
             << "  />\n"
             << "  <set"
             << " beg=\"1997 06 20 00 00 00\""
             << " end=\"1999 11 04 00 00 00\""
             << " rec=\"TRIMBLE 4000SSE\""
             << " ant=\"TRM14532.00     NONE\""
             << "  />\n"
             << "  <set"
             << " beg=\"1999 11 04 00 00 00\""
             << " end=\"2000 07 24 00 00 00\""
             << " rec=\"ASHTECH Z18\""
             << " ant=\"ASH701073.3     SNOW\""
             << "  />\n"
             << "  <set"
             << " beg=\"2000 07 24 00 00 00\""
             << " end=\"2000 10 04 00 00 00\""
             << " rec=\"TRIMBLE 4000SSE\""
             << " ant=\"TRM14532.00     NONE\""
             << "  />\n"
             << "  <set"
             << " beg=\"2000 10 04 00 00 00\""
             << " end=\"2001 07 18 00 00 00\""
             << " rec=\"ASHTECH Z18\""
             << " ant=\"ASH701073.3     SNOW\""
             << "  />\n"
             << "  <set"
             << " beg=\"2006 07 14 00 00 00\""
             << " end=\"2009 12 14 00 00 00\""
             << " rec=\"ASHTECH Z18\""
             << " ant=\"TPSCR3_GGD      CONE\""
             << "  />\n"
             << "  <set"
             << " beg=\"2009 12 14 00 00 00\""
             << " end=\"2013 03 19 00 00 00\""
             << " rec=\"TPS NETG3\""
             << " ant=\"TPSCR.G3        TPSH\""
             << "  />\n"
             << " </rec>\n";

        cerr << "\t<!-- receiver description:\n"
             << "\t id     .. ID name\n"
             << "\t name   .. site name\n"
             << "\t domes  .. site domes\n"
             << "\t desc   .. description\n"
             << "\t X[YZ]  .. X,Y,Z-coordinate [m]\n"
             << "\t dX[YZ] .. X,Y,Z-eccentricity [m]\n"
             << "\t -->\n\n";
        return;
    }

    shared_ptr<gnss_data_rec> set_rec::grec(std::string s, base_log spdlog)
    {
        std::string str;
        shared_ptr<gnss_data_rec> tmp = std::make_shared<gnss_data_rec>(spdlog);

        // return if no rec id found
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute("rec", "id", s.c_str());
        if (!site)
        {
            return 0;
        }

        base_time begdef(FIRST_TIME);
        base_time enddef(LAST_TIME);
        base_time beg(begdef), end(enddef);
        Triple xyz = _get_crd_xyz(s);
        Triple blh = _get_crd_blh(s);
        Triple eccneu = _get_ecc_neu(s);

#ifdef DEBUG
        Triple XYZ;
        ell2xyz(blh.crd_array(), XYZ.crd_array(), true);
        Triple ELL;
        xyz2ell(XYZ.crd_array(), ELL.crd_array(), true);

        cerr << "REC: " << std::fixed << setprecision(3)
            << setw(14) << xyz[0]
            << setw(14) << xyz[1]
            << setw(14) << xyz[2] << setprecision(5)
            << setw(14) << blh[0]
            << setw(14) << blh[1]
            << setw(14) << blh[2]
            << endl;
        cerr << "REC: " << std::fixed << setprecision(3)
            << setw(14) << XYZ[0]
            << setw(14) << XYZ[1]
            << setw(14) << XYZ[2] << setprecision(5)
            << setw(14) << ELL[0]
            << setw(14) << ELL[1]
            << setw(14) << ELL[2]
            << endl;
#endif

        if (double_eq(xyz[0], 0.0) &&
            double_eq(xyz[1], 0.0) &&
            double_eq(xyz[2], 0.0))
        {
            if (double_eq(blh[2], HSL_UNKNOWN))
            {
                SPDLOG_LOGGER_WARN(spdlog, "warning - invalid coordinates for rec: ");
            }
            else
            {
                ell2xyz(blh, xyz, true);
            }
        }

        // find first node for rec/site s
        std::string str_beg_gen = _doc.child(XMLKEY_ROOT).child("gen").child_value("beg");
        base_type_conv::substitute(str_beg_gen, "\"", "");
        base_time gen_beg(base_time::GPS);
        if (!str_beg_gen.empty())
            gen_beg.from_str("%Y-%m-%d %H:%M:%S", base_type_conv::trim(str_beg_gen));
        else
            gen_beg = FIRST_TIME;

        std::string str_end_gen = _doc.child(XMLKEY_ROOT).child("gen").child_value("end");
        base_type_conv::substitute(str_end_gen, "\"", "");
        base_time gen_end(base_time::GPS);
        if (!str_end_gen.empty())
            gen_end.from_str("%Y-%m-%d %H:%M:%S", base_type_conv::trim(str_end_gen));
        else
            gen_end = FIRST_TIME;

        tmp->spdlog(spdlog);
        tmp->overwrite(site.attribute("overwrite").as_bool());
        tmp->id(site.attribute("id").value());
        //tmp->desc(site.attribute("desc").value());
        //tmp->name(site.attribute("name").value());
        //tmp->domes(site.attribute("domes").value());

        Triple std(10, 10, 10);    // glfeng change from 999->10, same sigma with Rinex header aprox coordinates
        tmp->crd(xyz, std, beg, end); // FULL TIME

        tmp->eccneu(eccneu, beg, end);                     // FULL TIME
        tmp->rec(site.attribute("rec").value(), beg, end); // FULL TIME
        tmp->ant(site.attribute("ant").value(), beg, end); // FULL TIME

#ifdef DEBUG
        std::cout << site << " " << gen_beg.str_ymdhms() << " "
            << gen_end.str_ymdhms() << " "
            " "
            << beg.str_ymdhms() << " "
            << end.str_ymdhms() << " "
            << std::fixed << setprecision(3)
            << xyz[0] << " " << xyz[1] << endl;
#endif

        std::string x = site.attribute("X").value();
        std::string y = site.attribute("Y").value();
        std::string z = site.attribute("Z").value();
        if (!x.empty() && !y.empty() && !z.empty())
        {
            xyz[0] = base_type_conv::str2dbl(x);
            xyz[1] = base_type_conv::str2dbl(y);
            xyz[2] = base_type_conv::str2dbl(z);
            Triple std(10, 10, 10); // glfeng change from 999->10, same sigma with Rinex header aprox coordinates
            tmp->crd(xyz, std, beg, end);
        }

        std::string dx = site.attribute("DX").value();
        std::string dy = site.attribute("DY").value();
        std::string dz = site.attribute("DZ").value();
        if (!dx.empty() && !dy.empty() && !dz.empty())
        {
            xyz[0] = base_type_conv::str2dbl(dx);
            xyz[1] = base_type_conv::str2dbl(dy);
            xyz[2] = base_type_conv::str2dbl(dz);
            tmp->eccxyz(xyz, beg, end);
        }

        std::string dn = site.attribute("DN").value();
        std::string de = site.attribute("DE").value();
        std::string du = site.attribute("DU").value();
        if (!dx.empty() && !dy.empty() && !dz.empty())
        {
            xyz[0] = base_type_conv::str2dbl(dn);
            xyz[1] = base_type_conv::str2dbl(de);
            xyz[2] = base_type_conv::str2dbl(du);
            tmp->eccneu(xyz, beg, end);
        }

#ifdef DEBUG
        base_time tt(base_time::GPS); // today time
        tt.from_ymdhms(2010, 11, 2, 0, 0, 0.0);
        std::cout << "REC: " << tmp->id() << setw(12) << "[" + tmp->name() + "]"
            << " tim=" << tt.str("%Y-%m-%d %H:%M:%S[%T]")
            << " rec=" << tmp->rec(tt)
            << " ant=" << tmp->ant(tt)
            << " crd=" << tmp->crd(tt)[0]
            << endl;
#endif
        return tmp;
    }

    hwa_gnss::gnss_data_obj* set_rec::grec_all(std::string name, base_log spdlog)
    {
        hwa_gnss::gnss_data_obj* obj = nullptr;
        std::string sites;
        set<std::string> tmp = all_rec();

        if (tmp.find(name) == tmp.end())
        {
            return obj;
        }
        std::string sites_leo = _doc.child(XMLKEY_ROOT).child(XMLKEY_GEN).find_child_by_attribute("type", "leo").child_value();
        transform(sites_leo.begin(), sites_leo.end(), sites_leo.begin(), ::toupper);
        if (sites_leo.find(name) != std::string::npos)
        {
            obj = new gnss_data_rec_leo(spdlog);
        }
        else
        {
            obj = new gnss_data_rec(spdlog);
        }
        obj->id(name);
        return obj;
    }

    Triple set_rec::_get_crd_blh(std::string s)
    {
        hwa_gnss::gnss_model_gpt _ggpt;
        Triple zero(0.0, 0.0, HSL_UNKNOWN);
        xml_node site = _doc.child(XMLKEY_ROOT).child(XMLKEY_REC).find_child_by_attribute("rec", "id", s.c_str());
        xml_attribute attr;

        if ((attr = site.attribute("LAT")) &&
            (attr = site.attribute("LON")) &&
            (attr = site.attribute("HSL")))
        {
            Triple blh(site.attribute("LAT").as_double(),
                site.attribute("LON").as_double(),
                site.attribute("HSL").as_double());
            double pres, temp, undu = 0.0;
            _ggpt.gpt_v1(51544.0, blh[0] * D2R, blh[1] * D2R, blh[2], pres, temp, undu); // RAD
            blh[2] += undu;                                                              // ADD UNDULATION BECAUSE ONLY HSL (ABOVE MEAN SEE LEVEL) SUPPORTED
            return blh;
        }

        Triple xyz = _get_crd_xyz(s);

        if (!double_eq(xyz[0], 0.0) &&
            !double_eq(xyz[1], 0.0) &&
            !double_eq(xyz[2], 0.0))
        {
            Triple blh;
            if (xyz2ell(xyz, blh, true) == 1)
                return blh;
            else
                return zero;
        }

        return zero;
    }

} // namespace
