#include "hwa_set_npp.h"
#include "hwa_set_gbase.h"
#include <omp.h>

using namespace std;
using namespace pugi;

namespace hwa_set
{
    // Constructor
    // ----------
    set_npp::set_npp()
        : set_base()
    {
        _set.insert(XMLKEY_NPP);

        //_nppmodel = NPP_MODEL::NO;
    }

    // Destructor
    // ----------
    set_npp::~set_npp()
    {
    }

    // settings check
    // ----------
    void set_npp::check()
    {
        // check existence of nodes/attributes
        // xml_node parent = _doc.child(XMLKEY_ROOT);
        // xml_node node = _default_node(parent, XMLKEY_NPP);
        // _doc.child(XMLKEY_ROOT);
        // _default_node(parent, XMLKEY_NPP);
        return;
    }

    // help body
    // ----------
    void set_npp::help()
    {
        cerr << " <npp \n"
             //<< "   nppmodel=\"" << _nppmodel << "\" \n"
             << " />\n";

        cerr << "\t<!-- npp description:\n"
             << "\t nppmodel  .. npp model \n"
             << "\t -->\n\n";
        return;
    }

    bool set_npp::isClient()
    {
        set<string> src = set_base::_setvals(XMLKEY_GNSS, XMLKEY_NPP, "client");
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

    //NPP_MODEL set_npp::npp_model()
    //{
    //    //get rkf model node
    //    string npp_model = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_NPP).child_value("npp_model");
    //    if (npp_model.empty()) return NPP_MODEL::NO;

    //    // delete spaces
    //    npp_model.erase(remove(npp_model.begin(), npp_model.end(), ' '), npp_model.end());
    //    //npp_model = str2upper(npp_model);
    //    transform(npp_model.begin(), npp_model.end(), npp_model.begin(), ::toupper);

    //    if (npp_model == "NO")   return NPP_MODEL::NO;
    //    if (npp_model == "VRS")  return NPP_MODEL::VRS;
    //    if (npp_model == "UPD")  return NPP_MODEL::UPD;
    //    if (npp_model == "URTK")   return NPP_MODEL::URTK;
    //    if (npp_model == "PPP_RTK")    return NPP_MODEL::PPP_RTK;

    //    throw std::logic_error("ERROR type of iono order, you should use FIRST/SECOND/NONE");
    //}

    bool set_npp::npp_delaunay()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("npp_delaunay");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_npp::comp_aug()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("comp_aug");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return true;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_npp::grid_aug()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("grid_aug");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return true;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    void set_npp::aug_limit(double &comp_aug, double &ion_aug, double &trop_aug)
    {
        xml_node tmp_set;
        tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_NPP).child("aug_limit");
        string tmp = tmp_set.attribute("comp").value();
        comp_aug = tmp.empty() ? 0.0 : base_type_conv::str2dbl(tmp);
        tmp = tmp_set.attribute("ion").value();
        ion_aug = tmp.empty() ? 0.0 : base_type_conv::str2dbl(tmp);
        tmp = tmp_set.attribute("trop").value();
        trop_aug = tmp.empty() ? 0.0 : base_type_conv::str2dbl(tmp);
        return;
    }

    bool set_npp::reset_amb_ppprtk()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("reset_amb_ppprtk");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_npp::self_cor()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("self_cor");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return false;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    int set_npp::obs_level()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("obs_level");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return 3;
        int tmp_int = std::stoi(tmp); //default false
        return tmp_int;
    }

    bool set_npp::cor_obs()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_AMBIGUITY).child_value("self_cor");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return true;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    NPP_MODEL set_npp::npp_model()
    {
        //get rkf model node
        string npp_model = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_NPP).child_value("npp_model");
        if (npp_model.empty())
            return NPP_MODEL::NO; //default false

          // delete spaces
          //npp_model.erase(remove(npp_model.begin(), npp_model.end(), ' '), npp_model.end());
        str_erase(npp_model);
        //npp_model = str2upper(npp_model);
        transform(npp_model.begin(), npp_model.end(), npp_model.begin(), ::toupper);

        if (npp_model == "NO")
            return NPP_MODEL::NO;
        if (npp_model == "VRS")
            return NPP_MODEL::VRS;
        if (npp_model == "UPD")
            return NPP_MODEL::UPD;
        if (npp_model == "URTK")
            return NPP_MODEL::URTK;
        if (npp_model == "PPP_RTK")
            return NPP_MODEL::PPP_RTK;

        throw std::logic_error("ERROR type of iono order, you should use FIRST/SECOND/NONE");
    }
} // namespace
