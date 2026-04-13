#include "hwa_set_gflt.h"
#include "hwa_set_gbase.h"

using namespace pugi;

namespace hwa_set
{
    set_flt::set_flt()
        : set_base()
    {
        _set.insert(XMLKEY_FLT);
        _noise_clk = 1000.0;
        _noise_crd = 100.0;
        _noise_vel = 100.0;
        _noise_dclk = 1000.0;
        _rndwk_gps = 20.0;
        _rndwk_glo = 20.0;
        _rndwk_gal = 20.0;
        _rndwk_bds = 20.0;
        _rndwk_qzs = 20.0;
        _rndwk_ztd = 3.0;
        _rndwk_vion = 1000;
        _noise_vion = 0.1;
        _rndwk_grd = 0.3;
        _rndwk_amb = 0.1;
        _method_flt = "kalman";
        _method_smt = "RTSSVD";
        _reset_amb = 0;
        _reset_par = 0;
        _smt_delay = 0;
        _smooth = false;
        _e0 = 0.85;
        _g0 = 0.85;
        _max_res_norm = 3.0;
        _barrior = 9;
        _kappa_sig = 0;
        _alpha_sig = 0.001;
        _dof1 = 1;
        _dof2 = 15;
        _tau = 1000;
        _proc_noise = 0.05;
        _max_iter = 20;
        _num_particles = 500;
    }

    // Destructor
    // ----------
    set_flt::~set_flt()
    {
    }

    // Return value
    // ----------
    std::string set_flt::method_flt()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("method_flt");
        str_erase(tmp);
        if (tmp.empty()) tmp = "kalman";
        return tmp;
    }

    // Return value
    // ----------
    std::string set_flt::method_smt()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("method_smt");
        str_erase(tmp);
        if (tmp.empty()) tmp = _method_smt;
        return tmp;
    }

    double set_flt::noise_clk()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("noise_clk");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _noise_clk; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::noise_crd()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("noise_crd");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _noise_crd; //default value
        return tmp_double;
    }

    double set_flt::noise_dclk()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("noise_dclk");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _noise_dclk; //default value
        return tmp_double;
    }

    double set_flt::noise_vel()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("noise_vel");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _noise_vel; //default value
        return tmp_double;
    }
    // Return value
    // ----------
    double set_flt::rndwk_gps()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_gps");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_gps; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::rndwk_glo()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_glo");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_glo; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::rndwk_gal()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_gal");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_gal; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::rndwk_bds()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_bds");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_bds; //default value
        return tmp_double;
    }

    double set_flt::rndwk_amb()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_amb");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_amb; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::rndwk_qzs()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_qzs");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_qzs; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::rndwk_ztd()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_ztd");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_ztd; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::noise_vion()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("noise_vion");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _noise_vion; //default value
        return tmp_double;
    }

    double set_flt::rndwk_vion()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_vion");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_vion; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_flt::rndwk_grd()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("rndwk_grd");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _rndwk_grd; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    int set_flt::reset_amb()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("reset_amb");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _reset_amb; //default value
        return tmp_int;
    }

    // Return value
    // ----------
    int set_flt::reset_par()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("reset_par");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _reset_par; //default value
        return tmp_int;
    }

    int set_flt::reset_par(double d)
    {
        xml_node flt = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT);
        flt.remove_attribute("reset_par");
        xml_attribute attri = flt.append_attribute("reset_par");
        attri.set_value(std::to_string(d).c_str());
        return 0;
    }

    // Return value
    // ----------
    int set_flt::smt_delay()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("smt_delay");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _smt_delay; //default value
        return tmp_int;
    }

    // Return value
    // ----------
    bool set_flt::smooth()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("smooth");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty()) return _smooth;
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    double set_flt::barrior()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("barrior");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _barrior; //default value
        return tmp_double;
    }

    double set_flt::kappa_sig()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("kappa_sig");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _kappa_sig; //default value
        return tmp_double;
    }

    double set_flt::alpha_sig()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("alpha_sig");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _alpha_sig; //default value
        return tmp_double;
    }

    double set_flt::E0()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("e0");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _e0; //default value
        return tmp_double;
    }

    double set_flt::G0()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("g0");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _g0; //default value
        return tmp_double;
    }

    int set_flt::dof1()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("dof1");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _dof1; //default value
        return tmp_int;
    }

    int set_flt::dof2()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("dof2");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _dof2; //default value
        return tmp_int;
    }

    int set_flt::max_iter()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("max_iter");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _max_iter; //default value
        return tmp_int;
    }

    int set_flt::num_particles()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("num_particles");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _num_particles; //default value
        return tmp_int;
    }

    double set_flt::Tau()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("tau");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _tau; //default value
        return tmp_double;
    }

    double set_flt::proc_noise()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("proc_noise");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _proc_noise; //default value
        return tmp_double;
    }

    double set_flt::max_res_norm()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("max_res_norm");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _max_res_norm; //default value
        return tmp_double;
    }

    std::string set_flt::filter()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_FLT).child_value("filter");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty())tmp = "EKF";
        return tmp;
    }

    // settings check
    // ----------
    void set_flt::check()
    {
        // check existence of nodes/attributes
        xml_node parent = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS);
        xml_node node = _default_node(parent, "filter");

        // check existence of attributes
        _default_attr(node, "method_flt", _method_flt);
        _default_attr(node, "method_smt", _method_smt);
        _default_attr(node, "noise_clk", _noise_clk);
        _default_attr(node, "noise_dclk", _noise_dclk);
        _default_attr(node, "noise_crd", _noise_crd);
        _default_attr(node, "noise_vel", _noise_vel);
        _default_attr(node, "rndwk_glo", _rndwk_glo);
        _default_attr(node, "rndwk_gal", _rndwk_gal);
        _default_attr(node, "rndwk_bds", _rndwk_bds);
        _default_attr(node, "rndwk_qzs", _rndwk_qzs);
        _default_attr(node, "rndwk_ztd", _rndwk_ztd);
        _default_attr(node, "noise_vion", _noise_vion);
        _default_attr(node, "rndwk_grd", _rndwk_grd);
        _default_attr(node, "rndwk_vion", _rndwk_vion);
        _default_attr(node, "reset_amb", _reset_amb);
        _default_attr(node, "reset_par", _reset_par);
        _default_attr(node, "smt_delay", _smt_delay);
        _default_attr(node, "smooth", _smooth);

        _default_attr(node, "kappa_sig", 0);
        _default_attr(node, "alpha_sig", 0.001);
        _default_attr(node, "tau", 1000);
        _default_attr(node, "proc_noise", 0.05);
        _default_attr(node, "g0", 0.85);
        _default_attr(node, "e0", 0.85);
        _default_attr(node, "max_iter", 20);
        _default_attr(node, "_num_particles", 500);
        _default_attr(node, "barrior", 9);
        _default_attr(node, "dof1", 1);
        _default_attr(node, "dof2", 15);
        _default_attr(node, "update", "EKF");
        _default_attr(node, "max_res_norm", 3);

        return;
    }

    // help body
    // ----------
    void set_flt::help()
    {
        std::cerr << " <filter \n"
             << "   method_flt=\"" << _method_flt << "\" \n"
             << "   method_smt=\"" << _method_smt << "\" \n"
             << "   noise_clk=\"" << _noise_clk << "\" \n"
             << "   noise_crd=\"" << _noise_crd << "\" \n"
             << "   rndwk_ztd=\"" << _rndwk_ztd << "\" \n"
             << "   noise_vion=\"" << _noise_vion << "\" \n"
             << "   rndwk_grd=\"" << _rndwk_grd << "\" \n"
             << "   smt_delay=\"" << _smt_delay << "\" \n"
             << "   smooth=\"" << _smooth << "\" \n"
             << "  />\n";

        std::cerr << "\t<!-- filter description:\n"
             << "\t method_flt    .. type of filtering method (SRCF, Kalman)\n"
             << "\t method_smt    .. type of smoothing method (RTS)\n"
             << "\t noise_clk     .. white noise for clocks \n"
             << "\t noise_crd     .. white noise for coordinates \n"
             << "\t rndwk_ztd     .. random walk process for ZTD [mm/sqrt(hour)] \n"
             << "\t noise_vion    .. white noise process for VION [mm/sqrt(hour)] \n"
             << "\t -->\n\n";
        return;
    }

} // namespace
