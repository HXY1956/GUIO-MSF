#include "hwa_set_gproc.h"
#include "hwa_set_gbase.h"
#include "hwa_base_mutex.h"

using namespace pugi;

#define XMLKEY_PROC_ifcb "ifcb_model"
#define XMLKEY_PROC_ION_ORDER "iono_order"
#define XMLKEY_PROC_IFB "ifb_model"
#define XMLKEY_PROC_READ_OFILE "read_ofile_mode"
#define XMLKEY_PROC_TRIMCOR "base_type_conv::trimcor"
#define XMLKEY_PROC_EQU "write_equ"
#define XMLKEY_PROC_ISFIRSTSOL "isfirstsol"
#define XMLKEY_PROC_ISCONSPW "isconstrpw"
#define XMLKEY_PROC_SAT_PCV "sat_pcv"
#define XMLKEY_PROC_REC_PCV "rec_pcv"

namespace hwa_set
{
    set_gproc::set_gproc()
        : set_base()
    {
        _set.insert(XMLKEY_PROC);
        _phase = true;
        _tropo = true;
        _iono = true;
        _tropo_grad = false;
        _tropo_slant = false;
        _tropo_model = TROPMODEL::SAASTAMOINEN;
        _ztd_model = ZTDMODEL::PWC;
        _tropo_mf = ZTDMPFUNC::GMF;
        _iono_mf = IONMPFUNC::ICOSZ;
        _grad_mf = GRDMPFUNC::TILTING;
        _obs_weight = OBSWEIGHT::SINEL2;
        _res_type = RESIDTYPE::RES_NORM;
        _obs_combin = OBSCOMBIN::IONO_FREE;
        _attitudes = ATTITUDES::DEF_YAWMODEL;
        _cbiaschar = CBIASCHAR::DEF_CBIASCHAR;

        _sig_init_ztd = 0.1;
        _sig_init_vion = 10;
        _sig_init_grd = 0.0005;
        _sig_init_amb = 1000.0;
        _sig_init_crd = 100.0;
        _sig_init_vel = 10.0;
        _sig_init_glo = 1000.0;
        _sig_init_gal = 1000.0;
        _sig_init_bds = 1000.0;
        _sig_init_qzs = 1000.0;
        _minimum_elev = 10;
        _max_res_norm = 10;
        _crd_est = "EST";
        _pos_kin = false;
        _use_eclipsed = true;
        _auto_band = false;
        _frequency = 2;
        _realtime = false;
        _sd_sat = false;
        _simulation = false;
        _dynamics = 0;
        _basepos = base_pos::SPP;
        _minsat = static_cast<size_t>(6); //zzwu
        _sig_init_sat_pcv = 10.0;
        _sig_init_sat_pcox = 10.0;
        _sig_init_sat_pcoy = 10.0;
        _sig_init_sat_pcoz = 10.0;
        _sig_init_rec_pcv = 10.0;
        _sig_init_rec_pcox = 10.0;
        _sig_init_rec_pcoy = 10.0;
        _sig_init_rec_pcoz = 10.0;
        _sat_pco_xyz["pcox"] = false;
        _sat_pco_xyz["pcoy"] = false;
        _sat_pco_xyz["pcoz"] = false;
        _rec_pco_xyz["pcox"] = false;
        _rec_pco_xyz["pcoy"] = false;
        _rec_pco_xyz["pcoz"] = false;
        _sat_azi_beg = 0.0;
        _sat_azi_end = 360.0;
        _sat_dazi = 0.0;
        _sat_zen_beg = 0.0;
        _sat_zen_end = 14.0;
        _sat_dzen = 1.0;
        _rec_azi_beg = 0.0;
        _rec_azi_end = 360.0;
        _rec_dazi = 0.0;
        _rec_zen_beg = 0.0;
        _rec_zen_end = 90.0;
        _rec_dzen = 5.0;

        _nppmodel = NPP_MODEL::NO;
        _meanpolemodel = modeofmeanpole::cubic;
    }

    // Return value
    // ----------
    modeofmeanpole set_gproc::mean_pole_model()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("mean_pole_model");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        modeofmeanpole TM = str2meanpolemodel(tmp);
        return TM;
    }

    modeofmeanpole set_gproc::str2meanpolemodel(const std::string &tm)
    {
        if (tm == "linear" || tm == "LINEAR")
        {
            return modeofmeanpole::linear;
        }
        else if (tm == "cubic" || tm == "CUBIC")
        {
            return modeofmeanpole::cubic;
        }
        else
        {
            return modeofmeanpole::cubic;
        }
    }

    // Destructor
    // ----------
    set_gproc::~set_gproc() = default;

    // Return value
    // ----------
    bool set_gproc::isleo()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("isleo");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false);
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::tropo()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("tropo");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "FALSE" ? false : true); //default true
        return tmp_bool;
    }

    void set_gproc::tropo(bool b)
    {
        xml_node proc = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC);
        proc.remove_child("tropo");
        xml_node tropo = proc.append_child("tropo");
        tropo.append_child(xml_node_type::node_pcdata).set_value(b ? "true" : "false");
    }

    // Return value
    // ----------
    bool set_gproc::iono()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("iono");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = tmp == "FALSE" ? false : true; //default true
        return tmp_bool;
    }

    void set_gproc::iono(bool b)
    {
        xml_node proc = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC);
        proc.remove_child("iono");
        xml_node iono = proc.append_child("iono");
        iono.append_child(xml_node_type::node_pcdata).set_value(b ? "true" : "false");
    }

    // Return value
    // ----------
    bool set_gproc::tropo_slant()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("tropo_slant");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = tmp == "TRUE" ? true : false; //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::tropo_grad()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("tropo_gradient");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::phase()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("phase");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::doppler()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("doppler");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::pos_kin()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("pos_kin");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::use_eclipsed()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("use_eclipsed");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::auto_band()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("auto_band");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_gproc::sd_sat()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sd_sat");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::IRC()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("IRC");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::pre_fix_residuals()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("pre_fix_residuals");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // Return value
    // ----------
    bool set_gproc::carrier_range()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("carrier_range");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_gproc::apply_carrier_range()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("apply_carrier_range");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_gproc::ambfix()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("ambfix");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }
    bool set_gproc::realtime()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("realtime");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }
    bool set_gproc::ultrasp3()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("ultrasp3");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    base_pos set_gproc::basepos()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("basepos");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        base_pos type = str2basepos(tmp); //default SPP
        return type;
    }

    void set_gproc::basepos(std::string str)
    {
        xml_node proc = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC);
        proc.remove_child("basepos");
        xml_node basepose = proc.append_child("basepos");
        basepose.append_child(xml_node_type::node_pcdata).set_value(str.c_str());
    }

    base_pos set_gproc::str2basepos(const std::string &str)
    {
        if (str == "RINEXH")
            return base_pos::RINEXH;
        else if (str == "CFILE")
            return base_pos::CFILE;
        else if (str == "KIN2KIN")
            return base_pos::KIN2KIN;
        else if (str == "EXTERNAL")
            return base_pos::EXTERNAL;
        else
            return base_pos::SPP; //default value
    }

    // Return value
    // ----------
    double set_gproc::sig_init_ztd()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_ztd");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_ztd; //default value
        return tmp_double;
    }
    double set_gproc::sig_init_kbr()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_kbr");
        str_erase(tmp);
        double tmp_double = 0.0; //default value
        if (tmp != "")
            tmp_double = std::stod(tmp);
        return tmp_double;
    }

    double set_gproc::sig_init_lri()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_lri");
        str_erase(tmp);
        double tmp_double = 0.0; //default value
        if (tmp != "")
            tmp_double = std::stod(tmp);
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_vion()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_vion");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_vion; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_grd()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_grd");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_grd; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_crd()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_crd");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_crd; //default value
        return tmp_double;
    }

    double set_gproc::sig_init_vel()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_vel");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_vel; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_amb()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_amb");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_amb; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_glo()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_glo");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_glo; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_gal()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_gal");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_gal; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_bds()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_bds");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_bds; //default value
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::sig_init_qzs()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_init_qzs");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _sig_init_qzs; //default value
        return tmp_double;
    }

    double set_gproc::sig_ref_clk()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sig_ref_clk");
        str_erase(tmp);
        double tmp_double = 0.0; //default value
        if (tmp != "")
            tmp_double = std::stod(tmp);
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::minimum_elev()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("minimum_elev");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _minimum_elev; //default value
        return tmp_double;
    }

    // Return value for LEO
    // ----------
    double set_gproc::minimum_elev_leo()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("minimum_elev_leo");
        str_erase(tmp);
        double tmp_double = 0.0; //default value
        if (tmp != "")
            tmp_double = std::stod(tmp);
        return tmp_double;
    }

    // Return value
    // ----------
    double set_gproc::max_res_norm()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("max_res_norm");
        str_erase(tmp);
        double tmp_double;
        if (tmp != "")
            tmp_double = std::stod(tmp);
        else
            tmp_double = _max_res_norm; //default value
        return tmp_double;
    }

    double set_gproc::max_att_res_norm()
    {
        std::string str = _doc.child(XMLKEY_ROOT).child("integration").child("Attitude").child("max_res_norm").attribute("Value").value();
        double max_res_norm = hwa_base::base_type_conv::str2dbl(str);
        return max_res_norm;
    }

    int set_gproc::minsat()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("min_sat");
        str_erase(tmp);
        int tmp_int;
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        else
            tmp_int = _minsat; //default value
        return tmp_int;
    }

    std::string set_gproc::ref_clk()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("ref_clk");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        return tmp;
    }

    LSQ_MODE set_gproc::lsq_mode()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("lsq_mode");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "lsq")
            return LSQ_MODE::lsq;
        else
            return LSQ_MODE::EPO; //default value
    }

    int set_gproc::lsq_buffer_size()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("lsq_buffer_size");
        str_erase(tmp);
        int tmp_int = 10; //default value
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        return tmp_int;
    }
    KBRLRIMODE set_gproc::kbr_mode()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("kbr_mode");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "RANGE_RATE")
        {
            return KBRLRIMODE::RANGE_RATE;
        }
        else if (tmp == "RATE")
        {
            return KBRLRIMODE::RATE;
        }
        else
        {
            return KBRLRIMODE::RANGE; //default value
        }
    }

    KBRLRIMODE set_gproc::lri_mode()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("lri_mode");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "RANGE_RATE")
        {
            return KBRLRIMODE::RANGE_RATE;
        }
        else if (tmp == "RATE")
        {
            return KBRLRIMODE::RATE;
        }
        else
        {
            return KBRLRIMODE::RANGE; //default value
        }
    }

    SLIPMODEL set_gproc::slip_model()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("slip_model");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "TURBOEDIT")
            return SLIPMODEL::TURBO_EDIT;
        else
            return SLIPMODEL::DEF_DETECT_MODEL; //default value
    }

    IONMODEL set_gproc::ion_model()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("ion_model");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "SION")
            return IONMODEL::SION;
        else if (tmp == "VION")
            return IONMODEL::VION;
        else
            return IONMODEL::DEF_ION; //default value
    }

    // Return value
    // ----------
    TROPMODEL set_gproc::tropo_model()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("tropo_model");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        TROPMODEL TM = str2tropmodel(tmp);
        if (TM == TROPMODEL::DEF_TROPMODEL)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = tropmodel2str(_tropo_model);
            //_default_attr(node, "tropo_model", tmp);
            _default_node(node, "tropo_model", tmp.c_str());
            TM = _tropo_model; //default value
        }
        return TM;
    }

    ZTDMODEL set_gproc::ztd_model(double *dt)
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("ztd_model");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        *dt = 0.0;
        ZTDMODEL TM = str2ztdmodel(tmp);

        if (TM == ZTDMODEL::PWC)
        {
            int loc = tmp.find(':');
            *dt = hwa_base::base_type_conv::str2dbl(tmp.substr(loc + 1));
        }
        else if (TM == ZTDMODEL::DEF_ZTDMODEL)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = ztdmodel2str(_ztd_model);
            //_default_attr(node, "tropo_model", tmp);
            _default_node(node, "tropo_model", tmp.c_str());
            TM = _ztd_model; //default value
            *dt = 60;
        }
        return TM;
    }

    // Return value
    // ----------
    std::string set_gproc::trop_corr()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("trop_corr");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        return tmp;
    }

    // Return value
    // ----------
    ATTITUDES set_gproc::attitudes()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("attitudes");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        return str2attitudes(tmp); //default DEF_YAWMODEL
    }

    // Return value
    // ----------
    CBIASCHAR set_gproc::cbiaschar()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("cbiaschar");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        return str2cbiaschar(tmp); //default DEF_CBIASCHAR
    }

    // Return value
    // ----------
    SYSBIASMODEL set_gproc::sysbias_model()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sysbias_model");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        return str2sysbiasmodel(hwa_base::base_type_conv::trim(tmp)); //default AUTO_WHIT
    }

    // Return value
    // ----------
    SYSBIASMODEL set_gproc::ifbbias_model()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("ifbbias_model");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        return str2sysbiasmodel(hwa_base::base_type_conv::trim(tmp)); //default AUTO_WHIT
    }

    // Return value
    // ----------
    GRDMPFUNC set_gproc::grad_mf()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("grad_mf");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        GRDMPFUNC MF = str2grdmpfunc(tmp);
        if (MF == GRDMPFUNC::DEF_GRDMPFUNC)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = grdmpfunc2str(_grad_mf);
            //_default_attr(node, "grad_mf", tmp);
            _default_node(node, "grad_mf", tmp.c_str());

            MF = _grad_mf; //default TILTING
        }
        return MF;
    }

    // Return value
    // ----------
    OBSWEIGHT set_gproc::weighting()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("obs_weight");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        OBSWEIGHT WG = str2obsweight(tmp);
        if (WG == OBSWEIGHT::DEF_OBS_WEIGHT)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = obsweight2str(_obs_weight);
            //_default_attr(node, "obs_weight", tmp);
            _default_node(node, "obs_weight", tmp.c_str());
            WG = _obs_weight; //default SINEL2
        }
        return WG;
    }

    // Return value
    // ----------
    RESIDTYPE set_gproc::residuals()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("residuals");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        RESIDTYPE RS = str2residtype(tmp);
        if (RS == RESIDTYPE::DEF_RESIDTYPE)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = residtype2str(_res_type);
            //_default_attr(node, "residuals", tmp);
            _default_node(node, "residuals", tmp.c_str());
            RS = _res_type; //default RES_NORM
        }
        return RS;
    }

    // Return value
    // ----------
    OBSCOMBIN set_gproc::obs_combin()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("obs_combination");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        OBSCOMBIN OC = str2obscombin(tmp);
        if (OC == OBSCOMBIN::DEF_OBSCOMBIN)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = obscombin2str(_obs_combin);
            //_default_attr(node, "obs_combination", tmp);
            _default_node(node, "obs_combination", tmp.c_str());
            OC = _obs_combin; //default IONO_FREE
        }
        return OC;
    }

    void set_gproc::obs_combin(std::string str)
    {
        xml_node proc = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC);
        proc.remove_child("obs_combination");
        xml_node obs_combination = proc.append_child("obs_combination");
        obs_combination.append_child(xml_node_type::node_pcdata).set_value(str.c_str());
    }

    // Return value
    // ----------
    ZTDMPFUNC set_gproc::tropo_mf()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("tropo_mf");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        ZTDMPFUNC MF = str2ztdmpfunc(tmp);
        if (MF == ZTDMPFUNC::DEF_ZTDMPFUNC)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = ztdmpfunc2str(_tropo_mf);
            //_default_attr(node, "tropo_mf", tmp);
            _default_node(node, "tropo_mf", tmp.c_str());
            MF = _tropo_mf; //default GMF
        }

        return MF;
    }

    // Return value
    // ----------
    IONMPFUNC set_gproc::iono_mf()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("iono_mf");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        IONMPFUNC MF = str2ionmpfunc(tmp);
        if (MF == IONMPFUNC::DEF_IONMPFUNC)
        {
            xml_node parent = _doc.child(XMLKEY_ROOT);
            xml_node node = _default_node(parent, XMLKEY_PROC);
            tmp = ionmpfunc2str(_iono_mf);
            //_default_attr(node, "iono_mf", tmp);
            _default_node(node, "iono_mf", tmp.c_str());
            MF = _iono_mf; //default ICOSZ
        }

        return MF;
    }

    // Return value
    // ----------
    CONSTRPAR set_gproc::crd_est()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("crd_constr");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp.empty())
            tmp = _crd_est; // default EST

        if (tmp == "EST")
            return CONSTRPAR::EST;
        else if (tmp == "KIN")
            return CONSTRPAR::KIN;
        else if (tmp == "SIMU_KIN")
            return CONSTRPAR::SIMU_KIN;
        else
            return CONSTRPAR::FIX;
    }

    //HwangShih addded
    HMTCONSTR set_gproc::hmt_cst()
    {
        HMTCONSTR constr = HMTCONSTR_UNDEF;
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("helmert_constr");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        if (tmp.empty())
            tmp = HMTCONSTR_UNDEF; // default value

        if (tmp.compare("NNT") == 0)
            constr = NNT;
        else if (tmp.compare("NNR") == 0)
            constr = NNR;
        else if (tmp.compare("NNTNNR") == 0)
            constr = NNTNNR;
        else
            constr = HMTCONSTR_UNDEF;

        return constr;
    }

    bool set_gproc::geoc_est()
    {
        bool isEstGeoc = false;
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("geoc_estimate");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        if (tmp.empty())
            tmp = "NO"; // default value

        if (tmp.compare("NO") == 0)
            isEstGeoc = false;
        else if (tmp.compare("YES") == 0)
            isEstGeoc = true;
        else
            isEstGeoc = false;

        return isEstGeoc;
    }

    // settings check
    // ----------
    void set_gproc::check()
    {
        // check existence of nodes/attributes
        xml_node parent = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS);
        xml_node node = _default_node(parent, XMLKEY_PROC);

        // check existence of attributes
        //_default_attr(node, "phase", _phase);
        //_default_attr(node, "tropo", _tropo);
        //_default_attr(node, "iono", _iono);
        //_default_attr(node, "tropo_grad", _tropo_grad);
        //_default_attr(node, "tropo_model", tropmodel2str(_tropo_model));
        //_default_attr(node, "tropo_slant", _tropo_slant);
        //_default_attr(node, "sig_init_crd", _sig_init_crd);
        //_default_attr(node, "sig_init_vel", _sig_init_vel);
        //_default_attr(node, "sig_init_ztd", _sig_init_ztd);
        //_default_attr(node, "sig_init_grd", _sig_init_grd);
        //_default_attr(node, "sig_init_vion", _sig_init_vion);
        //_default_attr(node, "sig_init_amb", _sig_init_amb);
        //_default_attr(node, "sig_init_glo", _sig_init_glo);
        //_default_attr(node, "sig_init_gal", _sig_init_gal);
        //_default_attr(node, "sig_init_bds", _sig_init_bds);
        //_default_attr(node, "sig_init_qzs", _sig_init_qzs);
        //_default_attr(node, "minimum_elev", _minimum_elev);
        //_default_attr(node, "max_res_norm", _max_res_norm);
        //_default_attr(node, "crd_constr", _crd_est);
        //_default_attr(node, "pos_kin", _pos_kin);
        //_default_attr(node, "sd_sat", _sd_sat);
        //_default_attr(node, "min_sat", _minsat);
        //_default_attr(node, "use_eclipsed", _use_eclipsed);
        //_default_attr(node, "auto_band", _auto_band);
        //_default_attr(node, "realtime", _realtime);
        //_default_attr(node, "simulation", _simulation);
        //rtk process
        //_default_attr(node, "basepos", basepos2str(_basepos));
        //_default_attr(node, "nppmodel", _nppmodel);

        _default_node(node, "phase", _phase == true ? "true" : "false");
        _default_node(node, "tropo", _tropo == true ? "true" : "false");
        _default_node(node, "iono", _iono == true ? "true" : "false");
        _default_node(node, "gradient", _tropo_grad == true ? "true" : "false");
        _default_node(node, "tropo_model", (tropmodel2str(_tropo_model)).c_str());
        _default_node(node, "tropo_slant", _tropo_slant == true ? "true" : "false");
        _default_node(node, "sig_init_crd", (std::to_string(_sig_init_crd)).c_str());
        _default_node(node, "sig_init_vel", (std::to_string(_sig_init_vel)).c_str());
        _default_node(node, "sig_init_ztd", (std::to_string(_sig_init_ztd)).c_str());
        _default_node(node, "sig_init_grd", (std::to_string(_sig_init_grd)).c_str());
        _default_node(node, "sig_init_vion", (std::to_string(_sig_init_vion)).c_str());
        _default_node(node, "sig_init_amb", (std::to_string(_sig_init_amb)).c_str());
        _default_node(node, "sig_init_glo", (std::to_string(_sig_init_glo)).c_str());
        _default_node(node, "sig_init_gal", (std::to_string(_sig_init_gal)).c_str());
        _default_node(node, "sig_init_bds", (std::to_string(_sig_init_bds)).c_str());
        _default_node(node, "sig_init_qzs", (std::to_string(_sig_init_qzs)).c_str());
        _default_node(node, "minimum_elev", (std::to_string(_minimum_elev)).c_str());
        _default_node(node, "max_res_norm", (std::to_string(_max_res_norm)).c_str());
        _default_node(node, "crd_constr", _crd_est.c_str());
        _default_node(node, "pos_kin", _pos_kin == true ? "true" : "false");
        _default_node(node, "sd_sat", _sd_sat == true ? "true" : "false");
        _default_node(node, "min_sat", (std::to_string(_minsat)).c_str());
        _default_node(node, "use_eclipsed", _use_eclipsed == true ? "true" : "false");
        _default_node(node, "auto_band", _auto_band == true ? "true" : "false");
        _default_node(node, "realtime", _realtime == true ? "true" : "false");
        _default_node(node, "simulation", _simulation == true ? "true" : "false");
        _default_node(node, "basepos", basepos2str(_basepos).c_str());
    }

    // help body
    // ----------
    void set_gproc::help()
    {
        std::cerr << " <process \n"
             << "   phase=\"" << _phase << "\" \n"
             << "   tropo=\"" << _tropo << "\" \n"
             << "   iono=\"" << _iono << "\" \n"
             << "   tropo_grad=\"" << _tropo_grad << "\" \n"
             << "   tropo_model=\"" << tropmodel2str(_tropo_model) << "\" \n"
             << "   tropo_slant=\"" << _tropo_slant << "\" \n"
             << "   tropo_mf=\"" << ztdmpfunc2str(_tropo_mf) << "\" \n"
             << "   iono_mf=\"" << ionmpfunc2str(_iono_mf) << "\" \n"
             << "   grad_mf=\"" << grdmpfunc2str(_grad_mf) << "\" \n"
             << "   obs_weight=\"" << obsweight2str(_obs_weight) << "\" \n"
             << "   residuals=\"" << residtype2str(_res_type) << "\" \n"
             << "   obs_combination=\"" << obscombin2str(_obs_combin) << "\" \n"
             << "   sig_init_crd=\"" << _sig_init_crd << "\" \n"
             << "   sig_init_ztd=\"" << _sig_init_ztd << "\" \n"
             << "   sig_init_vion=\"" << _sig_init_vion << "\" \n"
             << "   minimum_elev=\"" << _minimum_elev << "\" \n"
             << "   max_res_norm=\"" << _max_res_norm << "\" \n"
             << "   basepos=\"" << basepos2str(_basepos) << "\" \n"
             //<< "   nppmodel=\"" << _nppmodel << "\" \n"
             << " />\n";

        std::cerr << "\t<!-- process description:\n"
             << "\t phase [0,1]   .. use carrier phase data\n"
             << "\t tropo [0,1]   .. estimate troposphere\n"
             << "\t tropo_grad    .. tropospheric horizontal gradient models\n"
             << "\t tropo_model   .. tropospheric model (SAAS, GPT, ...)\n"
             << "\t tropo_slant   .. tropospheric slant delays produced\n"
             << "\t tropo_mf      .. tropospheric mapPing function (COSZ, GMF, ...)\n"
             << "\t grad_mf       .. tropo gradient mapPing function (TILTING,CHEN_HERRING, BAR_SEVER ...)\n"
             << "\t obs_weight    .. observation elevation dependant weighting (EQUAL, SINEL, SINEL2, SINEL4, CODPHA, MLTPTH)\n"
             << "\t residuals     .. type of residuals (RES_ORIG, RES_NORM, RES_ALL)\n"
             << "\t sig_init_crd  .. accuracy of initial coordinates [m]\n"
             << "\t sig_init_ztd  .. accuracy of initial zenith path delay [m]\n"
             << "\t sig_init_vion .. accuracy of initial vertical iono path delay [m]\n"
             << "\t minimum_elev  .. elevation angle cut-off [degree]\n"
             << "\t max_res_norm  .. maximal normalized residuals\n"
             << "\t basepos  .. base site coordinate\n"
             << "\t nppmodel  .. npp model \n"
             << "\t -->\n\n";
    }

    // convert str to ATTITUDES enum
    // ----------
    ATTITUDES set_gproc::str2attitudes(const std::string &ati)
    {
        if (ati == "NOMINAL")
        {
            return ATTITUDES::YAW_NOMI;
        }
        else if (ati == "RTCM")
        {
            return ATTITUDES::YAW_RTCM;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported attitude model (" << attitude2str(_attitudes) << ")! Used default value ( YAW_model )";
            _add_log("gsetproc", ostr.str());
            return ATTITUDES::DEF_YAWMODEL;
        }
    }

    // convert str to GRDMPFUNC enum
    // ----------
    GRDMPFUNC set_gproc::str2grdmpfunc(const std::string &mf)
    {
        if (mf == "TILTING")
        {
            return GRDMPFUNC::TILTING;
        }
        else if (mf == "CHEN_HERRING")
        {
            return GRDMPFUNC::CHEN_HERRING;
        }
        else if (mf == "BAR_SEVER")
        {
            return GRDMPFUNC::BAR_SEVER;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported GRD mapPing function (" << grdmpfunc2str(_grad_mf) << ")! Used default value ("
                 << grdmpfunc2str(_grad_mf) << ")";
            _add_log("gsetproc", ostr.str());
            return GRDMPFUNC::DEF_GRDMPFUNC;
        }
    }

    // convert str to CBIASCHAR enum
    // ----------
    CBIASCHAR set_gproc::str2cbiaschar(const std::string &cb)
    {
        if (cb == "2CHAR")
        {
            return CBIASCHAR::CHAR2;
        }
        else if (cb == "3CHAR")
        {
            return CBIASCHAR::CHAR3;
        }
        else if (cb == "ORIG")
        {
            return CBIASCHAR::ORIG;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported forcing code bias signals! Used default value (" << cbiaschar2str(_cbiaschar) << ")";
            _add_log("gsetproc", ostr.str());
            return CBIASCHAR::DEF_CBIASCHAR;
        }
    }

    // convert str to SYSBIASMODEL enum
    // ----------
    SYSBIASMODEL set_gproc::str2sysbiasmodel(const std::string &sys)
    {
        if (sys == "AUTO+CON")
        {
            return SYSBIASMODEL::AUTO_CON;
        }
        else if (sys == "AUTO+RWK")
        {
            return SYSBIASMODEL::AUTO_RWK;
        }
        else if (sys == "ISB+CON")
        {
            return SYSBIASMODEL::ISB_CON;
        }
        else if (sys == "ISB+RWK")
        {
            return SYSBIASMODEL::ISB_RWK;
        }
        else if (sys == "AUTO+WHIT")
        {
            return SYSBIASMODEL::AUTO_WHIT;
        }
        else if (sys == "ISB+WHIT")
        {
            return SYSBIASMODEL::ISB_WHIT;
        }
        //else if (sys.compare("IFB+CON")  == 0) model = SYSBIASMODEL::IFB_CON;
        //else if (sys.compare("IFB+RWK")  == 0) model = SYSBIASMODEL::IFB_RWK;
        else
        {
            //    model = SYSBIASMODEL::AUTO_CON;
            std::stringstream ostr;
            ostr << "Unsupported forcing code bias signals! Used default value (AUTO+CON)";
            _add_log("gsetproc", ostr.str());
            return SYSBIASMODEL::AUTO_WHIT;
        }
    }

    // convert str to ZTDMPFUNC enum
    // ----------
    ZTDMPFUNC set_gproc::str2ztdmpfunc(const std::string &mf)
    {
        if (mf == "COSZ")
        {
            return ZTDMPFUNC::COSZ;
        }
        else if (mf == "GMF")
        {
            return ZTDMPFUNC::GMF;
        }
        else if (mf == "NO_MF")
        {
            return ZTDMPFUNC::NO_MF;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported ZTD mapPing function (" << mf << ")! Used default value (" << ztdmpfunc2str(_tropo_mf) << ")";
            _add_log("gsetproc", ostr.str());
            return ZTDMPFUNC::DEF_ZTDMPFUNC;
        }
    }

    // convert str to IONMPFUNC enum
    // ----------
    IONMPFUNC set_gproc::str2ionmpfunc(const std::string &mf)
    {
        if (mf == "ICOSZ")
        {
            return IONMPFUNC::ICOSZ;
        }
        else if (mf == "QFAC")
        {
            return IONMPFUNC::QFAC;
        }
        else if (mf == "NONE")
        {
            return IONMPFUNC::NONE;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported ION mapPing function (" << mf << ")! Used default value (" << ionmpfunc2str(_iono_mf) << ")";
            _add_log("gsetproc", ostr.str());
            return IONMPFUNC::DEF_IONMPFUNC;
        }
    }

    // convert str to OBSWEIGHT enum
    // ----------
    OBSWEIGHT set_gproc::str2obsweight(const std::string &wg)
    {
        if (wg == "EQUAL")
        {
            return OBSWEIGHT::EQUAL;
        }
        else if (wg == "SINEL" || wg == "SINEL1")
        {
            return OBSWEIGHT::SINEL;
        }
        else if (wg == "SINEL2")
        {
            return OBSWEIGHT::SINEL2;
        }
        else if (wg == "SINEL4")
        {
            return OBSWEIGHT::SINEL4;
        }
        else if (wg == "CODPHA")
        {
            return OBSWEIGHT::CODPHA;
        }
        else if (wg == "MLTPTH")
        {
            return OBSWEIGHT::MLTPTH;
        }
        else if (wg == "PARTELE")
        {
            return OBSWEIGHT::PARTELE;
        }
        else if (wg == "SNR")
        {
            return OBSWEIGHT::SNR;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported observation weighting model (" << wg << ")! Used default value (" << obsweight2str(_obs_weight)
                 << ")";
            _add_log("gsetproc", ostr.str());
            return OBSWEIGHT::DEF_OBS_WEIGHT;
        }
    }

    // convert str to TROPMODEL enum
    // ----------
    TROPMODEL set_gproc::str2tropmodel(const std::string &tm)
    {
        if (tm == "SAASTAMOINEN")
        {
            return TROPMODEL::SAASTAMOINEN;
        }
        else if (tm == "DAVIS")
        {
            return TROPMODEL::DAVIS;
        }
        else if (tm == "HOPFIELD")
        {
            return TROPMODEL::HOPFIELD;
        }
        else if (tm == "MOPS")
        {
            return TROPMODEL::MOPS;
        }
        else if (tm == "GPTW")
        {
            return TROPMODEL::GPTW;
        }
        else if (tm == "GPT2W")
        {
            return TROPMODEL::GPT2W;
        }
        else if (tm == "GAL27")
        {
            return TROPMODEL::GAL27;
        }
        else if (tm == "GALTROPO27")
        {
            return TROPMODEL::GALTROPO27;
        }
        else if (tm == "EXTERN")
        {
            return TROPMODEL::EXTERN;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported tropospheric model (" << tm << ")! Used default value (" << tropmodel2str(_tropo_model) << ")";
            _add_log("gsetproc", ostr.str());
            return TROPMODEL::DEF_TROPMODEL;
        }
    }

    ZTDMODEL set_gproc::str2ztdmodel(const std::string &ztd)
    {
        if (ztd.substr(0, 3) == "PWC")
            return ZTDMODEL::PWC;
        else if (ztd.substr(0, 3) == "STO")
            return ZTDMODEL::STO;
        else
            return ZTDMODEL::DEF_ZTDMODEL;
    }

    // convert str to RESIDTYPE enum
    // ----------
    RESIDTYPE set_gproc::str2residtype(const std::string &rs)
    {
        if (rs == "RES_ORIG")
        {
            return RESIDTYPE::RES_ORIG;
        }
        else if (rs == "RES_NORM")
        {
            return RESIDTYPE::RES_NORM;
        }
        else if (rs == "RES_ALL")
        {
            return RESIDTYPE::RES_ALL;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported type of residuals (" << rs << ")! Used default value (" << residtype2str(_res_type) << ")";
            _add_log("gsetproc", ostr.str());
            return RESIDTYPE::DEF_RESIDTYPE;
        }
    }

    // convert str to OBSCOMB enum
    // ----------
    OBSCOMBIN set_gproc::str2obscombin(const std::string &oc)
    {
        if (oc == "IONO_FREE")
        {
            return OBSCOMBIN::IONO_FREE;
        }
        else if (oc == "RAW_SINGLE")
        {
            return OBSCOMBIN::RAW_SINGLE;
        }
        else if (oc == "RAW_DOUBLE")
        {
            return OBSCOMBIN::RAW_DOUBLE;
        }
        else if (oc == "RAW_ALL")
        {
            return OBSCOMBIN::RAW_ALL;
        }
        else if (oc == "MW_COMBIN")
        {
            return OBSCOMBIN::MW_COMBIN;
        }
        else if (oc == "WL_COMBIN")
        {
            return OBSCOMBIN::WL_COMBIN;
        }
        else if (oc == "RAW_MIX")
        {
            return OBSCOMBIN::RAW_MIX;
        }
        else if (oc == "IF_P1")
        {
            return OBSCOMBIN::IF_P1;
        }
        else
        {
            std::stringstream ostr;
            ostr << "Unsupported observations combination (" << oc << ")! Used default value (" << obscombin2str(_obs_combin)
                 << ")";
            _add_log("gsetproc", ostr.str());
            return OBSCOMBIN::DEF_OBSCOMBIN;
        }
    }

    // convert GRDMPFUNC enum to str
    // ---------
    std::string set_gproc::grdmpfunc2str(GRDMPFUNC MF)
    {
        switch (MF)
        {
        case GRDMPFUNC::TILTING:
            return "TILTING";
        case GRDMPFUNC::CHEN_HERRING:
            return "CHEN_HERRING";
        case GRDMPFUNC::BAR_SEVER:
            return "BAR_SEVER";
        case GRDMPFUNC::DEF_GRDMPFUNC:
            return "NOT DEFINED";
        default:
            return "";
        }
    }

    // convert ZTDMPFUNC enum to str
    // ---------
    std::string set_gproc::ztdmpfunc2str(ZTDMPFUNC MF)
    {
        switch (MF)
        {
        case ZTDMPFUNC::COSZ:
            return "COSZ";
        case ZTDMPFUNC::GMF:
            return "GMF";
        case ZTDMPFUNC::NO_MF:
            return "NO_MF";
        case ZTDMPFUNC::DEF_ZTDMPFUNC:
            return "NOT DEFINED";
        default:
            return "";
        }
    }

    // convert IONMPFUNC enum to str
    // ---------
    std::string set_gproc::ionmpfunc2str(IONMPFUNC MF)
    {
        switch (MF)
        {
        case IONMPFUNC::ICOSZ:
            return "ICOSZ";
        case IONMPFUNC::QFAC:
            return "QFAC";
        case IONMPFUNC::NONE:
            return "NONE";
        case IONMPFUNC::DEF_IONMPFUNC:
            return "NOT DEFINED";
        default:
            return "";
        }
    }

    // convert OBSWEIGHT enum to str
    // ---------
    std::string set_gproc::obsweight2str(OBSWEIGHT WG)
    {
        switch (WG)
        {
        case OBSWEIGHT::EQUAL:
            return "EQUAL";
        case OBSWEIGHT::SINEL:
            return "SINEL";
        case OBSWEIGHT::SINEL2:
            return "SINEL2";
        case OBSWEIGHT::SINEL4:
            return "SINEL4";
        case OBSWEIGHT::CODPHA:
            return "CODPHA";
        case OBSWEIGHT::MLTPTH:
            return "MLTPTH";
        case OBSWEIGHT::PARTELE:
            return "PARTELE";
        case OBSWEIGHT::SNR:
            return "SNR";
        case OBSWEIGHT::DEF_OBS_WEIGHT:
            return "NOT DEFINED";
        default:
            return "";
        }
    }

    // convert TROPMODEL enum to str
    // ---------
    std::string set_gproc::tropmodel2str(TROPMODEL TM)
    {
        switch (TM)
        {
        case TROPMODEL::SAASTAMOINEN:
            return "SAASTAMOINEN";
        case TROPMODEL::DAVIS:
            return "DAVIS";
        case TROPMODEL::HOPFIELD:
            return "HOPFIELD";
        case TROPMODEL::MOPS:
            return "MOPS";
        case TROPMODEL::GPTW:
            return "GPTW";
        case TROPMODEL::GPT2W:
            return "GPT2W";
        case TROPMODEL::GAL27:
            return "GAL27";
        case TROPMODEL::GALTROPO27:
            return "GALTROPO27";
        case TROPMODEL::EXTERN:
            return "EXTERN";
        case TROPMODEL::DEF_TROPMODEL:
            return "NOT DEFINED";
        default:
            return "";
        }
    }

    // convert ZTD MODEL enum to str
    // ---------
    std::string set_gproc::ztdmodel2str(ZTDMODEL ZTD)
    {
        switch (ZTD)
        {
        case ZTDMODEL::DEF_ZTDMODEL:
            return "NOT DEFINED";
        case ZTDMODEL::PWC:
            return "PWC";
        case ZTDMODEL::STO:
            return "STO";
        default:
            return "";
        }
    }

    // convert RESIDTYPE enum to str
    // ---------
    std::string set_gproc::residtype2str(RESIDTYPE RS)
    {
        switch (RS)
        {
        case RESIDTYPE::RES_ORIG:
            return "RES_ORIG";
        case RESIDTYPE::RES_NORM:
            return "RES_NORM";
        case RESIDTYPE::RES_ALL:
            return "RES_ALL";
        case RESIDTYPE::DEF_RESIDTYPE:
            return "NOT DEFINED";
        default:
            return "";
        }
    }

    std::string set_gproc::attitude2str(ATTITUDES AT)
    {
        switch (AT)
        {
        case ATTITUDES::YAW_NOMI:
            return "YAW_NOMI";
        case ATTITUDES::YAW_RTCM:
            return "YAW_RTCM";
        case ATTITUDES::DEF_YAWMODEL:
            return "DEF_YAWMODEL";
        default:
            return "";
        }
    }

    // convert OBSCOMB enum to str
    // ---------
    std::string set_gproc::obscombin2str(OBSCOMBIN OC)
    {
        switch (OC)
        {
        case OBSCOMBIN::IONO_FREE:
            return "IONO_FREE";
        case OBSCOMBIN::RAW_SINGLE:
            return "RAW_SINGLE";
        case OBSCOMBIN::RAW_DOUBLE:
            return "RAW_DOUBLE";
        case OBSCOMBIN::RAW_ALL:
            return "RAW_ALL";
        case OBSCOMBIN::DEF_OBSCOMBIN:
            return "NOT DEFINED";
        case OBSCOMBIN::MW_COMBIN:
            return "MW_COMBIN";
        case OBSCOMBIN::RAW_MIX:
            return "RAW_MIX";
        case OBSCOMBIN::IF_P1:
            return "IF_P1";
        case OBSCOMBIN::WL_COMBIN:
            return "WL_COMBIN";
        case OBSCOMBIN::EWL_COMBIN:
            return "EWL_COMBIN";
        default:
            return "";
        }
    }

    // convert CBIASCHAR enum to str
    // ---------
    std::string set_gproc::cbiaschar2str(CBIASCHAR CB)
    {
        switch (CB)
        {
        case CBIASCHAR::CHAR2:
            return "2CHAR";
        case CBIASCHAR::CHAR3:
            return "3CHAR";
        case CBIASCHAR::ORIG:
            return "ORIG";
        case CBIASCHAR::DEF_CBIASCHAR:
            return "NOT DEFINED";
        default:
            return "";
        }
    }

    std::string set_gproc::basepos2str(base_pos BP)
    {
        switch (BP)
        {
        case base_pos::RINEXH:
            return "RINEXH";
        case base_pos::CFILE:
            return "CFILE";
        case base_pos::SPP:
            return "SPP";
        case base_pos::KIN2KIN:
            return "KIN2KIN";
        case base_pos::EXTERNAL:
            return "EXTERNAL";
        default:
            return "";
        }
    }

    ///< judge whether simulation data
    bool set_gproc::simulation()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("simulation");
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    // jdhuang : add
    IFCB_MODEL set_gproc::ifcb_model()
    {
        //get rkf model node
        std::string ifcb = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_ifcb);

        if (ifcb.empty())
            return IFCB_MODEL::COR; //default value

        // delete spaces
        //ifcb.erase(remove(ifcb.begin(), ifcb.end(), ' '), ifcb.end());
        str_erase(ifcb);
        transform(ifcb.begin(), ifcb.end(), ifcb.begin(), ::toupper);

        if (ifcb == "EST")
            return IFCB_MODEL::EST;
        if (ifcb == "COR")
            return IFCB_MODEL::COR;

        return IFCB_MODEL::DEF;
    }

    //IONO_ORDER set_gproc::iono_order() {
    //  //get rkf model node
    //  std::string order = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_ION_ORDER);
    //
    //  if (order.empty()) return IONO_ORDER::NONE;
    //
    //  // delete spaces
    //  order.erase(remove(order.begin(), order.end(), ' '), order.end());
    //  order = str2upper(order);
    //
    //  if (order == "FIRST") return IONO_ORDER::FIRST_ORDER;
    //  if (order == "SECOND") return IONO_ORDER::SECOND_ORDER;
    //  if (order == "THIRD") return IONO_ORDER::THIRD_ORDER;
    //  if (order == "NONE") return IONO_ORDER::NONE;
    //
    //  throw std::logic_error("ERROR type of iono order, you should use FIRST/SECOND/NONE");
    //}

    std::string set_gproc::range_smooth_mode(int *smt_windows)
    {
        *smt_windows = 20;
        xml_node tmp_set = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child("smooth_range");

        std::string tmp_mode = tmp_set.child_value("mode");
        if (tmp_mode.empty())
            return "NONE"; //default value

        // delete spaces
        //tmp_mode.erase(remove(tmp_mode.begin(), tmp_mode.end(), ' '), tmp_mode.end());
        str_erase(tmp_mode);
        transform(tmp_mode.begin(), tmp_mode.end(), tmp_mode.begin(), ::toupper);

        if (tmp_mode != "DOPPLER" && tmp_mode != "PHASE")
            return "NONE";

        std::string tmp_window = tmp_set.child_value("window");
        if (tmp_window.empty())
            return tmp_mode;

        // delete spaces
        //tmp_window.erase(remove(tmp_window.begin(), tmp_window.end(), ' '), tmp_window.end());
        str_erase(tmp_window);
        int win_int = hwa_base::base_type_conv::str2int(tmp_window);
        *smt_windows = win_int <= 0 ? 20 : win_int;

        return tmp_mode;
    }

    bool set_gproc::bds_code_bias_correction()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("bds_code_bias_corr");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    ISFIRSTSOL set_gproc::is_firstsol()
    {
        std::string isvlbifirst = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_ISFIRSTSOL);
        //isvlbifirst.erase(remove(isvlbifirst.begin(), isvlbifirst.end(), ' '), isvlbifirst.end());
        str_erase(isvlbifirst);
        if (isvlbifirst.empty())
            return ISFIRSTSOL::NO; //throw std::logic_error("freq model is empty.");//default value
        isvlbifirst = hwa_base::base_type_conv::trim(isvlbifirst);
        transform(isvlbifirst.begin(), isvlbifirst.end(), isvlbifirst.begin(), ::toupper);

        if (isvlbifirst == "YES")
            return ISFIRSTSOL::YES;
        if (isvlbifirst == "NO")
            return ISFIRSTSOL::NO;

        return ISFIRSTSOL::YES;
    }

    ISCONSPW set_gproc::is_conspw()
    {
        std::string isconstrpw = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_ISCONSPW);
        //isconstrpw.erase(remove(isconstrpw.begin(), isconstrpw.end(), ' '), isconstrpw.end());
        str_erase(isconstrpw);
        if (isconstrpw.empty())
            return ISCONSPW::NO; //throw std::logic_error("freq model is empty.");//default value
        isconstrpw = hwa_base::base_type_conv::trim(isconstrpw);
        transform(isconstrpw.begin(), isconstrpw.end(), isconstrpw.begin(), ::toupper);

        if (isconstrpw == "YES")
            return ISCONSPW::YES;
        if (isconstrpw == "NO")
            return ISCONSPW::NO;

        return ISCONSPW::NO;
    }

    IFB_model set_gproc::ifb_model()
    {
        //get rkf model node
        std::string ifb = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_IFB);

        if (ifb.empty())
            return IFB_model::NONE; //default value

        // delete spaces
        //ifb.erase(remove(ifb.begin(), ifb.end(), ' '), ifb.end());
        str_erase(ifb);
        transform(ifb.begin(), ifb.end(), ifb.begin(), ::toupper);

        if (ifb == "EST_REC_IFB")
            return IFB_model::EST_REC_IFB;
        if (ifb == "EST_SAT_IFB")
            return IFB_model::EST_SAT_IFB;
        if (ifb == "EST_IFB")
            return IFB_model::EST_IFB;
        if (ifb == "COR_REC_IFB")
            return IFB_model::COR_REC_IFB;
        if (ifb == "COR_SAT_IFB")
            return IFB_model::COR_SAT_IFB;
        if (ifb == "COR_IFB")
            return IFB_model::COR_IFB;
        if (ifb == "NONE")
            return IFB_model::NONE;

        throw std::logic_error(
            "ERROR type of ifb model, you should use EST_REC_IFB/EST_SAT_IFB/EST_IFB/COR_REC_IFB/COR_SAT_IFB/COR_IFB/NONE");
    }

    bool set_gproc::trimcor()
    {
        //get rkf model node
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_TRIMCOR);

        if (tmp.empty())
            return true; //default value

        // delete spaces
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        if (tmp == "TRUE")
            return true;
        if (tmp == "FALSE")
            return false;

        return false;
    }

    bool set_gproc::write_equ()
    {
        //get rkf model node
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_EQU);

        if (tmp.empty())
            return true; //default value

        // delete spaces
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        if (tmp == "TRUE")
            return true;
        if (tmp == "FALSE")
            return false;

        return true;
    }

    // Hwang Shih: add
    OFSTREAM_MODE set_gproc::read_ofile_mode()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value(XMLKEY_PROC_READ_OFILE);
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        return (tmp == "REALTIME" ? OFSTREAM_MODE::REAL_TIME : OFSTREAM_MODE::READ_all); //default READ_all
    }

    int set_gproc::frequency()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("frequency");
        str_erase(tmp);
        int tmp_int = 2; //default value
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        return tmp_int;
    }

    int set_gproc::num_threads()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("num_threads");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        int tmp_int = 1; //default value
        if (tmp != "")
            tmp_int = std::stoi(tmp);
#ifdef USE_OPENMP
        if (tmp == 0)
        {
            // default std::set 1
            tmp = 1;
        }
        else if (tmp == -1)
        {
            // std::set number of cpu cores
            tmp = omp_get_num_procs();
        }
#else
        // if no use openmp number of threads is 1
        tmp = 1;
#endif
        return tmp_int;
    }

    bool set_gproc::matrix_remove()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("matrix_remove");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_gproc::cmb_equ_multi_thread()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("cmb_equ_multi_thread");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
#ifndef USE_OPENMP
    tmp_bool = false;
#endif
        return tmp_bool;
    }

    GRDMODEL set_gproc::grd_model(double *dt)
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("grd_model");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        *dt = 0.0;
        GRDMODEL TM = str2grdmodel(tmp); //default DEF_GRDMODEL
        if (TM == GRDMODEL::GRD_PWC)
        {
            int loc = tmp.find(':');
            *dt = hwa_base::base_type_conv::str2dbl(tmp.substr(loc + 1));
        }
        else if (TM == GRDMODEL::DEF_GRDMODEL)
        {
            TM = GRDMODEL::GRD_PWC;
            *dt = 120;
        }

        return TM;
    }

    GRDMODEL set_gproc::str2grdmodel(const std::string &grd)
    {
        std::string tmp = hwa_base::base_type_conv::trim(grd);
        if (tmp.substr(0, 3) == "PWC")
            return GRDMODEL::GRD_PWC;
        else if (tmp.substr(0, 3) == "STO")
            return GRDMODEL::GRD_STO;
        else
            return GRDMODEL::DEF_GRDMODEL;
    }

    double set_gproc::sig_init_sat_pcv()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("sig_init_pcv");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_sat_pcv; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sig_init_sat_pcox()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("sig_init_pcox");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_sat_pcox; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sig_init_sat_pcoy()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("sig_init_pcoy");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_sat_pcoy; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sig_init_sat_pcoz()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("sig_init_pcoz");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_sat_pcoz; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sig_init_rec_pcv()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("sig_init_pcv");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_rec_pcv; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sig_init_rec_pcox()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("sig_init_pcox");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_rec_pcox; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sig_init_rec_pcoy()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("sig_init_pcoy");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_rec_pcoy; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sig_init_rec_pcoz()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("sig_init_pcoz");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sig_init_rec_pcoz; //default value
        }
        else
        {
            return tmp_double;
        }
    }

    bool set_gproc::sat_pcv_est()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sat_pcv_est"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_gproc::rec_pcv_est()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("rec_pcv_est"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_gproc::sat_zero_con()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("zero_con"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    bool set_gproc::rec_zero_con()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("zero_con"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

    std::map<std::string, bool> set_gproc::sat_pco_xyz()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("pco_est");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "")
        {
            return _sat_pco_xyz;
        }
        else
        {
            std::map<std::string, bool> pco_xyz;
            if (tmp.find("X") != tmp.npos)
            {
                pco_xyz["pcox"] = true;
            }
            else
            {
                pco_xyz["pcox"] = false;
            }
            if (tmp.find("Y") != tmp.npos)
            {
                pco_xyz["pcoy"] = true;
            }
            else
            {
                pco_xyz["pcoy"] = false;
            }
            if (tmp.find("Z") != tmp.npos)
            {
                pco_xyz["pcoz"] = true;
            }
            else
            {
                pco_xyz["pcoz"] = false;
            }
            return pco_xyz;
        }
    }

    std::map<std::string, bool> set_gproc::rec_pco_xyz()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("pco_est");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "")
        {
            return _rec_pco_xyz;
        }
        else
        {
            std::map<std::string, bool> pco_xyz;
            if (tmp.find("X") != tmp.npos)
            {
                pco_xyz["pcox"] = true;
            }
            else
            {
                pco_xyz["pcox"] = false;
            }
            if (tmp.find("Y") != tmp.npos)
            {
                pco_xyz["pcoy"] = true;
            }
            else
            {
                pco_xyz["pcoy"] = false;
            }
            if (tmp.find("Z") != tmp.npos)
            {
                pco_xyz["pcoz"] = true;
            }
            else
            {
                pco_xyz["pcoz"] = false;
            }
            return pco_xyz;
        }
    }

    PCV_MODE set_gproc::sat_pcv_mode()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("pcv_mod");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "ZEN")
        {
            return PCV_MODE::ZENITH;
        }
        if (tmp == "AZI")
        {
            return PCV_MODE::AZIMUTH;
        }
        if (tmp == "ALL")
        {
            return PCV_MODE::ALL;
        }
        if (tmp == "NON")
        {
            return PCV_MODE::NONE;
        }
        return PCV_MODE::NONE;
    }

    PCV_MODE set_gproc::rec_pcv_mode()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("pcv_mod");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "ZEN")
        {
            return PCV_MODE::ZENITH;
        }
        if (tmp == "AZI")
        {
            return PCV_MODE::AZIMUTH;
        }
        if (tmp == "ALL")
        {
            return PCV_MODE::ALL;
        }
        if (tmp == "NON")
        {
            return PCV_MODE::NONE;
        }
        return PCV_MODE::NONE;
    }

    double set_gproc::sat_azi_beg()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("azi_beg");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sat_azi_beg;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sat_azi_end()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("azi_end");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sat_azi_end;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sat_dazi()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("dazi");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sat_dazi;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sat_zen_beg()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("zen_beg");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sat_zen_beg;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sat_zen_end()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("zen_end");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sat_zen_end;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::sat_dzen()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("dzen");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _sat_dzen;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::rec_azi_beg()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("azi_beg");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _rec_azi_beg;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::rec_azi_end()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("azi_end");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _rec_azi_end;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::rec_dazi()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("dazi");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;

        if (tmp != "")
            tmp_double = std::stod(tmp);
        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _rec_dazi;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::rec_zen_beg()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("zen_beg");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _rec_zen_beg;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::rec_zen_end()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("zen_end");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _rec_zen_end;
        }
        else
        {
            return tmp_double;
        }
    }

    double set_gproc::rec_dzen()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("dzen");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        double tmp_double = 0.0;
        if (tmp != "")
            tmp_double = std::stod(tmp);

        if (hwa_base::double_eq(tmp_double, 0.0))
        {
            return _rec_dzen;
        }
        else
        {
            return tmp_double;
        }
    }

    std::map<GOBS_LC, std::set<std::string>> set_gproc::exc_sat_list()
    {
        std::map<GOBS_LC, std::set<std::string>> satlist;
        std::map<std::string, std::set<std::string>> sats = _setval_map(XMLKEY_GNSS, XMLKEY_PROC, XMLKEY_PROC_SAT_PCV, "excsat", "freq");
        for (auto tmp = sats.begin(); tmp != sats.end(); tmp++)
        {
            GOBS_LC lc;
            std::string lcstr = tmp->first;
            if (lcstr == "L1")
            {
                lc = GOBS_LC::LC_L1;
            }
            else if (lcstr == "L2")
            {
                lc = GOBS_LC::LC_L2;
            }
            else if (lcstr == "L3")
            {
                lc = GOBS_LC::LC_L3;
            }
            else if (lcstr == "L4")
            {
                lc = GOBS_LC::LC_L4;
            }
            else if (lcstr == "L5")
            {
                lc = GOBS_LC::LC_L5;
            }
            else if (lcstr == "LC")
            {
                lc = GOBS_LC::LC_IF;
            }
            else
            {
                lc = GOBS_LC::LC_UNDEF;
            }
            satlist[lc] = tmp->second;
        }
        return satlist;
    }

    std::map<GOBS_LC, std::set<std::string>> set_gproc::exc_rec_list()
    {
        std::map<GOBS_LC, std::set<std::string>> reclist;
        std::map<std::string, std::set<std::string>> recs = set_base::_setval_map(XMLKEY_GNSS, XMLKEY_PROC, XMLKEY_PROC_REC_PCV, "excrec", "freq");
        for (auto tmp = recs.begin(); tmp != recs.end(); tmp++)
        {
            GOBS_LC lc;
            std::string lcstr = tmp->first;
            if (lcstr == "L1")
            {
                lc = GOBS_LC::LC_L1;
            }
            else if (lcstr == "L2")
            {
                lc = GOBS_LC::LC_L2;
            }
            else if (lcstr == "L3")
            {
                lc = GOBS_LC::LC_L3;
            }
            else if (lcstr == "L4")
            {
                lc = GOBS_LC::LC_L4;
            }
            else if (lcstr == "L5")
            {
                lc = GOBS_LC::LC_L5;
            }
            else if (lcstr == "LC")
            {
                lc = GOBS_LC::LC_IF;
            }
            else
            {
                lc = GOBS_LC::LC_UNDEF;
            }
            reclist[lc] = tmp->second;
        }
        return reclist;
    }

    bool set_gproc::sat_ext_pco()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("ext_pco"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false

        return tmp_bool;
    }

    bool set_gproc::rec_ext_pco()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("ext_pco"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false

        return tmp_bool;
    }

    bool set_gproc::sat_pcv_neq()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_SAT_PCV).child_value("pcv_neq"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false

        return tmp_bool;
    }

    RECEIVERTYPE set_gproc::get_receiverType()
    {
        std::string recType = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("receiverType");
        //recType.erase(remove(recType.begin(), recType.end(), ' '), recType.end());
        str_erase(recType);
        transform(recType.begin(), recType.end(), recType.begin(), ::toupper);

        if (recType.empty() || recType == "DEF")
        {
            return RECEIVERTYPE::DEF;
        }

        if (recType == "AND") //"And"
        {
            return RECEIVERTYPE::And;
        }
        else if (recType == "F9P")
        {
            return RECEIVERTYPE::F9P;
        }
        else
        {
            return RECEIVERTYPE::DEF;
        }
    }

    NPP_MODEL set_gproc::npp_model()
    {
        //get rkf model node
        std::string npp_model = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("npp_model");
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

    bool set_gproc::rec_pcv_neq()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child(XMLKEY_PROC_REC_PCV).child_value("pcv_neq"); //default false
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false

        return tmp_bool;
    }

    bool set_gproc::sat_orb_est()
    {
        bool tmp_bool = false;
        //bool tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).attribute("sat_orb_est").empty();
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sat_orb_est");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "")
        {
            return false; //Default True
        }
        else
        {
            tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sat_orb_est");
            //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
            str_erase(tmp);
            std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
            tmp_bool = tmp == "TRUE" ? true : false;
        }
        return tmp_bool;
    }

    bool set_gproc::sat_clk_est()
    {
        bool tmp_bool = false;
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sat_clk_est");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "")
        {
            return false; //Default True
        }
        else
        {
            tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("sat_clk_est");
            //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
            str_erase(tmp);
            std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
            tmp_bool = tmp == "TRUE" ? true : false;
        }
        return tmp_bool;
    }

    bool set_gproc::motion_model()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_GNSS).child(XMLKEY_PROC).child_value("motion_model");
        //tmp.erase(remove(tmp.begin(), tmp.end(), ' '), tmp.end());
        str_erase(tmp);
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        bool tmp_bool = (tmp == "TRUE" ? true : false); //default false
        return tmp_bool;
    }

} // namespace
